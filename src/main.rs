#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

esp_bootloader_esp_idf::esp_app_desc!();

use core::{
    cell::RefCell,
    future::Future,
    pin::{Pin, pin},
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
    task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
};

use esp_backtrace as _;
use esp_hal::{
    Blocking, handler,
    time::Duration,
    timer::{PeriodicTimer, timg::TimerGroup},
};
use esp_println as _;
use esp_wifi_hal::{RxFilterBank, TxParameters, WiFi, WiFiRate, WiFiResources};
use static_cell::StaticCell;

// ---------------------------------------------------------------------------
// Timer interrupt globals
// ---------------------------------------------------------------------------

type Timer0<'d> = PeriodicTimer<'d, Blocking>;

static TICKS: AtomicU32 = AtomicU32::new(0);
static TIMER0: critical_section::Mutex<RefCell<Option<Timer0<'static>>>> =
    critical_section::Mutex::new(RefCell::new(None));

// ---------------------------------------------------------------------------
// Round-robin executor
//
// Each task occupies a numbered slot (0..MAX_TASKS). TASK_READY[i] is the
// "should poll" flag for slot i. When a future returns Pending it does NOT
// need to store its waker anywhere — the ISR (and the WiFi driver, via the
// waker we hand it) will flip the flag and the executor will re-poll.
//
// The waker data pointer IS the slot index, so the vtable can wake a single
// specific slot without touching the others.
// ---------------------------------------------------------------------------

const MAX_TASKS: usize = 4;

/// All tasks start ready so they each get at least one initial poll on boot.
static TASK_READY: [AtomicBool; MAX_TASKS] = [const { AtomicBool::new(true) }; MAX_TASKS];

static TASK_VTABLE: RawWakerVTable = RawWakerVTable::new(
    |p| RawWaker::new(p, &TASK_VTABLE),                               // clone
    |p| TASK_READY[p as usize].store(true, Ordering::Release),        // wake (consuming)
    |p| TASK_READY[p as usize].store(true, Ordering::Release),        // wake_by_ref
    |_| {},                                                            // drop
);

/// Drive a fixed slice of futures round-robin, sleeping when all are idle.
///
/// The waker given to slot `i` sets `TASK_READY[i]`, so any async driver
/// (WiFi, timers) can wake exactly the task that is waiting for it.
fn run_tasks(tasks: &mut [Pin<&mut dyn Future<Output = ()>>]) -> ! {
    loop {
        let mut any_polled = false;
        for (i, task) in tasks.iter_mut().enumerate() {
            if TASK_READY[i].swap(false, Ordering::AcqRel) {
                any_polled = true;
                // Safety: i fits in a pointer; the vtable never dereferences it.
                let waker = unsafe { Waker::new(i as *const (), &TASK_VTABLE) };
                let mut cx = Context::from_waker(&waker);
                let _ = task.as_mut().poll(&mut cx);
            }
        }
        // If no task was runnable, halt until the next interrupt. This is
        // Xtensa's atomic sleep-until-interrupt: it lowers INTLEVEL to 0
        // and idles in a single instruction, so a wake that arrives between
        // the flag check above and the waiti cannot be lost.
        if !any_polled {
            unsafe { core::arch::asm!("waiti 0") };
        }
    }
}

// ---------------------------------------------------------------------------
// Timer ISR
//
// Fires every TICK_PERIOD ms. Increments TICKS, then wakes ALL task slots.
// WaitForTick futures simply re-check their deadline on the next poll —
// no per-task waker storage needed.
// ---------------------------------------------------------------------------

#[handler]
fn tg0_t0_handler() {
    critical_section::with(|cs| {
        if let Some(t) = TIMER0.borrow_ref_mut(cs).as_mut() {
            t.clear_interrupt();
        }
    });
    TICKS.fetch_add(1, Ordering::Release);
    for slot in &TASK_READY {
        slot.store(true, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// WaitForTick
//
// Suspends until `target` timer interrupts have fired. Because the ISR sets
// all TASK_READY flags on every tick, no waker needs to be stored: the
// executor will re-poll this future on the tick after it became Pending.
// ---------------------------------------------------------------------------

struct WaitForTick {
    target: u32,
}

impl WaitForTick {
    fn after(ticks: u32) -> Self {
        Self {
            target: TICKS.load(Ordering::Relaxed).wrapping_add(ticks),
        }
    }
}

impl Future for WaitForTick {
    type Output = ();
    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if TICKS.load(Ordering::Acquire) >= self.target {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

// ---------------------------------------------------------------------------
// WiFi helpers
// ---------------------------------------------------------------------------

const WIFI_CHANNEL: u8 = 1;

/// Locally-administered MAC for device A (sender).
#[cfg(feature = "sender")]
const MY_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
#[cfg(feature = "sender")]
const PEER_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x02];

/// Locally-administered MAC for device B (receiver/echo).
#[cfg(feature = "receiver")]
const MY_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x02];
#[cfg(feature = "receiver")]
const PEER_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

/// Write a minimal 802.11 data frame into `buf` and return the byte count.
///
/// Frame layout (24-byte MAC header + payload):
///   FC(2) | Duration(2) | DA(6) | SA(6) | BSSID(6) | SeqCtrl(2) | payload
///
/// SeqCtrl is zeroed here; `override_seq_num: true` in TxParameters tells
/// the hardware to fill it in correctly before transmission.
#[cfg(any(feature = "sender", feature = "receiver"))]
fn build_frame(dst: &[u8; 6], src: &[u8; 6], payload: &[u8], buf: &mut [u8]) -> usize {
    const HDR: usize = 24;
    let total = HDR + payload.len();
    buf[0] = 0x08; buf[1] = 0x00;           // Frame Control: Data
    buf[2] = 0x00; buf[3] = 0x00;           // Duration/ID
    buf[4..10].copy_from_slice(dst);         // Address 1 — DA
    buf[10..16].copy_from_slice(src);        // Address 2 — SA
    buf[16..22].copy_from_slice(dst);        // Address 3 — BSSID (reuse DA)
    buf[22] = 0x00; buf[23] = 0x00;         // Sequence Control
    buf[HDR..total].copy_from_slice(payload);
    total
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

/// Heartbeat: logs the running tick count on every timer interrupt.
/// Runs on both devices to confirm the executor and timer are alive.
async fn heartbeat_task() {
    let mut count: u32 = 0;
    loop {
        defmt::info!("tick {}", count);
        count += 1;
        WaitForTick::after(1).await;
    }
}

/// Sender (device A): every 5 ticks transmit a HELLO frame, then wait for
/// the echo reply before sleeping again.
///
/// Build with: `cargo espflash flash --release --features sender`
#[cfg(feature = "sender")]
async fn sender_task(wifi: &WiFi<'_>) {
    loop {
        // --- transmit ---
        let mut frame = [0u8; 256];
        let len = build_frame(&PEER_MAC, &MY_MAC, b"HELLO", &mut frame);
        wifi.transmit(
            &mut frame[..len],
            &TxParameters {
                rate: WiFiRate::PhyRate1ML,
                override_seq_num: true,
                ..Default::default()
            },
            None,
        )
        .await
        .ok();
        defmt::info!("TX: HELLO");

        // --- wait for echo reply from peer ---
        // Discard any frames that didn't come from our peer (e.g. beacons).
        loop {
            let reply = wifi.receive().await;
            let mpdu = reply.mpdu_buffer();
            // Address 2 (SA) lives at bytes 10-15 of the 802.11 header.
            let from_peer = mpdu.len() >= 16 && &mpdu[10..16] == &PEER_MAC;
            if from_peer {
                let payload = if mpdu.len() > 24 { &mpdu[24..] } else { &[] };
                defmt::info!("RX echo: {} bytes — {:?}", payload.len(), payload);
                drop(reply);
                break;
            }
            drop(reply);
        }

        WaitForTick::after(5).await;
    }
}

/// Echo (device B): receive any frame from the peer and send it straight back
/// with the addresses swapped.
///
/// Build with: `cargo espflash flash --release --features receiver`
#[cfg(feature = "receiver")]
async fn echo_task(wifi: &WiFi<'_>) {
    loop {
        let frame = wifi.receive().await;
        let mpdu = frame.mpdu_buffer();

        // Filter: only echo frames whose SA (bytes 10-15) is our peer.
        if mpdu.len() < 16 || &mpdu[10..16] != &PEER_MAC {
            drop(frame);
            continue;
        }

        // Copy the full MPDU out before releasing the borrowed buffer.
        let mut buf = [0u8; 256];
        let len = mpdu.len().min(buf.len());
        buf[..len].copy_from_slice(&mpdu[..len]);
        drop(frame);

        // Swap DA ↔ SA in the 802.11 header so the frame goes back to sender.
        buf[4..10].copy_from_slice(&PEER_MAC); // Address 1 — DA (original sender)
        buf[10..16].copy_from_slice(&MY_MAC);  // Address 2 — SA (us)

        wifi.transmit(
            &mut buf[..len],
            &TxParameters {
                rate: WiFiRate::PhyRate1ML,
                override_seq_num: true,
                ..Default::default()
            },
            None,
        )
        .await
        .ok();
        defmt::info!("Echoed {} bytes", len);
    }
}

// ---------------------------------------------------------------------------
// Static storage for WiFi DMA descriptors (must outlive WiFi<'_>).
// ---------------------------------------------------------------------------

static WIFI_RESOURCES: StaticCell<WiFiResources<10>> = StaticCell::new();

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    // --- Timer: 3 s per tick ---
    let tg0 = TimerGroup::new(peripherals.TIMG0);
    let mut timer0 = PeriodicTimer::new(tg0.timer0);
    timer0.set_interrupt_handler(tg0_t0_handler);
    timer0.start(Duration::from_millis(3000)).unwrap();
    timer0.listen();
    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });

    // --- WiFi ---
    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.ADC2,
        WIFI_RESOURCES.init(WiFiResources::new()),
    );
    wifi.set_channel(WIFI_CHANNEL).ok();

    // Configure RX filters so the MAC hardware actually passes frames to DMA.
    // Without this the filter is in an undefined default state and receive()
    // waits forever — frames are dropped in hardware before reaching the driver.
    //
    // We register our MAC as the expected Receiver Address on interface 0,
    // disable the BSSID check (we're not in an infrastructure BSS), then
    // flush any stale frames left over from the channel-change reinit.
    wifi.set_filter(RxFilterBank::ReceiverAddress, 0, MY_MAC, [0xff; 6]).ok();
    wifi.set_filter_status(RxFilterBank::ReceiverAddress, 0, true).ok();
    wifi.set_filter_bssid_check(0, false).ok();
    wifi.clear_rx_queue();

    // --- Spawn tasks (role selected at compile time) ---
    let mut heartbeat = pin!(heartbeat_task());

    #[cfg(feature = "sender")]
    let mut role = pin!(sender_task(&wifi));
    #[cfg(feature = "receiver")]
    let mut role = pin!(echo_task(&wifi));

    let mut tasks: [Pin<&mut dyn Future<Output = ()>>; 2] =
        [heartbeat.as_mut(), role.as_mut()];

    run_tasks(&mut tasks)
}
