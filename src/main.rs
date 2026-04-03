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
use embedded_graphics::{
    Drawable,
    draw_target::DrawTarget,
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::{
    Blocking, handler,
    i2c::master::{Config, I2c},
    time::Duration,
    timer::{PeriodicTimer, timg::TimerGroup},
};
use esp_println as _;
use esp_wifi_hal::{RxFilterBank, TxParameters, WiFi, WiFiRate, WiFiResources};
use ssd1306::{
    I2CDisplayInterface, Ssd1306,
    mode::{BufferedGraphicsMode, DisplayConfig},
    prelude::{DisplayRotation, WriteOnlyDataCommand},
    size::{DisplaySize, DisplaySize128x64},
};
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
    |p| RawWaker::new(p, &TASK_VTABLE),                        // clone
    |p| TASK_READY[p as usize].store(true, Ordering::Release), // wake (consuming)
    |p| TASK_READY[p as usize].store(true, Ordering::Release), // wake_by_ref
    |_| {},                                                    // drop
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
// Shared display state
//
// MSG_INDEX is set by the WiFi tasks whenever a message is sent or received.
// The display task watches it and redraws only when the value changes.
//
// Both devices share the same MESSAGES array (compiled into each binary), so
// a single index byte in the frame payload is enough to keep screens in sync.
// ---------------------------------------------------------------------------

const MESSAGES: &[&str] = &[
    "Hello!",
    "How are you?",
    "Rust on ESP32",
    "No OS needed",
    "WiFi works!",
    "Open MAC ftw",
    "Ping!",
    "Reticulum?",
];

/// Set by WiFi tasks to tell the display task which message to show.
/// Initialised to MESSAGES.len() (out of range) so the display draws on boot.
static MSG_INDEX: AtomicU32 = AtomicU32::new(u32::MAX);

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
/// Payload layout: [msg_index: u8] — one byte is enough to identify the message.
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
    buf[0] = 0x08;
    buf[1] = 0x00; // Frame Control: Data
    buf[2] = 0x00;
    buf[3] = 0x00; // Duration/ID
    buf[4..10].copy_from_slice(dst); // Address 1 — DA
    buf[10..16].copy_from_slice(src); // Address 2 — SA
    buf[16..22].copy_from_slice(dst); // Address 3 — BSSID (reuse DA)
    buf[22] = 0x00;
    buf[23] = 0x00; // Sequence Control (overridden by HW)
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
        WaitForTick::after(10).await;
    }
}

/// Display: redraws the screen whenever MSG_INDEX changes.
///
/// Generic over the display hardware so it works with any SSD1306 interface.
async fn display_task<DI, SIZE>(mut display: Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>)
where
    DI: WriteOnlyDataCommand,
    SIZE: DisplaySize,
    Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>: DrawTarget<Color = BinaryColor>,
{
    #[cfg(feature = "sender")]
    let mut name = "sender:";
    #[cfg(feature = "receiver")]
    let mut name = "receiver:";

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut last_drawn = u32::MAX - 1; // different from initial MSG_INDEX sentinel

    loop {
        let idx = MSG_INDEX.load(Ordering::Acquire);
        if idx != last_drawn {
            last_drawn = idx;
            let msg = MESSAGES[idx as usize % MESSAGES.len()];
            display.clear(BinaryColor::Off).ok();
            Text::with_baseline(name, Point::zero(), text_style, Baseline::Top)
                .draw(&mut display)
                .ok();
            Text::with_baseline(msg, Point::new(0, 16), text_style, Baseline::Top)
                .draw(&mut display)
                .ok();
            display.flush().ok();
            defmt::info!("Display: {}", msg);
        }
        WaitForTick::after(1).await;
    }
}

/// Sender (device A): cycles through MESSAGES, transmits the index each round,
/// waits for the echo, then pauses before the next send.
///
/// Build with: `cargo espflash flash --release --features sender`
#[cfg(feature = "sender")]
async fn sender_task(wifi: &WiFi<'_>) {
    let mut counter: u32 = 0;
    loop {
        let idx = (counter as usize) % MESSAGES.len();

        // Update local display immediately so sender sees the message it's about to send.
        MSG_INDEX.store(idx as u32, Ordering::Release);

        // Payload: single byte carrying the message index.
        let payload = [idx as u8];
        let mut frame = [0u8; 256];
        let len = build_frame(&PEER_MAC, &MY_MAC, &payload, &mut frame);

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
        defmt::info!("TX idx={} \"{}\"", idx, MESSAGES[idx]);

        // --- wait for echo reply from peer ---
        // Discard any frames that didn't come from our peer (e.g. beacons).
        loop {
            let reply = wifi.receive().await;
            let mpdu = reply.mpdu_buffer();
            // Address 2 (SA) lives at bytes 10-15 of the 802.11 header.
            let from_peer = mpdu.len() >= 16 && &mpdu[10..16] == &PEER_MAC;
            if from_peer {
                defmt::info!("RX echo confirmed");
                drop(reply);
                break;
            }
            drop(reply);
        }

        counter = counter.wrapping_add(1);
        WaitForTick::after(10).await;
    }
}

/// Echo (device B): receives a frame from the peer, updates the display with
/// the embedded message index, then echoes the frame back.
///
/// Build with: `cargo espflash flash --release --features receiver`
#[cfg(feature = "receiver")]
async fn echo_task(wifi: &WiFi<'_>) {
    loop {
        let frame = wifi.receive().await;
        let mpdu = frame.mpdu_buffer();

        // Filter: only handle frames from our peer.
        if mpdu.len() < 16 || &mpdu[10..16] != &PEER_MAC {
            drop(frame);
            continue;
        }

        // Read the message index from the first payload byte (after the 24-byte header).
        if mpdu.len() > 24 {
            let idx = (mpdu[24] as usize) % MESSAGES.len();
            MSG_INDEX.store(idx as u32, Ordering::Release);
            defmt::info!("RX idx={} \"{}\"", idx, MESSAGES[idx]);
        }

        // Copy the full MPDU out before releasing the borrowed buffer.
        // Then swap DA ↔ SA so the frame goes back to the sender.
        let mut buf = [0u8; 256];
        let len = mpdu.len().min(buf.len());
        buf[..len].copy_from_slice(&mpdu[..len]);
        drop(frame);

        buf[4..10].copy_from_slice(&PEER_MAC); // DA = original sender
        buf[10..16].copy_from_slice(&MY_MAC); // SA = us

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

    // --- Display ---
    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_scl(peripherals.GPIO22)
        .with_sda(peripherals.GPIO21);

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // --- Timer: 3 s per tick ---
    let tg0 = TimerGroup::new(peripherals.TIMG0);
    let mut timer0 = PeriodicTimer::new(tg0.timer0);
    timer0.set_interrupt_handler(tg0_t0_handler);
    timer0.start(Duration::from_millis(100)).unwrap();
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
    wifi.set_filter(RxFilterBank::ReceiverAddress, 0, MY_MAC, [0xff; 6])
        .ok();
    wifi.set_filter_status(RxFilterBank::ReceiverAddress, 0, true)
        .ok();
    wifi.set_filter_bssid_check(0, false).ok();
    wifi.clear_rx_queue();

    // --- Spawn tasks ---
    // Slot 0: heartbeat  Slot 1: WiFi role  Slot 2: display
    let mut heartbeat = pin!(heartbeat_task());
    let mut disp = pin!(display_task(display));

    #[cfg(feature = "sender")]
    let mut role = pin!(sender_task(&wifi));
    #[cfg(feature = "receiver")]
    let mut role = pin!(echo_task(&wifi));

    let mut tasks: [Pin<&mut dyn Future<Output = ()>>; 3] =
        [heartbeat.as_mut(), role.as_mut(), disp.as_mut()];

    run_tasks(&mut tasks)
}
