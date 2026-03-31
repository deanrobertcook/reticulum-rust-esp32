#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

use core::{
    cell::RefCell,
    future::Future,
    pin::{Pin, pin},
    sync::atomic::{AtomicU32, Ordering},
    task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
};

use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::{
    Blocking, handler,
    i2c::master::{Config, I2c},
    time::{Duration, Instant},
    timer::{PeriodicTimer, timg::TimerGroup},
};
use esp_println as _;
use ssd1306::{
    I2CDisplayInterface, Ssd1306, mode::DisplayConfig, prelude::DisplayRotation,
    size::DisplaySize128x64,
};

// PeriodicTimer auto-reloads after each interrupt, so the ISR only needs
// to call clear_interrupt() — no manual reload required.
type Timer0<'d> = PeriodicTimer<'d, Blocking>;

// ---------------------------------------------------------------------------
// Timer interrupt globals
//
// TICKS   — incremented by the ISR each time the hardware timer fires.
// TIMER0  — the timer peripheral itself; the ISR needs it to clear the
//           interrupt flag and reload the countdown before the next tick.
// WAKER   — where a future parks its Waker so the ISR can call wake().
//           The ISR takes the waker out (Option → None) to avoid waking
//           twice; the future re-stores it on the next Pending return.
// ---------------------------------------------------------------------------
static TICKS: AtomicU32 = AtomicU32::new(0);
static TIMER0: critical_section::Mutex<RefCell<Option<Timer0>>> =
    critical_section::Mutex::new(RefCell::new(None));
static WAKER: critical_section::Mutex<RefCell<Option<Waker>>> =
    critical_section::Mutex::new(RefCell::new(None));

// ---------------------------------------------------------------------------
// Executor
//
// A "spin" executor: polls the future in a tight loop. No task queue, no
// sleep. The waker is a no-op because we unconditionally re-poll anyway.
// ---------------------------------------------------------------------------

static VTABLE: RawWakerVTable = RawWakerVTable::new(
    |p| RawWaker::new(p, &VTABLE), // clone  — keep the same data pointer
    |_| {},                        // wake (consuming)  — nothing to do
    |_| {},                        // wake_by_ref       — nothing to do
    |_| {},                        // drop              — nothing to free
);

fn block_on<F: Future>(future: F) -> F::Output {
    // Safety: the vtable functions are all no-ops so the null data pointer
    // is never dereferenced.
    let waker = unsafe { Waker::new(core::ptr::null(), &VTABLE) };
    let mut cx = Context::from_waker(&waker);
    // pin! fixes the future in place on the stack so its address is stable
    // across polls (futures may contain self-referential pointers).
    let mut future = pin!(future);
    loop {
        match future.as_mut().poll(&mut cx) {
            Poll::Ready(val) => return val,
            Poll::Pending => {} // spin: just try again immediately
        }
    }
}

// ---------------------------------------------------------------------------
// Futures
//
// Each struct is a hand-written state machine. `poll` advances it one step
// and returns Ready(val) when done or Pending to be polled again.
// ---------------------------------------------------------------------------

/// Yields control back to the executor exactly once, then completes.
/// Demonstrates that a future can return Pending without being "stuck" —
/// it will be re-polled on the next iteration of block_on's loop.
struct YieldNow(bool /* already_yielded */);

impl Future for YieldNow {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if self.0 {
            Poll::Ready(())
        } else {
            self.0 = true;
            // Tell the executor we want to be polled again.
            // In the spin executor this is a no-op, but it's the correct
            // contract: a future that returns Pending MUST arrange for
            // wake() to be called, otherwise the executor may park the task.
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

/// Busy-polls until `duration` has elapsed since first poll.
/// On each Pending return it calls wake_by_ref() to request re-polling.
struct AsyncDelay {
    deadline: Option<Instant>,
    duration: Duration,
}

impl AsyncDelay {
    fn new(duration: Duration) -> Self {
        Self {
            deadline: None,
            duration,
        }
    }
}

impl Future for AsyncDelay {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let now = Instant::now();
        // Set the deadline on the first poll; keep it on subsequent polls.
        let d = self.duration.clone();
        let deadline = self.deadline.get_or_insert(now + d);
        if now >= *deadline {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

/// Waits until the global TICKS counter reaches `target`.
///
/// On each Pending return it stores its Waker into the WAKER static so
/// that the timer ISR can call wake() when the next tick fires. This is
/// the correct contract for a non-spinning future: rather than calling
/// wake_by_ref() itself (like AsyncDelay does), it delegates that call
/// to external hardware.
///
/// In our spin executor the waker is still a no-op, so this future
/// effectively busy-waits — but the architecture is now correct. Swap
/// in a real executor (Embassy) and the CPU would sleep between ticks.
struct WaitForTick {
    target: u32,
}

impl WaitForTick {
    /// Complete after `ticks` timer interrupts have fired.
    fn after(ticks: u32) -> Self {
        Self {
            target: TICKS.load(Ordering::Relaxed).wrapping_add(ticks),
        }
    }
}

impl Future for WaitForTick {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if TICKS.load(Ordering::Acquire) >= self.target {
            return Poll::Ready(());
        }
        // Park the waker so the ISR can wake us on the next tick.
        critical_section::with(|cs| {
            *WAKER.borrow_ref_mut(cs) = Some(cx.waker().clone());
        });
        // Check again after storing the waker to close the race window:
        // the ISR might have fired between the first check and the store.
        if TICKS.load(Ordering::Acquire) >= self.target {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

// ---------------------------------------------------------------------------
// Timer ISR
//
// The handler attribute sets the correct calling convention and interrupt
// return instruction for Xtensa. The function is placed in IRAM (#[ram])
// so it runs from fast on-chip RAM rather than slower flash.
// ---------------------------------------------------------------------------

#[handler]
fn tg0_t0_handler() {
    // 1. Clear the interrupt flag. PeriodicTimer auto-reloads the countdown,
    //    so there's nothing else to do to make it fire again.
    critical_section::with(|cs| {
        if let Some(t) = TIMER0.borrow_ref_mut(cs).as_mut() {
            t.clear_interrupt();
        }
    });
    defmt::debug!("Tick handler called!");

    // 2. Count the tick. Release ordering so any future that loads with
    //    Acquire sees all writes done before this increment.
    TICKS.fetch_add(1, Ordering::Release);

    // 3. Wake the parked future, if any. We take it out of the Option so
    //    we don't double-wake; the future will re-store it on next Pending.
    let waker = critical_section::with(|cs| WAKER.borrow_ref_mut(cs).take());
    if let Some(w) = waker {
        w.wake();
    }
}

// ---------------------------------------------------------------------------
// Async application logic
// ---------------------------------------------------------------------------

async fn run() -> ! {
    // --- demonstrate YieldNow ---
    defmt::info!("poll 1: about to yield");
    YieldNow(false).await; // returns Pending on first poll, Ready on second
    defmt::info!("poll 2: resumed after yield");

    // --- main loop driven by the hardware timer ISR ---
    let mut count: u32 = 0;
    loop {
        defmt::info!("tick {}", count);
        count += 1;
        // WaitForTick suspends here; the ISR increments TICKS and calls
        // wake(), which causes block_on to poll this future again.
        WaitForTick::after(1).await;
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_println::logger::init_logger_from_env();

    // --- Set up TIMG0 Timer0 to fire every 500 ms ---
    //
    // PeriodicTimer wraps the raw timg timer and exposes the public API.
    // `start` sets the period and begins counting; `listen` enables the
    // interrupt line so the ISR fires when the period elapses.
    let tg0 = TimerGroup::new(peripherals.TIMG0);
    let mut timer0 = PeriodicTimer::new(tg0.timer0);
    timer0.set_interrupt_handler(tg0_t0_handler);
    timer0.start(Duration::from_millis(500)).unwrap();
    timer0.listen();
    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_scl(peripherals.GPIO22)
        .with_sda(peripherals.GPIO21);

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();

    // Hand off to the async world. block_on drives `run()` to completion
    // (which never happens — run() loops forever — so this also loops forever).
    block_on(run())
}
