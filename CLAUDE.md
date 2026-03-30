# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Bare-metal Rust firmware for an **ESP32** microcontroller, driving an SSD1306 128x64 OLED display over I2C. Uses `#![no_std]` / `#![no_main]` — no operating system, no standard library.

The project name is `embassy_test` and Embassy async executor dependencies exist in `Cargo.toml` (currently commented out) but are not yet active. The current runtime uses `esp-hal`'s synchronous `#[esp_hal::main]` entry point.

## Toolchain

This project requires the **Espressif Xtensa Rust fork**, not standard rustup. The `rust-toolchain.toml` pins to `channel = "esp"` with target `xtensa-esp32-none-elf`. Install via [espup](https://github.com/esp-rs/espup):

```sh
cargo install espup
espup install
. ~/export-esp.sh   # must source this in every new shell
```

## Commands

```sh
# Build (release recommended — dev builds with opt-level="s" per Cargo.toml)
cargo build --release

# Flash to connected ESP32
cargo espflash flash --release

# Flash and open serial monitor
cargo espflash flash --release --monitor

# Serial monitor only (device already flashed)
cargo espflash monitor
```

`cargo espflash` requires `espflash`:
```sh
cargo install espflash
```

## Architecture

- **`src/main.rs`** — entire firmware; single file, single synchronous loop
- **`build.rs`** — sets `linkall.x` as the last linker script and installs a linker error-handling script that prints human-readable hints for common undefined-symbol errors (`defmt`, `esp-alloc`, etc.)

### Hardware Wiring

| Signal | GPIO |
|--------|------|
| I2C SCL | GPIO22 |
| I2C SDA | GPIO21 |

The SSD1306 display is initialized in buffered graphics mode; text is drawn via `embedded-graphics` and flushed with `display.flush()`.

### Key Dependencies

- `esp-hal` — HAL for ESP32 peripherals (I2C, Delay, etc.)
- `esp-bootloader-esp-idf` — provides `esp_app_desc!()` macro required for the IDF bootloader
- `esp-backtrace` — panic handler that prints a backtrace over UART
- `esp-println` / `log` — logging via `RUST_LOG` env var (set before flashing via espflash monitor)
- `ssd1306` + `embedded-graphics` — display driver and graphics primitives

### Enabling Embassy

The async Embassy stack (executor, time, sync, rtos) is stubbed out in `Cargo.toml` comments. To activate it, uncomment the relevant dependencies and switch the entry point from `#[esp_hal::main]` to an Embassy executor setup.
