# Adaptive Sunrise Alarm & Ambient Monitor

**Student:** Stuparu Alessia-Ștefania · **Group:** 1222EEB  
**University:** Universitatea Politehnica din București — Facultatea de Inginerie în Limbi Străine  
**GitHub:** [fils-project-2026-alessiastuparu](https://github.com/UPB-PMRust-Students/fils-project-2026-alessiastuparu)

---

## Overview

A web-managed, dual-architecture smart alarm that wakes the user up by gradually increasing the intensity of the light and the frequency of the beeps. A **Raspberry Pi Pico 2W** hosts a Wi-Fi dashboard for setting alarms and monitoring the room. A **STM32 Nucleo-U545RE-Q** runs the hardware: it ticks a software clock, reads temperature and humidity, drives a 16-LED sunrise ring, pulses a buzzer, and listens to two physical buttons — all concurrently, without an RTOS, using Embassy's asynchronous task model.

---

## Architecture

The system is split across two microcontrollers that communicate over a UART serial bridge.

```
┌─────────────────────────────┐         UART 115200 baud        ┌──────────────────────────────┐
│    Raspberry Pi Pico 2W     │ ──── TX → RX / RX ← TX ──────   │   STM32 Nucleo-U545RE-Q      │
│                             │                                 │                              │
│  • Embassy Wi-Fi server     │   Commands:  SET_ALARM, SNOOZE  │  • Async software RTC        │
│  • HTML/JS dashboard        │   DISABLE, SYNC_TIME            │  • WS2812B LED ring (PA8)    │
│  • NTP time sync            │                                 │  • BME280 sensor (I2C1)      │
│  • Telemetry display        │   Telemetry: STATUS:TIME=...    │  • ST7735 LCD (SPI1)         │
│                             │ ◄─────────────────────────────  │  • Active buzzer (PB0)       │
└─────────────────────────────┘                                 │  • Snooze button (D10/PC9)   │
                                                                │  • Cancel button (D5/PB4)    │
                                                                └──────────────────────────────┘
```

### STM32 Task Map

The STM32 firmware runs **9 concurrent Embassy tasks**, all sharing state through a single `Mutex<CriticalSectionRawMutex, ClockState>`:

| Task | Pin(s) | What it does |
|---|---|---|
| `clock_ticker` | — | Increments HH:MM:SS every second; fires alarm at the set time; auto-cancels after 5 min |
| `buzzer_driver` | PB0 | Gentle triple-beep for the first minute, then escalating tone |
| `led_sunrise` | PA8 | WS2812B bit-bang; 5-min warm sunrise ramp before alarm; white flash when ringing |
| `display_task` | SPI1 (PA5/PA6/PA7), DC=PC7, RST=PC6 | ST7735 LCD: live clock, alarm time, status, temperature |
| `bme280_task` | I2C1 (PB6/PB7) | Reads temperature and humidity every 30 s |
| `uart_listener` | USART1 RX (PA10) | Parses commands from the Pico W |
| `telemetry_sender` | USART1 TX (PA9) | Sends STATUS frame to the Pico W every 2 s |
| `button_snooze` | PC9 / EXTI9 | Physical snooze — adds 5 min to alarm time |
| `button_cancel` | PB4 / EXTI4 | Physical cancel — fully disables alarm and LED |

---

## Hardware

| Component | Interface | Pin(s) | Purpose |
|---|---|---|---|
| STM32 Nucleo-U545RE-Q | — | — | Main ARM Cortex-M33 controller @ 80 MHz |
| Raspberry Pi Pico 2W | UART | PA9 TX / PA10 RX | Wi-Fi, web dashboard, NTP sync |
| BME280 sensor | I2C1 | PB6 SCL / PB7 SDA | Temperature & humidity readings |
| ST7735 1.8" LCD | SPI1 | PA5 CLK / PA7 MOSI / PA6 MISO / PC7 DC / PC6 RST | Visual clock and status display |
| WS2812B LED ring (16 LEDs) | Bit-bang GPIO | PA8 (D7) | Sunrise simulation & alarm flash |
| Active buzzer KY-012 | GPIO | PB0 | Audible alarm |
| Tactile button (snooze) | EXTI | PC9 (D10) | Snooze alarm +5 min |
| Tactile button (cancel) | EXTI | PB4 (D5) | Fully cancel alarm |

> **Note:** A DFPlayer Mini MP3 module and speaker are present on the breadboard but disabled — the unit was found to be faulty. The buzzer serves as the sole audio output.

### Wiring Notes

- All buttons are wired between their STM32 pin and **GND**, using the internal pull-up resistor (`Pull::Up`). A falling edge triggers the interrupt.
- The WS2812B data line runs directly from PA8 with no level shifter (3.3 V signal is accepted by most WS2812B modules at 5 V power).
- The BME280 is a 3.3 V module; SDA/SCL connect directly to the Nucleo without pull-up resistors (the I2C peripheral enables internal ones).

---

## Alarm Sequence

```
Alarm set via dashboard (SET_ALARM:HH:MM)
        │
        ▼
T − 5 min: LED ring begins sunrise ramp
        │   Deep red → orange → warm white
        │
        ▼
T − 0:  ALARM TRIGGERED
        ├── Buzzer: triple-beep pattern (first 60 s)
        ├── Buzzer: escalating continuous tone (60 s → 5 min)
        └── LED ring: full white flash
        │
        ├── SNOOZE pressed (button or dashboard)
        │       └── Alarm muted, rescheduled +5 min
        │
        ├── DISABLE pressed (button or dashboard)
        │       └── Alarm fully cancelled, LED cleared
        │
        └── 5 min elapsed with no interaction
                └── Auto-cancel
```

---

## UART Protocol

Commands travel **Pico → STM32** as plain ASCII lines terminated with `\n`:

| Command | Effect |
|---|---|
| `SET_ALARM:HH:MM` | Set and enable alarm |
| `SNOOZE` | Mute ringing, reschedule +5 min |
| `DISABLE` | Cancel alarm entirely |
| `SYNC_TIME:HH:MM:SS` | Sync software RTC to NTP time |

Telemetry travels **STM32 → Pico** every 2 seconds:

```
STATUS:TIME=HH:MM:SS,TEMP=XX.X,HUM=XX,ALARM=HH:MM,STATE=IDLE|SUNRISE|ALARMING,LED_PHASE=N,LED_PROG=N
```

---

## Software

**Language:** Rust (no\_std, no\_main)  
**Framework:** [Embassy](https://embassy.dev/) — async/await task executor for embedded  
**Toolchain:** `thumbv8m.main-none-eabihf` (ARM Cortex-M33)  
**Flash tool:** `probe-rs`

### Key Dependencies

```toml
embassy-stm32     # HAL for STM32U545 — GPIO, SPI, I2C, USART, EXTI, DMA
embassy-executor  # Async task executor
embassy-sync      # Mutex for shared state across tasks
embassy-time      # Timers and delays
st7735-lcd        # ST7735 display driver
embedded-graphics # Text and primitives for the display
cortex-m          # Low-level Cortex-M utilities (used for WS2812B bit-bang timing)
```

### Building & Flashing

```bash
# Clone the workspace
git clone https://github.com/UPB-PMRust-Students/fils-project-2026-alessiastuparu
cd alarm

# Flash the STM32 firmware
cd stm32-core
cargo run --release

# Flash the Pico firmware (separate terminal)
cd ../pico-core
cargo run --release
```

Requires `probe-rs` installed and an ST-Link debug probe (built into the Nucleo board).

---

## Bill of Materials

| Component | Unit Price |
|---|---|
| Nucleo STM32U545RE-Q | Already owned |
| Raspberry Pi Pico 2W | 39.66 RON |
| GroundStudio BME280 3V3 | 32.67 RON |
| ST7735 1.8" LCD TFT (SPI) | 19.18 RON |
| WS2812B LED Ring (16 LEDs) | 35.80 RON |
| Active Buzzer KY-012 | 13.21 RON |
| DFPlayer Mini MP3 Player* | 13.98 RON |
| Mini Speaker 3W 8Ω* | 16.67 RON |
| Tactile Buttons 6×6×5mm | 17.62 RON |
| Breadboard 830p MB-102 | 21.18 RON |
| MB102 Breadboard Power Supply | 4.69 RON |
| 9V 1A Power Adapter | 8.99 RON |
| Dupont Wires (M-M & M-F) | 19.24 RON |

*Present on breadboard, disabled due to hardware fault.

**Total: ~262 RON**

---

## Project Log

**Weeks 1–7** — Research, proposal, component selection and ordering.  
**Week 8** — Cargo workspace setup for dual-architecture cross-compilation; designed inter-chip communication protocol; wrote shared command library.  
**Week 9** — Pico 2W Wi-Fi server and HTML/JS dashboard; STM32 UART listener, time sync and alarm setting; internal software RTC and alarm state management.  
**Week 10** — Bidirectional UART telemetry; buzzer driver with escalating pattern; physical snooze/cancel buttons with EXTI; ST7735 SPI display; PLL to 80 MHz; WS2812B LED ring sunrise simulation.  
**Week 11** — Switched Pico 2W firmware to MicroPython with plain-text UART protocol; verified full system integration with live BME280 data and all peripherals working together.  
**Week 12** — WS2812B bit-bang timing resolved at 80 MHz.  
**Week 13** — KiCad schematic completed.  
**Week 14** — DFPlayer wired, tested and confirmed hardware-faulty, disabled; Physical button EXTI debugged and corrected; WS2812B boot-noise fixed with explicit OFF frame at startup.
