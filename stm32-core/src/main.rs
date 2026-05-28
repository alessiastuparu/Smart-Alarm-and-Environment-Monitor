// STM32 Alarm OS — main.rs
// Stuparu Alessia-Ștefania, 1222EEB

#![no_std]   
#![no_main]  

use defmt::{info, warn};
use defmt_rtt as _;          
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;        

use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale,
};
use embassy_stm32::i2c::I2c;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, Uart, UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X18_BOLD};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Text;

use heapless::String;
use core::fmt::Write as FmtWrite;

// ─── Shared state ─────────────────────────────────────────────────────────────
//
// This struct holds every piece of mutable state the firmware cares about.
// It lives in a static Mutex so any task can lock it, read/write what it needs,
// and then release the lock. Because we use CriticalSectionRawMutex, locking
// briefly disables interrupts — which is safe on a single-core Cortex-M33.

struct ClockState {
    hour:            u8,
    minute:          u8,
    second:          u8,
    alarm_hour:      u8,
    alarm_minute:    u8,
    alarm_enabled:   bool,
    is_ringing:      bool,
    ringing_seconds: u32,   // Counts up while the alarm is active
    temp_tenths:     i16,   // (285 = 28.5 °C)
    hum_pct:         u8,
    led_phase:       u8,    // 0 = off, 1–3 = sunrise phase
    led_prog:        u8,    // Sunrise progress 0–100%
}

static CLOCK_STATE: Mutex<CriticalSectionRawMutex, ClockState> = Mutex::new(ClockState {
    hour:            0,
    minute:          0,
    second:          0,
    alarm_hour:      7,
    alarm_minute:    0,
    alarm_enabled:   false,
    is_ringing:      false,
    ringing_seconds: 0,
    temp_tenths:     225,
    hum_pct:         50,
    led_phase:       0,
    led_prog:        0,
});

// ─── Interrupt bindings ───────────────────────────────────────────────────────

bind_interrupts!(struct Irqs {
    USART1          => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
    GPDMA1_CHANNEL0 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH1>;
    GPDMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH2>;
    GPDMA1_CHANNEL3 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH3>;
    GPDMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH4>;
    GPDMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH5>;
    EXTI9           => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9>;
    EXTI4           => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
    I2C1_EV         => embassy_stm32::i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER         => embassy_stm32::i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

// ─── WS2812B bit-bang on PA8 ────────────────────────────────────

#[derive(Copy, Clone)]
struct LedColor { r: u8, g: u8, b: u8 }

impl LedColor {
    const OFF: Self = Self { r: 0, g: 0, b: 0 };
}
const GPIOA_BSRR: *mut u32 = 0x4202_0018 as *mut u32;
const PA8_HIGH: u32 = 1 << 8;
const PA8_LOW:  u32 = 1 << 24;

fn send_ws2812_byte(byte: u8) {
    // Send 8 bits MSB-first. Each bit is a pulse: long high = 1, short high = 0.
    // Delay counts are tuned for 80 MHz: 1 cycle ≈ 12.5 ns.
    for i in (0..8).rev() {
        unsafe {
            if (byte >> i) & 1 == 1 {
                // Logic 1: ~800 ns high, ~450 ns low
                core::ptr::write_volatile(GPIOA_BSRR, PA8_HIGH);
                cortex_m::asm::delay(16);
                core::ptr::write_volatile(GPIOA_BSRR, PA8_LOW);
                cortex_m::asm::delay(9);
            } else {
                // Logic 0: ~400 ns high, ~850 ns low
                core::ptr::write_volatile(GPIOA_BSRR, PA8_HIGH);
                cortex_m::asm::delay(8);
                core::ptr::write_volatile(GPIOA_BSRR, PA8_LOW);
                cortex_m::asm::delay(17);
            }
        }
    }
}

fn ws2812_send(pin: &mut Output, colors: &[LedColor; 16]) {
    let _ = pin; 
    cortex_m::interrupt::free(|_| {
        for c in colors.iter() {
            send_ws2812_byte(c.g);
            send_ws2812_byte(c.r);
            send_ws2812_byte(c.b);
        }
        unsafe { core::ptr::write_volatile(GPIOA_BSRR, PA8_LOW); }
    });
    cortex_m::asm::delay(40_000);
}

fn lerp_color(a: LedColor, b: LedColor, t: u8) -> LedColor {
    let t = t as u16;
    LedColor {
        r: (a.r as u16 + (b.r as u16).wrapping_sub(a.r as u16).wrapping_mul(t) / 100) as u8,
        g: (a.g as u16 + (b.g as u16).wrapping_sub(a.g as u16).wrapping_mul(t) / 100) as u8,
        b: (a.b as u16 + (b.b as u16).wrapping_sub(a.b as u16).wrapping_mul(t) / 100) as u8,
    }
}

// Sunrise keyframes: 6 colour stops mapped to progress values 0–100.
const KF: [(u8, LedColor); 6] = [
    (0,   LedColor { r: 2,   g: 0,   b: 0  }),  // Near black
    (20,  LedColor { r: 20,  g: 3,   b: 0  }),  // Deep red
    (40,  LedColor { r: 80,  g: 15,  b: 0  }),  // Dark orange
    (60,  LedColor { r: 180, g: 60,  b: 5  }),  // Warm orange
    (80,  LedColor { r: 255, g: 140, b: 20 }),  // Golden
    (100, LedColor { r: 255, g: 220, b: 80 }),  // Warm white
];

fn sunrise_color(prog: u8) -> LedColor {
    for i in 0..KF.len() - 1 {
        let (p0, c0) = KF[i];
        let (p1, c1) = KF[i + 1];
        if prog >= p0 && prog <= p1 {
            let span  = (p1 - p0) as u16;
            let local = ((prog - p0) as u16 * 100 / span) as u8;
            return lerp_color(c0, c1, local);
        }
    }
    KF[KF.len() - 1].1
}

// ─── Task: Clock ticker ───────────────────────────────────────────────────────
//
// This task wakes every second, increments the software clock
// and checks whether the alarm should fire.
// If the alarm has been ringing for 5 minutes with no user interaction,
// it auto-cancels so the buzzer does not run indefinitely.

#[embassy_executor::task]
async fn clock_ticker() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        let mut s = CLOCK_STATE.lock().await;

        s.second += 1;
        if s.second >= 60 {
            s.second = 0;
            s.minute += 1;
            if s.minute >= 60 {
                s.minute = 0;
                s.hour += 1;
                if s.hour >= 24 { s.hour = 0; }
            }
        }

        // Trigger the alarm when the clock reaches the set time
        if s.alarm_enabled && !s.is_ringing
            && s.hour == s.alarm_hour
            && s.minute == s.alarm_minute
            && s.second == 0
        {
            s.is_ringing      = true;
            s.ringing_seconds = 0;
            info!("ALARM TRIGGERED!");
        }

        // Count how long the alarm has been ringing; auto-cancel after 5 minutes
        if s.is_ringing {
            s.ringing_seconds += 1;
            if s.ringing_seconds >= 300 {
                s.is_ringing      = false;
                s.alarm_enabled   = false;
                s.ringing_seconds = 0;
                info!("Alarm auto-cancelled after 5 minutes.");
            }
        }
    }
}

// ─── Task: Buzzer ─────────────────────────────────────────────────────────────
//
// The buzzer pattern escalates over time to encourage the user to wake up.
// For the first 60 seconds it plays a triple-beep every ~9 seconds.
// After that, it switches to a continuous tone whose frequency increases
// every minute until it hits the fastest rate at 3 minutes.

#[embassy_executor::task]
async fn buzzer_driver(mut pin: Output<'static>) {
    info!("Buzzer task started.");
    loop {
        let (ringing, secs) = {
            let s = CLOCK_STATE.lock().await;
            (s.is_ringing, s.ringing_seconds)
        };

        if ringing {
            if secs < 60 {
                // triple-beep pattern for the first minute
                for _ in 0..3u8 {
                    pin.set_high();
                    Timer::after(Duration::from_millis(80)).await;
                    pin.set_low();
                    Timer::after(Duration::from_millis(120)).await;
                }
                Timer::after(Duration::from_millis(8_400)).await;
            } else {
                // continuous tone: period halves every minute
                let period_ms: u64 = if secs < 120 { 350 }
                                     else if secs < 180 { 200 }
                                     else { 100 };
                pin.set_high();
                Timer::after(Duration::from_millis(period_ms)).await;
                pin.set_low();
                Timer::after(Duration::from_millis(period_ms)).await;
            }
        } else {
            pin.set_low();
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

// ─── Task: Buttons ────────────────────────────────────────────────────────────
//
// Both buttons use EXTI (External Interrupt) lines so the MCU does not need to poll them.

#[embassy_executor::task]
async fn button_snooze(mut btn: ExtiInput<'static, Async>) {
    loop {
        btn.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(50)).await; 
        info!("BTN snooze detected");
        let mut s = CLOCK_STATE.lock().await;
        if s.is_ringing {
            // snooze: silence the alarm and reschedule it 5 minutes later
            s.is_ringing      = false;
            s.ringing_seconds = 0;
            let total = s.alarm_minute as u16 + 5;
            s.alarm_minute = (total % 60) as u8;
            if total >= 60 { s.alarm_hour = (s.alarm_hour + 1) % 24; }
            info!("Physical snooze -> next alarm {:02}:{:02}", s.alarm_hour, s.alarm_minute);
        }
    }
}

#[embassy_executor::task]
async fn button_cancel(mut btn: ExtiInput<'static, Async>) {
    loop {
        btn.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(50)).await; 
        info!("BTN cancel detected");
        let mut s = CLOCK_STATE.lock().await;
        if s.is_ringing || s.alarm_enabled {
            // cancel: clear alarm state and reset the LED ring
            s.alarm_enabled   = false;
            s.is_ringing      = false;
            s.ringing_seconds = 0;
            s.led_phase       = 0;
            s.led_prog        = 0;
            info!("Physical cancel -- alarm fully disabled.");
        }
    }
}

// ─── Task: WS2812B LED Sunrise ────────────────────────────────────────────────

#[embassy_executor::task]
async fn led_sunrise(mut pin: Output<'static>) {
    info!("LED sunrise task started.");

    // Clear garbage colour the LEDs may have latched during power-up/flashing
    ws2812_send(&mut pin, &[LedColor::OFF; 16]);

    // Boot test: show red for 2s so we check if it's working
    ws2812_send(&mut pin, &[LedColor { r: 80, g: 0, b: 0 }; 16]);
    info!("LED ring: boot test (red, 2 s)");
    Timer::after(Duration::from_secs(2)).await;
    ws2812_send(&mut pin, &[LedColor::OFF; 16]);

    let mut flash_toggle = false;

    loop {
        let (hour, minute, second, alarm_hour, alarm_minute, alarm_enabled, is_ringing) = {
            let s = CLOCK_STATE.lock().await;
            (s.hour, s.minute, s.second, s.alarm_hour, s.alarm_minute,
             s.alarm_enabled, s.is_ringing)
        };

        let mut frame = [LedColor::OFF; 16];
        let mut phase = 0u8;
        let mut prog  = 0u8;

        if is_ringing {
            // Full-brightness warm white flash while alarm is active
            let color = if flash_toggle {
                LedColor { r: 255, g: 255, b: 200 }
            } else {
                LedColor::OFF
            };
            flash_toggle = !flash_toggle;
            frame = [color; 16];
            phase = 3;
            prog  = 100;

        } else if alarm_enabled {
            // Calculate seconds until the alarm fires
            let crt_secs   = hour as i32 * 3600 + minute as i32 * 60 + second as i32;
            let alarm_secs = alarm_hour as i32 * 3600 + alarm_minute as i32 * 60;
            let mut secs_left = alarm_secs - crt_secs;
            if secs_left < 0 { secs_left += 24 * 3600; }

            if secs_left <= 300 {
                // Pick the color that corresponds to the 0-100 progress
                prog  = ((300 - secs_left) * 100 / 300) as u8;
                phase = if prog < 33 { 1 } else if prog < 67 { 2 } else { 3 };
                let color = sunrise_color(prog);
                frame = [color; 16];
            }
        }

        // Write phase and progress back to shared state
        {
            let mut s = CLOCK_STATE.lock().await;
            s.led_phase = phase;
            s.led_prog  = prog;
        }

        ws2812_send(&mut pin, &frame);
        Timer::after(Duration::from_secs(1)).await;
    }
}

// ─── Task: Display ────────────────────────────────────────────────────────────
//
// Drives the 1.8" ST7735 LCD over SPI at 20 MHz. The display shows:
//   - Current time (HH:MM:SS) in large bold white text
//   - Alarm time (HH:MM) with a colour-coded status: grey=OFF, green=SET, red=RING!
//   - Current temperature in light cyan
//
// To avoid full-screen flicker on every update, only the dynamic regions are
// cleared and redrawn each second. The static labels (TIME, ALARM, TEMP) are
// drawn once at startup and never touched again.

#[embassy_executor::task]
async fn display_task(
    mut display: st7735_lcd::ST7735<
        Spi<'static, embassy_stm32::mode::Async, embassy_stm32::spi::mode::Master>,
        Output<'static>,
        Output<'static>,
    >,
) {
    info!("Display task started.");

    let time_style       = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::WHITE);
    let label_style      = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
    let alarm_on_style   = MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN);
    let alarm_ring_style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);
    let temp_style       = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_LIGHT_CYAN);
    let clear_style      = PrimitiveStyleBuilder::new().fill_color(Rgb565::BLACK).build();

    // Draw the static section labels once — they never change
    display.clear(Rgb565::BLACK).ok();
    Text::new("TIME",  Point::new(4, 16),  label_style).draw(&mut display).ok();
    Text::new("ALARM", Point::new(4, 64),  label_style).draw(&mut display).ok();
    Text::new("TEMP",  Point::new(4, 110), label_style).draw(&mut display).ok();

    loop {
        let (hour, minute, second, alarm_hour, alarm_minute,
             alarm_enabled, is_ringing, temp_tenths) = {
            let s = CLOCK_STATE.lock().await;
            (s.hour, s.minute, s.second, s.alarm_hour, s.alarm_minute,
             s.alarm_enabled, s.is_ringing, s.temp_tenths)
        };

        // Clear only the dynamic value regions to avoid full-screen flicker
        Rectangle::new(Point::new(4, 22),  Size::new(120, 22)).into_styled(clear_style).draw(&mut display).ok();
        Rectangle::new(Point::new(4, 68),  Size::new(70,  22)).into_styled(clear_style).draw(&mut display).ok();
        Rectangle::new(Point::new(76, 68), Size::new(50,  22)).into_styled(clear_style).draw(&mut display).ok();
        Rectangle::new(Point::new(4, 114), Size::new(120, 22)).into_styled(clear_style).draw(&mut display).ok();

        let mut buf: String<12> = String::new();
        core::write!(buf, "{:02}:{:02}:{:02}", hour, minute, second).ok();
        Text::new(buf.as_str(), Point::new(4, 40), time_style).draw(&mut display).ok();

        let mut buf: String<8> = String::new();
        core::write!(buf, "{:02}:{:02}", alarm_hour, alarm_minute).ok();
        Text::new(buf.as_str(), Point::new(4, 82), time_style).draw(&mut display).ok();

        // Alarm status text and colour change based on current alarm state
        let (status_str, style) = if is_ringing {
            ("RING!", alarm_ring_style)
        } else if alarm_enabled {
            ("SET  ", alarm_on_style)
        } else {
            ("OFF  ", label_style)
        };
        Text::new(status_str, Point::new(76, 82), style).draw(&mut display).ok();

        // Temperature is stored as tenths of a degree so we split it for display
        let mut buf: String<16> = String::new();
        let tw = temp_tenths / 10;
        let tf = (temp_tenths % 10).abs();
        core::write!(buf, "{}.{} C  ", tw, tf).ok();
        Text::new(buf.as_str(), Point::new(4, 128), temp_style).draw(&mut display).ok();

        Timer::after(Duration::from_secs(1)).await;
    }
}

// ─── Task: UART listener ──────────────────────────────────────────────────────
//
// Listens for ASCII command lines from the Pico W over USART1 at 115200 baud.
// read_until_idle() uses DMA and returns when the line goes idle — efficient
// because the CPU is not involved during the actual byte reception.
// Commands are buffered into a heapless::String and dispatched when '\n' arrives.

#[embassy_executor::task]
async fn uart_listener(mut rx: UartRx<'static, Async>) {
    let mut rx_buf: [u8; 64] = [0; 64];
    let mut line: String<64>  = String::new();
    info!("UART listener ready.");

    loop {
        match rx.read_until_idle(&mut rx_buf).await {
            Ok(n) if n > 0 => {
                for &b in &rx_buf[..n] {
                    if b == b'\n' {
                        process_command(line.as_str()).await;
                        line.clear();
                    } else if b != b'\r' {
                        let _ = line.push(b as char);
                    }
                }
            }
            Ok(_)  => {}
            Err(_) => warn!("UART RX error"),
        }
    }
}

// Parses and executes a single command string received from the Pico W.
async fn process_command(cmd: &str) {
    let cmd = cmd.trim();
    if cmd.is_empty() { return; }
    info!("CMD: {}", cmd);

    if let Some(rest) = cmd.strip_prefix("SET_ALARM:") {
        let mut parts = rest.splitn(2, ':');
        let h = parts.next().and_then(|s| parse_u8(s).ok());
        let m = parts.next().and_then(|s| parse_u8(s).ok());
        if let (Some(hour), Some(min)) = (h, m) {
            let mut s = CLOCK_STATE.lock().await;
            s.alarm_hour      = hour;
            s.alarm_minute    = min;
            s.alarm_enabled   = true;
            s.is_ringing      = false;
            s.ringing_seconds = 0;
            info!("Alarm set for {:02}:{:02}", hour, min);
        } else {
            warn!("SET_ALARM: bad format");
        }
    } else if cmd == "SNOOZE" {
        let mut s = CLOCK_STATE.lock().await;
        if s.is_ringing {
            s.is_ringing      = false;
            s.ringing_seconds = 0;
            let total = s.alarm_minute as u16 + 5;
            s.alarm_minute = (total % 60) as u8;
            if total >= 60 { s.alarm_hour = (s.alarm_hour + 1) % 24; }
            info!("Web snooze -> next {:02}:{:02}", s.alarm_hour, s.alarm_minute);
        }
    } else if cmd == "DISABLE" {
        let mut s = CLOCK_STATE.lock().await;
        s.alarm_enabled   = false;
        s.is_ringing      = false;
        s.ringing_seconds = 0;
        s.led_phase       = 0;
        s.led_prog        = 0;
        info!("Alarm disabled via web.");
    } else if let Some(rest) = cmd.strip_prefix("SYNC_TIME:") {
        // The Pico W queries an NTP server and sends the current time every ~60 s
        let segs = split3(rest, ':');
        if let (Ok(h), Ok(m), Ok(sc)) = (
            parse_u8(segs[0]), parse_u8(segs[1]), parse_u8(segs[2])
        ) {
            let mut s = CLOCK_STATE.lock().await;
            s.hour   = h;
            s.minute = m;
            s.second = sc;
            info!("Clock synced to {:02}:{:02}:{:02}", h, m, sc);
        }
    } else {
        warn!("Unknown command: {}", cmd);
    }
}

// ─── Task: Telemetry sender ───────────────────────────────────────────────────
//
// Sends a STATUS line to the Pico W every 2 seconds so the web dashboard can
// display live data (time, temperature, humidity, alarm state, LED state).
// The 3-second startup delay avoids sending before the Pico W is ready to receive.

#[embassy_executor::task]
async fn telemetry_sender(mut tx: UartTx<'static, Async>) {
    Timer::after(Duration::from_secs(3)).await;
    info!("Telemetry sender started.");

    loop {
        let (hour, minute, second, alarm_hour, alarm_minute,
             alarm_enabled, is_ringing, temp_tenths, hum_pct, led_phase, led_prog) = {
            let s = CLOCK_STATE.lock().await;
            (s.hour, s.minute, s.second, s.alarm_hour, s.alarm_minute,
             s.alarm_enabled, s.is_ringing, s.temp_tenths,
             s.hum_pct, s.led_phase, s.led_prog)
        };

        let state_str = if is_ringing {
            "ALARMING"
        } else if alarm_enabled && led_phase > 0 {
            "SUNRISE"
        } else {
            "IDLE"
        };

        let temp_whole = temp_tenths / 10;
        let temp_frac  = (temp_tenths % 10).unsigned_abs();

        let mut msg: String<128> = String::new();
        core::write!(
            msg,
            "STATUS:TIME={:02}:{:02}:{:02},TEMP={}.{},HUM={},ALARM={:02}:{:02},STATE={},LED_PHASE={},LED_PROG={}\n",
            hour, minute, second,
            temp_whole, temp_frac,
            hum_pct,
            alarm_hour, alarm_minute,
            state_str,
            led_phase,
            led_prog,
        ).ok();

        let _ = tx.write(msg.as_bytes()).await;
        Timer::after(Duration::from_secs(2)).await;
    }
}

// ─── Task: BME280 temperature & humidity sensor ───────────────────────────────
//
// The BME280 communicates over I2C. 
//
// Temperature is stored as tenths of a degree (i16) to avoid floating-point,
// since this is a no_std environment without an FPU-backed f32 library.

#[embassy_executor::task]
async fn bme280_task(mut i2c: I2c<'static, Async, embassy_stm32::i2c::Master>) {
    Timer::after(Duration::from_secs(2)).await;
    info!("BME280 task started.");

    // Probe both possible I2C addresses
    let mut addr_found: Option<u8> = None;
    for addr in 0x76u8..=0x77 {
        let mut buf = [0u8; 1];
        if embassy_time::with_timeout(
            Duration::from_millis(10),
            i2c.read(addr, &mut buf),
        ).await.is_ok() {
            info!("BME280 found at 0x{:02x}", addr);
            addr_found = Some(addr);
            break;
        }
    }

    let addr = match addr_found {
        Some(a) => a,
        None => { warn!("BME280 not found."); return; }
    };

    // Read temperature calibration coefficients from registers 0x88–0x8D
    let mut calib = [0u8; 6];
    if i2c.write_read(addr, &[0x88], &mut calib).await.is_err() {
        warn!("BME280: calib read failed"); return;
    }
    let dig_t1 = u16::from_le_bytes([calib[0], calib[1]]) as i64;
    let dig_t2 = i16::from_le_bytes([calib[2], calib[3]]) as i64;
    let dig_t3 = i16::from_le_bytes([calib[4], calib[5]]) as i64;

    // Read humidity calibration coefficients from registers 0xA1 and 0xE1–0xE7
    let mut hc1 = [0u8; 1];
    let mut hc2 = [0u8; 7];
    let _ = i2c.write_read(addr, &[0xA1], &mut hc1).await;
    let _ = i2c.write_read(addr, &[0xE1], &mut hc2).await;
    let dig_h1 = hc1[0] as i64;
    let dig_h2 = i16::from_le_bytes([hc2[0], hc2[1]]) as i64;
    let dig_h3 = hc2[2] as i64;
    let dig_h4 = ((hc2[3] as i64) << 4) | ((hc2[4] as i64) & 0x0F);
    let dig_h5 = ((hc2[5] as i64) << 4) | ((hc2[4] as i64) >> 4);
    let dig_h6 = hc2[6] as i8 as i64;

    // Configure oversampling: humidity ×1 (0xF2), temp+pressure ×1 normal mode (0xF4)
    let _ = i2c.write(addr, &[0xF2, 0x01]).await;
    let _ = i2c.write(addr, &[0xF4, 0x27]).await;
    Timer::after(Duration::from_millis(100)).await;
    info!("BME280 initialised.");

    loop {
        // Trigger a forced measurement (0x25 = forced mode, ×1 oversampling)
        let _ = i2c.write(addr, &[0xF4, 0x25]).await;
        Timer::after(Duration::from_millis(10)).await;

        let mut raw_t = [0u8; 3];
        let t_ok = i2c.write_read(addr, &[0xFA], &mut raw_t).await.is_ok();
        let mut raw_h = [0u8; 2];
        let h_ok = i2c.write_read(addr, &[0xFD], &mut raw_h).await.is_ok();

        if t_ok {
            let adc_t = ((raw_t[0] as i64) << 12)
                      | ((raw_t[1] as i64) << 4)
                      | ((raw_t[2] as i64) >> 4);
            let var1   = (adc_t / 8 - dig_t1 * 2) * dig_t2 / 2048;
            let tmp    = adc_t / 16 - dig_t1;
            let var2   = (tmp * tmp / 4096) * dig_t3 / 16384;
            let t_fine = var1 + var2;
            // Convert to tenths of a degree
            let temp_tenths = ((t_fine * 5 + 128) / 256 / 10) as i16;

            let mut state = CLOCK_STATE.lock().await;
            state.temp_tenths = temp_tenths;

            if h_ok {
                let adc_h = ((raw_h[0] as i64) << 8) | (raw_h[1] as i64);
                let mut x: i64 = t_fine - 76800;
                x = (((adc_h << 14) - (dig_h4 << 20) - (dig_h5 * x) + 16384) >> 15)
                  * (((((((x * dig_h6) >> 10)
                      * (((x * dig_h3) >> 11) + 32768)) >> 10)
                      + 2097152) * dig_h2 + 8192) >> 14);
                x = x - (((((x >> 15) * (x >> 15)) >> 7) * dig_h1) >> 4);
                x = x.clamp(0, 419430400);
                let hum_pct = ((x >> 12) / 1024).clamp(0, 100) as u8;
                state.hum_pct = hum_pct;
                info!("BME280: {}.{} C, {}% RH",
                    temp_tenths / 10, (temp_tenths % 10).abs(), hum_pct);
            } else {
                info!("BME280: {}.{} C", temp_tenths / 10, (temp_tenths % 10).abs());
            }
        } else {
            warn!("BME280: read failed");
        }

        Timer::after(Duration::from_secs(30)).await;
    }
}

// ─── DFPlayer (hardware broken, kept for reference) ───────────────────────────
// PC8 bit-bang UART at 9600 baud and the DFPlayer protocol frame builder.

#[allow(dead_code)]
const GPIOC_BSRR: *mut u32 = 0x4202_0818 as *mut u32;
#[allow(dead_code)]
const PC8_HIGH: u32 = 1 << 8;
#[allow(dead_code)]
const PC8_LOW:  u32 = 1 << 24;
#[allow(dead_code)]
const BAUD_DELAY: u32 = 2083; 

#[allow(dead_code)]
fn uart_send_byte_9600(byte: u8) {
    unsafe {
        core::ptr::write_volatile(GPIOC_BSRR, PC8_LOW);
        cortex_m::asm::delay(BAUD_DELAY);
        for i in 0..8u8 {
            if (byte >> i) & 1 == 1 {
                core::ptr::write_volatile(GPIOC_BSRR, PC8_HIGH);
            } else {
                core::ptr::write_volatile(GPIOC_BSRR, PC8_LOW);
            }
            cortex_m::asm::delay(BAUD_DELAY);
        }
        core::ptr::write_volatile(GPIOC_BSRR, PC8_HIGH);
        cortex_m::asm::delay(BAUD_DELAY);
    }
}

#[allow(dead_code)]
fn dfplayer_send(cmd: u8, param: u16) {
    // DFPlayer protocol: 10-byte frame with start byte 0x7E, version 0xFF,
    // command byte, parameter, and a two's-complement checksum
    let checksum = (0u16)
        .wrapping_sub(0xFF)
        .wrapping_sub(0x06)
        .wrapping_sub(cmd as u16)
        .wrapping_sub(0x00)
        .wrapping_sub(param >> 8)
        .wrapping_sub(param & 0xFF);
    let frame = [
        0x7Eu8, 0xFF, 0x06, cmd, 0x00,
        (param >> 8) as u8,
        (param & 0xFF) as u8,
        (checksum >> 8) as u8,
        (checksum & 0xFF) as u8,
        0xEF,
    ];
    for &b in frame.iter() {
        uart_send_byte_9600(b);
    }
}

#[allow(dead_code)]
#[embassy_executor::task]
async fn dfplayer_task(_pin: Output<'static>) {
    info!("DFPlayer task started (bit-bang 9600 baud on PC8).");
    Timer::after(Duration::from_millis(2100)).await;
    dfplayer_send(0x06, 24);
    Timer::after(Duration::from_millis(200)).await;
    info!("DFPlayer: volume set");

    let mut was_ringing = false;
    let mut escalated   = false;
    let mut ring_start  = 0u32;

    loop {
        let (is_ringing, ringing_seconds) = {
            let s = CLOCK_STATE.lock().await;
            (s.is_ringing, s.ringing_seconds)
        };
        if is_ringing && !was_ringing {
            dfplayer_send(0x06, 24);
            Timer::after(Duration::from_millis(100)).await;
            dfplayer_send(0x12, 1);
            info!("DFPlayer: play track 1");
            ring_start  = ringing_seconds;
            escalated   = false;
            was_ringing = true;
        } else if is_ringing && !escalated && ringing_seconds >= ring_start + 60 {
            dfplayer_send(0x06, 30);
            info!("DFPlayer: volume escalated");
            escalated = true;
        } else if !is_ringing && was_ringing {
            dfplayer_send(0x0E, 0);
            info!("DFPlayer: paused");
            was_ringing = false;
            escalated   = false;
        }
        Timer::after(Duration::from_millis(250)).await;
    }
}

// ─── Helpers ──────────────────────────────────────────────────────────────────

// Parse a decimal u8 from a string slice without using the standard library
fn parse_u8(s: &str) -> Result<u8, ()> {
    let s = s.trim();
    if s.is_empty() { return Err(()); }
    let mut v: u8 = 0;
    for c in s.chars() {
        let d = c as u8;
        if d < b'0' || d > b'9' { return Err(()); }
        v = v.wrapping_mul(10).wrapping_add(d - b'0');
    }
    Ok(v)
}

// Split a string into exactly 3 parts by a separator character.
// Used to parse SYNC_TIME:HH:MM:SS without heap allocation.
fn split3<'a>(s: &'a str, sep: char) -> [&'a str; 3] {
    let mut out   = [""; 3];
    let mut idx   = 0;
    let mut start = 0;
    for (i, c) in s.char_indices() {
        if c == sep && idx < 2 {
            out[idx] = &s[start..i];
            idx  += 1;
            start = i + 1;
        }
    }
    out[idx] = &s[start..];
    out
}

// ─── Main ─────────────────────────────────────────────────────────────────────
//
// Configures the clock tree, initialises all peripherals, spawns the 9 async
// tasks, then enters an infinite sleep loop. 

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Configure the PLL to run the system clock at 80 MHz from the 16 MHz HSI.
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV1,
        mul:    PllMul::MUL10,
        divp:   None,
        divq:   None,
        divr:   Some(PllDiv::DIV2),
    });
    config.rcc.sys           = Sysclk::PLL1_R;
    config.rcc.voltage_range = VoltageScale::RANGE1;
    config.rcc.ahb_pre       = AHBPrescaler::DIV1;
    config.rcc.apb1_pre      = APBPrescaler::DIV1;
    config.rcc.apb2_pre      = APBPrescaler::DIV1;
    config.rcc.apb3_pre      = APBPrescaler::DIV1;

    let p = embassy_stm32::init(config);
    info!("STM32 Alarm OS -- 80 MHz");

    // USART1 at 115200 baud — bidirectional link to the Pico W
    // TX = PA9 (sends telemetry), RX = PA10 (receives commands)
    // DMA channels 0 and 1 handle the transfers so the CPU is free during I/O
    let mut uart1_cfg = UartConfig::default();
    uart1_cfg.baudrate = 115200;
    let uart1 = Uart::new(
        p.USART1, p.PA10, p.PA9,
        p.GPDMA1_CH0, p.GPDMA1_CH1,
        Irqs, uart1_cfg,
    ).expect("USART1 init failed");
    let (uart1_tx, uart1_rx) = uart1.split();

    let buzzer     = Output::new(p.PB0, Level::Low,  Speed::Low);
    // Buttons use internal pull-up: pin sits at 3.3V, pressing pulls it to GND (falling edge)
    let snooze_btn = ExtiInput::new(p.PC9, p.EXTI9, Pull::Up, Irqs); 
    let cancel_btn = ExtiInput::new(p.PB4, p.EXTI4, Pull::Up, Irqs); 
    let led_pin    = Output::new(p.PA8, Level::Low, Speed::High);    
    // DFPlayer pin kept configured so PC8 is not left floating
    let _dfplayer_pin  = Output::new(p.PC8, Level::High, Speed::High);

    // SPI1 at 20 MHz for the ST7735 LCD
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(20_000_000);
    let spi = Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6,
        p.GPDMA1_CH2, p.GPDMA1_CH3,
        Irqs, spi_cfg,
    );
    let dc  = Output::new(p.PC7, Level::Low,  Speed::High);
    let rst = Output::new(p.PC6, Level::High, Speed::High);
    let mut display = st7735_lcd::ST7735::new(spi, dc, rst, false, false, 128, 160);
    display.init(&mut embassy_time::Delay).ok();
    display.set_orientation(&st7735_lcd::Orientation::Portrait).ok();

    // I2C1 for the BME280 sensor
    let i2c = I2c::new(
        p.I2C1, p.PB6, p.PB7,
        p.GPDMA1_CH4, p.GPDMA1_CH5,
        Irqs, Default::default(),
    );

    // Spawn all 9 tasks. Each task runs independently; the executor schedules them.
    spawner.spawn(clock_ticker().unwrap());
    spawner.spawn(buzzer_driver(buzzer).unwrap());
    spawner.spawn(button_snooze(snooze_btn).unwrap());
    spawner.spawn(button_cancel(cancel_btn).unwrap());
    spawner.spawn(led_sunrise(led_pin).unwrap());
    spawner.spawn(display_task(display).unwrap());
    spawner.spawn(uart_listener(uart1_rx).unwrap());
    spawner.spawn(telemetry_sender(uart1_tx).unwrap());
    spawner.spawn(bme280_task(i2c).unwrap());
    //spawner.spawn(dfplayer_task(_dfplayer_pin).unwrap()); 

    // Main task sleeps indefinitely. All work happens in the spawned tasks above.
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}