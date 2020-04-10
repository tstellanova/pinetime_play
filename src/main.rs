#![no_std]
#![no_main]

// configure panic behavior:
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use nrf52832_hal as p_hal;
use p_hal::nrf52832_pac as pac;
use p_hal::{
    clocks::ClocksExt, // LfOscConfiguration},
    gpio::{GpioExt, Level},
};
use p_hal::{delay::Delay, spim, twim};

use arrayvec::ArrayString;
use core::fmt;
use core::fmt::Arguments;
use cortex_m_rt as rt;

// use cortex_m_semihosting::hprintln;
use rtt_target::{rprintln, rtt_init_print};

use embedded_graphics::{
    egtext, fonts::Font12x16, pixelcolor::Rgb565, text_style,
};
use embedded_graphics::{prelude::*, primitives::*, style::*};

use bma421::BMA421;
use cst816s::{
    TouchEvent, CST816S, GESTURE_LONG_PRESS, GESTURE_SLIDE_DOWN,
    GESTURE_SLIDE_LEFT, GESTURE_SLIDE_RIGHT, GESTURE_SLIDE_UP,
};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use hrs3300::{AdcResolution, Hrs3300};
use rt::entry;
use st7789::Orientation;

use core::convert::TryInto;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

// mod sensor_value_tracker;
// use sensor_value_tracker::SensorValueTracker;
// use cortex_m::asm::wfe;
// use nrf52832_hal::prelude::TimerExt;
// use nrf52832_hal::Timer;

use pac::interrupt;

const SCREEN_WIDTH: i32 = 240;
const SCREEN_HEIGHT: i32 = 240;
const HALF_SCREEN_WIDTH: i32 = SCREEN_WIDTH / 2;
const MIN_SCREEN_DIM: i32 = SCREEN_HEIGHT;
const SCREEN_RADIUS: u32 = (MIN_SCREEN_DIM / 2) as u32;
const FONT_HEIGHT: i32 = 20; //for Font12x16

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);

    let mut cp = pac::CorePeripherals::take().unwrap();
    let mut delay_source = Delay::new(cp.SYST);

    // PineTime has a 32 MHz HSE (HFXO) and a 32.768 kHz LSE (LFXO)
    let mut dp = pac::Peripherals::take().unwrap();
    let _clockit = dp
        .CLOCK
        .constrain()
        //.set_lfclk_src_external(LfOscConfiguration::ExternalAndBypass)
        .start_lfclk()
        .enable_ext_hfosc();
    // TODO configure low-speed clock with LfOscConfiguration: currently hangs
    //.set_lfclk_src_external(LfOscConfiguration::ExternalNoBypass).start_lfclk();

    let port0 = dp.P0.split();

    rprintln!("\r\n--- BEGIN ---");

    // random number generator peripheral
    //let mut rng = dp.RNG.constrain();

    // pushbutton input GPIO: P0.13
    let mut _user_butt = port0.p0_13.into_floating_input().degrade();
    // must drive this pin high to enable pushbutton
    let mut _user_butt_en =
        port0.p0_15.into_push_pull_output(Level::High).degrade();

    // P0.12: when this pin is low, it indicates the battery is charging
    let charging_pin = port0.p0_12.into_floating_input();
    // P0.19: power presence? when this line is low, there's power connected
    let power_pin = port0.p0_19.into_floating_input();

    // TODO read battery voltage from P0.31 (AIN7
    // batteryVoltage = adcValue * 2000 / 1241 ; //millivolts

    // vibration motor output: drive low to activate motor
    let mut vibe = port0.p0_16.into_push_pull_output(Level::Low).degrade();
    pulse_vibe(&mut vibe, &mut delay_source, 25_000);

    // internal i2c0 bus devices: BMA421 (accel), HRS3300 (hrs), CST816S (TouchPad)
    // BMA421-INT:  P0.08
    // TP-INT: P0.28
    let i2c0_pins = twim::Pins {
        scl: port0.p0_07.into_floating_input().degrade(),
        sda: port0.p0_06.into_floating_input().degrade(),
    };
    let i2c_port = twim::Twim::new(dp.TWIM1, i2c0_pins, twim::Frequency::K400);
    let i2c_bus0 = shared_bus::CortexMBusManager::new(i2c_port);

    delay_source.delay_ms(10u8);

    let mut touchpad = CST816S::new(
        i2c_bus0.acquire(),
        port0.p0_28.into_pullup_input().degrade(), //P0.28/AIN4 (TP_INT)
        port0.p0_10.into_push_pull_output(Level::High).degrade(), //P0.10/NFC2 (TP_RESET)
    );
    touchpad.setup(&mut delay_source).unwrap();

    // watch for interrupts on TP_INT pin
    enable_gpiote_interrupt(
        &mut cp.NVIC,
        &mut dp.GPIOTE,
        28,
        pac::gpiote::config::POLARITYW::HITOLO,
    ); //p0.28 (TP_INT)
    let mut touch_target_state = false;

    // the accel driver is problematic -- sometimes needs a few tries to startup
    // let mut accel = loop {
    //     if let Ok(accel) = BMA421::new(i2c_bus0.acquire(), &mut delay_source) {
    //         break accel;
    //     }
    // };

    // let mut hrs = Hrs3300::new(i2c_bus0.acquire());
    // hrs.init().unwrap();
    // hrs.set_adc_resolution(AdcResolution::Bit18).unwrap();
    // hrs.enable_hrs().unwrap();
    // hrs.enable_oscillator().unwrap();
    // //hrs.set_conversion_delay(hrs3300::ConversionDelay::Ms800).unwrap();
    // let hrs_id = hrs.device_id().unwrap();
    // rprintln!("hrs device id: {}", hrs_id);
    // hrs.disable_oscillator().unwrap();
    // let mut hr_monitor = SensorValueTracker::new(0.1);

    // // find the BLE radio peripheral
    //
    // // let radio = dp.RADIO.
    // let radio = dp.RADIO;
    // // TODO configure dma before starting radio
    // radio.tasks_start.write(|w| {
    //     unsafe { w.bits(0) }
    // });

    let spim0_pins = spim::Pins {
        sck: port0.p0_02.into_push_pull_output(Level::Low).degrade(),
        miso: None,
        mosi: Some(port0.p0_03.into_push_pull_output(Level::Low).degrade()),
    };

    // create SPIM0 interface, 8 Mbps, use 122 as "over read character"
    let spim0 = spim::Spim::new(
        dp.SPIM0,
        spim0_pins,
        spim::Frequency::M8,
        spim::MODE_3,
        122,
    );
    let spi_bus0 = shared_bus::CortexMBusManager::new(spim0);

    let mut backlight_ctrl = PinetimeBacklightControl::new(
        port0.p0_14.into_push_pull_output(Level::High).degrade(), //LCD_BACKLIGHT_LOW
        port0.p0_22.into_push_pull_output(Level::High).degrade(), //LCD_BACKLIGHT_MID
        port0.p0_23.into_push_pull_output(Level::High).degrade(), //LCD_BACKLIGHT_HIGH
    );

    // create display driver
    let mut display = st7789::new_display_driver(
        spi_bus0.acquire(),
        port0.p0_25.into_push_pull_output(Level::High), // SPI chip select (CSX)
        port0.p0_18.into_push_pull_output(Level::Low),  // data/clock switch pin
        port0.p0_26.into_push_pull_output(Level::Low),  // reset pin
        SCREEN_WIDTH as u16,
        SCREEN_HEIGHT as u16,
    );

    display.init(&mut delay_source).unwrap();
    display.set_orientation(&Orientation::Portrait).unwrap();

    draw_background(&mut display);
    // let half_height = SCREEN_HEIGHT / 2;
    // let graph_area = Rectangle::new(
    //     Point::new(10, half_height - 50),
    //     Point::new(SCREEN_WIDTH - 20, half_height + 50),
    // );

    // //TODO // Setup SPI flash NOR memory
    // let flash_csn = port0.p0_05.into_push_pull_output(Level::High);
    // let mut flash =
    //     spi_memory::series25::Flash::init(spi_bus0.acquire(), flash_csn)
    //         .unwrap();
    // if let Ok(flash_id) = flash.find_device_identifier() {
    //     rprintln!(
    //         "flash ID: {} 0x{:x} {:?}",
    //         flash_id.continuation_count(),
    //         flash_id.mfr_code(),
    //         flash_id.device_id()
    //     );
    // }

    let mut idle_count = 0;

    loop {
        let is_charging = charging_pin.is_low().unwrap_or(false);
        let is_powered = power_pin.is_low().unwrap_or(false);
        render_power(&mut display, is_powered, is_charging);
        backlight_ctrl.set_level_by_power_sources(is_powered, is_charging);

        // let ambient_light = hrs.read_als().unwrap_or(0);
        // let heart_rate = hrs.read_hrs().unwrap_or(0);
        // if 0 != heart_rate  {
        //     //if heart_rate > 13000 { //TODO heuristic absorption value
        //         let heart_rate_f32 = heart_rate as f32;
        //         let avg_heart = hr_monitor.update(heart_rate_f32);
        //
        //         render_graph_bar(&mut display,
        //                          &graph_area,
        //                          x_idx,
        //                          heart_rate_f32,
        //                          avg_heart,
        //                          Rgb565::GREEN);
        //     //}
        //
        //     render_text(
        //         &mut display,
        //         HALF_SCREEN_WIDTH - 40,
        //         SCREEN_HEIGHT - (2 * FONT_HEIGHT),
        //         Rgb565::RED,
        //         format_args!("HR {}", heart_rate),
        //     );
        //     render_text(
        //         &mut display,
        //         HALF_SCREEN_WIDTH - 40,
        //         SCREEN_HEIGHT - (1 * FONT_HEIGHT),
        //         Rgb565::RED,
        //         format_args!("AL {}", ambient_light),
        //     );
        // }

        if let Some(evt) = touchpad.read_one_touch_event() {
            idle_count = 0;
            if evt.gesture == GESTURE_LONG_PRESS {
                pulse_vibe(&mut vibe, &mut delay_source, 10_000);
                // TOGGLE
                touch_target_state = !touch_target_state;
                render_touch_target(&mut display, touch_target_state);
            }
        } else {
            idle_count += 1;
            delay_source.delay_us(10u32);
            // if let Ok(accel_bytes) = accel.read_accel_bytes() {
            //     let vec3: [i16; 3] = [
            //         i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap()) / 16,
            //         i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap()) / 16,
            //         i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap()) / 16,
            //     ];
            //
            //     render_vec3_i16(
            //         &mut display,
            //         HALF_SCREEN_WIDTH - 40,
            //         30,
            //         vec3.as_ref(),
            //     );
            // }
        }

        if idle_count > 5 {
            rprintln!("Sleeping...");
            cortex_m::asm::wfe();
            rprintln!("AWAKE!");
            idle_count = 0;
        }
    }
}

fn draw_background(display: &mut impl DrawTarget<Rgb565>) {
    let clear_bg = Rectangle::new(
        Point::new(0, 0),
        Point::new(SCREEN_WIDTH, SCREEN_HEIGHT),
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK));
    let _ = clear_bg.draw(display);

    let center_circle = Circle::new(
        Point::new(HALF_SCREEN_WIDTH, SCREEN_HEIGHT / 2),
        SCREEN_RADIUS,
    )
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::YELLOW, 4));
    let _ = center_circle.draw(display);
}

fn render_touch_target(display: &mut impl DrawTarget<Rgb565>, state: bool) {
    let fill = if state { Rgb565::MAGENTA } else { Rgb565::CYAN };

    let touch_circle =
        Circle::new(Point::new(HALF_SCREEN_WIDTH, SCREEN_HEIGHT / 2), 20)
            .into_styled(PrimitiveStyle::with_fill(fill));
    let _ = touch_circle.draw(display);
}

fn render_power(
    display: &mut impl DrawTarget<Rgb565>,
    powered: bool,
    charging: bool,
) {
    //rprintln!("power: {} charging: {}", powered, charging);

    let power_fill = if powered { Rgb565::RED } else { Rgb565::BLACK };
    let charge_fill = if charging {
        Rgb565::BLUE
    } else {
        Rgb565::BLACK
    };

    let power_circle = Circle::new(Point::new(12, SCREEN_HEIGHT - 12), 10)
        .into_styled(PrimitiveStyle::with_fill(power_fill));
    let _ = power_circle.draw(display);

    let charge_circle =
        Circle::new(Point::new(SCREEN_WIDTH - 12, SCREEN_HEIGHT - 12), 10)
            .into_styled(PrimitiveStyle::with_fill(charge_fill));
    let _ = charge_circle.draw(display);
}

/// Render formatted text to the display
fn render_text(
    display: &mut impl DrawTarget<Rgb565>,
    x_pos: i32,
    y_pos: i32,
    color: Rgb565,
    args: Arguments<'_>,
) {
    let mut format_buf = ArrayString::<[u8; 16]>::new();
    if fmt::write(&mut format_buf, args).is_ok() {
        let _ = egtext!(
            text = &format_buf,
            top_left = Point::new(x_pos, y_pos),
            style = text_style!(
                font = Font12x16,
                text_color = color,
                background_color = Rgb565::BLACK,
            )
        )
        .draw(display);
    }
}

/// render a vector of three i16 to the display
fn render_vec3_i16(
    display: &mut impl DrawTarget<Rgb565>,
    x_pos: i32,
    start_y: i32,
    buf: &[i16],
) {
    //TODO dynamically reformat depending on font size
    let mut y_pos = start_y;
    render_text(
        display,
        x_pos,
        y_pos,
        Rgb565::GREEN,
        format_args!("X: {}", buf[0]),
    );
    y_pos += FONT_HEIGHT;
    render_text(
        display,
        x_pos,
        y_pos,
        Rgb565::GREEN,
        format_args!("Y: {}", buf[1]),
    );
    y_pos += FONT_HEIGHT;
    render_text(
        display,
        x_pos,
        y_pos,
        Rgb565::GREEN,
        format_args!("Z: {}", buf[2]),
    );
}

// fn render_graph_bar(
//     display: &mut impl DrawTarget<Rgb565>,
//     area: &Rectangle,
//     x_pos: i32,
//     value: f32, // this value
//     avg: f32,   // avg value
//     color: Rgb565,
// )
// {
//     // clear rect
//     let _ = Rectangle::new(
//         Point::new(x_pos, area.top_left.y),
//         Point::new(x_pos + 2, area.bottom_right.y),
//     )
//     .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
//     .draw(display);
//     // actual bar
//     //let y_off = (((area.bottom_right.y - area.top_left.y) as f32) * value) as i32;
//     let half_height = (area.bottom_right.y - area.top_left.y) / 2;
//     let y_ctr = area.top_left.y + half_height;
//     let delta = (value - avg) / avg; //normalized delta
//     let y_delta = (delta * (half_height as f32)) as i32;
//     let y_pos = y_ctr + y_delta;
//     let _ = Rectangle::new(
//         Point::new(x_pos, y_pos),
//         Point::new(x_pos + 2, area.bottom_right.y),
//     )
//     .into_styled(PrimitiveStyle::with_fill(color))
//     .draw(display);
//
//     // display.draw( Rect::new(Coord::new(xpos, 0), Coord::new(xpos + (2*BAR_WIDTH), SCREEN_HEIGHT)).with_fill(Some(0u8.into())).into_iter());
// }
//

fn pulse_vibe(
    vibe: &mut impl OutputPin,
    delay_source: &mut impl DelayUs<u32>,
    micros: u32,
) {
    if micros > 0 {
        let _ = vibe.set_low();
        delay_source.delay_us(micros);
        let _ = vibe.set_high();
    }
}

#[interrupt]
fn GPIOTE() {
    rprintln!("GPIOTE");
    // ..
}

fn enable_gpiote_interrupt(
    nvic: &mut pac::NVIC,
    gpiote: &mut pac::GPIOTE,
    pin_id: u8,
    polarity: pac::gpiote::config::POLARITYW,
) {
    gpiote.config[0].write(|w| {
        let w = w.mode().event().polarity().variant(polarity);

        unsafe { w.psel().bits(pin_id) }
    });
    gpiote.intenset.modify(|_, w| w.in0().set());

    cortex_m::interrupt::free(|_| {
        pac::NVIC::unpend(pac::Interrupt::GPIOTE);
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::GPIOTE);
        }
    });
}

pub fn disable_gpiote_interrupt(
    nvic: &mut pac::NVIC,
    gpiote: &mut pac::GPIOTE,
) {
    gpiote.events_in[0].write(|w| unsafe { w.bits(0) });
    gpiote.intenclr.modify(|_, w| w.in0().clear());
    //nrf52::NVIC::mask(Interrupt::GPIOTE);
}

struct PinetimeBacklightControl<T> {
    pub pin_low: T,
    pub pin_mid: T,
    pub pin_high: T,
}

impl<T> PinetimeBacklightControl<T>
where
    T: OutputPin,
{
    fn new(pin_low: T, pin_mid: T, pin_high: T) -> Self {
        PinetimeBacklightControl {
            pin_low,
            pin_mid,
            pin_high,
        }
    }

    pub fn set_level_by_power_sources(
        &mut self,
        powered: bool,
        charging: bool,
    ) {
        // vary backlight level with the power level
        let level: u8 = if powered {
            0x04
        } else if charging {
            0x02
        } else {
            0x01
        };
        self.set_level(level);
    }

    /// This display supports three bits of brightness
    pub fn set_level(&mut self, level: u8) {
        // All off
        let _ = self.pin_low.set_high();
        let _ = self.pin_mid.set_high();
        let _ = self.pin_high.set_high();

        if level & 0x01 == 0x01 {
            let _ = self.pin_low.set_low();
        }
        if level & 0x02 == 0x02 {
            let _ = self.pin_mid.set_low();
        }
        if level & 0x04 == 0x04 {
            let _ = self.pin_high.set_low();
        }
    }
}
