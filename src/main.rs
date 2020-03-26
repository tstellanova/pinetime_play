#![no_std]
#![no_main]

// configure panic behavior:
#[cfg(not(debug_assertions))]
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
#[cfg(debug_assertions)]
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use nrf52832_hal as p_hal;
use nrf52832_hal::nrf52832_pac as pac;
use p_hal::prelude::*;
use p_hal::{gpio::*, spim, Delay};

use arrayvec::ArrayString;
use core::fmt;
use core::fmt::Arguments;
use cortex_m_rt as rt;
use cortex_m_semihosting::hprintln;
use embedded_graphics::{egtext, fonts::Font24x32, pixelcolor::Rgb565, prelude::*, text_style};
use embedded_graphics::{prelude::*, primitives::*, style::*};
use embedded_hal::digital::v1::OutputPin;
use rt::entry;
use st7789::{Orientation, ST7789};

mod port_types;
use port_types::*;

const SCREEN_WIDTH: i32 = 240;
const SCREEN_HEIGHT: i32 = 240;

type DisplayType = st7789::ST7789<Spim0PortType, DisplaySckPinType, DisplayMosiPinType, Delay>;

#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let delay_source = Delay::new(cp.SYST);

    let dp = pac::Peripherals::take().unwrap();
    let port0 = dp.P0.split();

    let mut rng = dp.RNG.constrain();

    let spim0_pins = spim::Pins {
        sck: port0.p0_02.into_push_pull_output(Level::Low).degrade(),
        miso: None,
        mosi: Some(port0.p0_03.into_push_pull_output(Level::Low).degrade()),
    };

    // create SPI M0 interface, 8 Mbps, use 122 as "over read character"
    let spim0 = spim::Spim::new(dp.SPIM0, spim0_pins, spim::Frequency::M8, spim::MODE_3, 122);

    // backlight control pin for display
    let mut _backlight = port0.p0_22.into_push_pull_output(Level::Low);
    // SPI chip select (CSN) for the display.
    // There's only one device on SPIM0 ? so we always leave it selected
    let _display_csn = port0.p0_25.into_push_pull_output(Level::Low);

    // data/clock switch pin for display
    let display_dc = port0.p0_18.into_push_pull_output(Level::Low);
    // reset pin for display
    let display_rst = port0.p0_26.into_push_pull_output(Level::Low);

    // create display driver
    let mut display = ST7789::new(
        spim0,
        display_dc,
        display_rst,
        SCREEN_WIDTH as u16,
        SCREEN_HEIGHT as u16,
        delay_source,
    );
    display.init().unwrap();
    display.set_orientation(&Orientation::Landscape).unwrap();
    draw_background(&mut display);

    loop {
        let rando = [
            rng.random_u16() as i16,
            rng.random_u16() as i16,
            rng.random_u16() as i16,
        ];
        render_vec3_i16(&mut display, 20, "hi", rando.as_ref());
        hprintln!(".").unwrap();
    }
}

fn draw_background(display: &mut DisplayType) {
    let clear_bg = Rectangle::new(Point::new(0, 0), Point::new(SCREEN_WIDTH, SCREEN_HEIGHT))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK));
    clear_bg.draw(display).unwrap();

    let min_dim = SCREEN_WIDTH.min(SCREEN_HEIGHT) as u32;
    let center_circle = Circle::new(Point::new(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2), min_dim / 2)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 2));
    center_circle.draw(display).unwrap();
}

/// Render formatted text to the display
fn render_text(display: &mut DisplayType, y_pos: i32, args: Arguments<'_>) {
    // let mut format_buf = ArrayString::<[u8; 16]>::new();
    let mut format_buf = ArrayString::<[_; 16]>::new();
    if fmt::write(&mut format_buf, args).is_ok() {
        let _ = egtext!(
            text = &format_buf,
            top_left = Point::new(10, y_pos),
            style = text_style!(
                font = Font24x32,
                text_color = Rgb565::GREEN,
                background_color = Rgb565::BLACK,
            )
        )
        .draw(display);
    }
}

/// render a vector of three i16 to the display
fn render_vec3_i16(display: &mut DisplayType, start_y: i32, _label: &str, buf: &[i16]) {
    const LINE_HEIGHT: i32 = 36;
    let mut y_pos = start_y;
    //TODO dynamically reformat depending on display size
    render_text(display, y_pos, format_args!("X: {}", buf[0]));
    y_pos += LINE_HEIGHT;
    render_text(display, y_pos, format_args!("Y: {}", buf[1]));
    y_pos += LINE_HEIGHT;
    render_text(display, y_pos, format_args!("Z: {}", buf[2]));
}
