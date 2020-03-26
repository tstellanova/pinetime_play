

use nrf52832_hal as p_hal;
use nrf52832_hal::nrf52832_pac as pac;

pub type HalSpimError = p_hal::spim::Error;

pub type Spim0PortType =  p_hal::spim::Spim<pac::SPIM0>;
pub type DisplaySckPinType = p_hal::gpio::p0::P0_18<p_hal::gpio::Output<p_hal::gpio::PushPull>>;
pub type DisplayMosiPinType =  p_hal::gpio::p0::P0_26<p_hal::gpio::Output<p_hal::gpio::PushPull>>;



