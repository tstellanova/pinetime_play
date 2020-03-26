## pinetime_play 

An environment for experimenting with rust on 
the [PineTime](https://wiki.pine64.org/index.php/PineTime)
 nrf52-based smart watch.
 
Note that you will need to 
[clear the nrf52 flash protection](https://gist.github.com/tstellanova/8c8509ae3dd4f58697c3b487dc3393b2)
before you will be able to program the PineTime. 

For installation and debugging you can connect with the PineTime on its SWD debug port using, for example:
- openocd (built with proper support). We've used an inexpensive ST-Link adapter to with openocd. 
- [daily build of the Black Magic Probe firmware](https://github.com/blacksphere/blackmagic/wiki/Upgrading-Firmware)
- Segger J-Link or similar


## Status
This is work-in-progress

- [x] Debug build runs on PineTime
- [x] Release build runs on PineTime
- [x] Internal SPI bus access
- [x] Support for rendering to display (using st7789 driver)
- [x] Load driver for HRS3300 heart rate sensor
- [ ] Optimal clock configuration
- [ ] Internal I2C bus access
- [ ] Access all onboard sensors (requires building some embedded HAL drivers)
- [ ] Driver for BMA421 accelerometer
- [ ] Calibration routines for accelerometers
- [ ] Reading battery status ports (using ADC?)
- [ ] flash memory read/write at runtime
- [x] Semihosting debug support via swd port
- [ ] CI
- [ ] Documentation


## Clocks
- PineTime has a low speed 32.768 kHz crystal (LSE)

## Notes on buses

See the "PineTime Port Assignment" document
for notes on which devices are attached to which buse.


## License

BSD-3-Clause, see `LICENSE` file. 