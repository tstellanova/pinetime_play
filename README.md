## pinetime_play 

An environment for experimenting with rust on 
the [PineTime](https://wiki.pine64.org/index.php/PineTime)
 nrf52-based smart watch.
 
Note that you will need to 
[clear the nrf52 flash protection](https://gist.github.com/tstellanova/8c8509ae3dd4f58697c3b487dc3393b2)
before you will be able to program the PineTime. 

For installation and debugging you can use either 
openocd (built with proper support) or the 
[daily build of the Black Magic Probe firmware](https://github.com/blacksphere/blackmagic/wiki/Upgrading-Firmware)

We've used the an inexpensive ST-Link adapter to connect with the PineTime.

## Status
This is  work-in-progress

- [x] Debug build runs on PineTime
- [x] Release build runs on PineTime
- [x] Internal SPI bus access
- [x] Support for rendering to display (using st7789 driver)
- [ ] Optimal clock configuration
- [ ] Internal I2C bus access
- [ ] Access all onboard sensors (requires building some embedded HAL drivers)
- [ ] Driver for BMA421 accelerometer
- [ ] Driver for HRS3300 heart rate sensor
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