# sensor-temp-humidity-sht40

This is a rust [`embedded-hal`](https://github.com/japaric/embedded-hal) 
driver for the Sensirion SHT40 temperature and relative-humidity sensor.

## Status

### Software 

The driver is developed against the unstable 1.0.0 version of
embedded-hal (currently supporting embedded-hal-1.0.0-alpha.6). As such, it
should also be considered unstable. 

A release is planned for the older, stable version of embedded-hal.

### Hardware Features

- At the moment, the driver does not support activating the SHT40 built-in heater.
However, the support for those commands exists and should require minimal
changes (patches welcome!). 
- The driver should work with other sensors from the SHT4x family (like SHT41 or
SHT45) but this has not been tested.

## License

The crate is dually licensed under Apache License, Version 2.0 or the 
BSD 3-clause license (you can choose under which of the two licenses you use 
the code).

If you want to contribute code to this project, you accept that any code you
submit shall be dually licensed as above, with your copyright details added
to the relevant source files.
