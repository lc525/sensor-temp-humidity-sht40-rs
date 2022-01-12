# sensor-temp-humidity-sht40

This is a rust [`embedded-hal`](https://github.com/japaric/embedded-hal) 
driver for the Sensirion SHT40 temperature and relative-humidity sensor.

## Status

### Software 

The driver is developed against the stable 0.2.6 version of
embedded-hal. Please use the main branch for hal-1.0.0 support

### Hardware Features

- At the moment, the driver is feature-complete, supporting all the commands
that can be sent to the sensor.
- The driver should work with other sensors from the SHT4x family (like SHT41 or
SHT45) but this has not been tested.

## License

The crate is dually licensed under Apache License, Version 2.0 or the 
BSD 3-clause license (you can choose under which of the two licenses you use 
the code).

## Contributions

If you want to contribute code to this project, you accept that any code you
submit shall be dually licensed as above, with your copyright details added
to the relevant source files.
