# sensor-temp-humidity-sht40

This is a rust [`embedded-hal`](https://github.com/japaric/embedded-hal) 
driver for the Sensirion SHT40 temperature and relative-humidity sensor.

## Status

### Software 

The driver is developed against the stable 0.2.6 version of
embedded-hal. All releases from this branch are versioned 0.2.6xx with
xx incrementing from one release to the next.

Please use the [`main`](https://github.com/lc525/sensor-temp-humidity-sht40-rs/tree/main) branch for hal-1.0.0 support.

### Hardware Features

- At the moment, the driver is feature-complete, supporting all the commands
that can be sent to the sensor.
- The driver should work with other sensors from the SHT4x family (like SHT41 or
SHT45) but this has not been tested.

## Crate Features

- "fp": enable this feature if your CPU or MCU has a floating-point unit
  and you want to be able to convert measured values to common units (degrees
  Celsius, degreed Fahrenheit for temperature and % for relative humidity):

  `cargo build --features fp --release`

  By default, instead of common units, the driver stores measurements in 
  milli degrees Celsius or milli degrees Fahrenheit for temperature and in per 
  cent mille (pcm) for relative humidity, doing conversions using fixed-point
  arithmetic.

  A temperature of 23.89 degrees Celsius will be stored as 23890 and
  and a humidity of 56.2% will be stored as 56200.

  With the fp feature, users of the API can directly convert back to common units 
  using an utility function which takes a Measurement and returns an
  MeasurementFp. Unless you need this convenience, it is probably best not to
  enable "fp" and propagate values as integers until just before displaying them
  to an end-user.

## Usage

Detailed API documentation [`here`](https://docs.rs/crate/sensor-temp-humidity-sht40/latest)

Import this crate and an `embedded_hal` implementation, then instantiate the
driver:

* For example, assuming you have connected a SH40 sensor to a linux
machine and it is detected as an i2c device:

```
use linux_embedded_hal as hal;

use hal::{I2cdev, Delay};
use sensor_temp_humidity_sht40::{SHT40Driver, I2CAddr, Precision,
                                 Measurement, TempUnit};

fn main() {
    let i2c_dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sht40 = SHT40Driver::new(i2c_dev, I2CAddr::SHT4x_A, Delay);

    if let Ok(measurement) =
        sht40.get_temp_and_rh(Precision::High, TempUnit::MilliDegreesCelsius) {
      println!("Temp: {temp} C, Relative Humidity: {rh} %",
               temp = measurement.temp,
               rh = measurement.rel_hum_pcm);
    }
}
```

* Likewise, if you are using an ESP32-based board:

```
use esp_idf_sys;
use esp_idf_hal::delay;
use esp_idf_hal::i2c;
use esp_idf_hal::prelude::*;
use sensor_temp_humidity_sht40::{SHT40Driver, I2CAddr, Precision, TempUnit};

fn main() {
    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let i2c = peripherals.i2c0;
    let scl = pins.gpio9;
    let sda = pins.gpio8;
    let i2c_config = <i2c::config::MasterConfig as Default>::default()
                     .baudrate(400.kHz().into());

    let mut sensor_drv = SHT40Driver::new(
        i2c::Master::<i2c::I2C0, _, _>::new(i2c, 
                                            i2c::MasterPins { sda, scl }, 
                                            i2c_config).unwrap(), 
        I2CAddr::SHT4x_A, 
        delay::Ets);

    let measurement = sensor_drv.get_temp_and_rh(Precision::High,
                                                 TempUnit::MilliDegreesCelsius);
    println!("Measurement {:#?}", measurement);
}
```

## License

The crate is dually licensed under Apache License, Version 2.0 or the 
BSD 3-clause license (you can choose under which of the two licenses you use 
the code).

## Contributions

If you want to contribute code to this project, you accept that any code you
submit shall be dually licensed as above, with your copyright details added
to the relevant source files.
