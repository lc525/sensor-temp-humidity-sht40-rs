// Copyright (c) 2022 Lucian Carata <luc@rez.how>
//
// This file is part of the sensor-temp-humidity-sht40 crate, and is dually
// licensed under Apache License Version 2.0 or the BSD 3-clause License.
//
// For full licensing details, consult the LICENSE file in the root directory
// of the crate.
//
#![cfg_attr(not(test), no_std)]

/// This is a rust [`embedded-hal`](https://github.com/japaric/embedded-hal) 
/// driver for the Sensirion SHT40 temperature and relative-humidity sensor.
///
/// By depending on embedded-hal, this driver is platform-agnostic and can be 
/// used with any platform for which an implementation of the embedded-hal
/// traits exists. 
/// 
/// The full details about the SHT40 sensor can be read in its datasheet:
/// https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Datasheets/Sensirion_Humidity_Sensors_SHT4x_Datasheet.pdf
///
/// Any sensor-related data quoted in this documentation are orientative and are 
/// taken from Version 2 - July 2021 of the datasheet
///
/// ## Usage:
///
/// Import this crate and an `embedded_hal` implementation, then instantiate the
/// driver; For example, assuming you have connected a SH40 sensor to a linux
/// machine and it is detected as an i2c device:
///
/// ```no_run
/// use linux_embedded_hal as hal;
///
/// use hal::{I2cdev, Delay};
/// use sensor_temp_humidity_sht40::{SHT40Driver, I2CAddr, Precision,
///                                  Measurement, TempUnit};
///
/// fn main() {
///     let i2c_dev = I2cdev::new("/dev/i2c-1").unwrap();
///     let mut sht40 = SHT40Driver::new(i2c_dev, I2CAddr::SHT4x_A, Delay);
///     
///     if let Ok(measurement) =
///         sht40.get_temp_and_rh(Precision::High, TempUnit::Celsius) {
///       println!("Temp: {temp} C, Relative Humidity: {rh} %",
///                temp = measurement.temp,
///                rh = measurement.rel_hum_percent);
///     }
/// }
/// ```

use embedded_hal as hal;

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

use sensirion_i2c::{crc8, i2c};

// I2C Commands supported by the SHT40 sensor
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
enum Command {
    SoftReset,
    GetSerialNumber,
    MeasureTempAndHumidity(Precision),
    SetHeaterThenMeasureHighPrec(HeaterPower, HeaterDuration),
}

/// The accuracy of SHT40 is typically +/- 0.2C and at worst +/- 0.4C in the 
/// interval [0..60]C. The worst case outside this interval is +/- 1C (for the 
/// ends of the interval [-40..120]C. 
///
/// For relative humidity, the typical accuracy is +/- 1.8 %RH and at worst 
/// +/- 3.5 %RH in the interval [10...90]%RH, while the worst case outside the
/// interval is +/- 5%RH.
///
/// While the accuracy characteristics are not-user adjustable, the precision 
/// of the measurement can be configured at measurement time, with a trade-off 
/// of precision for faster measurement times and lower power consumption. 
///
/// Precision values given below refer to the repeatability of multiple 
/// consecutive measurements in constant conditions and state 3 times the
/// standard deviation (3*sigma) of the resulting distribution. Average current
/// consumption is given for continuous operation with one measurement per
/// second.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Precision {
    /// Relative humididy meas. repeatability: 0.08 %RH
    /// Temperature meas. repeatability:       0.04 C
    /// Avg. current consumption:              2.4 uA
    /// Measurement duration (typ):            6.9 ms
    /// Measurement duration (max):            8.2 ms
    High,

    /// Relative humididy meas. repeatability: 0.15 %RH
    /// Temperature meas. repeatability:       0.07 C
    /// Avg. current consumption:              1.3 uA
    /// Measurement duration (typ):            3.7 ms
    /// Measurement duration (max):            4.5 ms
    Medium,

    /// Relative humididy meas. repeatability: 0.25 %RH
    /// Temperature meas. repeatability:       0.1 C
    /// Avg. current consumption:              0.4 uA
    /// Measurement duration (typ):            1.3 ms
    /// Measurement duration (max):            1.7 ms
    Low
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaterPower {
    /// High is typically 200mW (for VDD=3.3V)
    High,

    /// Medium is typically 110mW (for VDD=3.3V)
    Medium,

    /// Low is typically 20mW (for VDD=3.3V)
    Low,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaterDuration {
    /// PulseLong is typically 1s (max 1.1s)
    PulseLong,

    /// PulseShort is typically 100ms (max 110ms)
    PulseShort,
}

/// I2C addresses for SHT40 are fixed, and depending on the product number
/// the sensor uses either 0x44 or 0x45. Product numbers SHT4x-Axxx get
/// address 0x44, and SHT4x-Bxxx get 0x45. For example, SHT40-AD1B has
/// address 0x44 and SHT40-BD1B has address 0x45.
#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2CAddr {
    /// use SHT4x_A for all products with I2C address 0x44, product numbers
    /// SHT4x-Axxx
    SHT4x_A = 0x44,

    /// use SHT4x_B for all products with I2C address 0x45, product numbers
    /// SHT4x-Bxxx
    SHT4x_B = 0x45,
}

/// Measurement unit for Temperature. 
///
/// The raw integer measurements from the sensor can be converted directly into
/// either unit, and this avoids users needing to do a subsequent floating-point
/// unit transformation which might reduce accuracy.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TempUnit {
    Farrenheit,
    Celsius,
}

/// SHT40Driver is the main structure with which a user of this driver
/// interracts. It is generic over two parameters implementing embedded_hal 
/// traits for the specific platform you're using: I2c which must implement 
/// embedded_hal::blocking::i2c::{Read, Write, WriteRead} and Delay which must
/// implement embedded_hal::blocking::delay::DelayMs.
/// 
/// Those implementations are needed for issuing I2C write commands to the 
/// sensor, waiting until the sensor has performed the actual measurement and
/// then issuing I2C read commands to get the measured data.
///
/// In this interraction, the device running the driver acts as the I2C master.
/// 
/// The actual types used to parameterise SHT40Driver depend on the embedded_hal
/// trait implementations. 
///
/// For example, for the esp32 platform there are two crates implementing 
/// embedded_hal, a bare-metal one (no_std) [`esp-hal`](https://github.com/esp-rs/esp32-hal)
/// and [`esp-idf-hal`](https://github.com/esp-rs/esp-idf-hal/tree/embedded-hal-1-compat)
/// which requires std and is built on top of the vendor-provided esp-idf C framework.
///
/// Each of those may in the future offer multiple implementations of i2c 
/// (bit-banged, using I2C hardware controllers on the main processor, using 
/// I2C hardware controllers on the ultra-low-power processor, etc). You would
/// need to instantiate the type best suited for your application and then
/// pass it to this driver.
///
/// Likewise, the nRF family of devices is supported using types implemented in
/// the [`nrf-hal`](https://github.com/nrf-rs/nrf-hal) crate.
///
/// Please consult the documentation of the embedded_hal implementation for 
/// your platform and examples using I2C to see exactly which types to use 
/// during SHT40Driver initialization.
/// 
#[derive(Debug)]
pub struct SHT40Driver<I2c, Delay> {
    i2c: I2c,
    address: u8,
    delay: Delay,
}

#[derive(Debug)]
pub enum Error<E> {
    /// Error on the I2C bus
    I2c(E),
    /// Failed checksum validation
    Crc,
    /// Error while blocking for set delay (typically waiting for I2C response)
    DelayError,
}

impl<E, I2cWrite, I2cRead> From<i2c::Error<I2cWrite, I2cRead>> for Error<E>
where
    I2cWrite: Write<Error = E>,
    I2cRead: Read<Error = E>,
{
    fn from(err: i2c::Error<I2cWrite, I2cRead>) -> Self {
        match err {
            i2c::Error::Crc => Error::Crc,
            i2c::Error::I2cWrite(e) |
            i2c::Error::I2cRead(e) => Error::I2c(e),
        }
    }
}


#[derive(Clone, Copy)]
struct DeviceCommand {
    cmd_code: u8,
    max_duration_ms: u16,
}

/// A sensor measurement result
///
/// You obtain a measurement by calling get_temp_and_rh(...) or 
/// set_heater_then_measure(...) on an instance of SHT40Driver.
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// Measured temperature
    pub temp: f32,
    /// Unit of temperature measurement (C or F)
    pub temp_unit: TempUnit,
    /// Measured relative humidity (%)
    pub rel_hum_percent: f32,
    /// The precision that was requested when performing the measurement.
    /// For measurements obtained through set_heater_then_measure(...),
    /// the sensor automatically performs a Precision::High measurement.
    pub precision: Precision,
}

/// The raw, integer measurement data from the sensor.
/// In order to get actual temperature/humidity values this needs to be 
/// transformed. Normally, the transformations are done using the equations
/// (1), (2) and (3) from the sensor's datasheet. This is what the usual
/// measurement API does, but accessing raw measurements is provided
/// as a convenience if you want to do your own calibration.
pub struct RawMeasurement {
    temp_ticks: u16,
    rel_hum_ticks: u16,
    precision: Precision,
}

/// Structure for storing the sensor serial number
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct SerialNumber(u16, u16);


impl Command {
    fn as_device_command(self) -> DeviceCommand {
       match self {
            Command::SoftReset => DeviceCommand { cmd_code: 0x94,
                                                  max_duration_ms: 1 },
            Command::GetSerialNumber => DeviceCommand { cmd_code: 0x89,
                                                        max_duration_ms: 0 },
            Command::MeasureTempAndHumidity(prec) => {
                match prec {
                    Precision::High => DeviceCommand { cmd_code: 0xFD,
                                                       max_duration_ms: 9 },
                    Precision::Medium => DeviceCommand { cmd_code: 0xF6,
                                                         max_duration_ms: 5 },
                    Precision::Low => DeviceCommand { cmd_code: 0xE0,
                                                      max_duration_ms: 2 },
                }
            },
            Command::SetHeaterThenMeasureHighPrec(hpow, hdur) => {
                match (hpow, hdur) {
                    (HeaterPower::High, HeaterDuration::PulseLong) =>
                        DeviceCommand { cmd_code: 0x39,
                                        max_duration_ms: 1100 },
                    (HeaterPower::High, HeaterDuration::PulseShort) =>
                        DeviceCommand { cmd_code: 0x32,
                                        max_duration_ms: 110 },
                    (HeaterPower::Medium, HeaterDuration::PulseLong) =>
                        DeviceCommand { cmd_code: 0x2F,
                                        max_duration_ms: 1100 },
                    (HeaterPower::Medium, HeaterDuration::PulseShort) =>
                        DeviceCommand { cmd_code: 0x24,
                                        max_duration_ms: 110 },
                    (HeaterPower::Low, HeaterDuration::PulseLong) =>
                        DeviceCommand { cmd_code: 0x1E,
                                        max_duration_ms: 1100 },
                    (HeaterPower::Low, HeaterDuration::PulseShort) =>
                        DeviceCommand { cmd_code: 0x15,
                                        max_duration_ms: 110 },
                }
            }
        }
    }
}


impl<I2C, D, E> SHT40Driver<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u16>
{
    /// Initialize a new instance of the driver. Any synchronization required
    /// for dealing with multiple instances needs to be managed externally to
    /// the driver (for example, synchronizing access to the underlying i2c
    /// hardware).
    ///
    /// # Arguments
    ///
    /// * `i2c` - the object instance implementing access to the I2C controller 
    ///           hardware or emulating the protocol in software for your target
    ///           platform
    /// * `address` - the I2C address of the SHT40 sensor. SHT4x sensors are
    ///               assigned fixed addresses in hardware, and two such
    ///               addresses are available (0x44 and 0x45). Please refer to
    ///               the I2CAddr documentation for details.
    /// * `delay` - the object instance implementing blocking delays for your
    ///             target platform. This is used to block while waiting for
    ///             the sensor to respond to a measurement request or for the
    ///             heater to be deactivated.
    pub fn new(i2c: I2C, address: I2CAddr, delay: D) -> Self {
        SHT40Driver { i2c, address: address as u8, delay }
    }

    fn parse_sensor_raw_measurement(rx_bytes: [u8; 6], precision: Precision, check_crc: bool)
        -> Result<RawMeasurement, Error<E>> {
            let temp_ticks = ((rx_bytes[0] as u16) << 8)  + (rx_bytes[1] as u16);
            let rel_hum_ticks = ((rx_bytes[3] as u16) << 8) + (rx_bytes[4] as u16);
            if check_crc == true {
               if let Err(_) = crc8::validate(&rx_bytes) {
                   return Err(Error::Crc);
               }
            }
            Ok(RawMeasurement { temp_ticks, rel_hum_ticks, precision })
        }

    fn convert_raw_to_units(raw_meas: RawMeasurement, temp_unit: TempUnit) -> Measurement {
        let mut rel_hum_percent = -6.0 + 125.0 * (raw_meas.rel_hum_ticks as f32)/65535.0;

        // calibration may result in non-physical values which according to the
        // datasheet are only of interest when comparing the distribution of results
        // between multiple sensors:
        if rel_hum_percent > 100.0 { rel_hum_percent = 100.0; }
        if rel_hum_percent < 0.0 { rel_hum_percent = 0.0; }

        let temp = match temp_unit {
            TempUnit::Celsius => -45.0 + 175.0 * (raw_meas.temp_ticks as f32)/65535.0,
            TempUnit::Farrenheit => -49.0 + 315.0 * (raw_meas.temp_ticks as f32)/65535.0
        };

        Measurement {
            temp,
            temp_unit,
            rel_hum_percent,
            precision: raw_meas.precision
        }
    }

    fn parse_measurement(rx_bytes: [u8; 6], precision: Precision, temp_unit: TempUnit, check_crc: bool)
        -> Result<Measurement, Error<E>> {
            let raw = SHT40Driver::<I2C, D>::parse_sensor_raw_measurement(rx_bytes,
                                                              precision,
                                                              check_crc)?;
            Ok(SHT40Driver::<I2C, D>::convert_raw_to_units(raw, temp_unit))
    }

    fn i2c_command_and_response(&mut self, cmd: Command, rx_bytes: Option<&mut [u8]>)
        -> Result<(), Error<E>>{
        let dev_cmd = cmd.as_device_command();
        self.i2c.write(self.address, &dev_cmd.cmd_code.to_be_bytes())
            .map_err(|err| { Error::I2c(err) })?;
        self.delay.delay_ms(dev_cmd.max_duration_ms);
        if let Some(rx_bytes) = rx_bytes {
            i2c::read_words_with_crc(&mut self.i2c, self.address, rx_bytes)?;
        };
        Ok(())
    }

    /// Get the results of a measurement in raw sensor format (ticks). Please
    /// consult the sensor datasheet for transformations which can be applied
    /// to this raw data to get temperature and humidity values.
    ///
    /// The driver blocks for the max time the measurement can take, depending
    /// on the requested Precision, waiting for the measurement results.
    ///
    /// # Arguments
    ///
    /// * `precision` - The precision with which to execute the measurement. 
    ///                 Higher precision measurements take more time and consume
    ///                 more power. Consult the Precision struct documentation
    ///                 for details.
    ///
    pub fn get_raw_temp_and_rh_ticks(&mut self, precision: Precision) 
        -> Result<RawMeasurement, Error<E>> {
        let mut rx_bytes = [0; 6];
        let cmd = Command::MeasureTempAndHumidity(precision);
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        SHT40Driver::<I2C, D>::parse_sensor_raw_measurement(rx_bytes,
                                                            precision,
                                                            false)
    }

    /// Get the results of a measurement, with temperature measured in the 
    /// requested units. Relative humidity is unitless and always expressed as
    /// a percent (%)
    ///
    /// The driver blocks for the max time the measurement can take, depending
    /// on the requested Precision, waiting for the measurement results.
    ///
    /// # Arguments
    ///
    /// * `precision` - The precision with which to execute the measurement. 
    ///                 Higher precision measurements take more time and consume
    ///                 more power. Consult the Precision struct documentation
    ///                 for details.
    /// * `temp_unit` - The measurement unit to be used for temperature. You can
    ///                 choose between TempUnit::Celsius and TempUnit::Farrenheit
    ///
    pub fn get_temp_and_rh(&mut self, precision: Precision, temp_unit: TempUnit)
        -> Result<Measurement, Error<E>> {
        let mut rx_bytes = [0; 6];
        let cmd = Command::MeasureTempAndHumidity(precision);
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        SHT40Driver::<I2C, D>::parse_measurement(rx_bytes, precision, temp_unit, false)
    }
    
    /// Activate the SHT40 built-in heater with the specified power and duration
    /// then perform a high-precision measurement just before deactivation.
    ///
    /// This can be used to remove condensed or sprayed water from the sensor's
    /// surface, which make the sensor unresponsive to relative humidity changes
    /// as long as there is still liquid water on the surface. Likewise, in
    /// high-humidity environments this could allow creep-free measurements for
    /// extended times.
    ///
    /// The on-chip heater of SHT40 is designed for a maximal duty cycle of less
    /// than 5% (total heater-on time should not be longer than 5% of the
    /// sensor's lifetime). It is the responsibility of the callers of this
    /// function to keep track of this. 
    ///
    /// Additionally, the temperature sensor can be affected by the 
    /// thermally-induced mechanical stress from the heater, ofsetting the
    /// measured temperature. Likewise, any offsets which need to be applied
    /// to the measured values are the responsibility of the user of this API.
    ///
    /// For more details, consult the sensor's datasheet or additional Sensirion
    /// application notes.
    ///
    /// The driver blocks for the max time the heater is active, depending on
    /// the requested HeaterDuration, in order to get the measurement results.
    ///
    /// # Arguments
    ///
    /// * `hpow` - The power at which the heater should be activated
    /// * `hdur` - The duration for which the heater should be active. The
    ///            longest (HeaterDuration::PulseLong) is ~ 1s. After this 
    ///            duration, the heater is automatically deactivated.
    /// * `temp_unit` - The measurement unit to be used for temperature in the
    ///                 measurement taken just before the heater is deactivated.
    ///                 You can choose between TempUnit::Celsius and 
    ///                 TempUnit::Farrenheit
    ///
    pub fn set_heater_then_measure(&mut self, hpow: HeaterPower, hdur: HeaterDuration, temp_unit: TempUnit)
        -> Result<Measurement, Error<E>> {
        let mut rx_bytes = [0; 6];
        let precision = Precision::High;
        let cmd = Command::SetHeaterThenMeasureHighPrec(hpow, hdur);
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        SHT40Driver::<I2C, D>::parse_measurement(rx_bytes, precision, temp_unit, false)
    }
    
    /// Activate the SHT40 built-in heater with the specified power and duration
    /// then perform a high-precision measurement just before deactivation.
    /// Similar to set_heater_then_measure but returns a RawMeasurement.
    ///
    /// Heater activation can be used to remove condensed or sprayed water 
    /// from the sensor's surface, which make the sensor unresponsive to 
    /// relative humidity changes as long as there is still liquid water on the
    /// surface. Likewise, in high-humidity environments this could allow 
    /// creep-free measurements for extended times.
    ///
    /// The on-chip heater of SHT40 is designed for a maximal duty cycle of less
    /// than 5% (total heater-on time should not be longer than 5% of the
    /// sensor's lifetime). It is the responsibility of the callers of this
    /// function to keep track of this. 
    ///
    /// Additionally, the temperature sensor can be affected by the 
    /// thermally-induced mechanical stress from the heater, ofsetting the
    /// measured temperature. Likewise, any offsets which need to be applied
    /// to the measured values are the responsibility of the user of this API.
    ///
    /// For more details, consult the sensor's datasheet or additional Sensirion
    /// application notes.
    ///
    /// The driver blocks for the max time the heater is active, depending on
    /// the requested HeaterDuration, in order to get the measurement results.
    ///
    /// # Arguments
    ///
    /// * `hpow` - The power at which the heater should be activated
    /// * `hdur` - The duration for which the heater should be active. The
    ///            longest (HeaterDuration::PulseLong) is ~ 1s. After this 
    ///            duration, the heater is automatically deactivated.
    ///
    pub fn set_heater_then_measure_raw(&mut self, hpow: HeaterPower, hdur: HeaterDuration)
        -> Result<RawMeasurement, Error<E>> {
        let mut rx_bytes = [0; 6];
        let precision = Precision::High;
        let cmd = Command::SetHeaterThenMeasureHighPrec(hpow, hdur);
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        SHT40Driver::<I2C, D>::parse_sensor_raw_measurement(rx_bytes,
                                                            precision,
                                                            false)
    }

    /// Get the SHT40 sensor serial number
    pub fn get_serial(&mut self) -> Result<SerialNumber, Error<E>> {
        let mut rx_bytes = [0; 6];
        let cmd = Command::GetSerialNumber;
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        
        let serial_a = ((rx_bytes[0] as u16) << 8)  + (rx_bytes[1] as u16);
        let serial_b = ((rx_bytes[3] as u16) << 8)  + (rx_bytes[4] as u16);

        Ok(SerialNumber(serial_a, serial_b))
    }

    /// Perform a soft reset of the sensor. After this command, the user of the
    /// API should wait for at least 1ms (the Soft reset time) before issuing
    /// further commands. The driver itself does not block for this duration,
    /// but just issues the soft reset command.
    pub fn soft_reset_device(&mut self) -> Result<(), Error<E>> {
        let cmd = Command::SoftReset;
        let dev_cmd = cmd.as_device_command();
        self.i2c.write(self.address, &dev_cmd.cmd_code.to_be_bytes())
            .map_err(|err| { Error::I2c(err) })?;
        Ok(())
    }


}

#[cfg(test)]
mod tests {
    use embedded_hal_mock as hal;
    use float_cmp::approx_eq;

    use self::hal::delay::MockNoop as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    const SHT40_I2C_ADDR: I2CAddr = I2CAddr::SHT4x_A;

    fn gen_measurement_buf(temp: f32, tunit: TempUnit, rel_hum: f32) -> [u8; 6] {
        let rel_hum_ticks = ((rel_hum + 6.0) / 125.0 * 65535.0) as u16;
        let rel_temp_ticks;
        if tunit == TempUnit::Celsius {
            rel_temp_ticks = ((temp + 45.0) / 175.0 * 65535.0) as u16;
        } else {
            rel_temp_ticks = ((temp + 49.0) / 315.0 * 65535.0) as u16;
        }
        let temp_buf = rel_temp_ticks.to_be_bytes();
        let temp_crc = crc8::calculate(&temp_buf);
        let rel_hum_buf = rel_hum_ticks.to_be_bytes();
        let rel_crc = crc8::calculate(&rel_hum_buf);
        [temp_buf[0], temp_buf[1], temp_crc, rel_hum_buf[0], rel_hum_buf[1], rel_crc]
    }

    fn gen_serial_buf(serial_a: u16, serial_b: u16) -> [u8; 6] {
        let sa_buf = serial_a.to_be_bytes();
        let sa_crc = crc8::calculate(&sa_buf);
        let sb_buf = serial_b.to_be_bytes();
        let sb_crc = crc8::calculate(&sb_buf);
        [sa_buf[0], sa_buf[1], sa_crc, sb_buf[0], sb_buf[1], sb_crc]
    }

    fn gen_mock_expectations(command_expectations: &[(Command, [u8; 6])]) -> Vec<Transaction> {
        let mut expectations = Vec::<Transaction>::new();
        for cmd_ex in command_expectations {
            let dev_cmd = cmd_ex.0.as_device_command();
            let cmd_code = dev_cmd.cmd_code.to_be_bytes();
            expectations.push(Transaction::write(SHT40_I2C_ADDR as u8, cmd_code.to_vec()));
            expectations.push(Transaction::read(SHT40_I2C_ADDR as u8, cmd_ex.1.to_vec()));
        }
        expectations
    }

    #[test]
    fn test_sht40_serialnumber() {
        let serial_a = 23100;
        let serial_b = 25;

        // Mock expected I2C transactions
        let command_expectations = [
            (Command::GetSerialNumber, gen_serial_buf(serial_a, serial_b))
        ];
        let i2c_expectations = gen_mock_expectations(&command_expectations);
        let i2c_mock = I2cMock::new(&i2c_expectations);

        let mut sht40 = SHT40Driver::new(i2c_mock, SHT40_I2C_ADDR, DelayMock);

        if let Ok(SerialNumber(a, b)) = sht40.get_serial() {
           assert_eq!(a, serial_a);
           assert_eq!(b, serial_b);
        }
    }

    #[test]
    fn test_sht40_measurement() {
        let temp: f32 = 23.8;
        let tunit = TempUnit::Celsius;
        let rel_hum: f32 = 65.2;
        let buf = gen_measurement_buf(temp, tunit, rel_hum); 

        // Mock expected I2C transactions
        let command_expectations = [
            (Command::MeasureTempAndHumidity(Precision::High), buf)
        ];
        let i2c_expectations = gen_mock_expectations(&command_expectations);
        let i2c_mock = I2cMock::new(&i2c_expectations);

        let mut sht40 = SHT40Driver::new(i2c_mock, SHT40_I2C_ADDR, DelayMock);

        if let Ok(m) = sht40.get_temp_and_rh(Precision::High, tunit) {
           assert_eq!(m.temp_unit, tunit);
           assert!(approx_eq!(f32, m.temp, temp, epsilon = 0.01));
           assert!(approx_eq!(f32, m.rel_hum_percent, rel_hum, epsilon = 0.01));
        }
    }
    
    #[test]
    fn test_sht40_heater_and_measurement() {
        let temp: f32 = 23.8;
        let tunit = TempUnit::Celsius;
        let rel_hum: f32 = 65.2;
        let hpow = HeaterPower::High;
        let hdur = HeaterDuration::PulseShort;
        let buf = gen_measurement_buf(temp, tunit, rel_hum); 

        // Mock expected I2C transactions
        let command_expectations = [
            (Command::SetHeaterThenMeasureHighPrec(hpow, hdur), buf)
        ];
        let i2c_expectations = gen_mock_expectations(&command_expectations);
        let i2c_mock = I2cMock::new(&i2c_expectations);

        let mut sht40 = SHT40Driver::new(i2c_mock, SHT40_I2C_ADDR, DelayMock);

        if let Ok(m) = sht40.set_heater_then_measure(hpow, hdur, tunit) {
           assert_eq!(m.temp_unit, tunit);
           assert_eq!(m.precision, Precision::High);
           assert!(approx_eq!(f32, m.temp, temp, epsilon = 0.01));
           assert!(approx_eq!(f32, m.rel_hum_percent, rel_hum, epsilon = 0.01));
        }
    }
}
