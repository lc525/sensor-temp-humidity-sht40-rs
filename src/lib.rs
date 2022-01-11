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
/// used with any physical device implementing the embedded-hal traits. Please
/// note that this branch tracks the unstable development version of
/// embedded-hal-1.0.0
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
/// ```ignore
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

use hal::delay::blocking::DelayUs;
use hal::i2c::blocking::{Read, Write, WriteRead};

use sensirion_i2c::{crc8, i2c};

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
#[derive(Clone, Copy)]
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

#[derive(Clone, Copy)]
pub enum HeaterPower {
    /// High is typically 200mW (for VDD=3.3V)
    High,

    /// Medium is typically 110mW (for VDD=3.3V)
    Medium,

    /// Low is typically 20mW (for VDD=3.3V)
    Low,
}

#[derive(Clone, Copy)]
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
#[derive(Clone, Copy)]
pub enum I2CAddr {
    /// use SHT4x_A for all products with I2C address 0x44, product numbers
    /// SHT4x-Axxx
    SHT4x_A = 0x44,

    /// use SHT4x_B for all products with I2C address 0x45, product numbers
    /// SHT4x-Bxxx
    SHT4x_B = 0x45,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TempUnit {
    Farrenheit,
    Celsius,
}

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

pub struct Measurement {
    pub temp: f32,
    pub temp_unit: TempUnit,
    pub rel_hum_percent: f32,
    pub precision: Precision,
}

struct RawMeasurement {
    temp_ticks: u16,
    rel_hum_ticks: u16,
    precision: Precision,
}

#[derive(PartialEq, Eq, Clone, Copy)]
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
    D: DelayUs
{
    pub fn new(i2c: I2C, address: I2CAddr, delay: D) -> Self {
        SHT40Driver { i2c, address: address as u8, delay }
    }

    fn parse_sensor_raw_measurement(rx_bytes: [u8; 6], check_crc: bool, precision: Precision)
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
                                                              check_crc,
                                                              precision)?;
            Ok(SHT40Driver::<I2C, D>::convert_raw_to_units(raw, temp_unit))
    }

    fn i2c_command_and_response(&mut self, cmd: Command, rx_bytes: Option<&mut [u8]>)
        -> Result<(), Error<E>>{
        let dev_cmd = cmd.as_device_command();
        i2c::write_command_u8(&mut self.i2c, self.address, dev_cmd.cmd_code)
            .map_err(|err| { Error::I2c(err) })?;
        if let Err(_) = self.delay.delay_ms(dev_cmd.max_duration_ms as u32) {
            return Err(Error::<E>::DelayError)
        }
        if let Some(rx_bytes) = rx_bytes {
            i2c::read_words_with_crc(&mut self.i2c, self.address, rx_bytes)?;
        };
        Ok(())
    }

    pub fn get_temp_and_rh(&mut self, precision: Precision, temp_unit: TempUnit)
        -> Result<Measurement, Error<E>> {
        let mut rx_bytes = [0; 6];
        let cmd = Command::MeasureTempAndHumidity(precision);
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        SHT40Driver::<I2C, D>::parse_measurement(rx_bytes, precision, temp_unit, false)
    }

    pub fn get_sensor_serial(&mut self) -> Result<SerialNumber, Error<E>> {
        let mut rx_bytes = [0; 6];
        let cmd = Command::GetSerialNumber;
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        
        let serial_a = ((rx_bytes[0] as u16) << 8)  + (rx_bytes[1] as u16);
        let serial_b = ((rx_bytes[3] as u16) << 8)  + (rx_bytes[4] as u16);

        Ok(SerialNumber(serial_a, serial_b))
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

        if let Ok(SerialNumber(a, b)) = sht40.get_sensor_serial() {
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
}
