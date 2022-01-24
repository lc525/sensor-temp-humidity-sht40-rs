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
/// ## Features
///
/// - "fp": enable this feature if your CPU or MCU has a floating-point unit
///   and you want to be able to convert measured values to SI units (degrees
///   Celsius, degreed Fahrenheit for temperature and % for relative humidity).
///
///   The driver stores measurements in milli degrees Celsius or milli degrees
///   Fahrenheit for temperature and in per cent mille (pcm) for relative 
///   humidity, and does conversions using fixed-point arithmetic to minimise
///   loss of precision. Therefore, a temperature of 23.89 degrees Celsius will
///   be stored as 23890 and and a humidity of 56.2% is stored as 56200.
///
///   The fp feature adds data structures and a function for converting a
///   Measurement structure to a SIMeasurement structure, converting to floating
///   point as the very last step (after any other calibration offsets have
///   been applied to the measured values).
///
/// ## Usage:
///
/// Import this crate and an `embedded_hal` implementation, then instantiate the
/// driver; 
///
/// * For example, assuming you have connected a SH40 sensor to a linux
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
///         sht40.get_temp_and_rh(Precision::High, TempUnit::MilliDegreesCelsius) {
///       println!("Temp: {temp} C, Relative Humidity: {rh} %",
///                temp = measurement.temp,
///                rh = measurement.rel_hum_pcm);
///     }
/// }
/// ```
///
/// * Likewise, if you are using an ESP32-based board:
///
/// ```ignore
/// use esp_idf_sys;
/// use esp_idf_hal::delay;
/// use esp_idf_hal::i2c;
/// use esp_idf_hal::prelude::*;
/// use sensor_temp_humidity_sht40::{SHT40Driver, I2CAddr, Precision, TempUnit};
///
/// fn main() {
///     let peripherals = Peripherals::take().unwrap();
///     let pins = peripherals.pins;
///
///     let i2c = peripherals.i2c0;
///     let scl = pins.gpio9;
///     let sda = pins.gpio8;
///     let i2c_config = <i2c::config::MasterConfig as Default>::default()
///                      .baudrate(400.kHz().into());
///
///     let mut sensor_drv = SHT40Driver::new(
///         i2c::Master::<i2c::I2C0, _, _>::new(i2c, 
///                                             i2c::MasterPins { sda, scl }, 
///                                             i2c_config).unwrap(), 
///         I2CAddr::SHT4x_A, 
///         delay::Ets);
///
///     let measurement = sensor_drv.get_temp_and_rh(Precision::High,
///                                                  TempUnit::MilliDegreesCelsius);
///     println!("Measurement {:#?}", measurement);
/// }
/// ```

use embedded_hal as hal;

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

use sensirion_i2c::{crc8, i2c};

const MAX_MILLICELSIUS_OFFSET:i16 = 17000;
const MAX_MILLIFAHRENHEIT_OFFSET:i16 = 32000;
const MAX_PCMHUM_OFFSET:i16 = 32000;

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
    MilliDegreesFahrenheit,
    MilliDegreesCelsius,
}

#[cfg(feature="fp")]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SITempUnit {
    Fahrenheit,
    Celsius,
}

#[derive(Debug, Clone, Copy)]
pub struct TempOffset(i16, TempUnit);

#[derive(Debug, Clone, Copy)]
pub struct HumPcmOffset(i16);

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
    temp_offset: Option<TempOffset>,
    hum_pcm_offset: Option<HumPcmOffset>,
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

#[derive(Debug)]
pub enum CalibrationError {
    OffsetTooLarge,
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
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Measurement {
    /// Measured temperature
    pub temp: i32,
    /// Unit of temperature measurement (milliC or milliF)
    pub temp_unit: TempUnit,
    /// Measured relative humidity (pcm = percent mille = per 100_000)
    pub rel_hum_pcm: u32,
    /// The precision that was requested when performing the measurement.
    /// For measurements obtained through set_heater_then_measure(...),
    /// the sensor automatically performs a Precision::High measurement.
    pub precision: Precision,
}

#[cfg(feature="fp")]
#[derive(Debug, Clone, Copy)]
pub struct SIMeasurement {
    /// Measured temperature
    pub temp: f32,
    /// Unit of temperature measurement (C or F)
    pub temp_unit: SITempUnit,
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


impl Command {
    fn as_device_command(self) -> DeviceCommand {
       match self {
            Command::SoftReset => DeviceCommand { cmd_code: 0x94,
                                                  max_duration_ms: 1 },
            Command::GetSerialNumber => DeviceCommand { cmd_code: 0x89,
                                                        max_duration_ms: 1 },
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
        SHT40Driver { i2c, 
                      address: address as u8, 
                      delay, 
                      temp_offset: None, 
                      hum_pcm_offset: None }
    }

    pub fn set_temp_offset(&mut self, offset: TempOffset) -> Result<(), CalibrationError> {
        let too_large = match offset.1 {
            TempUnit::MilliDegreesCelsius => { 
                if offset.0.abs() > MAX_MILLICELSIUS_OFFSET { 
                    true 
                } else { 
                    false 
                } 
            },
            TempUnit::MilliDegreesFahrenheit => { 
                if offset.0.abs() > MAX_MILLIFAHRENHEIT_OFFSET { 
                    true 
                } else { 
                    false
                } 
            }
        };
        if !too_large {
            self.temp_offset = Some(offset);
            Ok(())
        } else {
            Err(CalibrationError::OffsetTooLarge)
        }
    }

    pub fn clear_temp_offset(&mut self) {
        self.temp_offset = None;
    }

    pub fn set_hum_offset(&mut self, offset: HumPcmOffset) -> Result<(), CalibrationError> {
        if offset.0.abs() < MAX_PCMHUM_OFFSET {
            self.hum_pcm_offset = Some(offset);
            Ok(())
        } else {
            Err(CalibrationError::OffsetTooLarge)
        }
    }
    
    pub fn clear_hum_offset(&mut self) {
        self.hum_pcm_offset = None;
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


    fn parse_measurement(rx_bytes: [u8; 6], precision: Precision, temp_unit: TempUnit,
                         check_crc: bool, temp_offset: Option<i16>, hum_pcm_offset: Option<i16>)
        -> Result<Measurement, Error<E>> {
            let raw = SHT40Driver::<I2C, D>::parse_sensor_raw_measurement(rx_bytes,
                                                              precision,
                                                              check_crc)?;
            Ok(convert_raw_to_units(raw, temp_unit, temp_offset, hum_pcm_offset))
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
    /// No offsets set using `set_temp_offset` or `set_hum_offset` are applied,
    /// as it is assumed the user of the API fully deals with any calibration
    /// required beyond the one done on the sensor itself
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


    /// The calibration offset and the measurement units are provided 
    /// independently by users of this API. For example, the offset might
    /// be set in milli degrees Fahrenheit, but the measurement is done in
    /// milli degrees Celsius.
    ///
    /// Although this is not necessarily recommended because the need of
    /// a further fixed-precision conversion of the offset, the API supports
    /// the scenario by detecting the mismatch and converting.
    ///
    /// This is not a classical conversion of the offset from milliFahrenheit
    /// to milliCelsius, because we're interested in the finite difference
    /// (in a mathematical sense). If C is the temperature in Celsius and f(C)
    /// is the conversion in Fahrenheit, we have:
    ///
    /// ```ignore
    ///
    /// f(C) = 9/5 * C + 32
    /// f(C + off) = 9/5 * (C + off) + 32
    /// 
    /// ```
    ///
    /// Therefore, for an offset of "off" we have
    /// 
    /// ```ignore
    /// 
    /// f(C + off) - f(C) = 9/5 * off =>
    /// f(C + off) - f(C) = 1.8 * off
    /// 
    /// ```
    ///
    /// This means that applying the offset 'off' to the the Celsius temperature
    /// is equivalent to applying the offset 9/5 * off to the Fahrenheit 
    /// temperature
    ///
    /// The function below implements exactly that using fixed precision
    /// arithmetic.
    ///
    fn convert_temp_offset(temp_off: Option<TempOffset>, to_unit: TempUnit, 
                           hum_pcm_off: Option<HumPcmOffset>) 
        -> (Option<i16>, Option<i16>) {
        let temp_offset: Option<i16>;
        let hum_offset: Option<i16>;
        
        if let Some(t_off) = temp_off {
            if t_off.1 != to_unit {
               temp_offset = match to_unit {
                   TempUnit::MilliDegreesCelsius => {
                       // t_off.0 in milliFahrenheit, convert to milliCelsius
                       Some(((t_off.0 as i32) * 555 / 1000) as i16)
                       
                   },
                   TempUnit::MilliDegreesFahrenheit => {
                       // t_off.0 delta in milliCelsius, convert to 
                       // milliFahrenheit offset
                       Some(((t_off.0 as i32) * 18 / 10) as i16)
                   }
               }
            } else {
                temp_offset = Some(t_off.0);
            }
        } else {
            temp_offset = None;
        }
        hum_offset = hum_pcm_off.map(|off|{ off.0 });
        (temp_offset, hum_offset)
    }

    /// Get the results of a measurement, with temperature measured in the 
    /// requested units (milli degrees C or milli degrees F).
    ///
    /// Relative humidity is unitless and always reported as
    /// pcm (percent mille = per 100_000).
    ///
    /// You can easily convert into degrees C, degrees F (for temp) and percent
    /// (%) for relative humidity by dividing the returned values by 1000. They
    /// are stored as milli-* so that all internal conversions can be done using
    /// integer arithmetic, without loss of precision. That way, MCUs without
    /// a floating-point unit are supported.
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
    ///                 choose between TempUnit::MilliDegreesCelsius and 
    ///                 TempUnit::MilliDegreesFahrenheit
    ///
    pub fn get_temp_and_rh(&mut self, precision: Precision, temp_unit: TempUnit)
        -> Result<Measurement, Error<E>> {
        let mut rx_bytes = [0; 6];
        let cmd = Command::MeasureTempAndHumidity(precision);
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        let (temp_offset, hum_offset) = 
            SHT40Driver::<I2C, D>::convert_temp_offset(self.temp_offset, 
                                                       temp_unit, 
                                                       self.hum_pcm_offset);
        SHT40Driver::<I2C, D>::parse_measurement(rx_bytes, precision, temp_unit,
                                                 false, temp_offset, hum_offset)
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
    ///                 You can choose between TempUnit::MilliDegreesCelsius and 
    ///                 TempUnit::MilliDegreesFahrenheit
    ///
    pub fn set_heater_then_measure(&mut self, hpow: HeaterPower, hdur: HeaterDuration, temp_unit: TempUnit)
        -> Result<Measurement, Error<E>> {
        let mut rx_bytes = [0; 6];
        let precision = Precision::High;
        let cmd = Command::SetHeaterThenMeasureHighPrec(hpow, hdur);
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        let (temp_offset, hum_offset) = 
            SHT40Driver::<I2C, D>::convert_temp_offset(self.temp_offset, 
                                                       temp_unit, 
                                                       self.hum_pcm_offset);
        SHT40Driver::<I2C, D>::parse_measurement(rx_bytes, precision, temp_unit,
                                                 false, temp_offset, hum_offset)
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
    /// measured temperature. If you want a more accurate temperature 
    /// measurement, it is recommended you wait ~1s after calling this function
    /// and then get another normal measurement (not activating the heater).
    ///
    /// For more details, consult the sensor's datasheet or additional Sensirion
    /// application notes.
    ///
    /// No offsets set using `set_temp_offset` or `set_hum_offset` are applied,
    /// as it is assumed the user of the API fully deals with any calibration
    /// required beyond the one done on the sensor itself
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
    pub fn get_serial(&mut self) -> Result<u32, Error<E>> {
        let mut rx_bytes = [0; 6];
        let cmd = Command::GetSerialNumber;
        self.i2c_command_and_response(cmd, Some(&mut rx_bytes))?;
        
        let serial_a = ((rx_bytes[0] as u16) << 8)  + (rx_bytes[1] as u16);
        let serial_b = ((rx_bytes[3] as u16) << 8)  + (rx_bytes[4] as u16);

        Ok( ((serial_a as u32) << 16) + (serial_b as u32) )
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


///
/// The SHT40 sensor uses a 16-bit ADC for converting the analog 
/// measurements into data, which is then calibrated on-sensor and then 
/// sent via I2C. In the raw measurement, temp_ticks and rel_hum_ticks are
/// direct outputs after the calibration process and take values in the
/// [0 .. 2^16 - 1] interval.
///
/// In order to convert the ticks into usable values, we map this interval
/// onto the following intervals:
///
/// For temp. in Celsius: [-45 .. 130]
/// For temp. in Fahrenheit: [-49 .. 266] (equivalent to the C interval)
/// For humidity (in %): [-6 .. 119], with values below 0 and above 100 not
///                                   having physical meaning.
///
/// Mathematically, this means that if a and b are the limits of the interval
/// [a .. b] for a particular metric (temp in C, temp in F, humidity) then 
/// the value of the metric, M is computed according to:
///
/// ```ignore
///
///     (b - a) * Mticks 
/// M = ----------------- + a                                       (1)
///          2^16 - 1
/// ```
///
/// However, we will not directly implement this formula but optimise it
/// for fixed-point algebra. Therefore, the Measurement will hold only
/// integer values rather than floating-point ones, and instead of the
/// typical measurement units (C, F or %) we will output values 1000-times
/// greater (units in milliC, milliF, pcm). 
///
/// Concretely, a temperature of 23.5 C will be returned as 23500 milliC and
/// a humidity of 45.64 % will be returned as 45640 pcm (part per 100_000)
///
/// We write (1) as:
///
/// ```ignore
///
///               (b - a) * 1000 * Mticks
/// M * 1000 = ---------------------------- + (a * 1000)           (2)
///                    (               1  )
///              2^i * ( 2^(16-i) -  -----)
///                    (              2^i )
/// ```
///
/// and we search for i that minimizes the fractional part, implementing the
/// first division by 2^i as a bit shifit and the pre-computing an integer
/// for ((b - a) * 1000) / (2^(16-i) - 1/2^i)
/// 
/// User provided offsets for temp and humidity can be used when externally
/// calibrating the sensor. It is assumed that the unit for temp_offset is
/// the same as temp_unit and that hum_pcm_offset is expressed in per cent 
/// mille (pcm, part per 100_000).
///
/// The relative humidity returned as part of the measurement result is set to
/// 0 if negative and to 100_000 if greater than 100_000 pcm (100%)
/// 
pub fn convert_raw_to_units(raw_meas: RawMeasurement, temp_unit: TempUnit,
                            temp_offset: Option<i16>, hum_pcm_offset: Option<i16>)
    -> Measurement {

    let temp_offset = temp_offset.unwrap_or(0);
    let hum_pcm_offset = hum_pcm_offset.unwrap_or(0);

    // compute (2) with a = -6, b = 119, i = 13
    let mut rel_hum_pcm = ((15625 * (raw_meas.rel_hum_ticks as i32)) >> 13) - 6000;
    rel_hum_pcm = rel_hum_pcm + (hum_pcm_offset as i32);

    // calibration may result in non-physical values which according to the
    // datasheet are only of interest when comparing the distribution of results
    // between multiple sensors:
    if rel_hum_pcm > 100000 { rel_hum_pcm = 100000; }
    if rel_hum_pcm < 0 { rel_hum_pcm = 0; }

    let temp = match temp_unit {
        // compute (2) with a = -45, b = 130, i = 13
        TempUnit::MilliDegreesCelsius =>  ((21875 * (raw_meas.temp_ticks as i32)) >> 13) - 45000,
        // compute (2) with a = -49, b = 266, i = 14
        TempUnit::MilliDegreesFahrenheit => ((78751 * (raw_meas.temp_ticks as i32)) >> 14) - 49000
    };


    Measurement {
        temp: temp + (temp_offset as i32),
        temp_unit,
        rel_hum_pcm: rel_hum_pcm as u32,
        precision: raw_meas.precision
    }
}

#[cfg(feature="fp")]
pub fn convert_measurement_to_si(meas: Measurement) -> SIMeasurement {
    SIMeasurement {
        temp: (meas.temp as f32) / 1000.0,
        temp_unit: match meas.temp_unit {
            TempUnit::MilliDegreesCelsius => SITempUnit::Celsius,
            TempUnit::MilliDegreesFahrenheit => SITempUnit::Fahrenheit,
        },
        rel_hum_percent: (meas.rel_hum_pcm as f32) / 1000.0,
        precision: meas.precision
    }
}


#[cfg(test)]
mod tests {
    use embedded_hal_mock as hal;

    use self::hal::delay::MockNoop as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    const SHT40_I2C_ADDR: I2CAddr = I2CAddr::SHT4x_A;

    fn gen_measurement_buf(temp: i32, tunit: TempUnit, rel_hum_pcm: u32) -> [u8; 6] {
        let rel_hum_ticks = (((rel_hum_pcm + 6000) << 13) / 15625) as u16;
        let rel_temp_ticks;
        if tunit == TempUnit::MilliDegreesCelsius {
            rel_temp_ticks = (((temp + 45000) << 13 ) / 21875 ) as u16;
        } else {
            rel_temp_ticks = (((temp + 49000) << 14) / 78751 ) as u16;
        }
        let temp_buf = rel_temp_ticks.to_be_bytes();
        let temp_crc = crc8::calculate(&temp_buf);
        let rel_hum_buf = rel_hum_ticks.to_be_bytes();
        let rel_crc = crc8::calculate(&rel_hum_buf);
        [temp_buf[0], temp_buf[1], temp_crc, rel_hum_buf[0], rel_hum_buf[1], rel_crc]
    }

    fn gen_serial_buf(serial_mock: u32) -> [u8; 6] {
        let s_buf = serial_mock.to_be_bytes();
        let sa_crc = crc8::calculate(&s_buf[0..2]);
        let sb_crc = crc8::calculate(&s_buf[2..4]);
        [s_buf[0], s_buf[1], sa_crc, s_buf[2], s_buf[3], sb_crc]
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
        let serial_mock: u32 = 231997321;

        // Mock expected I2C transactions
        let command_expectations = [
            (Command::GetSerialNumber, gen_serial_buf(serial_mock))
        ];
        let i2c_expectations = gen_mock_expectations(&command_expectations);
        let i2c_mock = I2cMock::new(&i2c_expectations);

        let mut sht40 = SHT40Driver::new(i2c_mock, SHT40_I2C_ADDR, DelayMock);

        if let Ok(serial) = sht40.get_serial() {
           assert_eq!(serial, serial_mock);
        }
    }

    #[test]
    fn test_sht40_measurement() {
        let temp: i32 = 20625;
        let tunit = TempUnit::MilliDegreesCelsius;
        let rel_hum_pcm: u32 = 9625;
        let buf = gen_measurement_buf(temp, tunit, rel_hum_pcm); 

        // Mock expected I2C transactions
        let command_expectations = [
            (Command::MeasureTempAndHumidity(Precision::High), buf)
        ];
        let i2c_expectations = gen_mock_expectations(&command_expectations);
        let i2c_mock = I2cMock::new(&i2c_expectations);

        let mut sht40 = SHT40Driver::new(i2c_mock, SHT40_I2C_ADDR, DelayMock);

        if let Ok(m) = sht40.get_temp_and_rh(Precision::High, tunit) {
           assert_eq!(m.temp_unit, tunit);
           assert_eq!(m.temp, temp);
           assert_eq!(m.rel_hum_pcm, rel_hum_pcm);
        }
    }
    
    #[test]
    fn test_sht40_measurement_with_offset_conversion() {
        let cmp_eps = 5;
        let temp_f: i32 = 77000;
        let temp_c: i32 = 25000;
        let tfunit = TempUnit::MilliDegreesFahrenheit;
        let tcunit = TempUnit::MilliDegreesCelsius;
        let rel_hum_pcm: u32 = 9625;
        let buf = gen_measurement_buf(temp_f, tfunit, rel_hum_pcm); 
        let buf_2 = gen_measurement_buf(temp_c, tcunit, rel_hum_pcm); 

        // Mock expected I2C transactions
        let command_expectations = [
            (Command::MeasureTempAndHumidity(Precision::High), buf),
            (Command::MeasureTempAndHumidity(Precision::High), buf_2)
        ];
        let i2c_expectations = gen_mock_expectations(&command_expectations);
        let i2c_mock = I2cMock::new(&i2c_expectations);

        let mut sht40 = SHT40Driver::new(i2c_mock, SHT40_I2C_ADDR, DelayMock);
        let mut t1_offset:i16 = -5000;
        sht40.set_temp_offset(TempOffset(t1_offset, TempUnit::MilliDegreesCelsius)).unwrap();
        let h1_offset:i16 = -1200;
        sht40.set_hum_offset(HumPcmOffset(h1_offset)).unwrap();

        if let Ok(m) = sht40.get_temp_and_rh(Precision::High, tfunit) {
           assert_eq!(m.temp_unit, tfunit);
           assert!((m.temp >= 68000 - cmp_eps) && (m.temp <= 68000 + cmp_eps));
           assert_eq!(m.rel_hum_pcm, rel_hum_pcm - 1200);
           sht40.clear_temp_offset();
           sht40.clear_hum_offset();
        }
        
        t1_offset = -9000;
        sht40.set_temp_offset(TempOffset(t1_offset, TempUnit::MilliDegreesFahrenheit)).unwrap();
        if let Ok(m) = sht40.get_temp_and_rh(Precision::High, tcunit) {
           assert_eq!(m.temp_unit, tcunit);
           assert!((m.temp >= 20000 - cmp_eps) && (m.temp <= 20000 + cmp_eps));
           assert_eq!(m.rel_hum_pcm, rel_hum_pcm);
        }

    }
    
    #[test]
    fn test_sht40_heater_and_measurement() {
        let temp: i32 = 20625;
        let tunit = TempUnit::MilliDegreesCelsius;
        let rel_hum_pcm: u32 = 9625;
        let hpow = HeaterPower::High;
        let hdur = HeaterDuration::PulseShort;
        let buf = gen_measurement_buf(temp, tunit, rel_hum_pcm); 

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
           assert_eq!(m.temp, temp);
           assert_eq!(m.rel_hum_pcm, rel_hum_pcm);
        }
    }

    #[cfg(feature="fp")]
    #[test]
    fn test_sht40_si_measurement() {
        let temp_si: f32 = 20.625;
        let tunit_si = SITempUnit::Celsius;
        let rel_hum_si: f32 = 9.625;

        let temp: i32 = 20625;
        let tunit = TempUnit::MilliDegreesCelsius;
        let rel_hum_pcm: u32 = 9625;
        let buf = gen_measurement_buf(temp, tunit, rel_hum_pcm); 

        // Mock expected I2C transactions
        let command_expectations = [
            (Command::MeasureTempAndHumidity(Precision::High), buf)
        ];
        let i2c_expectations = gen_mock_expectations(&command_expectations);
        let i2c_mock = I2cMock::new(&i2c_expectations);

        let mut sht40 = SHT40Driver::new(i2c_mock, SHT40_I2C_ADDR, DelayMock);

        if let Ok(m) = sht40.get_temp_and_rh(Precision::High, tunit) {
           let m_si = convert_measurement_to_si(m);
           assert_eq!(m_si.temp_unit, tunit_si);
           assert_eq!(m_si.temp, temp_si);
           assert_eq!(m_si.rel_hum_percent, rel_hum_si);
        }
    }
}
