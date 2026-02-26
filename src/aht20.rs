use crc::{CRC_8_NRSC_5, Crc};
use defmt::{bitflags, debug, info, warn};
use embassy_nrf::twim::{Instance, Twim};

const AHT20_ADDR: u8 = 0x38;
const AHT20_CMD_INIT: &[u8] = &[0xBE, 0x08, 0x00];
const AHT20_CMD_TRIGGER: &[u8] = &[0xAC, 0x33, 0x00];
const AHT20_CHECK_STATUS_COMMAND: &[u8] = &[0x71];

bitflags! {
    struct SensorStatus: u8 {
        const BUSY = 0b1000_0000;
        const CALIBRATED = 0b0000_1000;
        const UNKNOWN = 24;
    }
}

impl SensorStatus {
    fn is_calibrated(&self) -> bool {
        self.contains(SensorStatus::CALIBRATED)
    }

    fn is_ready(&self) -> bool {
        !self.contains(SensorStatus::BUSY)
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct Aht20Reading {
    pub temperature: f32,
    pub humidity: f32,
}

pub struct Aht20<'d, T: Instance> {
    twim: Twim<'d, T>,
    initialized: bool,
}

impl<'d, T: Instance> Aht20<'d, T> {
    pub fn new(twim: Twim<'d, T>) -> Self {
        Self {
            twim,
            initialized: false,
        }
    }

    pub fn init(&mut self) -> Result<(), Error> {
        while !self.check_status().unwrap().is_calibrated() {
            info!("AHT20 wait for calibration");
            self.send_initialize()?;
            self.delay_ms(10);
        }

        self.initialized = true;
        info!("AHT20 initialized");
        Ok(())
    }

    fn check_status(&mut self) -> Result<SensorStatus, Error> {
        let mut buffer = [0];
        self.twim
            .blocking_write_read(AHT20_ADDR, AHT20_CHECK_STATUS_COMMAND, &mut buffer)
            .map_err(|_| Error::I2c)?;
        Ok(SensorStatus::from_bits(buffer[0]).unwrap())
    }
    fn delay_ms(&self, arg: u64) {
        embassy_time::block_for(embassy_time::Duration::from_millis(arg));
    }

    fn send_initialize(&mut self) -> Result<(), Error> {
        self.twim
            .blocking_write(AHT20_ADDR, AHT20_CMD_INIT)
            .map_err(|_| Error::I2c)?;
        Ok(())
    }
    pub fn read(&mut self) -> Result<Aht20Reading, Error> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        self.twim
            .blocking_write(AHT20_ADDR, AHT20_CMD_TRIGGER)
            .map_err(|_| Error::I2c)?;

        self.delay_ms(80);

        while !self.check_status().unwrap().is_ready() {
            self.delay_ms(1);
        }
        let mut buf = [0u8; 7];
        self.twim
            .blocking_read(AHT20_ADDR, &mut buf)
            .map_err(|_| Error::I2c)?;
        let status = SensorStatus::from_bits(buf[0]).unwrap();
        if !status.is_ready() {
            return Err(Error::Busy);
        }
        let data = &buf[1..6];
        let crc = &buf[6];
        debug!("data: {}", data);
        debug!("crc: {}", crc);
        let crc_d = Crc::<u8>::new(&CRC_8_NRSC_5);
        let mut digest = crc_d.digest();
        digest.update(data);
        if digest.finalize() != *crc {
            warn!("crc failed");
        }
        debug!("data: {}", data);
        let humidity_raw: u32 =
            ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] >> 4) as u32);
        let temp_raw: u32 =
            (((data[2] & 0x0F) as u32) << 16) | ((data[3] as u32) << 8) | (data[4] as u32);
        let humidity = ((humidity_raw as f32) / ((1 << 20) as f32)) * 100.0;
        let temperature = ((temp_raw as f32) / ((1 << 20) as f32)) * 200.0 - 50.0;
        info!("AHT20 reading: {}°C, {}%", temperature, humidity);

        Ok(Aht20Reading {
            temperature,
            humidity,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum Error {
    I2c,
    Busy,
    NotInitialized,
}
