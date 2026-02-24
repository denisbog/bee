use defmt::info;
use embassy_nrf::twim::{Instance, Twim};

const AHT20_ADDR: u8 = 0x38;
const AHT20_CMD_INIT: &[u8] = &[0xE1, 0x08, 0x00];
const AHT20_CMD_TRIGGER: &[u8] = &[0xAC, 0x33, 0x00];

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
        self.twim
            .blocking_write(AHT20_ADDR, AHT20_CMD_INIT)
            .map_err(|_| Error::I2c)?;

        embassy_time::block_for(embassy_time::Duration::from_millis(10));

        self.initialized = true;
        info!("AHT20 initialized");
        Ok(())
    }

    pub fn read(&mut self) -> Result<Aht20Reading, Error> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        self.twim
            .blocking_write(AHT20_ADDR, AHT20_CMD_TRIGGER)
            .map_err(|_| Error::I2c)?;

        embassy_time::block_for(embassy_time::Duration::from_millis(80));

        let mut buf = [0u8; 6];
        self.twim
            .blocking_read(AHT20_ADDR, &mut buf)
            .map_err(|_| Error::I2c)?;

        let status = buf[0];
        if status & 0x80 != 0 {
            return Err(Error::Busy);
        }

        let humidity_raw =
            ((buf[1] as u32) << 12) | ((buf[2] as u32) << 4) | ((buf[3] as u32) >> 4);
        let temp_raw = (((buf[3] as u32) & 0x0F) << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32);

        let humidity = (humidity_raw as f32 / 1048576.0) * 100.0;
        let temperature = ((temp_raw as f32 / 1048576.0) * 200.0) - 50.0;

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
