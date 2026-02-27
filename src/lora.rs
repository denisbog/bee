use defmt::{error, info};
use embassy_nrf::{
    gpio::Output,
    spim::{Instance, Spim},
};

pub mod packet {
    #[repr(C)]
    pub struct SensorPacket {
        pub device_id: u8,
        pub temperature: i16,
        pub humidity: u16,
        pub battery_mv: u16,
        pub counter: u16,
    }

    impl SensorPacket {
        pub fn new(device_id: u8) -> Self {
            Self {
                device_id,
                temperature: 0,
                humidity: 0,
                battery_mv: 0,
                counter: 0,
            }
        }

        pub fn increment(&mut self) {
            self.counter = self.counter.wrapping_add(1);
        }
    }
}

const SX1278_FREQ: u32 = 433_200_000;

const REG_OP_MODE: u8 = 0x01;
const REG_FRF_MSB: u8 = 0x06;
const REG_FRF_MID: u8 = 0x07;
const REG_FRF_LSB: u8 = 0x08;
const REG_PA_CONFIG: u8 = 0x09;
const REG_PA_RAMP: u8 = 0x0A;
const REG_LNA: u8 = 0x0C;
const REG_FIFO_ADDR_PTR: u8 = 0x0D;
const REG_FIFO_TX_BASE_ADDR: u8 = 0x0E;
const REG_IRQ_FLAGS: u8 = 0x12;
const REG_PA_DAC: u8 = 0x4D;
const REG_MODEM_CONFIG_1: u8 = 0x1D;
const REG_MODEM_CONFIG_2: u8 = 0x1E;
const REG_MODEM_CONFIG_3: u8 = 0x26;
const REG_PAYLOAD_LENGTH: u8 = 0x22;

const MODE_LONG_RANGE: u8 = 0x80;
const MODE_SLEEP: u8 = 0x00;
const MODE_STDBY: u8 = 0x01;
const MODE_TX: u8 = 0x03;

const IRQ_TX_DONE: u8 = 0x08;

const PA_DAC_HIGH: u8 = 0x87;
const REG_VERSION: u8 = 0x42;

pub struct LoraRadio<'d, T: Instance> {
    nss: Output<'d>,
    reset: Output<'d>,
    spi: Spim<'d, T>,
}

impl<'d, T: Instance> LoraRadio<'d, T> {
    pub fn new(nss: Output<'d>, reset: Output<'d>, spi: Spim<'d, T>) -> Result<Self, ()> {
        Ok(Self { nss, reset, spi })
    }
    pub fn init(&mut self) -> Result<(), Error> {
        self.nss.set_high();
        self.reset.set_low();
        delay(10_000);
        self.reset.set_high();
        delay(10_000);

        info!("SX1278 reset complete");

        let version = spi_read(&mut self.spi, &mut self.nss, REG_VERSION);
        info!("SX1278 version: {:#04x}", version);

        if version != 0x12 {
            error!("Unexpected SX1278 version! Expected 0x12");
            Err(Error::SPI)?
        } else {
            info!("SX1278 initialized");
            Ok(())
        }
    }
    pub fn setup(&mut self) {
        spi_write(
            &mut self.spi,
            &mut self.nss,
            REG_OP_MODE,
            MODE_LONG_RANGE | MODE_SLEEP,
        );
        delay(10_000);

        spi_write(&mut self.spi, &mut self.nss, 0x31, 0x18);
        spi_write(&mut self.spi, &mut self.nss, 0x2E, 0x0);
        spi_write(&mut self.spi, &mut self.nss, 0x3E, 0x0);

        spi_write(&mut self.spi, &mut self.nss, REG_FIFO_TX_BASE_ADDR, 0);
        spi_write(&mut self.spi, &mut self.nss, REG_FIFO_ADDR_PTR, 0);

        spi_write(&mut self.spi, &mut self.nss, REG_LNA, 0x23);
        spi_write(&mut self.spi, &mut self.nss, REG_PA_RAMP, 0x09);
        spi_write(&mut self.spi, &mut self.nss, REG_PA_CONFIG, 0xFF);

        set_frequency(&mut self.spi, &mut self.nss, SX1278_FREQ);
        info!("Frequency set to {} Hz", SX1278_FREQ);

        spi_write(&mut self.spi, &mut self.nss, REG_MODEM_CONFIG_1, 0x72);
        spi_write(&mut self.spi, &mut self.nss, REG_MODEM_CONFIG_2, 0x74);
        spi_write(&mut self.spi, &mut self.nss, REG_MODEM_CONFIG_3, 0x04);
        info!("LoRa config: BW=125kHz, SF=7, CR=4/5");

        // spi_write(&mut self.spi, &mut self.nss, REG_PA_DAC, PA_DAC_HIGH);
        // info!("PA_DAC enabled for +20 dBm");

        spi_write(
            &mut self.spi,
            &mut self.nss,
            REG_OP_MODE,
            MODE_LONG_RANGE | MODE_STDBY,
        );
        delay(10_000);
    }

    pub fn write(&mut self) {
        let packet_count = 1;
        info!("TX packet #{}", packet_count);
        let mut packet_data = [0u8; 16];

        packet_data[0] = (packet_count >> 24) as u8;
        packet_data[1] = (packet_count >> 16) as u8;
        packet_data[2] = (packet_count >> 8) as u8;
        packet_data[3] = packet_count as u8;

        for i in 4..16 {
            packet_data[i] = i as u8;
        }

        tx_packet(&mut self.spi, &mut self.nss, &packet_data);
        info!("TX packet #{}.done", packet_count);
    }
}

fn delay(delay: u64) {
    embassy_time::block_for(embassy_time::Duration::from_micros(delay));
}

fn tx_packet<'d, T: Instance>(spi: &mut Spim<'d, T>, nss: &mut Output<'d>, data: &[u8]) {
    spi_write(spi, nss, REG_FIFO_ADDR_PTR, 0);

    nss.set_low();
    spi.blocking_write(&[REG_FIFO_ADDR_PTR as u8 | 0x80]).ok();
    spi.blocking_write(data).ok();
    nss.set_high();

    spi_write(spi, nss, REG_PAYLOAD_LENGTH, data.len() as u8);

    spi_write(spi, nss, REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

    wait_tx_done(spi, nss);
}

fn wait_tx_done<'d, T: Instance>(spi: &mut Spim<'d, T>, nss: &mut Output<'d>) {
    loop {
        let irq_flags = spi_read(spi, nss, REG_IRQ_FLAGS);
        if irq_flags & IRQ_TX_DONE != 0 {
            spi_write(spi, nss, REG_IRQ_FLAGS, IRQ_TX_DONE);
            break;
        }
    }
}

fn set_frequency<'d, T: Instance>(spi: &mut Spim<'d, T>, nss: &mut Output<'d>, freq: u32) {
    let frf = (freq as u64) << 19;
    let frf = frf / 32_000_000;
    let msb = ((frf >> 16) & 0xFF) as u8;
    let mid = ((frf >> 8) & 0xFF) as u8;
    let lsb = (frf & 0xFF) as u8;
    spi_write(spi, nss, REG_FRF_MSB, msb);
    spi_write(spi, nss, REG_FRF_MID, mid);
    spi_write(spi, nss, REG_FRF_LSB, lsb);
}

fn spi_write<'d, T: Instance>(spi: &mut Spim<'d, T>, nss: &mut Output<'d>, reg: u8, value: u8) {
    nss.set_low();
    let buffer = [reg | 0x80, value];
    spi.blocking_write(&buffer).ok();
    nss.set_high();
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum Error {
    SPI,
    Busy,
    NotInitialized,
}

fn spi_read<T: Instance>(spi: &mut Spim<'_, T>, nss: &mut Output<'_>, reg: u8) -> u8 {
    nss.set_low();
    let mut buffer = [reg, 0x00];
    spi.blocking_transfer_in_place(&mut buffer).ok();
    nss.set_high();
    buffer[1]
}
