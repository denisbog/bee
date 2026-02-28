use alloc::vec;
use defmt::{error, info};
use embassy_nrf::{
    gpio::Output,
    spim::{Instance, Spim},
};
use embassy_time::{Duration, Timer};
use meshtastic_protobufs::MeshPacket;
use prost::Message;
pub mod meshtastic {
    use meshtastic_protobufs::MeshPacket;
    use prost::{DecodeError, Message, bytes::Buf};

    pub fn parse(data: impl Buf) -> Result<MeshPacket, DecodeError> {
        MeshPacket::decode(data)
    }
}

const SX1278_FREQ: u32 = 433_875_000;

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
const REG_MODEM_CONFIG_1: u8 = 0x1D;
const REG_MODEM_CONFIG_2: u8 = 0x1E;
const REG_MODEM_CONFIG_3: u8 = 0x26;
const REG_PAYLOAD_LENGTH: u8 = 0x22;

const MODE_LONG_RANGE: u8 = 0x80;
const MODE_SLEEP: u8 = 0x00;
const MODE_STDBY: u8 = 0x01;
const MODE_TX: u8 = 0x03;

const IRQ_TX_DONE: u8 = 0x08;
const REG_VERSION: u8 = 0x42;
const REG_RX_NB_BYTES: u8 = 0x13;
const REG_FIFO_RX_CURRENT: u8 = 0x10;
const REG_IRQ_FLAGS_MASK: u8 = 0x11;
const REG_IRQ_FLAGS_RX_DONE: u8 = 0x40;
const REG_IRQ_FLAGS_VALID_HEADER: u8 = 0x10;
const REG_RX_TX_ADDR: u8 = 0x4A;

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

        spi_write(&mut self.spi, &mut self.nss, REG_MODEM_CONFIG_1, 0b10000010);
        spi_write(&mut self.spi, &mut self.nss, REG_MODEM_CONFIG_2, 0b10110100);
        spi_write(&mut self.spi, &mut self.nss, REG_MODEM_CONFIG_3, 0b00000100);
        info!("LoRa config: BW=250kHz, SF=11, CR=4/5");

        spi_write(
            &mut self.spi,
            &mut self.nss,
            REG_OP_MODE,
            MODE_LONG_RANGE | MODE_STDBY,
        );
        delay(10_000);
    }

    pub fn write_meshtastic_telemetry(
        &mut self,
        temp_celsius: f32,
        humidity_percent: f32,
        voltage: f32,
        packet_id: u32,
    ) {
        info!(
            "Sending Meshtastic telemetry: temp={}C, humidity={}%, voltage={}V, id={}",
            temp_celsius, humidity_percent, voltage, packet_id
        );

        // tx_packet(&mut self.spi, &mut self.nss, &mhpacket.data[..len]);
        info!("TX complete");
    }

    pub async fn write(&mut self) {
        let packet = MeshPacket {
            from: 11,
            to: 0xFFFFFFFF,
            channel: 0,
            id: 11,
            rx_time: 0,
            rx_snr: 1.0,
            hop_limit: 0,
            want_ack: false,
            priority: 0,
            rx_rssi: 1,
            delayed: 0,
            via_mqtt: false,
            hop_start: 0,
            public_key: vec![],
            pki_encrypted: true,
            next_hop: 0,
            relay_node: 0,
            tx_after: 0,
            transport_mechanism: 0,
            payload_variant: None,
        };

        tx_packet(
            &mut self.spi,
            &mut self.nss,
            packet.encode_to_vec().as_slice(),
        )
        .await;
    }

    pub fn start_rx(&mut self) {
        spi_write(&mut self.spi, &mut self.nss, REG_FIFO_ADDR_PTR, 0);
        spi_write(&mut self.spi, &mut self.nss, REG_IRQ_FLAGS_MASK, 0);
        spi_write(
            &mut self.spi,
            &mut self.nss,
            REG_OP_MODE,
            MODE_LONG_RANGE | 0x05,
        );
        info!("LoRa RX mode started");
    }

    pub fn check_rx(&mut self) -> Option<(usize, u8)> {
        let irq_flags = spi_read(&mut self.spi, &mut self.nss, REG_IRQ_FLAGS);

        if irq_flags & REG_IRQ_FLAGS_RX_DONE != 0 {
            spi_write(&mut self.spi, &mut self.nss, REG_IRQ_FLAGS, 0xFF);

            let rx_nb_bytes = spi_read(&mut self.spi, &mut self.nss, REG_RX_NB_BYTES);
            let fifo_rx_addr = spi_read(&mut self.spi, &mut self.nss, REG_FIFO_RX_CURRENT);

            spi_write(
                &mut self.spi,
                &mut self.nss,
                REG_FIFO_ADDR_PTR,
                fifo_rx_addr,
            );

            let mut packet = [0u8; 256];
            self.nss.set_low();
            self.spi.blocking_write(&[0x00 | 0x80]).ok();
            let bytes_read = rx_nb_bytes as usize;
            self.spi.blocking_read(&mut packet[..bytes_read]).ok();
            self.nss.set_high();

            info!("RX: {} bytes", bytes_read);
            for i in 0..bytes_read {
                if i < 32 {
                    info!("  [{:02}] = 0x{:02x}", i, packet[i]);
                }
            }

            spi_write(
                &mut self.spi,
                &mut self.nss,
                REG_OP_MODE,
                MODE_LONG_RANGE | MODE_STDBY,
            );

            Some((bytes_read, packet[0]))
        } else {
            None
        }
    }

    pub fn receive(&mut self, buffer: &mut [u8]) -> Option<usize> {
        let irq_flags = spi_read(&mut self.spi, &mut self.nss, REG_IRQ_FLAGS);

        if irq_flags & REG_IRQ_FLAGS_RX_DONE != 0 {
            spi_write(&mut self.spi, &mut self.nss, REG_IRQ_FLAGS, 0xFF);

            let rx_nb_bytes = spi_read(&mut self.spi, &mut self.nss, REG_RX_NB_BYTES);
            let fifo_rx_addr = spi_read(&mut self.spi, &mut self.nss, REG_FIFO_RX_CURRENT);

            spi_write(
                &mut self.spi,
                &mut self.nss,
                REG_FIFO_ADDR_PTR,
                fifo_rx_addr,
            );

            self.nss.set_low();
            self.spi.blocking_write(&[0x00 | 0x80]).ok();
            let len = rx_nb_bytes as usize;
            if len > buffer.len() {
                self.nss.set_high();
                return None;
            }
            self.spi.blocking_read(&mut buffer[..len]).ok();
            self.nss.set_high();

            info!("Received {} bytes", len);

            spi_write(
                &mut self.spi,
                &mut self.nss,
                REG_OP_MODE,
                MODE_LONG_RANGE | MODE_STDBY,
            );

            Some(len)
        } else {
            None
        }
    }
}

fn delay(delay: u64) {
    embassy_time::block_for(embassy_time::Duration::from_micros(delay));
}

async fn tx_packet<'d, T: Instance>(spi: &mut Spim<'d, T>, nss: &mut Output<'d>, data: &[u8]) {
    spi_write(spi, nss, REG_FIFO_ADDR_PTR, 0);

    nss.set_low();
    spi.blocking_write(&[REG_FIFO_ADDR_PTR as u8 | 0x80]).ok();
    spi.blocking_write(data).ok();
    nss.set_high();

    spi_write(spi, nss, REG_PAYLOAD_LENGTH, data.len() as u8);

    spi_write(spi, nss, REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

    wait_tx_done(spi, nss).await;
}

async fn wait_tx_done<'d, T: Instance>(spi: &mut Spim<'d, T>, nss: &mut Output<'d>) {
    let mut retries = 0;
    loop {
        let irq_flags = spi_read(spi, nss, REG_IRQ_FLAGS);
        if irq_flags & IRQ_TX_DONE != 0 {
            info!(
                "done transmiting {:#010b} in {} retries",
                irq_flags, retries
            );
            spi_write(spi, nss, REG_IRQ_FLAGS, IRQ_TX_DONE);
            break;
        } else {
            //device busy wait
            Timer::after(Duration::from_millis(500)).await;
            // delay(100_000);
            retries += 1;
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
