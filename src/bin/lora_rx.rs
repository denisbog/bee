#![no_std]
#![no_main]
#![macro_use]

use bee::lora;
use defmt_rtt as _;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::{self as _, interrupt, spim};
use nrf_softdevice as _;
use panic_probe as _;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::config::Config;
use embassy_nrf::peripherals::SPI3;
use embassy_time::{Duration, Timer};

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<SPI3>;
});

fn init_lora<'a>(
    nss: Output<'a>,
    reset: Output<'a>,
    sck: AnyPin,
    miso: AnyPin,
    mosi: AnyPin,
    spi3: SPI3,
) -> lora::LoraRadio<'a, SPI3> {
    let config = spim::Config::default();
    interrupt::SPIM3.set_priority(interrupt::Priority::P3);

    let spim = spim::Spim::new(spi3, Irqs, sck, miso, mosi, config);

    let mut lora = lora::LoraRadio::new(nss, reset, spim).unwrap();
    if let Err(e) = lora.init() {
        defmt::warn!("lora init failed: {:?}", e);
    } else {
        lora.setup();
    }
    lora
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    bee::init_heap();
    info!("Bee Mesh RX starting...");

    let mut config = Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let spi_nss = Output::new(p.P1_04, Level::High, OutputDrive::Standard);
    let spi_reset = Output::new(p.P1_06, Level::High, OutputDrive::Standard);
    let spi_sck = p.P0_20;
    let spi_miso = p.P0_22;
    let spi_mosi = p.P0_24;

    let mut lora = init_lora(
        spi_nss,
        spi_reset,
        spi_sck.into(),
        spi_miso.into(),
        spi_mosi.into(),
        p.SPI3,
    );

    lora.start_rx();
    info!("Listening for incoming packets...");

    let mut buffer = [0u8; 256];
    let mut packet_count: u32 = 0;

    loop {
        if let Some(len) = lora.receive(&mut buffer) {
            packet_count += 1;
            info!("=== Packet #{} ({} bytes) ===", packet_count, len);

            info!("Hex: ");
            // for (i, &byte) in buffer[..len].iter().enumerate() {
            //     if i > 0 && i % 16 == 0 {
            //         defmt::info!("");
            //     }
            //     defmt::info!("{:02x}", byte);
            // }
            // defmt::info!("");

            if let Ok(packet) = lora::meshtastic::parse(&buffer[..len]) {
                info!(
                    "Meshtastic: from=0x{:08x}, port={}",
                    packet.from, packet.channel
                );
            } else {
                warn!("failed to decode message")
            }

            lora.start_rx();
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}
