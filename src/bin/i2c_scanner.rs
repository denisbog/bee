#![no_std]
#![no_main]
#![macro_use]

use defmt_rtt as _;
use embassy_nrf as _;
use nrf_softdevice as _;
use panic_probe as _;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::config::Config;
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::{self, Twim};

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<TWISPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("I2C Scanner starting...");

    let mut config = Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let mut twi_config = twim::Config::default();
    twi_config.scl_pullup = true;
    twi_config.sda_pullup = true;

    let mut twi = Twim::new(p.TWISPI0, Irqs, p.P0_08, p.P0_06, twi_config);

    info!("Scanning I2C bus...");

    let mut found_devices = false;

    for addr in 0..=127 {
        let result = twi.blocking_write_read(addr, &[0x00], &mut [0u8; 1]);

        match result {
            Ok(_) => {
                info!("Found device at address 0x{:02X} ({})", addr, addr);
                found_devices = true;
            }
            Err(_) => {
                // No device at this address
            }
        }
    }

    if !found_devices {
        warn!("No I2C devices found!");
    }

    info!("Scan complete.");

    loop {
        embassy_time::block_for(embassy_time::Duration::from_secs(1));
    }
}
