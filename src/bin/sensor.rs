#![no_std]
#![no_main]
#![macro_use]

use defmt_rtt as _;
use embassy_nrf as _;
use embassy_nrf::interrupt::Priority;
use nrf_softdevice as _;
use panic_probe as _;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::config::Config;
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::{self, Twim};
use embassy_time::Duration;

use bee::aht20::Aht20;

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<TWISPI0>;
});

async fn run_sensor<SCL, SDA>(twim: TWISPI0, irqs: Irqs, sda: SDA, scl: SCL)
where
    SCL: embassy_nrf::gpio::Pin,
    SDA: embassy_nrf::gpio::Pin,
{
    info!("Starting as SENSOR");

    let twi_config = twim::Config::default();
    let twi = Twim::new(twim, irqs, sda, scl, twi_config);

    let mut aht20 = Aht20::new(twi);
    if let Err(e) = aht20.init() {
        warn!("AHT20 init failed: {:?}", e);
    }

    let mut last_reading = embassy_time::Instant::now();

    loop {
        if last_reading.elapsed() > Duration::from_secs(1) {
            if let Ok(reading) = aht20.read() {
                info!("Sensor: {}°C, {}%", reading.temperature, reading.humidity);
            }
            last_reading = embassy_time::Instant::now();
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Bee Mesh starting...");

    let mut config = Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    run_sensor(p.TWISPI0, Irqs, p.P0_08, p.P0_06).await;
}
