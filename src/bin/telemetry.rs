//! This example showcases how to notify a connected client via BLE of new SAADC data.
//! Using, for example, nRF-Connect on iOS/Android we can connect to the device "HelloRust"
//! and see the battery level characteristic getting updated in real-time.
//!
//! The SAADC is initialized in single-ended mode and a single measurement is taken every second.
//! This value is then used to update the battery_level characteristic.
//! We are using embassy-time for time-keeping purposes.
//! Everytime a new value is recorded, it gets sent to the connected clients via a GATT Notification.
//!
//! The ADC doesn't gather data unless a valid connection exists with a client. This is guaranteed
//! by using the "select" crate to wait for either the `gatt_server::run` future or the `adc_fut` future
//! to complete.
//!
//! Only a single BLE connection is supported in this example so that RAM usage remains minimal.
//!
//! The internal RC oscillator is used to generate the LFCLK.
//!

#![no_std]
#![no_main]

use bee::aht20::Aht20;
use core::mem;
use defmt_rtt as _;
use embassy_nrf::gpio::AnyPin;
use embassy_nrf::twim::Twim;
use embassy_nrf::{self as _, twim};
use panic_probe as _; // time driver // global logger

use defmt::{info, *};
use embassy_executor::Spawner;
use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::peripherals::SAADC;
use embassy_nrf::saadc::{AnyInput, Input, Saadc};
use embassy_nrf::{bind_interrupts, interrupt, saadc};
use embassy_time::{Duration, Timer};
use futures::future::{Either, select};
use futures::pin_mut;
use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::ble::{Connection, gatt_server, peripheral};
use nrf_softdevice::{Softdevice, raw};

use embassy_nrf::peripherals::TWISPI0;

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    TWISPI0 => twim::InterruptHandler<TWISPI0>;
});

/// Initializes the SAADC peripheral in single-ended mode on the given pin.
fn init_adc(adc_pin: AnyInput, adc: SAADC) -> Saadc<'static, 1> {
    // Then we initialize the ADC. We are only using one channel in this example.
    let config = saadc::Config::default();
    let channel_cfg = saadc::ChannelConfig::single_ended(adc_pin.degrade_saadc());
    interrupt::SAADC.set_priority(interrupt::Priority::P3);
    let saadc = saadc::Saadc::new(adc, Irqs, config, [channel_cfg]);
    saadc
}
fn init_aht(scl: AnyPin, sda: AnyPin, twispi0: TWISPI0) -> Aht20<'static, TWISPI0> {
    let twi_config = twim::Config::default();
    interrupt::TWISPI0.set_priority(interrupt::Priority::P3);
    let twi = Twim::new(twispi0, Irqs, sda, scl, twi_config);

    let mut aht20 = Aht20::new(twi);
    if let Err(e) = aht20.init() {
        warn!("AHT20 init failed: {:?}", e);
    }

    aht20
}

/// Reads the current ADC value every second and notifies the connected client.
async fn notify_adc_value<'a>(
    saadc: &'a mut Saadc<'_, 1>,
    server: &'a Server,
    connection: &'a Connection,
) {
    loop {
        let mut buf = [0i16; 1];
        saadc.sample(&mut buf).await;

        // We only sampled one ADC channel.
        let adc_raw_value: i16 = buf[0];

        // Try and notify the connected client of the new ADC value.
        match server.ts.battery_level_notify(connection, &adc_raw_value) {
            Ok(_) => info!("Battery adc_raw_value: {=i16}", &adc_raw_value),
            Err(_) => unwrap!(server.ts.battery_level_set(&adc_raw_value)),
        };

        // Sleep for one second.
        Timer::after(Duration::from_secs(1)).await
    }
}

async fn notify_temperature_value<'a>(
    aht20: &'a mut Aht20<'static, TWISPI0>,
    server: &'a Server,
    connection: &'a Connection,
) {
    loop {
        if let Ok(reading) = aht20.read() {
            let temp = (reading.temperature * 100.0) as i16;
            let humidity = (reading.humidity * 100.0) as u16;

            info!(
                "Sensor reading: {}°C, {}%",
                reading.temperature, reading.humidity
            );
            // Try and notify the connected client of the new ADC value.
            match server.ts.temperature_notify(connection, &temp) {
                Ok(_) => info!("Temperature: {=i16}", &temp),
                Err(_) => unwrap!(server.ts.temperature_set(&temp)),
            };
            match server.ts.humidity_notify(connection, &humidity) {
                Ok(_) => info!("Humidity: {=u16}", &humidity),
                Err(_) => unwrap!(server.ts.humidity_set(&humidity)),
            };
        }

        // Sleep for one second.
        Timer::after(Duration::from_secs(1)).await
    }
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[nrf_softdevice::gatt_service(uuid = "1819")]
struct TelemetryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: i16,
    #[characteristic(uuid = "2a6e", read, notify)]
    temperature: i16,
    #[characteristic(uuid = "2a6f", read, notify)]
    humidity: u16,
}

#[nrf_softdevice::gatt_server]
struct Server {
    ts: TelemetryService,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Telemetry Service!");

    // First we get the peripherals access crate.
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = interrupt::Priority::P2;
    config.time_interrupt_priority = interrupt::Priority::P2;
    let p = embassy_nrf::init(config);

    // Then we initialize the ADC. We are only using one channel in this example.
    let adc_pin = p.P0_29.degrade_saadc();

    let scl = p.P0_06;
    let sda = p.P0_08;

    let mut saadc = init_adc(adc_pin, p.SAADC);
    let mut aht20 = init_aht(scl.into(), sda.into(), p.TWISPI0);
    // Indicated: wait for ADC calibration.
    saadc.calibrate().await;

    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 1,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: raw::BLE_GAP_ADV_SET_COUNT_DEFAULT as u8,
            periph_role_count: raw::BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT as u8,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"TelemetryService" as *const u8 as _,
            current_len: 16,
            max_len: 16,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&config);
    let server = unwrap!(Server::new(sd));

    unwrap!(spawner.spawn(softdevice_task(sd)));

    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
        .full_name("Telemetry Service")
        .build();

    static SCAN_DATA: [u8; 0] = [];

    loop {
        let config = peripheral::Config::default();

        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };
        let conn = unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);
        info!("advertising done! I have a connection.");

        // We have a GATT connection. Now we will create two futures:
        //  - An infinite loop gathering data from the ADC and notifying the clients.
        //  - A GATT server listening for events from the connected client.
        //
        // Event enums (ServerEvent's) are generated by nrf_softdevice::gatt_server
        // proc macro when applied to the Server struct above
        // let adc_fut = notify_adc_value(&mut saadc, &server, &conn);

        let adc_fut = notify_adc_value(&mut saadc, &server, &conn);
        let aht20_fut = notify_temperature_value(&mut aht20, &server, &conn);

        let gatt_fut = gatt_server::run(&conn, &server, |e| match e {
            ServerEvent::Ts(e) => match e {
                TelemetryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    info!("battery notifications: {}", notifications)
                }
                TelemetryServiceEvent::TemperatureCccdWrite { notifications } => {
                    info!("temperature notifications: {}", notifications)
                }
                TelemetryServiceEvent::HumidityCccdWrite { notifications } => {
                    info!("humidity notifications: {}", notifications)
                }
            },
        });

        pin_mut!(adc_fut);
        pin_mut!(aht20_fut);
        pin_mut!(gatt_fut);
        // We are using "select" to wait for either one of the futures to complete.
        // There are some advantages to this approach:
        //  - we only gather data when a client is connected, therefore saving some power.
        //  - when the GATT server finishes operating, our ADC future is also automatically aborted.

        let _result = match select(select(adc_fut, aht20_fut), gatt_fut).await {
            Either::Left((Either::Left((r, _)), _)) => {
                info!("ADC encountered an error and stopped! {:?}", r)
            }
            Either::Left((Either::Right((r, _)), _)) => {
                info!("Data service encountered an error and stopped! {:?}", r)
            }
            Either::Right((e, _)) => info!("gatt_server run exited with error: {:?}", e),
        };
    }
}
