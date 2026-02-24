#![no_std]
#![no_main]
#![macro_use]

use core::mem;
use defmt_rtt as _;
use embassy_nrf as _;
use panic_probe as _;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::gpio::Pin;
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::{self, Twim};
use embassy_time::Duration;
use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::ble::peripheral;
use nrf_softdevice::{Softdevice, raw};

use bee::aht20::Aht20;

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<TWISPI0>;
});

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

fn create_sd_config() -> nrf_softdevice::Config {
    nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 1,
            central_role_count: 1,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"BeeMesh" as *const u8 as _,
            current_len: 7,
            max_len: 7,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    }
}

const SENSOR_SERVICE_UUID: u16 = 0x1850;
const MASTER_SERVICE_UUID: u16 = 0x1852;

async fn run_sensor<SCL: Pin, SDA: Pin>(
    sd: &'static Softdevice,
    device_id: u8,
    twim: TWISPI0,
    irqs: Irqs,
    scl: SCL,
    sda: SDA,
) {
    info!("Starting as SENSOR, device_id={}", device_id);

    let config = twim::Config::default();
    let twi = Twim::new(twim, irqs, scl, sda, config);

    let mut aht20 = Aht20::new(twi);
    if let Err(e) = aht20.init() {
        warn!("AHT20 init failed: {:?}", e);
    }

    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(
            ServiceList::Complete,
            &[ServiceUuid16::from_u16(SENSOR_SERVICE_UUID)],
        )
        .build();

    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new().build();

    loop {
        info!("Advertising as sensor...");

        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };

        match peripheral::advertise_connectable(sd, adv, &config).await {
            Ok(_conn) => {
                info!("Client connected!");
            }
            Err(e) => {
                warn!("Advertising failed: {:?}", e);
            }
        }
    }
}

async fn run_master(sd: &'static Softdevice, interval_secs: u32) {
    info!("Starting as MASTER");

    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(
            ServiceList::Complete,
            &[ServiceUuid16::from_u16(MASTER_SERVICE_UUID)],
        )
        .full_name("BeeMaster")
        .build();

    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new().build();

    loop {
        info!("Master advertising, waiting for client...");

        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };

        if let Ok(_conn) = peripheral::advertise_connectable(sd, adv, &config).await {
            info!("Master client connected");
        }

        info!("Sleeping for {} seconds...", interval_secs);
        embassy_time::block_for(Duration::from_secs(interval_secs as u64));
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Bee Mesh starting...");

    let p = embassy_nrf::init(Default::default());

    let config = create_sd_config();
    let sd = Softdevice::enable(&config);
    let sd: &'static Softdevice = unsafe { &*(core::ptr::addr_of!(sd) as *const Softdevice) };

    let device_config = bee::config::ConfigStorage::load();
    info!(
        "Loaded config: role={}, device_id={}, interval={}",
        device_config.role, device_config.device_id, device_config.interval_secs
    );

    if spawner.spawn(softdevice_task(sd)).is_err() {
        warn!("Failed to spawn softdevice task");
    }

    if device_config.is_master() {
        run_master(sd, device_config.interval_secs).await;
    } else {
        run_sensor(
            sd,
            device_config.device_id,
            p.TWISPI0,
            Irqs,
            p.P0_26,
            p.P0_27,
        )
        .await;
    }
}
