#![no_std]
#![no_main]
#![macro_use]

use core::mem;
use defmt_rtt as _;
use embassy_nrf as _;
use panic_probe as _;

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::config::Config;
use embassy_nrf::gpio::Pin;
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::{self, Twim};
use embassy_time::Duration;
use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::ble::central;
use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
use nrf_softdevice::ble::gatt_server::{self, RegisterError, WriteOp};
use nrf_softdevice::ble::peripheral;
use nrf_softdevice::ble::{Connection, Uuid};
use nrf_softdevice::{Softdevice, raw};

use bee::aht20::Aht20;

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<TWISPI0>;
});

const MAX_SENSORS: usize = 4;

const SENSOR_SERVICE_UUID: ServiceUuid16 = ServiceUuid16::BATTERY;
const MASTER_SERVICE_UUID: u16 = 0x1852;

const TEMP_UUID: u16 = 0x2A1C;
const HUMIDITY_UUID: u16 = 0x2A1F;

struct SensorService {
    temp_handle: u16,
    temp_cccd_handle: u16,
    humidity_handle: u16,
    humidity_cccd_handle: u16,
}

impl SensorService {
    fn register(sd: &mut Softdevice) -> Result<Self, RegisterError> {
        let mut svc = ServiceBuilder::new(sd, Uuid::new_16(SENSOR_SERVICE_UUID.to_u16()))?;

        let temp_char = svc.add_characteristic(
            Uuid::new_16(TEMP_UUID),
            Attribute::new(&[0u8, 0u8]),
            Metadata::new(Properties::new().read().notify()),
        )?;
        let temp_handles = temp_char.build();
        let temp_handle = temp_handles.value_handle;
        let temp_cccd_handle = temp_handles.cccd_handle;

        let humidity_char = svc.add_characteristic(
            Uuid::new_16(HUMIDITY_UUID),
            Attribute::new(&[0u8, 0u8]),
            Metadata::new(Properties::new().read().notify()),
        )?;
        let humidity_handles = humidity_char.build();
        let humidity_handle = humidity_handles.value_handle;
        let humidity_cccd_handle = humidity_handles.cccd_handle;

        svc.build();

        Ok(Self {
            temp_handle,
            temp_cccd_handle,
            humidity_handle,
            humidity_cccd_handle,
        })
    }
}

struct Server {
    sensor: SensorService,
}

impl Server {
    fn new(sd: &mut Softdevice) -> Result<Self, RegisterError> {
        let sensor = SensorService::register(sd)?;
        Ok(Self { sensor })
    }
}

impl gatt_server::Server for Server {
    type Event = ();

    fn on_write(
        &self,
        _conn: &Connection,
        handle: u16,
        _op: WriteOp,
        _offset: usize,
        data: &[u8],
    ) -> Option<Self::Event> {
        if handle == self.sensor.temp_cccd_handle {
            info!("Temp CCCD written: {:?}", data);
        } else if handle == self.sensor.humidity_cccd_handle {
            info!("Humidity CCCD written: {:?}", data);
        }
        None
    }
}

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
            conn_count: (MAX_SENSORS + 1) as u8,
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

async fn run_sensor<SCL: Pin, SDA: Pin>(
    sd: &'static Softdevice,
    device_id: u8,
    twim: TWISPI0,
    irqs: Irqs,
    scl: SCL,
    sda: SDA,
    server: Server,
) {
    info!("Starting as SENSOR, device_id={}", device_id);

    let twi_config = twim::Config::default();
    let twi = Twim::new(twim, irqs, sda, scl, twi_config);

    let mut aht20 = Aht20::new(twi);
    if let Err(e) = aht20.init() {
        warn!("AHT20 init failed: {:?}", e);
    }

    let mut current_temp: i16 = 0;
    let mut current_humidity: u16 = 0;

    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .full_name("TSensor")
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(ServiceList::Complete, &[SENSOR_SERVICE_UUID])
        .build();

    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new().build();

    let mut last_reading = embassy_time::Instant::now();

    loop {
        if last_reading.elapsed() > Duration::from_secs(10) {
            if let Ok(reading) = aht20.read() {
                current_temp = (reading.temperature * 100.0) as i16;
                current_humidity = (reading.humidity * 100.0) as u16;
                info!(
                    "Sensor {}: {}°C, {}%",
                    device_id, reading.temperature, reading.humidity
                );
            }
            last_reading = embassy_time::Instant::now();
        }

        info!("Advertising as sensor...");
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };

        match peripheral::advertise_connectable(&sd, adv, &config).await {
            Ok(conn) => {
                info!("Master connected!");

                let _ = gatt_server::set_value(
                    &sd,
                    server.sensor.temp_handle,
                    &current_temp.to_le_bytes(),
                );
                let _ = gatt_server::set_value(
                    &sd,
                    server.sensor.humidity_handle,
                    &current_humidity.to_le_bytes(),
                );

                gatt_server::run(&conn, &server, |_| {}).await;

                info!("Master disconnected");
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
        info!("Advertising as master...");

        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };

        match peripheral::advertise_connectable(sd, adv, &config).await {
            Ok(_conn) => {
                info!("Sensor connected!");
            }
            Err(e) => {
                warn!("Advertising failed: {:?}", e);
            }
        }

        info!("Scanning for sensors...");

        let scan_config = central::ScanConfig::default();
        let mut found_count = 0;

        let _ = central::scan(sd, &scan_config, |_report| {
            found_count += 1;
            info!("Found sensor #{}", found_count);
            Some(())
        })
        .await;

        info!(
            "Scan complete. Found {} sensors. Waiting {}s...",
            found_count, interval_secs
        );
        embassy_time::block_for(Duration::from_secs(interval_secs as u64));
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Bee Mesh starting...");

    let mut config = Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let config = create_sd_config();
    let mut sd = Softdevice::enable(&config);

    let device_config = bee::config::ConfigStorage::load();
    info!(
        "Loaded config: role={}, device_id={}, interval={}",
        device_config.role, device_config.device_id, device_config.interval_secs
    );

    let server = if !device_config.is_master() {
        match Server::new(&mut sd) {
            Ok(s) => Some(s),
            Err(e) => {
                error!("Failed to register GATT server: {:?}", e);
                None
            }
        }
    } else {
        None
    };

    let sd: &'static Softdevice = unsafe { &*(core::ptr::addr_of!(sd) as *const Softdevice) };

    if spawner.spawn(softdevice_task(sd)).is_err() {
        warn!("Failed to spawn softdevice task");
    }

    if device_config.is_master() {
        run_master(sd, device_config.interval_secs).await;
    } else if let Some(server) = server {
        run_sensor(
            sd,
            device_config.device_id,
            p.TWISPI0,
            Irqs,
            p.P0_06,
            p.P0_08,
            server,
        )
        .await;
    }
}
