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
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::{self, Twim};
use embassy_time::{Duration, Timer};
use nrf_softdevice::ble::Connection;
use nrf_softdevice::ble::Uuid;
use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::ble::central;
use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
use nrf_softdevice::ble::gatt_server::{self, RegisterError, WriteOp};
use nrf_softdevice::ble::peripheral;
use nrf_softdevice::{Softdevice, raw};

use bee::aht20::Aht20;

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

static mut CONNECTION: Option<Connection> = None;
static DISCONNECTED: AtomicBool = AtomicBool::new(true);

struct SensorReadings {
    temperature: i16,
    humidity: u16,
}

impl SensorReadings {
    const fn new() -> Self {
        Self {
            temperature: 0,
            humidity: 0,
        }
    }
}

struct SharedReadings {
    inner: UnsafeCell<SensorReadings>,
    notify_temp: UnsafeCell<bool>,
    notify_humidity: UnsafeCell<bool>,
}

impl SharedReadings {
    const fn new() -> Self {
        Self {
            inner: UnsafeCell::new(SensorReadings::new()),
            notify_temp: UnsafeCell::new(false),
            notify_humidity: UnsafeCell::new(false),
        }
    }

    fn set_notify_temp(&self, enabled: bool) {
        unsafe {
            *self.notify_temp.get() = enabled;
        }
    }

    fn set_notify_humidity(&self, enabled: bool) {
        unsafe {
            *self.notify_humidity.get() = enabled;
        }
    }

    fn get_notify_temp(&self) -> bool {
        unsafe { *self.notify_temp.get() }
    }

    fn get_notify_humidity(&self) -> bool {
        unsafe { *self.notify_humidity.get() }
    }
}

static READINGS: SharedReadings = SharedReadings::new();

unsafe impl Sync for SharedReadings {}

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
            if data.len() >= 2 && (data[1] & 0x01) != 0 {
                READINGS.set_notify_temp(true);
            }
        } else if handle == self.sensor.humidity_cccd_handle {
            info!("Humidity CCCD written: {:?}", data);
            if data.len() >= 2 && (data[1] & 0x01) != 0 {
                READINGS.set_notify_humidity(true);
            }
        }
        None
    }
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[allow(static_mut_refs)]
#[embassy_executor::task]
async fn gatt_server_run(sd: &'static Softdevice, temp_handle: u16, humidity_handle: u16) {
    let mut last_temp: i16 = 0;
    let mut last_humidity: u16 = 0;

    loop {
        Timer::after(Duration::from_secs(2)).await;

        if DISCONNECTED.load(Ordering::Relaxed) {
            break;
        }

        let readings = unsafe { &*READINGS.inner.get() };
        let notify_temp = READINGS.get_notify_temp();
        let notify_humidity = READINGS.get_notify_humidity();

        if let Some(conn) = unsafe { CONNECTION.as_ref() } {
            if readings.temperature != last_temp {
                let _ =
                    gatt_server::set_value(sd, temp_handle, &readings.temperature.to_le_bytes());
                if notify_temp {
                    let _ = gatt_server::notify_value(
                        conn,
                        temp_handle,
                        &readings.temperature.to_le_bytes(),
                    );
                }
                last_temp = readings.temperature;
            }

            if readings.humidity != last_humidity {
                let _ =
                    gatt_server::set_value(sd, humidity_handle, &readings.humidity.to_le_bytes());
                if notify_humidity {
                    let _ = gatt_server::notify_value(
                        conn,
                        humidity_handle,
                        &readings.humidity.to_le_bytes(),
                    );
                }
                last_humidity = readings.humidity;
            }
        }
    }
}

#[embassy_executor::task]
async fn sensor_reading_task(
    twim: TWISPI0,
    irqs: Irqs,
    scl: embassy_nrf::peripherals::P0_06,
    sda: embassy_nrf::peripherals::P0_08,
    readings: &'static SharedReadings,
) {
    let twi_config = twim::Config::default();
    let twi = Twim::new(twim, irqs, sda, scl, twi_config);

    let mut aht20 = Aht20::new(twi);
    if let Err(e) = aht20.init() {
        warn!("AHT20 init failed: {:?}", e);
    }

    loop {
        Timer::after(Duration::from_secs(10)).await;

        if let Ok(reading) = aht20.read() {
            let temp = (reading.temperature * 100.0) as i16;
            let humidity = (reading.humidity * 100.0) as u16;

            unsafe {
                let r = &mut *readings.inner.get();
                r.temperature = temp;
                r.humidity = humidity;
            }

            info!(
                "Sensor reading: {}°C, {}%",
                reading.temperature, reading.humidity
            );
        }
    }
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

#[allow(clippy::too_many_arguments)]
async fn run_sensor(
    sd: &'static Softdevice,
    device_id: u8,
    twim: TWISPI0,
    irqs: Irqs,
    scl: embassy_nrf::peripherals::P0_06,
    sda: embassy_nrf::peripherals::P0_08,
    server: Server,
    spawner: Spawner,
) {
    info!("Starting as SENSOR, device_id={}", device_id);

    if spawner
        .spawn(sensor_reading_task(twim, irqs, scl, sda, &READINGS))
        .is_err()
    {
        warn!("Failed to spawn sensor reading task");
    }

    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .full_name("TSensor")
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(ServiceList::Complete, &[SENSOR_SERVICE_UUID])
        .build();

    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new().build();

    let temp_handle = server.sensor.temp_handle;
    let humidity_handle = server.sensor.humidity_handle;

    loop {
        info!("Advertising as sensor...");
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };

        match peripheral::advertise_connectable(sd, adv, &config).await {
            Ok(conn) => {
                info!("Master connected!");
                DISCONNECTED.store(false, Ordering::Relaxed);

                let conn_ref = &conn;
                unsafe { CONNECTION = Some(core::ptr::read(conn_ref)) };
                spawner
                    .spawn(gatt_server_run(sd, temp_handle, humidity_handle))
                    .ok();
                gatt_server::run(&conn, &server, |_| {}).await;
                unsafe { CONNECTION = None };
                DISCONNECTED.store(true, Ordering::Relaxed);

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
        Timer::after(Duration::from_secs(interval_secs as u64)).await;
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

    #[allow(clippy::needless_borrow)]
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
            spawner,
        )
        .await;
    }
}
