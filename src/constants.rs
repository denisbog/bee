use nrf_softdevice::ble::Uuid;

pub const ROLE_SENSOR: u8 = 0;
pub const ROLE_MASTER: u8 = 1;

pub const DEFAULT_ROLE: u8 = ROLE_SENSOR;
pub const DEFAULT_DEVICE_ID: u8 = 0;
pub const DEFAULT_INTERVAL_SECS: u32 = 300;

pub const SENSOR_SERVICE_UUID: Uuid = Uuid::new_16(0x1850);
pub const CONFIG_SERVICE_UUID: Uuid = Uuid::new_16(0x1851);
pub const MASTER_DATA_SERVICE_UUID: Uuid = Uuid::new_16(0x1852);

pub const TEMP_CHAR_UUID: Uuid = Uuid::new_16(0x2A1F);
pub const DEVICE_ID_CHAR_UUID: Uuid = Uuid::new_16(0x2A20);
pub const ROLE_CHAR_UUID: Uuid = Uuid::new_16(0x2A21);
pub const INTERVAL_CHAR_UUID: Uuid = Uuid::new_16(0x2A22);

pub const SENSOR_ADV_NAME_PREFIX: &str = "BeeSensor";
