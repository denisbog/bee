use core::mem;
use defmt::info;

const ROLE_SENSOR: u8 = 0;
const ROLE_MASTER: u8 = 1;
const DEFAULT_ROLE: u8 = ROLE_SENSOR;
const DEFAULT_DEVICE_ID: u8 = 0;
const DEFAULT_INTERVAL_SECS: u32 = 300;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct DeviceConfig {
    pub role: u8,
    pub device_id: u8,
    pub interval_secs: u32,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            role: DEFAULT_ROLE,
            device_id: DEFAULT_DEVICE_ID,
            interval_secs: DEFAULT_INTERVAL_SECS,
        }
    }
}

impl DeviceConfig {
    pub fn is_sensor(&self) -> bool {
        self.role == ROLE_SENSOR
    }

    pub fn is_master(&self) -> bool {
        self.role == ROLE_MASTER
    }
}

const CONFIG_MAGIC: u32 = 0xBEE1BEE1;
const CONFIG_FLASH_OFFSET: usize = 0x1F000;

#[repr(C, packed)]
struct FlashConfig {
    magic: u32,
    role: u8,
    device_id: u8,
    interval_secs: u32,
    _padding: [u8; 2],
}

impl FlashConfig {
    fn from_config(config: &DeviceConfig) -> Self {
        Self {
            magic: CONFIG_MAGIC,
            role: config.role,
            device_id: config.device_id,
            interval_secs: config.interval_secs,
            _padding: [0; 2],
        }
    }

    fn to_config(&self) -> Option<DeviceConfig> {
        if self.magic == CONFIG_MAGIC {
            Some(DeviceConfig {
                role: self.role,
                device_id: self.device_id,
                interval_secs: self.interval_secs,
            })
        } else {
            None
        }
    }
}

const _: () = assert!(mem::size_of::<FlashConfig>() == 12);

pub struct ConfigStorage;

impl ConfigStorage {
    pub fn load() -> DeviceConfig {
        let flash_ptr = CONFIG_FLASH_OFFSET as *const FlashConfig;
        let flash_config = unsafe { &*flash_ptr };

        if let Some(config) = flash_config.to_config() {
            info!(
                "Loaded config from flash: role={}, device_id={}, interval={}",
                config.role, config.device_id, config.interval_secs
            );
            config
        } else {
            info!("No valid config in flash, using defaults");
            DeviceConfig::default()
        }
    }

    pub fn save(config: &DeviceConfig) {
        let _flash_config = FlashConfig::from_config(config);

        let flash_ptr = CONFIG_FLASH_OFFSET as *mut FlashConfig;
        unsafe {
            let ptr = flash_ptr as *mut u32;
            ptr.write_volatile(CONFIG_MAGIC);

            let ptr = (flash_ptr as *mut u8).add(4);
            ptr.write_volatile(config.role);
            ptr.add(1).write_volatile(config.device_id);

            let ptr = (flash_ptr as *mut u32).add(2);
            ptr.write_volatile(config.interval_secs);
        }

        info!(
            "Saved config to flash: role={}, device_id={}, interval={}",
            config.role, config.device_id, config.interval_secs
        );
    }
}
