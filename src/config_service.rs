use defmt::info;
use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
use nrf_softdevice::ble::gatt_server::{CharacteristicHandles, RegisterError, WriteOp};
use nrf_softdevice::Softdevice;

use crate::config::ConfigStorage;
use crate::config::DeviceConfig;
use crate::constants::{
    CONFIG_SERVICE_UUID, DEVICE_ID_CHAR_UUID, INTERVAL_CHAR_UUID, ROLE_CHAR_UUID,
};

pub struct ConfigService {
    role_handle: CharacteristicHandles,
    device_id_handle: CharacteristicHandles,
    interval_handle: CharacteristicHandles,
    pub config: DeviceConfig,
}

impl ConfigService {
    pub fn new(sd: &mut Softdevice) -> Result<Self, RegisterError> {
        let config = ConfigStorage::load();

        let mut sb = ServiceBuilder::new(sd, CONFIG_SERVICE_UUID)?;

        let role_bytes = [config.role];
        let role_attr = Attribute::new(&role_bytes);
        let role_md = Metadata::new(Properties::new().read().write());
        let role_handle = sb
            .add_characteristic(ROLE_CHAR_UUID, role_attr, role_md)?
            .build();

        let device_id_bytes = [config.device_id];
        let device_id_attr = Attribute::new(&device_id_bytes);
        let device_id_md = Metadata::new(Properties::new().read().write());
        let device_id_handle = sb
            .add_characteristic(DEVICE_ID_CHAR_UUID, device_id_attr, device_id_md)?
            .build();

        let interval_bytes = config.interval_secs.to_le_bytes();
        let interval_attr = Attribute::new(&interval_bytes);
        let interval_md = Metadata::new(Properties::new().read().write());
        let interval_handle = sb
            .add_characteristic(INTERVAL_CHAR_UUID, interval_attr, interval_md)?
            .build();

        let _service_handle = sb.build();

        Ok(Self {
            role_handle,
            device_id_handle,
            interval_handle,
            config,
        })
    }

    pub fn apply_config(&self, sd: &Softdevice) {
        ConfigStorage::save(&self.config);
        info!(
            "Config saved: role={}, device_id={}, interval={}",
            self.config.role, self.config.device_id, self.config.interval_secs
        );
    }

    pub fn handle_write(&mut self, handle: u16, data: &[u8], sd: &Softdevice) {
        if handle == self.role_handle.value_handle && !data.is_empty() {
            self.config.role = data[0];
            info!("Role changed to: {}", self.config.role);
            self.apply_config(sd);
        } else if handle == self.device_id_handle.value_handle && !data.is_empty() {
            self.config.device_id = data[0];
            info!("Device ID changed to: {}", self.config.device_id);
            self.apply_config(sd);
        } else if handle == self.interval_handle.value_handle && data.len() >= 4 {
            let interval = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            self.config.interval_secs = interval;
            info!("Interval changed to: {}s", self.config.interval_secs);
            self.apply_config(sd);
        }
    }
}

impl nrf_softdevice::ble::gatt_server::Server for ConfigService {
    type Event = ();

    fn on_write(
        &self,
        _conn: &nrf_softdevice::ble::Connection,
        handle: u16,
        _op: WriteOp,
        _offset: usize,
        data: &[u8],
    ) -> Option<Self::Event> {
        info!("Config write: handle={}, data={:?}", handle, data);
        None
    }
}
