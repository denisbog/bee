use defmt::info;
use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
use nrf_softdevice::ble::gatt_server::{CharacteristicHandles, RegisterError};
use nrf_softdevice::ble::{Connection, Uuid};
use nrf_softdevice::Softdevice;

use crate::aht20::Aht20Reading;
use crate::constants::{DEVICE_ID_CHAR_UUID, SENSOR_SERVICE_UUID, TEMP_CHAR_UUID};

pub struct SensorService {
    temp_handle: CharacteristicHandles,
    device_id_handle: CharacteristicHandles,
    current_reading: Option<Aht20Reading>,
    pub device_id: u8,
}

impl SensorService {
    pub fn new(sd: &mut Softdevice, device_id: u8) -> Result<Self, RegisterError> {
        let mut sb = ServiceBuilder::new(sd, SENSOR_SERVICE_UUID)?;

        let temp_attr = Attribute::new(&[0u8; 4]);
        let temp_md = Metadata::new(Properties::new().read().notify());
        let temp_handle = sb
            .add_characteristic(TEMP_CHAR_UUID, temp_attr, temp_md)?
            .build();

        let device_id_bytes = [device_id];
        let device_id_attr = Attribute::new(&device_id_bytes);
        let device_id_md = Metadata::new(Properties::new().read().write());
        let device_id_handle = sb
            .add_characteristic(DEVICE_ID_CHAR_UUID, device_id_attr, device_id_md)?
            .build();

        let _service_handle = sb.build();

        Ok(Self {
            temp_handle,
            device_id_handle,
            current_reading: None,
            device_id,
        })
    }

    pub fn update_reading(&self, reading: &Aht20Reading, conn: &Connection, sd: &Softdevice) {
        let temp_bytes = reading.temperature.to_be_bytes();
        let _ = nrf_softdevice::ble::gatt_server::notify_value(
            conn,
            self.temp_handle.value_handle,
            &temp_bytes,
        );
        info!("Notified temperature: {}C", reading.temperature);
    }

    pub fn get_reading(&self) -> Option<Aht20Reading> {
        self.current_reading
    }

    pub fn set_reading(&mut self, reading: Aht20Reading) {
        self.current_reading = Some(reading);
    }

    pub fn set_device_id(&mut self, device_id: u8, sd: &Softdevice, conn: &Connection) {
        self.device_id = device_id;
        let _ = nrf_softdevice::ble::gatt_server::set_value(
            sd,
            self.device_id_handle.value_handle,
            &[device_id],
        );
    }
}
