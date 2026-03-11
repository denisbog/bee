#![no_std]
#![no_main]
#![macro_use]

use bee::decrypt::{DEFAULT_KEY, decrypt_aes_ctr, parse_packet_header};
use bee::lora::{self, meshtastic};
use defmt_rtt as _;
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::gpiote::{InputChannel, InputChannelPolarity};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::{self as _, interrupt, spim};
use meshtastic_protobufs::{PortNum, Position, Telemetry};
use nrf_softdevice as _;
use panic_probe as _;

use defmt::{debug, info, trace, warn};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::config::Config;
use embassy_nrf::peripherals::{GPIOTE_CH0, SPI3};
use prost::Message;

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<SPI3>;
});

fn init_lora<'a>(
    nss: Output<'a>,
    reset: Output<'a>,
    sck: AnyPin,
    miso: AnyPin,
    mosi: AnyPin,
    dio0: Input<'a>,
    gpiote_ch0: GPIOTE_CH0,
    spi3: SPI3,
) -> (lora::LoraRadio<'a, SPI3>, InputChannel<'a>) {
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::K125;
    // config.mode = MODE_3;
    interrupt::SPIM3.set_priority(interrupt::Priority::P3);

    let spim = spim::Spim::new(spi3, Irqs, sck, miso, mosi, config);

    let mut lora = lora::LoraRadio::new(nss, reset, spim).unwrap();
    if let Err(e) = lora.init() {
        defmt::warn!("lora init failed: {:?}", e);
    } else {
        lora.setup();
    }

    let channel = InputChannel::new(gpiote_ch0, dio0, InputChannelPolarity::LoToHi);

    (lora, channel)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    bee::init_heap();
    info!("Bee Mesh RX starting...");

    let mut config = Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let spi_nss = Output::new(p.P1_04, Level::High, OutputDrive::Standard);
    let spi_reset = Output::new(p.P1_06, Level::High, OutputDrive::Standard);
    let spi_sck = p.P0_20;
    let spi_miso = p.P0_22;
    let spi_mosi = p.P0_24;
    let dio0 = Input::new(p.P0_11, Pull::Down);

    let (mut lora, channel) = init_lora(
        spi_nss,
        spi_reset,
        spi_sck.into(),
        spi_miso.into(),
        spi_mosi.into(),
        dio0,
        p.GPIOTE_CH0,
        p.SPI3,
    );

    lora.start_rx();
    info!("Listening for incoming packets (interrupt-driven)...");

    let mut buffer = [0u8; 256];
    let mut packet_count: u32 = 0;

    loop {
        channel.wait().await;
        debug!("RX interrupt triggered!");

        if let Some(len) = lora.receive(&mut buffer) {
            packet_count += 1;
            info!("=== Packet #{} ({} bytes) ===", packet_count, len);
            debug!("plain");
            print_to_hex(&buffer, len);
            if len > 16 {
                if let Ok(header) = parse_packet_header(&buffer[..len]) {
                    debug!("parsed");
                    print_to_hex(&header.encrypted_payload, header.encrypted_payload.len());
                    if let Ok(result) = decrypt_aes_ctr(
                        &DEFAULT_KEY,
                        header.from_node,
                        header.packet_id,
                        &header.encrypted_payload,
                    ) {
                        debug!("decoded");
                        print_to_hex(&result, result.len());

                        debug!("try to parse the packet");

                        if let Ok(message) = meshtastic::parse(result.as_slice()) {
                            info!("port num {}", message.portnum);
                            match message.portnum() {
                                PortNum::PositionApp => {
                                    if let Ok(position) =
                                        Position::decode(message.payload.as_slice())
                                    {
                                        info!(
                                            "------- telemetry location lat {} long {}",
                                            position.latitude_i(),
                                            position.longitude_i()
                                        );
                                    };
                                }
                                PortNum::TelemetryApp => {
                                    if let Ok(telemetry) =
                                        Telemetry::decode(message.payload.as_slice())
                                    {
                                        if let Some(variant) = telemetry.variant {
                                            match variant {
                                                meshtastic_protobufs::telemetry::Variant::DeviceMetrics(device_metrics) => {
                                                  if let Some (battery) = device_metrics.battery_level {
                                    info!("------- telemetry app battery {}", battery);
                                                  }
                                                },
                                                meshtastic_protobufs::telemetry::Variant::EnvironmentMetrics(environment_metrics) => {
                                    info!("------- telemetry app temperature {} humidity {}", environment_metrics.temperature, environment_metrics.relative_humidity);
                                                },
                                                _=> {}
                                            };
                                        } else {
                                            warn!("variant not provided");
                                        }
                                    } else {
                                        warn!("failed to decode telemetry data");
                                    };
                                    // info!("telemetry app {:x}", message.payload.as_slice());
                                }
                                PortNum::TextMessageApp => {
                                    if let Ok(message) = str::from_utf8(message.payload.as_slice())
                                    {
                                        info!("------- message {}", message);
                                    } else {
                                        warn!(
                                            "failed to read the message {:02x}",
                                            message.payload.as_slice()
                                        );
                                    }
                                }
                                _ => {
                                    debug!(
                                        "unrecognized packet decoded {:x}",
                                        message.payload.as_slice()
                                    );
                                }
                            };
                        } else {
                            // info!("failed to decode the packet {:x}", result.as_slice());
                            info!("failed to decode the packet");
                        };
                    } else {
                        warn!("failed to decrypt");
                    }
                } else {
                    warn!("failed to decode header");
                }
            }
            debug!("=== Packet #{} ({} bytes).end ===", packet_count, len);
            // lora.start_rx();
        }
    }

    fn print_to_hex(buffer: &[u8], len: usize) {
        let mut hex = [0u8; 512];
        let mut pos = 0;
        for &byte in buffer[..len].iter() {
            let hex_chars = b"0123456789abcdef";
            hex[pos] = hex_chars[(byte >> 4) as usize];
            hex[pos + 1] = hex_chars[(byte & 0xf) as usize];
            pos += 2;
        }
        trace!("Hex: {}", core::str::from_utf8(&hex[..pos]).unwrap());
    }
}
