## bootable

```sh 
cargo b --release
cargo objcopy --release  -- -O binary firmware.bin 
uf2conv firmware.bin --family 0xADA52840 --output firmware.uf2
```

## to check

https://lib.rs/crates/nrf-softdevice-s140

check for the `.text` section

```sh
readelf -S target/thumbv7em-none-eabihf/release/bee

probe-rs erase --chip nrf52840_xxAA
probe-rs download --verify --binary-format hex --chip nRF52840_xxAA s140_nrf52_7.3.0_softdevice.hex 

```
