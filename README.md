# rome

cli companion for [marisko](https://github.com/softmodded/marisko) - custom firmware for the teenage engineering sp-01 stem player.

flashes firmware over the bootloader and manages stems (songs) on the device's storage over usb.

## features

- **firmware flash** — write a `.bin` to the device via the sp-1 bootloader (uart)
- **stem upload** — encode 4 stereo stems (any format, any rate) to 8-channel ima adpcm and upload at ~390 KB/s
- **disk management** — list / add / remove songs, format, read disk info
- **diagnostics** — codec bring-up, ext_csd dump, block read/decode, write probe

## usage

```
# flash firmware
rome flash -p /dev/ttyACM0 build/sp1_firmware.bin

# add a song (4 stems → 8ch ADPCM)
rome song add "my song" drums.flac vocals.flac bass.flac other.flac

# list / remove / format
rome song list
rome song rm 0
rome format --yes

# device info
rome info
rome codec
```

## permissions

stem management talks raw usb bulk (libusb) to the running firmware so i nee dto bypass the
kernel cdc-acm tty for full throughput. install a udev rule so it works without sudo:

```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2fe3", ATTRS{idProduct}=="0101", MODE="0666", TAG+="uaccess"' \
  | sudo tee /etc/udev/rules.d/99-sp1.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

(firmware flashing uses the bootloaders serial and doesnt need sudo)

## build

```
cargo build --release
```

requires libusb (`pacman -S libusb` / `brew install libusb`).

## license

MIT
