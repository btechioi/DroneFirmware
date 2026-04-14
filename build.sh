#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$SCRIPT_DIR/.venv/lib/python3.12/site-packages"

FIRMWARE_DIR="$SCRIPT_DIR/firmware"
mkdir -p "$FIRMWARE_DIR"

echo "Building DroneFirmware..."
pio run && echo "✓ Flight Controller (Pico)"

echo ""
echo "Building ESP32 RC Receiver..."
pio run -d esp32-rc -e esp32c3-rc -e esp32-s3-rc && echo "✓ ESP32 RC Receiver (C3 & S3)"

echo ""
echo "Building ESP32 RC Transmitter..."
pio run -d esp32-rc -e esp32c3-tx -e esp32-s3-tx && echo "✓ ESP32 RC Transmitter (C3 & S3)"

echo ""
echo "============================================"
echo "Copying firmware files..."
echo "============================================"

cp .pio/build/pico/firmware.uf2 "$FIRMWARE_DIR/"
cp esp32-rc/.pio/build/esp32c3-rc/firmware.bin "$FIRMWARE_DIR/rc_receiver_esp32c3.bin"
cp esp32-rc/.pio/build/esp32-s3-rc/firmware.bin "$FIRMWARE_DIR/rc_receiver_esp32s3.bin"
cp esp32-rc/.pio/build/esp32c3-tx/firmware.bin "$FIRMWARE_DIR/rc_transmitter_esp32c3.bin"
cp esp32-rc/.pio/build/esp32-s3-tx/firmware.bin "$FIRMWARE_DIR/rc_transmitter_esp32s3.bin"

echo ""
echo "Built files in $FIRMWARE_DIR:"
ls -lh "$FIRMWARE_DIR/"
