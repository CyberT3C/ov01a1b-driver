#!/usr/bin/env bash

echo "OV01A1B Test Script"
echo "=================="
echo "Available modules:"

echo "Loading OV01A1B test module..."
./load.sh
sleep 3 

echo ""
echo "=== ACPI devices ==="
find /sys/bus/acpi/devices -name "*OVTI01AB*" 2>/dev/null | head -5

echo ""
echo "=== dmesg output ==="
sudo dmesg | tail -50 | grep -E "ov01a1b|OV01A1B|OVTI01AB"
sudo dmesg | grep -A 30 "OV01A1B Power Test Probe"

echo ""
echo "Unloading module..."
./unload.sh
