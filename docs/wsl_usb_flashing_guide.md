# WSL USB Flashing Guide - ESP32 on Windows 11

This guide explains how to flash an ESP32 device from WSL Ubuntu 22.04 on Windows 11 using usbipd-win.

---

## One-Time Setup

### 1. Install usbipd-win on Windows
Open PowerShell as **Administrator** and run:
```powershell
winget install --interactive --exact dorssel.usbipd-win
```

After installation, **restart your computer** or at minimum close all PowerShell/WSL terminals and reopen them.

### 2. Install USB/IP tools in WSL Ubuntu
Open your Ubuntu WSL terminal:
```bash
sudo apt update
sudo apt install linux-tools-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
```

### 3. Add User to dialout Group (for serial port access)
```bash
sudo usermod -a -G dialout $USER
```

After this command, **log out and back in to WSL** (or run `newgrp dialout`).

---

## Every Time You Want to Flash

### Step 1: Identify Your Device
In a regular PowerShell window (doesn't need admin):
```powershell
usbipd list
```

Example output:
```
Connected:
BUSID  VID:PID    DEVICE                                                        STATE
1-3    13d3:56ff  Integrated Camera                                             Not shared
1-4    1a86:7523  USB-SERIAL CH340 (COM4)                                       Not shared
2-4    8087:0029  Intel(R) Wireless Bluetooth(R)                                Not shared
```

Look for your ESP32 device. Common identifiers:
- **CH340**: `1a86:7523` - USB-SERIAL CH340
- **CP2102**: `10c4:ea60` - Silicon Labs CP210x
- **FTDI**: `0403:6001` - Future Technology Devices

In this example, the ESP32 is at **BUSID 1-4**.

### Step 2: Bind the Device (First Time Only)
Open PowerShell as **Administrator**:
```powershell
# Right-click PowerShell icon → "Run as Administrator"
# OR from within PowerShell:
Start-Process powershell -Verb RunAs
```

In the Administrator PowerShell:
```powershell
# Bind the device (only needed once per BUSID)
usbipd bind --busid 1-4
```

After binding, `usbipd list` should show `STATE: Shared` instead of `Not shared`.

### Step 3: Attach Device to WSL
Still in the **Administrator PowerShell**:
```powershell
usbipd attach --wsl --busid 1-4
```

Expected output:
```
usbipd: info: Using WSL distribution 'Ubuntu-22.04' to attach; the device will be available in all WSL 2 distributions.
usbipd: info: Loading vhci_hcd module.
usbipd: info: Detected networking mode 'nat'.
usbipd: info: Using IP address 172.19.16.1 to reach the host.
```

### Step 4: Verify Device in WSL
Switch to your WSL Ubuntu terminal:
```bash
# Check if device appeared
ls /dev/ttyUSB*
# Should show: /dev/ttyUSB0 (or similar)

# Check detailed USB info
lsusb
# Should show something like: "QinHeng Electronics CH340 serial converter"

# Check kernel messages (optional)
dmesg | tail -20
# Should show: ch341-uart converter detected, ttyUSB0 attached
```

### Step 5: Flash Your ESP32
In the WSL terminal, navigate to your project and flash:
```bash
cd /home/qcarver/dev/soem_poe32

# Flash to the device
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Or flash and monitor in one command
idf.py -p /dev/ttyUSB0 flash monitor
```

To exit the monitor, press `Ctrl+]`.

### Step 6: When Done - Detach Device (Optional)
In the **Administrator PowerShell**:
```powershell
usbipd detach --busid 1-4
```

**Note:** You can leave the device attached if you'll be flashing again soon. It will auto-detach when you unplug the USB cable or restart WSL.

---

## Troubleshooting

### "Permission denied" when accessing /dev/ttyUSB0
```bash
# Verify you're in the dialout group
groups
# Should show: ... dialout ...

# If not, add yourself:
sudo usermod -a -G dialout $USER

# Log out and back in to WSL, or run:
newgrp dialout
```

### Device not showing as /dev/ttyUSB0
```bash
# Check what USB devices are present
lsusb
# Should show your USB-to-serial chip (CH340, CP2102, etc.)

# Check kernel messages
dmesg | grep -i usb
dmesg | grep -i ch340  # or cp210x, ftdi, etc.

# If still not showing, try reattaching in Windows
```

### "usbipd: error: Device is already attached"
```powershell
# Detach first
usbipd detach --busid 1-4

# Wait 2-3 seconds, then attach again
usbipd attach --wsl --busid 1-4
```

### "usbipd: error: Access denied; this operation requires administrator privileges"
Make sure you're running PowerShell **as Administrator**:
```powershell
# From a regular PowerShell window:
Start-Process powershell -Verb RunAs
```

### WSL can't see the device after attach
```bash
# In WSL, verify usbip client is working
sudo modprobe vhci-hcd

# Check if the kernel module loaded
lsmod | grep vhci

# Check USB/IP status (use the IP from the attach message)
usbip list --remote=172.19.16.1
```

### ESP32 not responding / "Failed to connect"
1. **Press and hold BOOT button** on ESP32 while running `idf.py flash`
2. Release BOOT button after "Connecting..." message appears
3. If it still fails, try:
   ```bash
   # Erase flash first
   idf.py -p /dev/ttyUSB0 erase-flash
   
   # Then flash again
   idf.py -p /dev/ttyUSB0 flash
   ```

### Multiple /dev/ttyUSB* devices showing up
```bash
# List all with details
ls -l /dev/ttyUSB*

# Check which one is your ESP32
dmesg | grep ttyUSB

# Try each one if unsure
idf.py -p /dev/ttyUSB0 flash  # Try USB0
idf.py -p /dev/ttyUSB1 flash  # Try USB1
```

---

## Quick Reference Commands

### Windows (Admin PowerShell)
```powershell
# Show all USB devices
usbipd list

# Bind device once (if showing "Not shared")
usbipd bind --busid 1-4

# Attach to WSL (each time you want to flash)
usbipd attach --wsl --busid 1-4

# Detach when done
usbipd detach --busid 1-4
```

### WSL Ubuntu
```bash
# Verify device is attached
ls /dev/ttyUSB*
lsusb

# Flash ESP32
idf.py -p /dev/ttyUSB0 flash

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor

# Monitor only (after flash)
idf.py -p /dev/ttyUSB0 monitor

# Exit monitor
# Press Ctrl+]
```

---

## Understanding the Workflow

### Device States in Windows

1. **Not shared** - Device is only available to Windows
   - Run: `usbipd bind --busid X-X`
   
2. **Shared** - Device is bound but not attached
   - Run: `usbipd attach --wsl --busid X-X`
   
3. **Attached** - Device is connected to WSL
   - Device appears in WSL as `/dev/ttyUSB0`
   - Windows can't use it until detached

### When to Attach/Detach

**Attach when:**
- You want to flash from WSL
- You need to use `idf.py monitor`
- You're debugging via serial console

**Detach when:**
- You want to use Windows tools (e.g., Arduino IDE, PuTTY)
- You're done working with the device
- The device is unplugged/replugged

**Auto-detach happens when:**
- You unplug the USB cable
- You restart WSL
- You restart Windows

---

## Common USB-to-Serial Chips

| Chip | VID:PID | Device Name | Notes |
|------|---------|-------------|-------|
| CH340 | 1a86:7523 | USB-SERIAL CH340 | Most common on cheap ESP32 boards |
| CP2102 | 10c4:ea60 | Silicon Labs CP210x | Common on NodeMCU, DevKits |
| FTDI | 0403:6001 | FT232 USB UART | Higher quality, less common |

Your board uses **CH340** (1a86:7523), which is widely used on budget ESP32 development boards.

---

## Notes

- You need to **attach** the device every time you restart WSL or reconnect your USB cable
- The **bind** command only needs to be run **once per BUSID** (converts "Not shared" → "Shared")
- If you see "STATE: Attached" in `usbipd list`, the device is already connected to WSL
- Multiple ESP32 devices can be attached simultaneously (they'll show as `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.)
- The device BUSID (e.g., 1-4) may change if you plug it into a different physical USB port

---

## Additional Resources

- **usbipd-win GitHub**: https://github.com/dorssel/usbipd-win
- **ESP-IDF Documentation**: https://docs.espressif.com/projects/esp-idf/
- **WSL USB Support**: https://learn.microsoft.com/en-us/windows/wsl/connect-usb
