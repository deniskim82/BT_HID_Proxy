# BT_HID_Proxy

Bluetooth Keyboard to USB HID Proxy using ESP32 and CH9329.

## Overview

This project enables using a Bluetooth keyboard with PCs that don't have Bluetooth support. The ESP32 acts as a Bluetooth HID Host, receiving keyboard input from a paired BT/BLE keyboard and forwarding it to a PC via the CH9329 UART-to-USB-HID converter chip.

```
[BT/BLE Keyboard] ---(Bluetooth)---> [ESP32] ---(UART)---> [CH9329] ---(USB HID)---> [PC]
```

## Hardware

### Target Board
- **M5Stamp Pico** (ESP32-PICO-D4)
  - Compact form factor
  - Built-in button (GPIO39)
  - RGB LED SK6812 (GPIO27) - Reserved for future use

### Required Components
- M5Stamp Pico or compatible ESP32 board
- CH9329 UART-to-USB-HID module
- BT/BLE Keyboard

### Wiring

| M5Stamp Pico | CH9329 Module |
|--------------|---------------|
| GPIO26 (TX)  | RXD           |
| GPIO36 (RX)  | TXD           |
| GND          | GND           |
| 3.3V         | VCC (optional if USB powered) |

**Note:** The CH9329 module is typically powered via USB from the target PC.

## Features

- **Dual-mode Bluetooth**: Supports both Classic Bluetooth and BLE keyboards
- **Auto-reconnect**: Automatically reconnects to the last paired device
- **Persistent pairing**: Stores paired device info in NVS flash
- **Pairing mode**: Long-press button (5 seconds) to pair new devices
- **Key release safety**: Automatic key release on disconnect to prevent stuck keys

## Operation

### Normal Operation
1. On power-up, the device loads the last paired keyboard from NVS
2. Scans for the paired device
3. Connects and starts relaying keyboard input
4. If disconnected, automatically attempts to reconnect

### Pairing a New Keyboard
1. Put your Bluetooth keyboard in pairing mode
2. Long-press the M5Stamp Pico button for 5+ seconds
3. The device enters pairing mode and scans for HID keyboards
4. Once found, it connects and saves the new pairing
5. Previous pairing is overwritten

### States
| State       | Description |
|-------------|-------------|
| IDLE        | Not scanning or connected |
| SCANNING    | Searching for paired device |
| CONNECTING  | Establishing connection |
| CONNECTED   | Relaying keyboard input |
| PAIRING     | Scanning for new devices |
| ERROR       | Connection error (auto-retry) |

## Building

### Prerequisites
- [ESP-IDF v5.0+](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/)

### Build Commands
```bash
# Set target
idf.py set-target esp32

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor
idf.py -p /dev/ttyUSB0 monitor
```

## Configuration

### Pin Configuration (`main/config.h`)
```c
// UART for CH9329
#define CH9329_UART_TX_PIN      GPIO_NUM_26
#define CH9329_UART_RX_PIN      GPIO_NUM_36
#define CH9329_UART_BAUD_RATE   9600

// Button
#define BUTTON_GPIO             GPIO_NUM_39

// Timing
#define BUTTON_LONG_PRESS_MS    5000   // Pairing mode trigger
#define BT_SCAN_TIMEOUT_SEC     10     // Scan duration
#define BT_RETRY_DELAY_MS       3000   // Reconnect delay
```

### CH9329 Baud Rate
The CH9329 defaults to 9600 baud. If your module is configured differently, update `CH9329_UART_BAUD_RATE` in `config.h`.

## Project Structure

```
BT_HID_Proxy/
├── CMakeLists.txt          # Project build file
├── sdkconfig.defaults      # SDK configuration
├── main/
│   ├── CMakeLists.txt      # Component build file
│   ├── main.c              # Application entry point
│   ├── config.h            # Configuration constants
│   ├── bt_hid_host.c/h     # Bluetooth HID Host module
│   ├── ch9329.c/h          # CH9329 UART driver
│   ├── storage.c/h         # NVS storage for pairing
│   └── button.c/h          # Button input handler
└── README.md
```

## Technical Details

### CH9329 Protocol
The CH9329 uses a simple frame-based UART protocol:

```
[0x57][0xAB][ADDR][CMD][LEN][DATA...][CHECKSUM]
```

- **Header**: `0x57 0xAB`
- **Address**: `0x00` (default)
- **Command**: `0x02` for keyboard report
- **Length**: Data payload length
- **Data**: 8-byte USB HID keyboard report
- **Checksum**: Sum of all bytes mod 256

### USB HID Keyboard Report
Standard 8-byte format:
```
[Modifier][Reserved][Key1][Key2][Key3][Key4][Key5][Key6]
```

- **Modifier bits**: Ctrl, Shift, Alt, GUI (left/right)
- **Keys**: Up to 6 simultaneous key codes (USB HID usage codes)

### Known Issues & Solutions

#### Stuck Keys on Multi-paired Keyboard Switch
When switching a multi-paired keyboard between devices, the disconnect may not properly release keys. This implementation includes:
- Explicit key release on BT disconnect event
- Periodic key release timeout check
- Forced release on reconnection

#### Input Latency
To minimize latency:
- Duplicate report filtering (skip if identical to last)
- Reduced logging in data path
- Direct UART write without buffering

## References

### ESP32 Bluetooth HID Host
- [ESP-IDF HID Host API Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/esp_hidh.html)
- [ESP-IDF HID Host Example](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/esp_hid_host)

### CH9329
- [WCH Official Product Page](https://www.wch-ic.com/products/CH9329.html)
- [CH9329 Datasheet (PDF)](https://www.wch-ic.com/downloads/CH9329DS1_PDF.html)
- [Protocol Reference Implementation (Python)](https://github.com/Blue-Beaker/9329KeyboardRemote)
- [CH9329 Keyboard Controller](https://github.com/sjmf/ch9329-keyboard)

## Future Improvements

- [ ] RGB LED status indication
- [ ] Multiple paired device storage
- [ ] Media key support
- [ ] Mouse input support
- [ ] Web-based configuration interface
- [ ] OTA firmware update

## License

MIT License - See [LICENSE](LICENSE) file.

## Troubleshooting

### Keyboard not connecting
1. Ensure keyboard is in pairing mode
2. Long-press button to enter pairing mode on the proxy
3. Check serial monitor for scan results

### Keys not registering
1. Verify CH9329 wiring (TX→RXD, RX→TXD)
2. Check CH9329 baud rate configuration
3. Test CH9329 module with PC directly

### Random key inputs after switching devices
This is a known issue with multi-paired keyboards. The implementation includes safeguards, but if it persists:
1. Press any key to stop the repeat
2. The issue should resolve on next connection

### Build errors
1. Ensure ESP-IDF v5.0+ is installed
2. Run `idf.py set-target esp32` before building
3. Clean and rebuild: `idf.py fullclean && idf.py build`
