# BT_HID_Proxy

![Build Status](https://github.com/deniskim82/BT_HID_Proxy/actions/workflows/build.yml/badge.svg)

BLE Keyboard to USB HID Proxy using ESP32 and CH9329.

*[한국어 문서](README.ko.md)*

## Overview

This project enables using a BLE keyboard with PCs that don't have Bluetooth support. The ESP32 acts as a BLE HID (HOGP) Host, receiving keyboard input from a bonded BLE keyboard and forwarding it to a PC via the CH9329 UART-to-USB-HID converter chip.

```
[BLE Keyboard] ---(BLE / HOGP)---> [ESP32 / NimBLE] ---(UART)---> [CH9329] ---(USB HID)---> [PC]
```

> **Note:** This is a BLE-only rewrite (NimBLE stack). Classic Bluetooth keyboards
> are not supported. Virtually every keyboard sold in the last decade supports BLE;
> BLE also gives lower input latency (7.5 ms connection interval) and far more
> reliable reconnection than the previous dual-mode (Bluedroid) implementation.

## Hardware

### Target Board
- **M5Stamp Pico** (ESP32-PICO-D4)
  - Compact form factor
  - Built-in button (GPIO39)
  - RGB LED SK6812 (GPIO27) - Status indicator

### Required Components
- M5Stamp Pico or compatible ESP32 board
- CH9329 UART-to-USB-HID module
- BLE Keyboard

### Wiring

| M5Stamp Pico | CH9329 Module |
|--------------|---------------|
| GPIO32 (TX)  | RXD           |
| GPIO33 (RX)  | TXD           |
| GND          | GND           |
| 3.3V         | VCC (optional if USB powered) |

**Note:** The CH9329 module is typically powered via USB from the target PC. UART runs at 115200 baud (unchanged from previous firmware; CH9329 factory default is 9600 - reconfigure the module if yours is still at the default).

## Features

- **BLE HOGP host on NimBLE**: lightweight stack, fast and reliable reconnection
- **HID report map parsing**: the keyboard input report is positively identified
  from the device's report descriptor. Only that report is subscribed to and
  forwarded - media/vendor/battery reports can never be misinterpreted as
  keystrokes (this was the root cause of the old "random key repeats forever
  after reconnection" bug)
- **NKRO support**: bitmap-style (N-key rollover) reports are translated to
  standard 6KRO boot reports
- **Boot protocol fallback** for keyboards with unusual report maps
- **Auto-reconnect**: alternates between scanning for the keyboard's
  advertisements and direct connection attempts
- **Bond persistence**: pairing survives power cycles (NimBLE bond store + NVS)
- **Re-pairing resilience**: if the keyboard dropped its bond (e.g. its pairing
  slot was reused on another host), the proxy detects it and re-pairs
  automatically
- **Media and system keys**: volume, mute, play/pause, track skip, browser and
  application shortcuts, plus power/sleep/wake
- **LED state sync**: Caps/Num/Scroll lock state from the PC is forwarded to
  the keyboard, polled without ever blocking the input path
- **Key release safety**: all keys are released on the USB side whenever the
  BLE link is not up
- **Single-writer UART**: all CH9329 frames go through one TX task and queue,
  so frames can never interleave (previously a source of dropped keys)
- **Pairing mode**: long-press button (5 s) to pair a new keyboard
- **Static passkey**: keyboards that require Passkey Entry use code `123456`

## Operation

### Normal Operation
1. On power-up, the device loads the bonded keyboard from NVS
2. Scans for its advertisements (and periodically tries a direct connection)
3. Connects, encrypts, discovers the HID service, subscribes to the keyboard
   input report, and starts relaying
4. If disconnected, automatically reconnects

### Pairing a New Keyboard
1. Put your BLE keyboard in pairing mode
2. Long-press the M5Stamp Pico button for 5+ seconds
3. The proxy scans for ~6 s and connects to the strongest HID keyboard found
4. If the keyboard asks for a passkey, type `123456` on it and press Enter
5. The new bond replaces any previous pairing

### States & LED Indicators

Color = meaning, blink speed = urgency. Red is reserved for errors only.

| State       | Description                        | LED Pattern              |
|-------------|------------------------------------|--------------------------|
| IDLE        | No pairing stored                  | Slow amber blink (1 s)   |
| SCANNING    | Searching for bonded keyboard      | Slow blue blink (1 s)    |
| PAIRING     | Pairing mode, discoverable         | Fast blue blink (200 ms) |
| CONNECTING  | Establishing connection            | Solid blue               |
| CONNECTED   | Relaying keyboard input            | Solid dim green          |
| ERROR       | Connection error (auto-retry)      | Fast red blink (200 ms)  |

## Building

### Option 1: Download Pre-built Binaries (Recommended)

Pre-built firmware binaries are automatically generated via GitHub Actions:

1. Go to the [Actions](https://github.com/deniskim82/BT_HID_Proxy/actions) tab
2. Select the latest successful build
3. Download the artifacts (contains `.bin` files and flash script)
4. Extract and run the flash script:
   ```bash
   chmod +x flash.sh
   ./flash.sh /dev/ttyUSB0
   ```

### Option 2: Build from Source

#### Prerequisites
- [ESP-IDF v5.1.x](https://docs.espressif.com/projects/esp-idf/en/v5.1.2/esp32/get-started/)

#### Build Commands
```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

## Configuration

### Pin Configuration (`main/config.h`)
```c
// UART for CH9329
#define CH9329_UART_TX_PIN      GPIO_NUM_32
#define CH9329_UART_RX_PIN      GPIO_NUM_33
#define CH9329_UART_BAUD_RATE   115200

// Button
#define BUTTON_GPIO             GPIO_NUM_39

// RGB LED (SK6812)
#define RGB_LED_GPIO            GPIO_NUM_27

// Pairing
#define BUTTON_LONG_PRESS_MS    5000     // Pairing mode trigger
#define BLE_STATIC_PASSKEY      123456   // Passkey Entry code
```

### CH9329 Baud Rate
The project is configured for 115200 baud. If your CH9329 module uses a different rate (factory default is 9600), update `CH9329_UART_BAUD_RATE` in `config.h` or reconfigure the CH9329 module.

## Project Structure

```
BT_HID_Proxy/
├── CMakeLists.txt          # Project build file
├── sdkconfig.defaults      # SDK configuration (NimBLE, BLE-only)
├── main/
│   ├── CMakeLists.txt      # Component build file
│   ├── main.c              # Application entry point / wiring
│   ├── config.h            # Configuration constants
│   ├── ble_hid_host.c/h    # BLE HID (HOGP) host on NimBLE
│   ├── hid_parser.c/h      # HID report descriptor parser & translator
│   ├── ch9329.c/h          # CH9329 UART driver (single-writer TX queue)
│   ├── storage.c/h         # NVS storage for pairing
│   ├── button.c/h          # Button input handler
│   └── led_status.c/h      # RGB LED status indicator
└── README.md
```

## Technical Details

### Data path

```
NimBLE host task          CH9329 TX task            CH9329 RX task
----------------          --------------            --------------
notification arrives
  -> report map based
     translation to
     8-byte boot report
  -> queue post  ------->  dedup + frame build
                           -> uart_write_bytes
                                                    LED status frames
                                                    -> BLE LED output write
```

No blocking calls (delays, log-heavy paths) exist anywhere on the input path.

### CH9329 Protocol
```
[0x57][0xAB][ADDR][CMD][LEN][DATA...][CHECKSUM]
```

| Command | Meaning |
|---|---|
| 0x01 | GET_INFO (chip version, USB enumeration state, LED state) |
| 0x02 | Keyboard report |
| 0x03 | Media / system control keys |
| 0x08 | Read stored configuration |
| 0x0D | **RESET** — not a LED query, see below |

Checksum is the sum of all preceding bytes, mod 256.

> **Trap:** `0x0D` is the reset command. Earlier firmware mistook it for "get
> LED status" and sent it on every connection and every lock-key press, which
> reset the CH9329 and dropped its USB enumeration — surfacing on Windows as
> *"Unknown USB Device (Device Descriptor Request Failed)"*. LED state is
> carried in the `GET_INFO` (0x01) reply instead.

### Media key translation

The CH9329 takes a fixed bitmap rather than HID usage codes, so consumer-page
usages reported by the keyboard are translated to CH9329 bit positions
(volume, mute, transport, browser and application keys, plus power/sleep/wake).
Usages with no CH9329 equivalent — screen brightness, for instance — are
dropped rather than guessed at.

### USB HID Keyboard Report
Standard 8-byte boot format:
```
[Modifier][Reserved][Key1][Key2][Key3][Key4][Key5][Key6]
```

### Why the rewrite fixed the old bugs

| Old symptom | Root cause | Fix |
|---|---|---|
| Random key repeating forever after the keyboard came back from another host | Any 8-byte report (media/vendor/handshake) was forwarded as keystrokes; a phantom key-down never got its release | Report map parsing: only the verified keyboard input report is subscribed and forwarded |
| Occasional dropped keys | Concurrent `uart_write_bytes` from several tasks interleaved frame bytes; CH9329 discarded corrupted frames | Single TX task + queue serializes all UART writes |
| Input lag after Caps/Num/Scroll | `vTaskDelay(50)` inside the BT input callback stalled the event queue | LED polling moved to a one-shot esp_timer; input path never blocks |
| Sporadic freeze/reboot | Unaligned `*(uint64_t *)` read of report data (Xtensa LoadStoreAlignment exception) | Plain `memcmp` on the TX task |

## References

- [NimBLE (Apache Mynewt) Host API](https://mynewt.apache.org/latest/network/index.html)
- [HID over GATT Profile (HOGP) 1.0](https://www.bluetooth.com/specifications/specs/hid-over-gatt-profile-1-0/)
- [CH9329 Datasheet](https://www.wch-ic.com/downloads/CH9329DS1_PDF.html)

## Future Improvements

- [x] RGB LED status indication
- [x] LED (Caps/Num/Scroll) state sync
- [x] Media key support (consumer control report -> CH9329 multimedia command)
- [ ] Mouse input support (for keyboards with a trackpoint or touchpad)
- [ ] Multiple paired device storage
- [ ] OTA firmware update

## License

MIT License - See [LICENSE](LICENSE) file.

## Troubleshooting

### Keyboard not connecting
1. Ensure the keyboard is in pairing mode
2. Long-press the button (5 s) to enter pairing mode on the proxy
3. Check serial monitor for scan results; keyboards are matched by HID service
   UUID or appearance in their advertisement

### Keyboard asks for a code
Type `123456` on the keyboard and press Enter (`BLE_STATIC_PASSKEY`).

### Keys not registering
1. Verify CH9329 wiring (TX→RXD, RX→TXD)
2. Check CH9329 baud rate configuration (115200 expected)
3. Test the CH9329 module with a PC directly

### Build errors
1. Ensure ESP-IDF v5.1.x is installed
2. Run `idf.py set-target esp32` before building
3. Clean and rebuild: `idf.py fullclean && idf.py build`
