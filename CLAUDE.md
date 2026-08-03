# BT_HID_Proxy — project notes

BLE keyboard → USB HID proxy. ESP32 (NimBLE) acts as a BLE HID host and relays
input to a PC through a CH9329 UART-to-USB-HID chip.

```
[BLE keyboard] --(BLE/HOGP)--> [ESP32/NimBLE] --(UART)--> [CH9329] --(USB)--> [PC]
```

## Hardware constraints (why it is built this way)

- Board is an **M5Stamp Pico** (ESP32-PICO-D4). It is a minimal module with no
  USB OTG header broken out, so USB HID cannot be produced by the ESP32 itself
  — hence the external CH9329.
- UART1: TX=GPIO32 → CH9329 **RXD**, RX=GPIO33 ← CH9329 **TXD**, 115200 8-N-1,
  no flow control. Crossed; GND must be common.
- Button GPIO39, SK6812 RGB LED GPIO27.
- The CH9329 RXD line (GPIO33) is not needed for keystrokes to flow, but
  without it there is no LED state sync and no diagnostics.

## Known ceilings

- **6 keys maximum to the PC.** The CH9329 keyboard command (0x02) takes an
  8-byte boot report, so six non-modifier keys is a hard protocol limit
  regardless of what the source keyboard sends. Modifiers are a separate byte
  and never compete for those slots.
- **~125 Hz input ceiling** comes from BLE itself (7.5 ms minimum connection
  interval), not from the UART. At 115200 a 14-byte key frame costs ~1.2 ms, so
  the link runs at roughly 12 % utilisation — raising the baud rate buys
  nothing for BLE sources.

## Future direction (discussed, not implemented)

Moving to an **ESP32-S3 based board with native USB OTG** (e.g. an M5Stamp S3)
would let one chip be both the BLE host and the USB HID device, removing the
CH9329 entirely and with it the 6-key limit. Mouse and composite devices become
straightforward too.

Porting cost is low by design: `hid_parser` and `key_state` are transport
agnostic and carry over unchanged; only `ch9329.c` would be replaced with a
TinyUSB backend.

Other items considered but deliberately deferred:

- **Mouse support.** Feasible, but must accumulate X/Y deltas and emit at a
  fixed rate (~125 Hz) rather than queueing every report — a deeper queue is
  the wrong lever, since it converts a rate mismatch into latency. Never block
  the NimBLE host task on a full queue; that is what caused the freezes and
  reboots seen with an earlier mouse attempt.
- **Multiple simultaneous devices.** NimBLE supports it (raise
  `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` and `CONFIG_BTDM_CTRL_BLE_MAX_CONN`,
  currently 1). Three is the practical limit; radio scheduling, not memory, is
  the constraint, and connection intervals should relax to 11.25–15 ms. The
  merge layer (`key_state`) and the device index in the callbacks already
  account for this.

## Traps discovered the hard way

- **CH9329 `0x0D` is `CMD_RESET`, not "get LED status".** There is no LED query
  command; LED state rides in the `GET_INFO` (0x01) reply. Earlier firmware
  sent 0x0D on every connect and every lock-key press, resetting the chip and
  dropping its USB enumeration — which surfaced on Windows as *"Unknown USB
  Device (Device Descriptor Request Failed)"* and made LED sync never work.
- **Only forward reports positively identified as keyboard input.** Treating
  any 8-byte report as keystrokes is what caused random keys to repeat forever
  after the keyboard came back from another host.
- **Keyboards commonly declare several keyboard input reports** (6KRO *and*
  NKRO) and send on whichever matches their current mode. Subscribing to only
  one leaves the link up but silent. Subscribe to all of them.
- **Explicit `Usage()` items beat `Usage Minimum`.** Descriptors often
  enumerate usages and then declare a placeholder `Usage Minimum (1)`; the
  system-control report on the test keyboard does exactly this.
- **Never truncate the key list in the parser.** Capping belongs where the boot
  report is assembled, so held keys keep their slots.
- **Serialise all UART writes** through the single TX task. Concurrent
  `uart_write_bytes` calls interleave frame bytes and the CH9329 discards them.
- **No blocking calls on the input path** — a `vTaskDelay` in the BLE callback
  stalls the whole event queue.
- `sdkconfig.defaults` changes do not apply to an existing `sdkconfig`; delete
  it and rebuild. Plain source edits need no `fullclean`.

## Build

ESP-IDF v5.1.x (CI pins v5.1.2). `idf.py set-target esp32 && idf.py build`.
CI builds on pull requests and publishes flashable artifacts.

## Diagnostics

Budgeted logging, quiet in steady state: report map hex dump, discovered report
characteristics, chosen subscriptions, first 10 input reports, first 10 CH9329
TX frames (with a FIFO drain check), CH9329 RX frames including ACKs, plus a
boot-time UART loopback self-test and `GET_INFO`/`GET_PARA_CFG` dumps.
