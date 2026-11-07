# ESP32-P4 BitScrambler Test

This standalone ESP-IDF app runs a nibble-swap program on the ESP32-P4
BitScrambler in loopback mode. It needs no matrix panel or external wiring.
The app prints the input and transformed bytes over the serial monitor and
reports `PASS` when they match the expected result.

## Build and run

Select the `esp32-p4-bitscrambler` environment from PlatformIO, or build it from
the project root:

```sh
pio run -e esp32-p4-bitscrambler
```

Flash and monitor the connected board (replace `COMx` with its serial port):

```sh
pio run -e esp32-p4-bitscrambler -t upload --upload-port COMx
pio device monitor -e esp32-p4-bitscrambler
```

The expected output is the input with the high and low nibble of each byte
swapped. For example, `01 23 45 67` becomes `10 32 54 76`.