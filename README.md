# Nim-on-CH32V003

A pure Nim, bare-metal HAL for the **CH32V003F4P6** (16 KB Flash / 2 KB RAM). The `blink` example toggles PC5 every 250 ms.

This project is a pure Nim implementation, removing the dependency on WCH's C libraries.

*   **Pure Nim HAL:** Peripherals are accessed via a type-safe HAL generated from SVD files. No C library wrappers.
*   Nim -> C -> riscv-none-elf-gcc (`rv32ec / ilp32e`).
*   Minimal C/ASM for startup (`startup_ch32v003.S`).
*   `panicoverride.nim` stub for bare-metal panic handling.
*   Build rules in `Makefile` and flags in `nim.cfg`.

## build 

```bash
make clean
make examples       # blink.bin, uart_blink.bin
```

## flash

```bash
make flash    # flashes via minichlink
```
If you do not possess a WCH Link-e adapter, you can use a Raspberry Pi Pico, ESP32, or an Arduino to flash the .bin file.
Some helpful resources:
* [ESP32-S2 CH32V003 Programmer](https://github.com/cnlohr/esp32s2-cookbook/tree/master/ch32v003programmer)
* My fork of [Pico CH32V003 Programmer](https://github.com/snacsnoc/pico_ch32v003_prog)
  * See [original repo](https://github.com/hexagon5un/pico_ch32v003_prog)
* [NHC-Link042](https://github.com/NgoHungCuong/NHC-Link042)
* [Flipper Zero WCH SWIO Flasher and debugger](https://github.com/sukvojte/wch_swio_flasher)
* [Ardulink firmware (PlatformIO)](https://github.com/Community-PIO-CH32V/Ardulink-pio)

### Credits

*   Linker script from cjacker’s CH32V003EVT GCC/Makefile repo:  
    <https://github.com/cjacker/ch32v003evt_gcc_makefile>
*   Fixed ch32v003 SVD file from the ch32-rs project:  
    <https://github.com/ch32-rs/ch32-rs/tree/main/svd/fixed>
*   Original startup code (`startup_ch32v00x.S`) © WCH — Apache-2.0.