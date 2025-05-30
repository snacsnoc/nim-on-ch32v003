# Nim-on-CH32V003 — blink PoC

Bare-metal Nim demo for **CH32V003F4P6** (16 KB Flash / 2 KB RAM).  
PC5 toggles every 250 ms.

* Nim -> C -> riscv-none-elf-gcc (`rv32ec / ilp32e`)


* linker script from Adi Hamulic’s CH32V003-Bare-Metal demo  
  <https://github.com/AdiHamulic/CH32V003-Bare-Metal>

* device headers (`ch32v00x.h`, `system_ch32v00x.*`, `core_riscv.*`) © WCH — Apache-2.0

## build / flash

```bash
make clean
make          # blink.bin
make flash    # flashes via minichlink
```