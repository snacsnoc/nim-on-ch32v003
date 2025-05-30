# Nim-on-CH32V003 — blink PoC

Bare-metal Nim demo for **CH32V003F4P6** (16 KB Flash / 2 KB RAM).  
PC5 toggles every 250 ms.

* Nim -> C -> riscv-none-elf-gcc (`rv32ec / ilp32e`)
* minimal C/ASM in `c_src/` (startup_ch32v003.S + nim_interface.c)  
* `panicoverride.nim` stub to disable Nim’s panic on bare-metal  
* build rules in `Makefile` and flags in `nim.cfg`

## build / flash

```bash
make clean
make          # blink.bin
make flash    # flashes via minichlink
```

### Credits

* linker script from cjacker’s pre-converted WCH CH32V003EVT GCC/Makefile repo  
  <https://github.com/cjacker/ch32v003evt_gcc_makefile>  

* system libs + device headers (`ch32v00x.h`, `system_ch32v00x.*`, `core_riscv.*`, `startup_ch32v00x.s`, `link.ld`) © WCH — Apache-2.0
