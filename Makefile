NIM_COMPILER = nim
NIM_FLAGS    = --cc:gcc --skipUserCfg -d:release --nimcache:./nimcache

ELF_NAME     = blink
BIN_NAME     = $(ELF_NAME).bin

# note to self: 
# use https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/
# export PATH="$HOME/xpack-riscv-none-elf-gcc-14.2.0-3/bin:$PATH"
# brew riscv-software-src/homebrew-riscv  doesn't include rv32 E-class multilib
# must have rv32e 

TOOLCHAIN_PREFIX = riscv-none-elf-
AS               = $(TOOLCHAIN_PREFIX)gcc
OBJCOPY          = $(TOOLCHAIN_PREFIX)objcopy
FLASHER          = minichlink

HEX_NAME = $(ELF_NAME).hex

ASM_FLAGS     = -march=rv32ec_zicsr -mabi=ilp32e -Os

EXAMPLES        := $(basename $(notdir $(filter-out examples/panicoverride.nim, $(wildcard examples/*.nim))))
ELFS            := $(addsuffix .elf,$(EXAMPLES))
BINS            := $(addsuffix .bin,$(EXAMPLES))

.PHONY: all examples clean flash $(EXAMPLES)

all: blink.bin

examples: $(BINS)

system/startup_ch32v003.o: system/startup_ch32v003.S
	@echo "AS $<"
	$(AS) $(ASM_FLAGS) -c $< -o $@

%.elf: examples/%.nim system/startup_ch32v003.o
	@echo "NIM → $@"
	$(NIM_COMPILER) c $(NIM_FLAGS) -o:$@ $<

%.bin: %.elf
	@echo "BIN → $@"
	$(OBJCOPY) -O binary $< $@

flash: $(BIN_NAME)
	@echo "FLASH $@"
	$(FLASHER) --erase --flash $<

%.hex: %.elf
	@echo "HEX → $@"
	$(OBJCOPY) -O ihex $< $@

clean:
	@echo "CLEAN"
	rm -f *.elf *.bin *.hex system/startup_ch32v003.o *.map
	rm -rf ./nimcache