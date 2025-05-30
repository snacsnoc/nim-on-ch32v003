NIM_COMPILER = nim
NIM_FLAGS    = --skipCfg -d:release --nimcache:./nimcache 

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

ASM_FLAGS     = -march=rv32ec_zicsr -mabi=ilp32e -Os -g
CFLAGS     = -march=rv32ec_zicsr -mabi=ilp32e -ffreestanding -Os -msmall-data-limit=8 $(INCLUDE_DIR)
      

.PHONY: all flash clean

all: $(BIN_NAME)

startup_ch32v003.o: c_src/startup_ch32v003.S
	@echo "AS $<"
	$(AS) $(ASM_FLAGS) -c $< -o $@

$(ELF_NAME): blink.nim startup_ch32v003.o
	@echo "NIM $@"
	$(NIM_COMPILER) c $(NIM_FLAGS) \
		--passC:"$(CFLAGS)" \
		--passL:"startup_ch32v003.o -lgcc -Wl,-Map=$(ELF_NAME).map -Wl,-gc-sections -Tlink.ld -nostdlib" \
		-o:$(ELF_NAME) blink.nim

$(BIN_NAME): $(ELF_NAME)
	@echo "OBJCOPY $@"
	$(OBJCOPY) -O binary $< $@

flash: $(BIN_NAME)
	@echo "FLASH $@"
	$(FLASHER) --erase --flash $<


$(HEX_NAME): $(ELF_NAME)
	@echo "HEX $@"
	$(OBJCOPY) -O ihex $< $@

clean:
	@echo "CLEAN"
	rm -f $(ELF_NAME) $(BIN_NAME) startup_ch32v003.o *.map
	rm -rf ./nimcache
