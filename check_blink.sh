#!/usr/bin/env bash
# check_blink.sh – build-time sanity tester for CH32V003 firmware
# usage:  ./check_blink.sh blink.bin [blink.elf]

set -e

BIN="$1"
ELF="${2:-${BIN%.bin}}"

if [[ ! -f "$BIN" ]]; then
  echo "❌  binary '$BIN' not found"; exit 1
fi
if [[ ! -f "$ELF" ]]; then
  echo "❌  ELF '$ELF' not found (pass it as 2nd arg or keep same name w/o .bin)"; exit 1
fi

PREFIX=riscv-none-elf
SIZE_OK_FLASH=16384     # 16 KB
SIZE_OK_RAM=2048        # 2 KB

echo "=== size check ====================================================="
$PREFIX-size "$ELF"

# extract numbers (text data bss)
read -r TEXT DATA BSS _ <<<"$($PREFIX-size -A -d "$ELF" | awk '/^\.(text|data|bss)/{sum[$1]+=$2} END{print sum[".text"], sum[".data"], sum[".bss"]}')"
FLASH=$((TEXT+DATA))
RAM=$((DATA+BSS))

[[ $FLASH -le $SIZE_OK_FLASH ]] || { echo "❌ Flash usage $FLASH > $SIZE_OK_FLASH"; exit 1; }
[[ $RAM   -le $SIZE_OK_RAM   ]] || { echo "❌ RAM   usage $RAM   > $SIZE_OK_RAM";   exit 1; }

echo "✅  size within limits"

echo "=== ELF header / ISA ==============================================="
$PREFIX-readelf -h "$ELF" | grep -E 'Machine|Entry|Flags'

echo "=== section layout (first 10) ====================================="
$PREFIX-objdump -h "$ELF" | head -n 12

echo "=== first 40 instructions =========================================="
$PREFIX-objdump -d "$ELF" | head -n 40

echo "=== generate HEX & checksum ========================================"
HEX="${BIN%.bin}.hex"
$PREFIX-objcopy -O ihex "$ELF" "$HEX"
sha1sum "$BIN"

echo "✅  all checks passed"