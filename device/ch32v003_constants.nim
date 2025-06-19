
type
  FlashLatency* = enum
    Latency0 = 0
    Latency1 = 1

  SystemClockSource* = enum
    HSI = 0b00
    HSE = 0b01
    PLL = 0b10

  PLLSource* = enum
    HSI_Div2 = false
    HSE = true

  AHBPrescaler* = enum
    Div1 = 0b0000
    Div2 = 0b1000
    Div3 = 0b0001
    Div4 = 0b1001
    Div5 = 0b0010
    Div6 = 0b1010
    Div7 = 0b0011
    Div8 = 0b1011
    Div16 = 0b1100
    Div32 = 0b1101
    Div64 = 0b1110
    Div128 = 0b1111

  APBPrescaler* = enum
    APB_Div1 = 0b000
    APB_Div2 = 0b100
    APB_Div4 = 0b101
    APB_Div8 = 0b110
    APB_Div16 = 0b111

# special addresses
const
  CFG0_PLL_TRIM* = 0x1FFFF7D4'u32
  FLASH_SIZE_REG* = 0x1FFFF7E0'u32
  DEVICE_ID* = 0x1FFFF7E8'u32
