import device/ch32v00xxx
import device/ch32v003_constants

type
  ClockSource* = enum
    HSI = 0b00
    HSE = 0b01
    PLL = 0b10

# GPIO clocks
proc enable_gpio_a*() = RCC.APB2PCENR.modifyIt: it.IOPAEN = true
proc enable_gpio_c*() = RCC.APB2PCENR.modifyIt: it.IOPCEN = true
proc enable_gpio_d*() = RCC.APB2PCENR.modifyIt: it.IOPDEN = true

# APB2 peripherals
proc enable_afio*() = RCC.APB2PCENR.modifyIt: it.AFIOEN = true
proc enable_adc1*() = RCC.APB2PCENR.modifyIt: it.ADC1EN = true
proc enable_tim1*() = RCC.APB2PCENR.modifyIt: it.TIM1EN = true
proc enable_spi1*() = RCC.APB2PCENR.modifyIt: it.SPI1EN = true
proc enable_usart1*() = RCC.APB2PCENR.modifyIt: it.USART1EN = true

# APB1 peripherals
proc enable_tim2*() = RCC.APB1PCENR.modifyIt: it.TIM2EN = true
proc enable_wwdg*() = RCC.APB1PCENR.modifyIt: it.WWDGEN = true
proc enable_i2c1*() = RCC.APB1PCENR.modifyIt: it.I2C1EN = true
proc enable_pwr*() = RCC.APB1PCENR.modifyIt: it.PWREN = true

# AHB peripherals
proc enable_dma1*() = RCC.AHBPCENR.modifyIt: it.DMA1EN = true

# Clock status
proc get_system_clock_source*(): ClockSource =
  ClockSource(RCC.CFGR0.read().SWS)

proc is_pll_ready*(): bool = RCC.CTLR.read().PLLRDY
proc is_hse_ready*(): bool = RCC.CTLR.read().HSERDY
proc is_hsi_ready*(): bool = RCC.CTLR.read().HSIRDY

# Resets
proc reset_i2c1*() =
  RCC.APB1PRSTR.modifyIt: it.I2C1RST = true
  RCC.APB1PRSTR.modifyIt: it.I2C1RST = false

proc reset_usart1*() =
  RCC.APB2PRSTR.modifyIt: it.USART1RST = true
  RCC.APB2PRSTR.modifyIt: it.USART1RST = false

proc reset_tim1*() =
  RCC.APB2PRSTR.modifyIt: it.TIM1RST = true
  RCC.APB2PRSTR.modifyIt: it.TIM1RST = false

proc reset_tim2*() =
  # TIM2RST is bit 0 of APB1PRSTR. The SVD file is missing this field,
  # so we have to write the bit manually
  const TIM2RST_BIT = 1'u32 shl 0
  RCC.APB1PRSTR.modifyIt:
    var bits = it.uint32
    it = (bits or TIM2RST_BIT).RCC_APB1PRSTR_Fields
  RCC.APB1PRSTR.modifyIt:
    var bits = it.uint32
    it = (bits and not TIM2RST_BIT).RCC_APB1PRSTR_Fields


# System clock configuration
proc set_pll_source*(source: PLLSource) =
  RCC.CFGR0.modifyIt:
    it.PLLSRC = source.bool

proc set_system_clock*(source: ClockSource) =
  RCC.CFGR0.modifyIt:
    it.SW = source.uint32

proc set_ahb_prescaler*(prescaler: AHBPrescaler) =
  RCC.CFGR0.modifyIt:
    it.HPRE = prescaler.uint32

proc set_apb1_prescaler*(prescaler: APBPrescaler) =
  RCC.CFGR0.modifyIt:
    it.PPRE1 = prescaler.uint32

proc set_apb2_prescaler*(prescaler: APBPrescaler) =
  RCC.CFGR0.modifyIt:
    it.PPRE2 = prescaler.uint32

proc wait_pll_ready*() =
  while not RCC.CTLR.read().PLLRDY:
    discard

proc wait_hse_ready*(): bool =
  var cnt = 0'u32
  while cnt < 0x2000:
    if RCC.CTLR.read().HSERDY:
      return true
    inc cnt
  false
