import device/ch32v00xxx
import device/ch32v003_constants
import hal/hw_rcc

# Clock configuration
const
  USE_HSI_8MHZ* = false
  USE_HSI_24MHZ* = false
  USE_HSI_48MHZ* = true
  USE_HSE_8MHZ* = false
  USE_HSE_24MHZ* = false
  USE_HSE_48MHZ* = false

const
  HSI_VALUE* = 24_000_000'u32
  HSE_VALUE* = 24_000_000'u32

var SystemCoreClock* {.exportc.}: uint32 =
  when USE_HSI_8MHZ: 8_000_000'u32
  elif USE_HSI_24MHZ: HSI_VALUE
  elif USE_HSI_48MHZ: 48_000_000'u32
  elif USE_HSE_8MHZ: 8_000_000'u32
  elif USE_HSE_24MHZ: HSE_VALUE
  elif USE_HSE_48MHZ: 48_000_000'u32
  else: HSI_VALUE

const AHBPrescTable: array[16, uint8] = [1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5,
    6, 7, 8]

proc set_sys_clock_to_8mhz_hsi() =
  # set flash latency to 0 wait states using the modifyIt template
  FLASH.ACTLR.modifyIt:
    # `it` is injected by the template and holds the current register value.
    # We perform the read-modify-write logic on it.
    var regVal = it.uint32
    regVal = (regVal and 0xFFFFFFF8'u32) or uint32(Latency0)
    it = FLASH_ACTLR_Fields(regVal)

  set_ahb_prescaler(Div3)
proc set_sys_clock_to_24mhz_hsi() =
  # set flash latency to 0 wait states
  FLASH.ACTLR.modifyIt:
    it = FLASH_ACTLR_Fields( (it.uint32 and not 0x7'u32) or Latency0.uint32)

  set_ahb_prescaler(Div1)

proc set_sys_clock_to_48mhz_hsi() =
  # set flash latency to 1 wait state
  FLASH.ACTLR.modifyIt:
    it = FLASH_ACTLR_Fields( (it.uint32 and not 0x7'u32) or Latency1.uint32)

  set_ahb_prescaler(Div1)
  set_pll_source(HSI_Div2)

  RCC.CTLR.modifyIt:
    it.PLLON = true

  wait_pll_ready()
  set_system_clock(PLL)

  while get_system_clock_source() != PLL:
    discard

proc set_sys_clock_to_48mhz_hse() =
  # disable PA0-PA1 gpio for crystal
  RCC.APB2PCENR.modifyIt:
    it.AFIOEN = true

  AFIO.PCFR.modifyIt:
    it.PA12RM = true

  RCC.CTLR.modifyIt:
    it.HSEON = true

  if not wait_hse_ready():
    return

  # set flash latency to 1 wait state
  FLASH.ACTLR.modifyIt:
    it = FLASH_ACTLR_Fields( (it.uint32 and not 0x7'u32) or Latency1.uint32)

  set_ahb_prescaler(Div1)
  set_pll_source(HSE)

  RCC.CTLR.modifyIt:
    it.PLLON = true

  wait_pll_ready()
  set_system_clock(PLL)

  while get_system_clock_source() != PLL:
    discard

proc set_sys_clock() =
  when USE_HSI_8MHZ:
    set_sys_clock_to_8mhz_hsi()
  elif USE_HSI_24MHZ:
    set_sys_clock_to_24mhz_hsi()
  elif USE_HSI_48MHZ:
    set_sys_clock_to_48mhz_hsi()
  elif USE_HSE_8MHZ:
    discard
  elif USE_HSE_24MHZ:
    discard
  elif USE_HSE_48MHZ:
    set_sys_clock_to_48mhz_hse()

proc system_init*() {.exportc.} =
  # enable hsi
  RCC.CTLR.modifyIt:
    it.HSION = true

  # reset cfgr0
  var cfgr0 = RCC.CFGR0.read()
  cfgr0 = RCC_CFGR0_Fields(uint32(cfgr0) and 0xF8FF0000'u32)
  RCC.CFGR0.write(cfgr0)

  # reset hseon, csson, pllon
  var ctlr = RCC.CTLR.read()
  ctlr = RCC_CTLR_Fields(uint32(ctlr) and 0xFEF6FFFF'u32)
  RCC.CTLR.write(ctlr)

  # reset hsebyp
  ctlr = RCC.CTLR.read()
  ctlr = RCC_CTLR_Fields(uint32(ctlr) and 0xFFFBFFFF'u32)
  RCC.CTLR.write(ctlr)

  # reset pll bits
  cfgr0 = RCC.CFGR0.read()
  cfgr0 = RCC_CFGR0_Fields(uint32(cfgr0) and 0xFFFEFFFF'u32)
  RCC.CFGR0.write(cfgr0)

  # clear interrupts
  RCC.INTR.write(0x009F0000)

  set_sys_clock()
proc system_core_clock_update*() {.exportc.} =
  case get_system_clock_source()
  of HSI:
    SystemCoreClock = HSI_VALUE
  of HSE:
    SystemCoreClock = HSE_VALUE
  of PLL:
    if RCC.CFGR0.read().PLLSRC:
      SystemCoreClock = HSE_VALUE * 2
    else:
      SystemCoreClock = HSI_VALUE * 2

  let prescalerIdx = (RCC.CFGR0.read().HPRE and 0xF)
  let prescaler = AHBPrescTable[prescalerIdx]

  if prescalerIdx < 8:
    SystemCoreClock = SystemCoreClock div prescaler
  else:
    SystemCoreClock = SystemCoreClock shr prescaler
