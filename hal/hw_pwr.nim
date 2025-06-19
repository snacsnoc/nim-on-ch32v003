import device/ch32v00xxx

type
  PvdThreshold* = enum
    Pvd2v9 = 0b000 # 2.9V
    Pvd3v1 = 0b001
    Pvd3v3 = 0b010
    Pvd3v5 = 0b011
    Pvd3v7 = 0b100
    Pvd3v9 = 0b101
    Pvd4v1 = 0b110
    Pvd4v3 = 0b111

proc set_pvd_threshold*(threshold: PvdThreshold) =
  PWR.CTLR.modifyIt:
    it.PLS = threshold.uint32

proc enable_pvd*() =
  PWR.CTLR.modifyIt:
    it.PVDE = true

proc disable_pvd*() =
  PWR.CTLR.modifyIt:
    it.PVDE = false

proc is_power_good*(): bool =
  not PWR.CSR.read().PVDO

# Standby mode
proc enter_standby*() =
  PWR.CTLR.modifyIt:
    it.PDDS = true
  # Set sleepdeep
  asm """
    li t0, 0x00000004
    csrs 0x804, t0
    wfi
  """

# Auto wakeup
proc set_auto_wakeup*(enable: bool, prescaler: uint32 = 0,
    reload: uint32 = 63) =
  PWR.AWUCSR.modifyIt:
    it.AWUEN = enable
  if enable:
    PWR.AWUPSC.write(AWUPSC = prescaler and 0xF)
    PWR.AWUAPR.write(AWUAPR = reload and 0x3F)
