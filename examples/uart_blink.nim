# UART + blink

import system/system_ch32v00x
import hal/hw_gpio
import device/ch32v00xxx

proc NimMain() {.importc.}

proc delay_ms(ms: uint32) =
  for i in 0..<ms:
    for j in 0..<7500:
      asm "nop"

proc initUsart1*(baud: uint32) =
  # enable clocks

  # for PD5/PD6
  enable_clock(PortD)
  # for our LED on PC5
  enable_clock(PortC)
  RCC.APB2PCENR.modifyIt:
    # alt-func
    it.AFIOEN = true
  RCC.APB2PCENR.modifyIt:
    it.USART1EN = true

  # PD6 = TX, PD5 = RX
  configure_pin(PortD, 6, Output50MHz, OutputAltPushPull)
  configure_pin(PortD, 5, Input, FloatingIn)

  # configure baud
  let usart = unsafeAddr USART1
  usart.BRR.write((SystemCoreClock div baud).uint32)

  # enable TE, RE, UE
  usart.CTLR1.modifyIt:
    it.TE = true
    it.RE = true
    it.UE = true

proc uartWrite*(c: char) =
  let usart = unsafeAddr USART1
  # wait TXE
  while not usart.STATR.read().TXE: discard
  usart.DATAR.write(c.uint32)

proc uartWrite*(s: string) =
  for c in s:
    uartWrite(c)

proc main() {.exportc, cdecl.} =
  system_init()
  initUsart1(115200)

  # setup LED on PC5
  configure_pin(PortC, 5, Output10MHz, OutputPushPull)

  var count = 0'u32
  while true:
    # blink
    write_pin(PortC, 5, High)
    delay_ms(250)
    write_pin(PortC, 5, Low)
    delay_ms(250)

    # send over UART
    uartWrite($"Count = {count}\r\n")
    inc count
