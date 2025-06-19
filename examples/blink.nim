# Blink LED example for CH32V003
# LED connected to PC5

import system/system_ch32v00x
import hal/hw_gpio

proc NimMain() {.importc.}

proc delay_ms(ms: uint32) =
  for i in 0..<ms:
    for j in 0..<7500:
      asm "nop"

proc main() {.exportc, cdecl.} =
  NimMain()

  # Initialize system to 48MHz
  system_init()

  # Configure LED pin
  let led = GpioPin.init(PortC, 5)
  led.as_output(Output10MHz)

  # Blink forever
  while true:
    led.high()
    delay_ms(250)
    led.low()
    delay_ms(250)
