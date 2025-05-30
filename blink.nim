{.compile: "c_src/system_ch32v00x.c".}
{.compile: "c_src/core_riscv.c".}
{.compile: "c_src/nim_interface.c".}

proc initLed*() {.importc: "NimHelper_PC5_Output_Init", header: "nim_helpers.h".}
proc setLed*(high: cint) {.importc: "NimHelper_PC5_Write", header: "nim_helpers.h".}
proc delayMs*(ms: uint32) {.importc: "NimHelper_DelayMs", header: "nim_helpers.h".}

proc NimMain() {.importc.}

proc main() {.exportc, cdecl.} =
  NimMain()  # Initialize minimal Nim runtime
  initLed()
  while true:
    setLed(1)   # LED on
    delayMs(250)
    setLed(0)   # LED off
    delayMs(250)