proc nimPanic(msg: string) {.noconv, noreturn.} =
  while true:
    discard

proc panic*(s: string) {.noconv, noreturn, noinline.} =
  # this is to stub out an internal 'panic' that fatal.nim might be trying to call
  # behavior should be same as nimPanic
  while true:
    discard

proc rawoutput*(s: string) {.noconv, noinline.} =
  discard 
