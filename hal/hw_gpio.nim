import device/ch32v00xxx

type
  Port* = enum
    PortA = 0
    PortC = 1
    PortD = 2

  Pin* = range[0..7]

  PinMode* = enum
    Input = 0b00
    Output10MHz = 0b01
    Output2MHz = 0b10
    Output50MHz = 0b11

  PinConfig* = enum
    # Input modes (CNF bits when MODE is Input)
    AnalogIn = 0b00
    FloatingIn = 0b01
    PullUpDown = 0b10
    # Output modes (CNF bits when MODE is Output)
    OutputPushPull = 0b100 # Start from a unique value (4)
    OutputOpenDrain = 0b101
    OutputAltPushPull = 0b110
    OutputAltOpenDrain = 0b111

  PinState* = enum
    Low = false
    High = true

# Get GPIO peripheral pointer based on port
proc get_gpio_port(port: Port): ptr GPIOA_Type =
  case port
  of PortA: return unsafeAddr GPIOA
  of PortC: return unsafeAddr GPIOC
  of PortD: return unsafeAddr GPIOD

# Enable clock for GPIO port
proc enable_clock*(port: Port) =
  RCC.APB2PCENR.modifyIt:
    case port
    of PortA: it.IOPAEN = true
    of PortC: it.IOPCEN = true
    of PortD: it.IOPDEN = true

# Configure a single pin
proc configure_pin*(port: Port, pin: Pin, mode: PinMode, config: PinConfig) =
  let gpio = get_gpio_port(port)

  # Each pin uses 4 bits in CFGLR (pins 0-7)
  let shift = pin.uint32 * 4
  let mask = 0xF'u32 shl shift

  # We subtract 4 to get the actual 2-bit CNF value for output modes
  var cnf_val = config.uint32
  if mode != Input and cnf_val >= 4:
    cnf_val -= 4

  let value = (mode.uint32 or (cnf_val shl 2)) shl shift

  # Read-modify-write the configuration register
  gpio.CFGLR.modifyIt:
    var bits = it.uint32
    bits = (bits and not mask) or value
    it = bits.GPIOA_CFGLR_Fields

# Write to a pin
proc write_pin*(port: Port, pin: Pin, state: PinState) =
  let gpio = get_gpio_port(port)
  if state == High:
    gpio.BSHR.write((1'u32 shl pin).GPIOA_BSHR_Fields) # Set bit
  else:
    gpio.BSHR.write((1'u32 shl (pin + 16)).GPIOA_BSHR_Fields) # Reset bit

# Read from a pin
proc read_pin*(port: Port, pin: Pin): PinState =
  let gpio = get_gpio_port(port)
  let indr = gpio.INDR.read()
  result = if (indr.uint32 and (1'u32 shl pin)) != 0: High else: Low

# Toggle a pin specific pin's bit using XOR
proc toggle_pin*(port: Port, pin: Pin) =
  let gpio = get_gpio_port(port)

  gpio.OUTDR.modifyIt:
    var bits = it.uint32
    bits = bits xor (1'u32 shl pin)
    it = bits.GPIOA_OUTDR_Fields

# Configure pin as output push-pull
proc set_pin_as_output*(port: Port, pin: Pin, speed: PinMode = Output10MHz) =
  configure_pin(port, pin, speed, OutputPushPull)

# Configure pin as input
proc set_pin_as_input*(port: Port, pin: Pin, pullup: bool = false) =
  if pullup:
    configure_pin(port, pin, Input, PullUpDown)
    # Set ODR bit to enable pull-up
    let gpio = get_gpio_port(port)
    gpio.OUTDR.modifyIt:
      case pin
      of 0: it.ODR0 = true
      of 1: it.ODR1 = true
      of 2: it.ODR2 = true
      of 3: it.ODR3 = true
      of 4: it.ODR4 = true
      of 5: it.ODR5 = true
      of 6: it.ODR6 = true
      of 7: it.ODR7 = true
  else:
    configure_pin(port, pin, Input, FloatingIn)

type
  GpioPin* = object
    port*: Port
    pin*: Pin

proc init*(T: type GpioPin, port: Port, pin: Pin): GpioPin =
  GpioPin(port: port, pin: pin)

proc as_output*(gpio: GpioPin, speed: PinMode = Output10MHz) =
  gpio.port.enable_clock()
  gpio.port.set_pin_as_output(gpio.pin, speed)

proc as_input*(gpio: GpioPin, pullup: bool = false) =
  gpio.port.enable_clock()
  gpio.port.set_pin_as_input(gpio.pin, pullup)

proc write*(gpio: GpioPin, state: PinState) =
  gpio.port.write_pin(gpio.pin, state)

proc read*(gpio: GpioPin): PinState =
  gpio.port.readPin(gpio.pin)

proc toggle*(gpio: GpioPin) =
  gpio.port.toggle_pin(gpio.pin)

proc high*(gpio: GpioPin) = gpio.write(High)
proc low*(gpio: GpioPin) = gpio.write(Low)

proc `<-`*(gpio: GpioPin, state: PinState) = gpio.write(state)
proc `<-`*(gpio: GpioPin, state: bool) = gpio.write(if state: High else: Low)
