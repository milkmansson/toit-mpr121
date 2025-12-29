# Toit Driver library for the MPR121 Touch/Proximity sensor
This is a Toit driver for the MPR121 is a capacitive touch sensor controller,
made by NXP Semiconductors.  It detects when a finger (or any conductive object)
approaches or touches one of the pads.

> [!WARNING]
> This driver is functional but still under development.

![Front and back of an as5600](images/mpr121.jpg)

## Quickstart
For a quick overview and to get started, see the [examples](./examples/)

## Features
The MPR121 uses a constant DC current capacitance sensing scheme. It can measure
capacitances ranging from 10 pF to over 2000 pF with a resolution up to 0.01 pF.
The device does this by varying the amount of charge current and charge time
applied to the sensing inputs.

### Package Features
- Most functions specified in the datasheet are featured in the driver, `Mpr121`.
- 'Callback'/Event handler feature has been added, `Mpr121Events`.

### Not Yet Implemented
- Proximity feature hasn't been implemented in `Mpr121Events`
- GPIO function has not been implemented.
- LED driver function has not yet been implemented.

## Examples

### Starting the Driver
A starter pattern for the driver is shown in [this example](./examples/simple-touch-debugger.toit).

### Touch Debugger
After getting started, to simply see that something is being sensed/recognised
by the driver, the interrupt pin could be wired to an LED and used.  In addition, the driver also has a rudimentary feature to print the binary mask of what is being touched, shown in [the example](./examples/simple-touch-debugger.toit).  This example does not require an interrupt pin be configured.
```Toit
// I2C setup omitted.

// Start touch debugger.
mpr121-driver.debug-touched

// Stop touch debugger.
mpr121-driver.stop-all-tasks
```

### Event Handler
An event handler class has been written such that a Toit [Lambda](https://docs.toit.io/language/blocks-and-lambdas) can be assigned to a button.  The lambda will be executed on touch (or release) of the chosen sensor channel.  This version requires the use of a wired interrupt pin (pin 18 shown in this example) - similar operation to the debug mechanism is possible, but not implemented in this code.
```Toit
// I2C setup Omitted (see files in ./examples/).

// Start the MPR121 device/driver:
mpr121-device  := bus.device Mpr121.I2C-ADDRESS
mpr121-driver  := Mpr121 mpr121-device --logger=logger

// Start the event driver, supplying the mpr121-driver:
event-driver = Mpr121Events mpr121-driver --intrpt-pin=(gpio.Pin 18)

mpr121-driver.on-press Mpr121Events.CHANNEL-03 --callback=(:: print "touch channel 03")
mpr121-driver.on-release Mpr121Events.CHANNEL-04 --callback=(:: print "touch channel 04")

// Remove tasks:
mpr121-driver.remove Mpr121Events.CHANNEL-03

```
Note: _Assigning Lambdas to a combinations of channel/touches, is not implemented
yet._

## Issues
If there are any issues, changes, or any other kind of feedback, please
[raise an issue](toit-mpr121/issues). Feedback is welcome and appreciated!

## Disclaimer
- This driver has been written and tested with an unbranded INA226 module.
- All trademarks belong to their respective owners.
- No warranties for this work, express or implied.

## Credits
- Several works were pivotal in understanding the device and how it works.
  - [Janelia Arduino Driver](https://github.com/janelia-arduino/MPR121)
  - [Bare Conductive Arduino Driver](https://github.com/BareConductive/mpr121)
  - [Adafruit Driver](https://github.com/adafruit/Adafruit_MPR121/)
- [Florian](https://github.com/floitsch) for the tireless help and encouragement
- The wider Toit developer team (past and present) for a truly excellent product
- AI has been used for code and text reviews, analysing and compiling data and
  results, and assisting with ensuring accuracy.

## About Toit
One would assume you are here because you know what Toit is.  If you dont:
> Toit is a high-level, memory-safe language, with container/VM technology built
> specifically for microcontrollers (not a desktop language port). It gives fast
> iteration (live reloads over Wi-Fi in seconds), robust serviceability, and
> performance that’s far closer to C than typical scripting options on the
> ESP32. [[link](https://toitlang.org/)]
- [Review on Soracom](https://soracom.io/blog/internet-of-microcontrollers-made-easy-with-toit-x-soracom/)
- [Review on eeJournal](https://www.eejournal.com/article/its-time-to-get-toit)
