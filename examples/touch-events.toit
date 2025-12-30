// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import log
import mpr121 show *

/**
Example starts I2C and connects to the MPR121.  Device starts a debugging task
  which logs to the debug the channel being activated/touched.  Use `jag
  monitor` to see the output.

In this example, output and function will be similar to 'simple-touch-debugger',
  however, example code is given for the proximity function.  Note that only one
  proximity channel exists, and it is constructed using channels in fixed groups
  of 0..1, 0..3, or 0..11, configured using statics 'PROXIMITY-MODE-COMBINE-*''

This example also shows how to set the desired project logging level.

Note that for this example, the 13th bit is set *ALONGSIDE* the pins to be
combined to the proximity sensor.  Also note that if debugging is enabled, all
pin triggers are displayed, but only the touch event for Channel 02 executes the
indicated lambda function, as an example:

```
[mpr121.events] DEBUG: interrupt tripped {touched:             0, released:  111100000111}
[mpr121.events] DEBUG: interrupt tripped {touched: 1110100000111, released:             0}
CHANNEL-02 touched.
[mpr121.events] DEBUG: interrupt tripped {touched:          1000, released:             0}
[mpr121.events] DEBUG: interrupt tripped {touched:             0, released: 1110100001010}
[mpr121.events] DEBUG: interrupt tripped {touched:             0, released:           101}
[mpr121.events] DEBUG: interrupt tripped {touched:  110000000000, released:             0}
[mpr121.events] DEBUG: interrupt tripped {touched:           111, released:             0}
CHANNEL-02 touched.
[mpr121.events] DEBUG: interrupt tripped {touched:             0, released:  110000000000}
[mpr121.events] DEBUG: interrupt tripped {touched:             0, released:           111}
[mpr121.events] DEBUG: interrupt tripped {touched:  111000000101, released:             0}
```
*/

// Please set these correctly for your device:
SDA-PIN := 8
SCL-PIN := 9
FREQUENCY := 400_000
I2C-ADDRESS := 0x5a
INTERRUPT-PIN := 4

main:
  // Establish Log.
  logger := log.default.with-name "callback-test"

  // To enable DEBUG level.
  logger.with-level log.DEBUG-LEVEL

  // To enable INFO level.
  //logger.with-level log.INFO-LEVEL

  // Enable and drive I2C:
  print "Opening I2C..."
  sda-pin := gpio.Pin SDA-PIN
  scl-pin := gpio.Pin SCL-PIN
  bus := i2c.Bus --sda=sda-pin --scl=scl-pin --frequency=FREQUENCY
  scandevices := bus.scan

  // Check its there first before trying to use it:
  if not scandevices.contains I2C-ADDRESS:
    print "No MPR121 device found [0x$(%02x I2C-ADDRESS)]"
  else:
    mpr121-device := bus.device I2C-ADDRESS
    mpr121-driver := Mpr121 mpr121-device

    // Set Proximity for combining channels 0-3
    mpr121-driver.proximity-mode Mpr121.PROXIMITY-MODE-COMBINE-0-TO-3

    // Set some rudimentary touch events - display text if a button is touched.
    mpr121-events := Mpr121Events mpr121-driver --intrpt-pin=(gpio.Pin INTERRUPT-PIN)

    logger.warn "INTERRUPT-PIN $INTERRUPT-PIN needs to be tied to INT on MPR121."

    // Attach event to the touch of the pin
    mpr121-events.on-touch Mpr121Events.CHANNEL-02 --callback=(:: print "CHANNEL-02 touched.")
