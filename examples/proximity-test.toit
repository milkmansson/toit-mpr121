// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import log
import mpr121 as mpr121

/**
Example starts I2C and connects to the MPR121.  Device starts a debugging task
  which logs to the debug the channel being activated/touched.  Use `jag
  monitor` to see the output.

In this example, output and function will be similar to 'simple-touch-debugger',
  however, example code is given for the proximity function.  Note that only one
  proximity channel exists, and it is constructed using channels in fixed groups
  of 0..1, 0..3, or 0..11, configured using statics 'PROXIMITY-MODE-COMBINE-*''

This example also sets the logging level lower.

*/

main:
  // Establish Log.
  logger := log.default.with-name "proximity-test"

  // To enable DEBUG level.
  logger.with-level log.DEBUG-LEVEL

  // To enable INFO level.
  //logger.with-level log.INFO-LEVEL

  sda-pin-number := 26    // please set these correctly for your device
  scl-pin-number := 25    // please set these correctly for your device

  // Enable and drive I2C:
  frequency := 400_000
  sda-pin := gpio.Pin sda-pin-number
  scl-pin := gpio.Pin scl-pin-number
  bus := i2c.Bus --sda=sda-pin --scl=scl-pin --frequency=frequency
  scandevices := bus.scan

  if not scandevices.contains mpr121.Mpr121.I2C-ADDRESS:
    print "No MPR121 device found [0x$(%02x  mpr121.Mpr121.I2C-ADDRESS)]"
  else:
    mpr121-device := bus.device mpr121.Mpr121.I2C-ADDRESS
    mpr121-driver := mpr121.Mpr121 mpr121-device --logger=logger

    // Enable simple touch debugging
    mpr121-driver.debug-touched
