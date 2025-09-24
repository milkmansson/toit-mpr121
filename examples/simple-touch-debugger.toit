
// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.
import gpio
import i2c
import ..src.mpr121 as mpr121

main:
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
    mpr121-driver := mpr121.Mpr121 mpr121-device

    // Enable simple touch debugging
    mpr121-driver.debug-touched
