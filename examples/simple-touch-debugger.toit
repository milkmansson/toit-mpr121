// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import ..src.mpr121 as mpr121

/**
Example starts I2C and connects to the MPR121.  Device starts a debugging task
which logs to the debug the channel being activated/touched.  Use `jag monitor`
to see the output.  Note that
- channel 1 is on the LEFT of the device, but the LSB of the binary output
  (eg channel 1) is on the RIGHT of the output.
- if several channels are recieving touch, they will all show in the output
  (eg, where output looks like 0000.0000.0000.1110) touch is occurring on three
  channels, 2, 3, and 4, simultaneously.)
- In practice, alerts are generated for both the press, and the release.  If
  an LED is connected to the alert pin, it will flash twice for each
  touch/release cycle.

Example output:
```
[jaguar] INFO: program 8d1036b7-08ea-41af-0bfa-ca32a1cc0ab0 started
[mpr121] DEBUG: soft-reset: Wrote SOFT-RESET-REGISTER-ADDRESS_ with 0x0063 [0110.0011]
[mpr121] DEBUG: initialise-device_: Wrote ECR-REGISTER-ADDRESS_[5a] with 0x8c [1000.1100]
[mpr121] INFO: debug-touched: task is set
[mpr121] DEBUG: debug-touched: 0000.0000.0000.0001
[mpr121] DEBUG: debug-touched: 0000.0000.0000.0011
[mpr121] DEBUG: debug-touched: 0000.0000.0000.0011
[mpr121] DEBUG: debug-touched: 0000.0000.0000.0111
[mpr121] DEBUG: debug-touched: 0000.0000.0000.1110
[mpr121] DEBUG: debug-touched: 0000.0000.0001.1100
[mpr121] DEBUG: debug-touched: 0000.0000.0011.1100
[mpr121] DEBUG: debug-touched: 0000.0000.0011.1000
[mpr121] DEBUG: debug-touched: 0000.0000.0111.1000
[mpr121] DEBUG: debug-touched: 0000.0000.1111.0000
[mpr121] DEBUG: debug-touched: 0000.0000.1110.0000
[mpr121] DEBUG: debug-touched: 0000.0001.1100.0000
[mpr121] DEBUG: debug-touched: 0000.0011.1100.0000
[mpr121] DEBUG: debug-touched: 0000.0011.1100.0000
[mpr121] DEBUG: debug-touched: 0000.0111.1000.0000
[mpr121] DEBUG: debug-touched: 0000.1111.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1111.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1000.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1000.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1100.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.1110.0000.0000
[mpr121] DEBUG: debug-touched: 0000.0111.0000.0000
[mpr121] DEBUG: debug-touched: 0000.0011.1000.0000
[mpr121] DEBUG: debug-touched: 0000.0011.1100.0000
[mpr121] DEBUG: debug-touched: 0000.0001.1100.0000
[mpr121] DEBUG: debug-touched: 0000.0001.1100.0000
[mpr121] DEBUG: debug-touched: 0000.0000.1110.0000
[mpr121] DEBUG: debug-touched: 0000.0000.0111.0000
[mpr121] DEBUG: debug-touched: 0000.0000.0111.0000
[mpr121] DEBUG: debug-touched: 0000.0000.0011.1000
[mpr121] DEBUG: debug-touched: 0000.0000.0001.1100
[mpr121] DEBUG: debug-touched: 0000.0000.0001.1100
[mpr121] DEBUG: debug-touched: 0000.0000.0000.1100
[mpr121] DEBUG: debug-touched: 0000.0000.0000.1110
[mpr121] DEBUG: debug-touched: 0000.0000.0000.0111
[mpr121] DEBUG: debug-touched: 0000.0000.0000.0111
[mpr121] DEBUG: debug-touched: 0000.0000.0000.0111
[mpr121] DEBUG: debug-touched: 0000.0000.0000.0001
```
*/

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
