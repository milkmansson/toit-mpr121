// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import mpr121 show *

/**
Example showing the MPR121 GPIO pins, used as an LED output and a button input.

Electrodes are always enabled from ELE0 upwards, so to free pins ELE4..ELE11 for
  GPIO the number of touch electrodes is first reduced to 4 (ELE0..ELE3).  Only
  the pins that are not electrodes can then be configured.

Wiring (MPR121 pin numbers):
- LED from pin 11 through a resistor to ground.  The high-side only open-drain
  mode drives the pin high when set to 1 and leaves it floating when set to 0.
  Check the wiring and current limits against AN3894 and the datasheet before
  connecting an LED.
- Push button between pin 10 and ground.  The internal pull-up is used.

Every operation is an I2C transaction, so GPIO on this device is only suited to
  slow signals, as the README explains.
*/

// Please set these correctly for your device:
SDA-PIN := 8
SCL-PIN := 9
FREQUENCY := 400_000
I2C-ADDRESS := 0x5a

main:
  sda-pin := gpio.Pin SDA-PIN
  scl-pin := gpio.Pin SCL-PIN
  bus := i2c.Bus --sda=sda-pin --scl=scl-pin --frequency=FREQUENCY

  if not bus.scan.contains I2C-ADDRESS:
    print "No MPR121 device found [0x$(%02x I2C-ADDRESS)]"
    return

  driver := Mpr121 (bus.device I2C-ADDRESS)

  // Keep ELE0..ELE3 for touch; frees ELE4..ELE11.
  driver.touch-pins-enabled --number-of-pins=4

  led    := Mpr121Pin driver 11
  button := Mpr121Pin driver 10
  led.configure --output --led --value=0
  button.configure --input --pull-up

  20.repeat:
    // The button pulls the pin low when pressed.
    pressed := button.get == 0
    led.set (pressed ? 1 : 0)
    print "button $(pressed ? "pressed" : "released")"
    sleep --ms=250

  led.close
  button.close
