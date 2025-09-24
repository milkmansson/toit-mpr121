
// Copyright (C) 2025 Toit Contributors
// This file includes both original work, and derivative work based on:
//   - A - https://github.com/janelia-arduino, licensed under the BSD 3-Clause License.
//         Retrieved from: https://github.com/janelia-arduino/MPR121 
//   - B - https://github.com/BareConductive/mpr121, licensed under the MIT License.
//   - C - https://github.com/adafruit/Adafruit_MPR121/blob/master/Adafruit_MPR121.cpp
// See LICENSE-BSD and LICENSE-B-MIT for the original license texts.
//
// Significant influence from the following works:
//   - https://docs.cirkitdesigner.com/component/59d4ac7a-7613-4869-833a-07450eb89a32/ina3221
//
// Datasheet for this device:
//   - https://www.nxp.com/docs/en/data-sheet/MPR121.pdf
// In some places, text has been copied from this document directly to assist the user
// in understanding the function of this device.
//
// Use of this source code is governed by an MIT-style license that can be
// found in the package's LICENSE file.
//

import log
import binary
import serial.device as serial
import serial.registers as registers


DEFAULT-I2C-ADDRESS                     ::= 0x005a
DEFAULT-I2C-ADDRESS-TOUCH               ::= 0x005c
I2C-ADDRESS-5B                          ::= 0x005b
I2C-ADDRESS-5D                          ::= 0x005d
 
MPR121-PROXIMITY-MODE-DISABLED          := 0    // default set by this driver
MPR121-PROXIMITY-MODE-COMBINE_0_TO_1    := 1
MPR121-PROXIMITY-MODE-COMBINE_0_TO_3    := 2
MPR121-PROXIMITY-MODE-COMBINE_0_TO_11   := 3

MPR121-BASELINE-TRACKING-DISABLED       := 0x01
MPR121-BASELINE-TRACKING-INIT-0         := 0x00 // default in chipset
MPR121-BASELINE-TRACKING-INIT-5BIT      := 0x02
MPR121-BASELINE-TRACKING-INIT-10BIT     := 0x03

MPR121-CHARGE_DISCHARGE_TIME_DISABLED   := 0x00
MPR121-CHARGE_DISCHARGE_TIME_HALF_US    := 0x01 // default
MPR121-CHARGE_DISCHARGE_TIME_1US        := 0x02
MPR121-CHARGE_DISCHARGE_TIME_2US        := 0x03
MPR121-CHARGE_DISCHARGE_TIME_4US        := 0x04
MPR121-CHARGE_DISCHARGE_TIME_8US        := 0x05
MPR121-CHARGE_DISCHARGE_TIME_16US       := 0x06
MPR121-CHARGE_DISCHARGE_TIME_32US       := 0x07

MPR121-FIRST_FILTER_ITERATIONS_6        := 0x00 // default
MPR121-FIRST_FILTER_ITERATIONS_10       := 0x01
MPR121-FIRST_FILTER_ITERATIONS_18       := 0x02
MPR121-FIRST_FILTER_ITERATIONS_34       := 0x03

MPR121-SECOND_FILTER_ITERATIONS_4       := 0x00 // default
MPR121-SECOND_FILTER_ITERATIONS_6       := 0x01
MPR121-SECOND_FILTER_ITERATIONS_10      := 0x02
MPR121-SECOND_FILTER_ITERATIONS_18      := 0x03

/** Sample periods of the MPR121 - the time between capacitive readings. 
    Higher values consume less power, but are less responsive. */
MPR121-SAMPLE_PERIOD_1MS                := 0x00
MPR121-SAMPLE_PERIOD_2MS                := 0x01
MPR121-SAMPLE_PERIOD_4MS                := 0x02
MPR121-SAMPLE_PERIOD_8MS                := 0x03
MPR121-SAMPLE_PERIOD_16MS               := 0x04 // default
MPR121-SAMPLE_PERIOD_32MS               := 0x05
MPR121-SAMPLE_PERIOD_64MS               := 0x06
MPR121-SAMPLE_PERIOD_128MS              := 0x07


class Driver:
  // PUBLIC
  static DEVICE_COUNT_MAX_              ::= 4  // implied given the address limitation

  // Device can be configured between 0x58 and 0x5d
  static DEFAULT-I2C-ADDRESS            ::= 0x005a
  static DEFAULT-I2C-ADDRESS-TOUCH      ::= 0x005c
  static I2C-ADDRESS-5B                 ::= 0x005b
  static I2C-ADDRESS-5D                 ::= 0x005d
  static PHYSICAL_CHANNELS_PER_DEVICE_  ::= 12
  static CHANNELS_PER_DEVICE_           ::= 13
  // Attempting to build class to handle more than one device.  In cases where two are used, will allow one query to handle all devices which will be much harder without.

  // register addresses
  TOUCH-STATUS-REGISTER-ADDRESS_        := 0x00  // Low, high = 0x01
  OUT-OF-RANGE-STATUS-REGISTER-ADDRESS_ := 0x02
  FILTERED-DATA0-REGISTER-ADDRESS_      := 0x04  // Low, high = 0x05
  BASELINE-DATA0-REGISTER-ADDRESS_      := 0x1E

  // general electrode touch sense baseline filters
  // rising filter
  static MHDR-REGISTER-ADDRESS_         ::= 0x2B
  static NHDR-REGISTER-ADDRESS_         ::= 0x2C
  static NCLR-REGISTER-ADDRESS_         ::= 0x2D
  static FDLR-REGISTER-ADDRESS_         ::= 0x2E

  // falling filter
  static MHDF-REGISTER-ADDRESS_         ::= 0x2F
  static NHDF-REGISTER-ADDRESS_         ::= 0x30
  static NCLF-REGISTER-ADDRESS_         ::= 0x31
  static FDLF-REGISTER-ADDRESS_         ::= 0x32

  // touched filter
  static NHDT-REGISTER-ADDRESS_         ::= 0x33
  static NCLT-REGISTER-ADDRESS_         ::= 0x34
  static FDLT-REGISTER-ADDRESS_         ::= 0x35

  // proximity electrode touch sense baseline filters
  // rising filter
  static MHDPROXR-REGISTER-ADDRESS_     ::= 0x36
  static NHDPROXR-REGISTER-ADDRESS_     ::= 0x37
  static NCLPROXR-REGISTER-ADDRESS_     ::= 0x38
  static FDLPROXR-REGISTER-ADDRESS_     ::= 0x39

  // falling filter
  static MHDPROXF-REGISTER-ADDRESS_     ::= 0x3A
  static NHDPROXF-REGISTER-ADDRESS_     ::= 0x3B
  static NCLPROXF-REGISTER-ADDRESS_     ::= 0x3C
  static FDLPROXF-REGISTER-ADDRESS_     ::= 0x3D

  // touched filter
  static NHDPROXT-REGISTER-ADDRESS_     ::= 0x3E
  static NCLPROXT-REGISTER-ADDRESS_     ::= 0x3F
  static FDLPROXT-REGISTER-ADDRESS_     ::= 0x40

  // electrode touch and release thresholds
  static TOUCH-THRESHOLD0-REGISTER-ADDRESS_    ::= 0x41
  static RELEASE-THRESHOLD0-REGISTER-ADDRESS_  ::= 0x42
  static TOUCH-THRESHOLD-DEFAULT_              ::= 40
  static RELEASE-THRESHOLD-DEFAULT_            ::= 20

  // debounce settings
  static DEBOUNCE-REGISTER-ADDRESS_            ::= 0x5B
  static DEBOUNCE-REGISTER-RELEASE-MASK_       ::= 0b0111_0000
  static DEBOUNCE-REGISTER-RELEASE-OFFSET_     ::= 4
  static DEBOUNCE-REGISTER-TOUCH-MASK_         ::= 0b0000_0111
  static DEBOUNCE-REGISTER-TOUCH-OFFSET_       ::= 0

  // channel charge-discharge CURRENTS
  static CDC0-REGISTER-ADDRESS_                ::= 0x5F
  static CHARGE-DISCHARGE-CURRENT-MIN_         ::= 0     // 0 = defer to global
  static CHARGE-DISCHARGE-CURRENT-MAX_         ::= 63

  // channel charge-discharge TIMES
  static CDT0-REGISTER-ADDRESS_                ::= 0x6C
  static CDT-CHARGE-TIME-MASK_                 ::= 0x07
  static CDT-FIELD-DIVISOR_                    ::= 2
  static CDT-FIELD-OFFSET_                     ::= 4

  // GPIO
  static GPIO-CTL0-REGISTER-ADDRESS_           ::= 0x73
  static GPIO-CTL1-REGISTER-ADDRESS_           ::= 0x74
  static GPIO-DAT-REGISTER-ADDRESS_            ::= 0x75
  static GPIO-DIR-REGISTER-ADDRESS_            ::= 0x76
  static GPIO-EN-REGISTER-ADDRESS_             ::= 0x77
  static GPIO-SET-REGISTER-ADDRESS_            ::= 0x78
  static GPIO-CLR-REGISTER-ADDRESS_            ::= 0x79
  static GPIO-TOG-REGISTER-ADDRESS_            ::= 0x7A

  // Configuration registers
  static FILTER-GLOBAL-CDC-REGISTER-ADDRESS_   ::= 0x5C
  static FILTER-GLOBAL-CDC-REGISTER-DEFAULT_   ::= 0b0001_0000
  static FILTER-GLOBAL-CDT-REGISTER-ADDRESS_   ::= 0x5D
  static FILTER-GLOBAL-CDT-REGISTER-DEFAULT_   ::= 0b0010_0100

  //auto-config
  static ACCR0-REGISTER-ADDRESS_               ::= 0x7B
  static ACCR1-REGISTER-ADDRESS_               ::= 0x7C
  static USL-REGISTER-ADDRESS_                 ::= 0x7D  // Upper Limit
  static LSL-REGISTER-ADDRESS_                 ::= 0x7E  // Lower Limit
  static TL-REGISTER-ADDRESS_                  ::= 0x7F  // Target Limit

  //soft reset
  static SOFT-RESET-REGISTER-ADDRESS_          ::= 0x80
  static SOFT-RESET-VALUE_                     ::= 0x63

  //PWM
  //static PWM0_                                 ::= 0x81
  //static PWM1_                                 ::= 0x82
  //static PWM2_                                 ::= 0x83
  //static PWM3_                                 ::= 0x84

  //ECR 
  static ECR-REGISTER-ADDRESS_                  ::= 0x5E
  static ECR-ELECTRODE-MASK_                    ::= 0b0000_1111
  static ECR-ELECTRODE-OFFSET_                  ::= 0
  static ECR-PROXIMITY-MASK_                    ::= 0b0011_0000
  static ECR-PROXIMITY-OFFSET_                  ::= 4
  static ECR-CONFIGURATION-LOCK-MASK_           ::= 0b1100_0000
  static ECR-CONFIGURATION-LOCK-OFFSET_         ::= 6

  static DEVICE-INDEX-NONE_                     ::= -1
  static BASELINE-DATA-BIT-SHIFT_               ::= 2

  //Something funky here:
  static BITS-PER-BYTE                         ::= 8
  static BITS-PER-TWO-BYTES                    ::= 16
  static ONE-BYTE-MAX                          ::= 0xFF
  static TWO-BYTE-MAX                          ::= 0xFFFF

  //Other Statics
  static OVER-CURRENT-REXT_                    ::= 0b1000_0000_0000_0000
  static TOUCH-STATUS-MASK-BASE_               ::= 0x1FFF
  static OUT-OF-RANGE-STATUS-ACFF_             ::= 0x8000
  static OUT-OF-RANGE-STATUS-ARFF_             ::= 0x4000


  // Baseline Configuration
  static BASELINE-CONFIGURATION-JANELIA_/Map   ::= {
    MHDR-REGISTER-ADDRESS_     : 0x01,
    NHDR-REGISTER-ADDRESS_     : 0x01,
    NCLR-REGISTER-ADDRESS_     : 0x10,
    FDLR-REGISTER-ADDRESS_     : 0x20,
    MHDF-REGISTER-ADDRESS_     : 0x01,
    NHDF-REGISTER-ADDRESS_     : 0x01,
    NCLF-REGISTER-ADDRESS_     : 0x10,
    FDLF-REGISTER-ADDRESS_     : 0x20,
    NHDT-REGISTER-ADDRESS_     : 0x01,
    NCLT-REGISTER-ADDRESS_     : 0x10,
    FDLT-REGISTER-ADDRESS_     : 0xFF,
    MHDPROXR-REGISTER-ADDRESS_ : 0x0F,
    NHDPROXR-REGISTER-ADDRESS_ : 0x0F,
    NCLPROXR-REGISTER-ADDRESS_ : 0x00,
    FDLPROXR-REGISTER-ADDRESS_ : 0x00,
    MHDPROXF-REGISTER-ADDRESS_ : 0x01,
    NHDPROXF-REGISTER-ADDRESS_ : 0x01,
    NCLPROXF-REGISTER-ADDRESS_ : 0xFF,
    FDLPROXF-REGISTER-ADDRESS_ : 0xFF,
    NHDPROXT-REGISTER-ADDRESS_ : 0x00,
    NCLPROXT-REGISTER-ADDRESS_ : 0x00,
    FDLPROXT-REGISTER-ADDRESS_ : 0x00,
    DEBOUNCE-REGISTER-ADDRESS_ : 0x11,
    ACCR0-REGISTER-ADDRESS_    : 0x00,
    ACCR1-REGISTER-ADDRESS_    : 0x00,
    USL-REGISTER-ADDRESS_      : 0x00,
    LSL-REGISTER-ADDRESS_      : 0x00,
    TL-REGISTER-ADDRESS_       : 0x00 }

  // Globals
  logger_/log.Logger         := ?
  touch-status-mask_/int     := ?
  devices_/Map               := {:}
  last-ecr-register_/Map     := {:}
  tasks_/Map                 := {:}

  /** Class Constructor: Always constructs with 1 device, but others can be added */
  constructor device/serial.Device --identifier=DEFAULT-I2C-ADDRESS --logger/log.Logger=(log.default.with-name "mpr121"):
    devices_[DEFAULT-I2C-ADDRESS] = device.registers
    logger_ = logger
    touch-status-mask_ = TOUCH-STATUS-MASK-BASE_
    initialise-device_

  /** add-device: adding second device to class for more channels */
  add-device device/serial.Device --identifier -> none:
    // Add the device to the class after the class was already constructed
    if devices_.size >= DEVICE_COUNT_MAX_:
      logger_.debug "add-device: add-device: device count max reached count=$(devices_.size)"
      return 

    // Already have one with this identifier
    if devices_.contains identifier:  
      logger_.debug "add-device: add-device: already a device with id=$(identifier)"
      return
    
    // We got here, so add it
    devices_[identifier] = device
    initialise-device_ --identifier=identifier


  initialise-device_ -> none:
    devices_.keys.do: initialise-device_ --identifier=it

  initialise-device_ --identifier -> none:
    // Perform soft reset - ensures reset of values especially if the project/code resets but power is not interrupted
    soft-reset --identifier=identifier

    // Set baseline configuration, as per Janelia code:
    BASELINE-CONFIGURATION-JANELIA_.keys.do:
      devices_[identifier].write-u8 it BASELINE-CONFIGURATION-JANELIA_[it]
      //logger_.debug "initialise-device_: Wrote 0x$(%02x it) with   0x$(%02x BASELINE-CONFIGURATION-JANELIA_[it]) [$(bits-16 BASELINE-CONFIGURATION-JANELIA_[it])]"
 
    set-thresholds --touch=TOUCH-THRESHOLD-DEFAULT_ --release=RELEASE-THRESHOLD-DEFAULT_

    // Reset/disable GPIO mode for all pins
    devices_[identifier].write-u8 GPIO-EN-REGISTER-ADDRESS_ 0

    // DEFAULT CONFIGURATION: enable X electrodes = start MPR121
    // - CL Calibration Lock bits  : 0b10 = 5 bits for baseline tracking
    // - ELEPROX_EN proximity bits : disabled
    // - ELE_EN Electrode Enable   : amount of electrodes running (12)
    // Thus:
    initial-ecr-setting/int := 0b10000000 + 12
    devices_[identifier].write-u8 ECR-REGISTER-ADDRESS_ initial-ecr-setting
    logger_.debug "initialise-device_: Wrote ECR-REGISTER-ADDRESS_[$(%02x identifier)] with  0x$(%02x initial-ecr-setting) [$(bits-16 initial-ecr-setting)]"

    // Testing
    read-afe1-register --identifier=identifier
    read-afe2-register --identifier=identifier

  /** touch-pins-enabled: sets the number of touch pins enabled
      Must be set as a group, a number of consecutive pins from pin 0.  Cannot enable/disable them individually.
      */
  touch-pins-enabled --identifier --number-of-pins/int -> none:
    assert: 0 <= number-of-pins <= 12
    old-value := devices_[identifier].read-u8 ECR-REGISTER-ADDRESS_
    new-value := (old-value & 0xF0) & 0xFF  // Zero old value
    new-value = new-value + number-of-pins
    devices_[identifier].write-u8 ECR-REGISTER-ADDRESS_ new-value
    logger_.debug "touch-pins-enabled (0x$(%02x identifier)): enable pins $(number-of-pins) from=$(bits-16 old-value) to=$(bits-16 new-value)"
    sleep --ms=1

  /** set-thresholds:  Set the touch and release thresholds for some, or all, of the 13 channels on the device.
      The threshold is defined as a deviation value from the baseline value, so it remains constant even if the baseline value changes. Typically the touch threshold is a little bigger than the release threshold to touch debounce and hysteresis. For typical touch application, the value can be in range 0x05~0x30. The setting of the threshold is depended on the actual application.
      For operational details and how to set the threshold refer to application note AN3892 and MPR121 design guidelines.
      - touch: the touch threshold value from 0 to 255.
      - release: the release threshold from 0 to 255.
      For a specfic channel - higher values = less sensitive */
  set-thresholds --touch/int --release/int -> none:
    devices_.keys.do: set-thresholds --identifier=it --touch=touch --release=release

  /** set-thresholds: set all thresholds (the same)
      */
  set-thresholds --identifier --touch/int --release/int -> none:
    for i := 0; i <= PHYSICAL_CHANNELS_PER_DEVICE_; i += 1:
      set-thresholds --identifier=identifier --channel=i --touch=touch
      set-thresholds --identifier=identifier --channel=i --release=release

  /** set-thresholds: set thresholds for a specific device and channel
      */
  set-thresholds --identifier --channel --touch -> none:
    //devices_[identifier].write-u8 (TOUCH-THRESHOLD0-REGISTER-ADDRESS_ + (2 * channel)) touch
    write-register-u8 --identifier=identifier --register=(TOUCH-THRESHOLD0-REGISTER-ADDRESS_ + (2 * channel)) --value=touch 
    //logger_.debug "set-thresholds_: Wrote TOUCH-THRESHOLD0-REGISTER-ADDRESS_   [$(channel)] + $(2 * channel) with 0x$(%04x touch)"

  set-thresholds --identifier --channel --release -> none:
    // to do - measure and ensure release is greater than touch
    //devices_[identifier].write-u8 (RELEASE-THRESHOLD0-REGISTER-ADDRESS_ + (2 * channel)) release
    write-register-u8 --identifier=identifier --register=(RELEASE-THRESHOLD0-REGISTER-ADDRESS_ + (2 * channel)) --value=release
    //logger_.debug "set-thresholds_: Wrote RELEASE-THRESHOLD0-REGISTER-ADDRESS_ [$(channel)] + $(2 * channel) with 0x$(%04x release)"

  soft-reset -> none:
    devices_.keys.do: soft-reset --identifier=it

  soft-reset --identifier -> none:
    write-register-u8 --identifier=identifier --register=SOFT-RESET-REGISTER-ADDRESS_ --value=SOFT-RESET-VALUE_
    logger_.debug "soft-reset: Wrote SOFT-RESET-REGISTER-ADDRESS_ with    0x$(%04x SOFT-RESET-VALUE_) [$(bits-16 SOFT-RESET-VALUE_)]"
    sleep --ms=1

  /** read-filtered-channel-data:
      Read the filtered data from specified channel. The ADC raw data outputs run through 3 levels of digital filtering to filter out the high frequency and low frequency noise encountered. For detailed information on this filtering see page 6 of the device datasheet.
      Returns the filtered reading, as a 10 bit unsigned value
      */
  read-filtered-channel-data --identifier --channel -> int:
    if (channel > 12): return 0
    return devices_[identifier].read-u16-le (FILTERED-DATA0-REGISTER-ADDRESS_ + (channel * 2))

  /** read-baseline-channel-data
      Returns the baseline value for the channel. The 3rd level filtered result is internally 10bit but only high 8 bits are readable from registers 0x1E~0x2A as the baseline value output for each channel.
      */
  read-baseline-channel-data --identifier --channel -> int:
    if (channel > 12): return 0
    value := devices_[identifier].read-u8 (BASELINE_DATA0_REGISTER_ADDRESS_ + channel)
    return (value << BASELINE-DATA-BIT-SHIFT_)

  /** touched: Read the touch status of all 13 channels as bit values in a 12 bit integer.
      Returns a 12 bit integer with each bit corresponding to the touch status of a sensor channel. For example, if bit 5 is equal to 1, then that channel of the device is currently deemed to be touched. */
  touched --identifier -> int:
    value := devices_[identifier].read-u16-le TOUCH-STATUS-REGISTER-ADDRESS_
    output := value & 0x0FFF
    return output

  stop-all-channels_ -> none:
    devices_.keys.do: stop-all-channels_ --identifier=it

  stop-all-channels_ --identifier -> none:
    value := devices_[identifier].read-u8 ECR-REGISTER-ADDRESS_
    last-ecr-register_[identifier] = value
    write-register-u8 --identifier=identifier --register=ECR-REGISTER-ADDRESS_ --value=0x0

  start-all-channels_ -> none:
    devices_.keys.do: start-all-channels_ --identifier=it

  start-all-channels_ --identifier -> none:
    write-register-u8 --identifier=identifier --register=ECR-REGISTER-ADDRESS_ --value=last-ecr-register_[identifier] 
    logger_.debug "Write ECR-REGISTER-ADDRESS_ restored to 0x$(%02x last-ecr-register_[identifier] ) [$(bits-16 last-ecr-register_[identifier])]"

  write-register-u8 --identifier --register/int --value/int -> none:
    // MPR121 must be put in Stop Mode to write to most registers
    stop_required/bool := true
    // first get the current set value of the MPR121_ECR register
    last-ecr-register_[identifier] = devices_[identifier].read-u8 ECR-REGISTER-ADDRESS_
    if ((register == ECR-REGISTER-ADDRESS_) or ((0x73 <= register) and (register <= 0x7A))):
      stop_required = false
    if stop_required:
      // clear this register to set stop mode
      devices_[identifier].write-u8 ECR-REGISTER-ADDRESS_ 0x0
    devices_[identifier].write-u8 register value
    if stop_required:
      // write back the previous set ECR settings
      devices_[identifier].write-u8 ECR-REGISTER-ADDRESS_ last-ecr-register_[identifier]

  /** clear-overcurrent-flag:
      When over current detected, the OVCF bit 0b1000_0000 is set on the pin status register, and MPR121 goes to Stop Mode immediately. The ExTS bits in status registers,output registers 0x04~0x2A, and bit D5~D0 in ECR will be also cleared on over current condition. When the bit is “1”, writing ECR register (to try to enter Run mode) will be discarded.
      Write “1” to OVCF will clear this bit and MPR121 fault condition is cleared so that MPR121 can be configured into Run Mode again.
      */
  clear-overcurrent-flag --identifier -> none:
    devices_[identifier].write-u16-le TOUCH-STATUS-REGISTER-ADDRESS_ OVER-CURRENT-REXT_
    write-register-u8 --identifier=identifier  --register=TOUCH-STATUS-REGISTER-ADDRESS_ --value=OVER-CURRENT-REXT_

  // looks like it will set the bit of the channel of the out of range status
  get-out-of-range-status --identifier -> int:
    value := devices_[identifier].u16-le OUT-OF-RANGE-STATUS-REGISTER-ADDRESS_
    logger_.debug "get-out-of-range-status: returned 0x$(%02x value) [$(bits-16 value)]"
    return value

  auto-config-failed --identifier --out-of-range-status/int -> int:
    return (get-out-of-range-status --identifier=identifier) & OUT-OF-RANGE-STATUS-ACFF_

  auto-reconfig-failed --identifier --out-of-range-status/int -> int:
    return (get-out-of-range-status --identifier=identifier) & OUT-OF-RANGE-STATUS-ARFF_

  /** set-debounce: sets debounce on all channels for the device.
      A channels' status change will only take place after the number of consecutive touch or release detection meets the debounce number. If the number detected does not meet the debounce number, the status will not change. 
      Valid values are 0-7 */
  set-debounce --touch/int -> none:
    devices_.keys.do: set-debounce --identifier=it --touch=touch 

  set-debounce --release/int -> none:
    devices_.keys.do: set-debounce --identifier=it --release=release 

  set-debounce --touch/int --release/int -> none:
    devices_.keys.do: set-debounce --identifier=it --touch=touch 
    devices_.keys.do: set-debounce --identifier=it --release=release 

  set-debounce --identifier --release/int -> none:
    assert: 0 <= release <= 7 
    old-value := devices_[identifier].read-u8 DEBOUNCE-REGISTER-ADDRESS_
    new-value/int := old-value
    new-value     &= ~DEBOUNCE-REGISTER-RELEASE-MASK_
    new-value     |= (release << DEBOUNCE-REGISTER-RELEASE-OFFSET_)
    write-register-u8 --identifier=identifier  --register=DEBOUNCE-REGISTER-ADDRESS_ --value=new-value
    logger_.debug "set-debounce --release (0x$(%02x identifier)): from=$(bits-16 old-value) to=$(bits-16 new-value)"

  set-debounce --identifier --touch/int -> none:
    assert: 0 <= touch <= 7 
    old-value := devices_[identifier].read-u8 DEBOUNCE-REGISTER-ADDRESS_
    new-value/int := old-value
    new-value     &= ~DEBOUNCE-REGISTER-TOUCH-MASK_
    new-value     |= (touch << DEBOUNCE-REGISTER-TOUCH-OFFSET_)
    write-register-u8 --identifier=identifier  --register=DEBOUNCE-REGISTER-ADDRESS_ --value=new-value
    logger_.debug "set-debounce --touch   (0x$(%02x identifier)): from=$(bits-16 old-value) to=$(bits-16 new-value)"

  /** proximity-mode: set number of channels to used to generate virtual "13th" proximity channel.
      When configured, enables the use of a 13th channel, used for a single 'Proximity Detection' channel.  This requires a number of the existing channels, and this configuration selects how many of them are to be used:
      - 0 - 0b00 - Proximity Detection is disabled (Default)
      - 1 - 0b01 - Run Mode with ELE0~ELE1  combined for proximity detection enabled
      - 2 - 0b10 - Run Mode with ELE0~ELE3  combined for proximity detection enabled
      - 3 - 0b11 - Run Mode with ELE0~ELE11 (all 12 channles) combined for the single proximity detection channel enabled
      See https://www.nxp.com/docs/en/application-note/AN3893.pdf
      */
  proximity-mode --mode/int -> none:
    devices_.keys.do: proximity-mode --identifier=it --mode=mode

  proximity-mode --identifier --mode/int -> none:
    assert: 0 <= mode <= 3
    old-value := devices_[identifier].read-u8 ECR-REGISTER-ADDRESS_
    new-value/int := old-value
    new-value     &= ~ECR-PROXIMITY-MASK_
    new-value     |= (mode << ECR-PROXIMITY-OFFSET_)
    write-register-u8 --identifier=identifier  --register=ECR-REGISTER-ADDRESS_ --value=new-value
    logger_.debug "proximity-mode (0x$(%02x identifier)): from=$(bits-16 old-value) to=$(bits-16 new-value)"

  /** configuration-lock:  Sets Configuration Lock register.  Values:
     - 0 Baseline tracking enable (Default). The baseline values updates every {ESI x SFI} period by MPR121 per baseline filter operation. The initial value is ????.
     - 1 Calibration lock. Baseline tracking disabled. Baseline values are unchanged by MPR121.
     - 2 Baseline tracking and initialize enable. At the first {ESI x SFI}, MPR121 copy 5MSBs of the 2nd filter output to 10bit baseline value (5LSBs become zero.  Subsequent update is per nominal baseline filter operation.
     - 3 Baseline tracking and initialize enable. At the first {ESI x SFI}, MPR121 copy the 2nd filter output to 10bit baseline value. Subsequent update is per nominal baseline filter operation. */

  configuration-lock --mode/int -> none:
    devices_.keys.do: configuration-lock --identifier=it --mode=mode

  configuration-lock --identifier --mode/int -> none:
    assert: 0 <= mode <= 3
    old-value := devices_[identifier].read-u8 ECR-REGISTER-ADDRESS_
    new-value/int := old-value
    new-value     &= ~ECR-CONFIGURATION-LOCK-MASK_
    new-value     |= (mode << ECR-CONFIGURATION-LOCK-OFFSET_)
    write-register-u8 --identifier=identifier  --register=ECR-REGISTER-ADDRESS_ --value=new-value
    logger_.debug "configuration-lock (0x$(%02x identifier)): from=$(bits-16 old-value) to=$(bits-16 new-value)"
   
  /** set-autoconfig_: Enable autoconfig option - enable / disable autoconfig.
      When enabled, the MPR121 automatically searches and sets the charging parameters for every enabled pad. This happens on each time the MPR121 transitions from Stop Mode to Run Mode.
      */
  set-autoconfig_ --autoconfig/bool -> none:
    devices_.keys.do: set-autoconfig_ --identifier --autoconfig=autoconfig

  set-autoconfig_ --identifier --autoconfig/bool -> none:
    // register map at
    // https://www.nxp.com/docs/en/data-sheet/MPR121.pdf#page=17&zoom=auto,-416,747
    if autoconfig:
      // recommend settings found at
      // https://www.nxp.com/docs/en/application-note/AN3889.pdf#page=9&zoom=310,-42,373
      // FFI (First Filter Iterations) same as FFI in CONFIG1
      // FFI           → 00 Sets samples taken to 6 (Default)
      // RETRY
      // RETRY
      // BVA same as CL in ECR
      // BVA same as CL in ECR
      // ARE Auto-Reconfiguration Enable
      // ACE Auto-Configuration Enable
      // 0x0B == 0b00001011
      devices_[identifier].write-u8 ACCR0-REGISTER-ADDRESS_ 0b00001011
      logger_.debug "Wrote ACCR0-REGISTER-ADDRESS_ with    0x$(%04x 0b00001011) [$(bits-16 0b00001011)]"
    
      // details on values
      // https://www.nxp.com/docs/en/application-note/AN3889.pdf#page=7&zoom=310,-42,792
      // correct values for Vdd = 3.3V
      //devices_[identifier].write-u8 USL-REGISTER-ADDRESS_ 200                 // ((Vdd - 0.7)/Vdd) * 256
      write-register-u8 --identifier=identifier --register=USL-REGISTER-ADDRESS_ --value=200     // ((Vdd - 0.7)/Vdd) * 256
      logger_.debug "Wrote USL-REGISTER-ADDRESS_ with    200"
      //devices_[identifier].write-u8 TL-REGISTER-ADDRESS_  180                 // UPLIMIT * 0.9
      write-register-u8 --identifier=identifier --register=TL-REGISTER-ADDRESS_  --value=180     // UPLIMIT * 0.9
      logger_.debug "Wrote TL-REGISTER-ADDRESS_ with     180"
      //devices_[identifier].write-u8 LSL-REGISTER-ADDRESS_ 130                 // UPLIMIT * 0.65
      write-register-u8 --identifier=identifier --register=LSL-REGISTER-ADDRESS_ --value=130     // UPLIMIT * 0.65
      logger_.debug "Wrote LSL-REGISTER-ADDRESS_ with    130"
    else:
      // really only disable ACE.
      write-register-u8 --identifier=identifier --register=ACCR0-REGISTER-ADDRESS_ --value=0b00001010
      logger_.debug "Wrote ACCR0-REGISTER-ADDRESS_ with    0x$(%04x 0b00001010) [$(bits-16 0b00001010)]"

  afe1-register-reset -> none:
    devices_.keys.do: afe1-register-reset --identifier=it

  afe1-register-reset --identifier -> none:
    value := devices_[identifier].read-u8 FILTER-GLOBAL-CDC-REGISTER-ADDRESS_
    write-register-u8 --identifier=identifier  --register=FILTER-GLOBAL-CDC-REGISTER-ADDRESS_ --value=FILTER-GLOBAL-CDC-REGISTER-DEFAULT_
    logger_.debug "afe1-register-reset: (0x$(%02x identifier)): from=$(bits-16 value) to=$(bits-16 FILTER-GLOBAL-CDC-REGISTER-DEFAULT_)"

  afe2-register-reset -> none:
    devices_.keys.do: afe2-register-reset --identifier=it

  afe2-register-reset --identifier -> none:
    value := devices_[identifier].read-u8 FILTER-GLOBAL-CDT-REGISTER-ADDRESS_
    write-register-u8 --identifier=identifier  --register=FILTER-GLOBAL-CDT-REGISTER-ADDRESS_ --value=FILTER-GLOBAL-CDT-REGISTER-DEFAULT_
    logger_.debug "afe2-register-reset: (0x$(%02x identifier)): from=$(bits-16 value) to=$(bits-16 FILTER-GLOBAL-CDT-REGISTER-DEFAULT_)"


  /** The MPR121 uses a constant DC current capacitance sensing scheme. It can measure capacitances ranging from 10 pF to over 2000 pF with a resolution up to 0.01 pF. The device does this by varying the amount of charge current and charge time applied to the sensing inputs.*/

  /** charge-discharge-current: Sets individual Charge Discharge Current per channel if Global value is not used.
      The Individual Charge Discharge Current field selects the supply current to be used when charging and discharging a specific channel. Programmable in 1uA steps, up to 64uA.  When the CDCx is zero, the global value is used.  If auto configuration is enabled and run once, the individual CDC will be automatically updated by MPR121 internally after autoconfiguration is finished.
      */
  channel-charge-discharge-current --identifier/int --channel/int -> int:
    assert: 0 <= channel <= 12
    value := devices_[identifier].read-u8 (CDC0-REGISTER-ADDRESS_ + channel)
    return value

  channel-charge-discharge-current --identifier/int --channel/int --current/int -> none:
    assert: 0 <= channel <= 12
    assert: 0 <= current <= CHARGE-DISCHARGE-CURRENT-MAX_
    value := devices_[identifier].read-u8 (CDC0-REGISTER-ADDRESS_ + channel)
    write-register-u8 --identifier=identifier  --register=(CDC0-REGISTER-ADDRESS_ + channel) --value=current
    logger_.debug "channel-charge-discharge-current: (0x$(%02x identifier)): channel$(channel) register=$(bits-16 (CDC0-REGISTER-ADDRESS_ + channel)) value=$(current)"

  /** channel-charge-time - Sets the charge time applied to each channel. 
      Similar to the global CDT value, the range is 0~32 μS, from 0b000-0b111  When the CDTx is zero, the global CDT value is used for that channel.
      */
  channel-charge-discharge-time --identifier/int --channel/int --time/int-> none:
    // 3 bits are held for 12 channels, across 6 registers, two channels per register, with one bit unused.
    assert: 0 <= time <= 7
    assert: 0 <= channel <= 12
    register := CDT0-REGISTER-ADDRESS_ + (channel / CDT-FIELD-DIVISOR_)
    shift    := (CDT-FIELD-OFFSET_ * (channel % CDT-FIELD-DIVISOR_))
    //old-value := devices_[identifier].read-u8 register
    //old-time := old-value >> mask
    //old-time &= CDT-CHARGE-TIME-MASK_
    //new-value/int := old-value
    //new-value     &= ~CDT-CHARGE-TIME-MASK_
    //new-value     |= time << mask
    old-value := devices_[identifier].read-u8 register
    new-value := old-value
    new-value &= ~((CDT-CHARGE-TIME-MASK_) << shift)
    new-value |= (time & CDT-CHARGE-TIME-MASK_) << shift
    write-register-u8 --identifier=identifier  --register=register --value=new-value
    logger_.debug "channel-charge-discharge-time (0x$(%02x identifier)): from=$(bits-16 old-value) to=$(bits-16 time)"
    logger_.debug "channel-charge-discharge-time (0x$(%02x identifier)): register=0x$(%02x register) from=$(bits-16 old-value) to=$(bits-16 new-value)"
    sleep --ms=1

  channel-charge-discharge-time --identifier/int --channel/int -> int:
    // 3 bits are held for 12 channels, across 6 registers, two channels per register, with one bit unused.
    register := CDT0-REGISTER-ADDRESS_ + (channel / CDT-FIELD-DIVISOR_)
    mask     := (CDT-FIELD-OFFSET_ * (channel % CDT-FIELD-DIVISOR_))
    old-value := devices_[identifier].read-u8 register
    old-time := old-value >> mask
    old-time &= (CDT-CHARGE-TIME-MASK_ << mask)
    sleep --ms=1
    return old-time

  /** GPIO and LED Driver function
      Among the 12 electrode inputs, 8 inputs are designed as multifunctional pins (D0 - D7 bits in the register correspond to GPIO and LED functions on ELE4 - ELE11). When these pins are not configured as electrodes, they may be used to drive LEDs or used for general purpose input or output. For more details on this feature, please refer to application note AN3894.
      https://www.nxp.com/docs/en/application-note/AN3894.pdf
      
      Note: The number of touch sensing electrodes, and therefore the number of GPIO ports left available is configured by the ECR (0x5E) and GPIO Enable Register (0x77). ECR has higher priority and overrides the GPIO enabled in 0x77 - that is, when a pin is enabled as GPIO but is also selected as electrode by ECR, the GPIO function is disabled immediately and it becomes an electrode during Run Mode.

      Note: Be aware that the channels/electrodes cannot be turned on or off without following the 'total number of touch channels enabled' sequence.  This means that using the first 6 channels for touch and wanting to use the second channel for GPIO, won't work.  This means that using the higher end channels for GPIO might need to be done first, and then to start reducing the number of channels for touch one by one.   
      */

  gpio-pin-function --identifier --pin/int --disable -> none:
    assert: 4 <= pin <= 11
    old-value := devices_[identifier].read-u8 GPIO-EN-REGISTER-ADDRESS_
    new-value       := (old-value &  ~(1 << pin - 4)) & 0xFF
    devices_[identifier].write-u8 GPIO-EN-REGISTER-ADDRESS_ new-value
    logger_.debug "gpio-pin-function (0x$(%02x identifier)):  disable pin $(pin) from=$(bits-16 old-value) to=$(bits-16 new-value)"
    sleep --ms=1

  gpio-pin-function --identifier --pin/int --enable --input -> none:
    assert: 4 <= pin <= 11
    old-EN-reg-value := devices_[identifier].read-u8 GPIO-EN-REGISTER-ADDRESS_
    new-EN-reg-value := (old-EN-reg-value |  (1 << pin - 4)) & 0xFF                 //0 in EN
    devices_[identifier].write-u8 GPIO-EN-REGISTER-ADDRESS_ new-EN-reg-value
    logger_.debug "gpio-pin-function (0x$(%02x identifier)):  enable  pin $(pin) INPUT from=$(bits-16 old-EN-reg-value) to=$(bits-16 new-EN-reg-value)"
    old-CTL0-reg-value := devices_[identifier].read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := devices_[identifier].read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL1
    devices_[identifier].write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    devices_[identifier].write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    old-DIR-reg-value := devices_[identifier].read-u8 GPIO-DIR-REGISTER-ADDRESS_
    new-DIR-reg-value := (old-DIR-reg-value &  ~(1 << pin - 4)) & 0xFF              //0 in DIR
    devices_[identifier].read-u8 GPIO-DIR-REGISTER-ADDRESS_ new-DIR-reg-value
    sleep --ms=1

  gpio-pin-function --identifier --pin/int --enable --input --pulldown -> none:
    assert: 4 <= pin <= 11
    gpio-pin-function --identifier --pin=pin --enable --input
    old-CTL0-reg-value := devices_[identifier].read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := devices_[identifier].read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL1
    devices_[identifier].write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    devices_[identifier].write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    logger_.debug "gpio-pin-function (0x$(%02x identifier)):  enable internal pulldown on pin $(pin)" // from=$(bits-16 old-value) to=$(bits-16 new-value)"
    sleep --ms=1

  gpio-pin-function --identifier --pin/int --enable --input --pullup -> none:
    assert: 4 <= pin <= 11
    gpio-pin-function --identifier --pin=pin --enable --input
    old-CTL0-reg-value := devices_[identifier].read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := devices_[identifier].read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL1
    devices_[identifier].write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    devices_[identifier].write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    logger_.debug "gpio-pin-function (0x$(%02x identifier)):  enable internal pullup on pin $(pin)" // from=$(bits-16 old-value) to=$(bits-16 new-value)"
    sleep --ms=1

  gpio-pin-function --identifier --pin/int --enable --output -> none:
    assert: 4 <= pin <= 11
    old-EN-reg-value := devices_[identifier].read-u8 GPIO-EN-REGISTER-ADDRESS_
    new-EN-reg-value := (old-EN-reg-value |  (1 << pin - 4)) & 0xFF                 //0 in EN
    devices_[identifier].write-u8 GPIO-EN-REGISTER-ADDRESS_ new-EN-reg-value
    logger_.debug "gpio-pin-function (0x$(%02x identifier)):  enable  pin $(pin) OUTPUT from=$(bits-16 old-EN-reg-value) to=$(bits-16 new-EN-reg-value)"
    old-CTL0-reg-value := devices_[identifier].read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := devices_[identifier].read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL1
    devices_[identifier].write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    devices_[identifier].write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    old-DIR-reg-value := devices_[identifier].read-u8 GPIO-DIR-REGISTER-ADDRESS_
    new-DIR-reg-value := (old-DIR-reg-value  |  (1 << pin - 4)) & 0xFF              //1 in DIR
    sleep --ms=1

  gpio-pin-function --identifier --pin/int --enable --output --hi-side-open-drain-led -> none:
    assert: 4 <= pin <= 11
    gpio-pin-function --identifier --pin=pin --enable --output
    old-CTL0-reg-value := devices_[identifier].read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := devices_[identifier].read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL1
    devices_[identifier].write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    devices_[identifier].write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    logger_.debug "gpio-pin-function (0x$(%02x identifier)):  enable hi-side-open-drain-led on pin $(pin)" // from=$(bits-16 old-value) to=$(bits-16 new-value)"
    sleep --ms=1

  gpio-pin-function --identifier --pin/int --enable --output --lo-side-open-drain -> none:
    assert: 4 <= pin <= 11
    gpio-pin-function --identifier --pin=pin --enable --output
    old-CTL0-reg-value := devices_[identifier].read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := devices_[identifier].read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL1
    devices_[identifier].write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    devices_[identifier].write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    logger_.debug "gpio-pin-function (0x$(%02x identifier)):  enable lo-side-open-drain on pin $(pin)" // from=$(bits-16 old-value) to=$(bits-16 new-value)"
    sleep --ms=1

  /** get-gpio-pin: returns DAT for the pin.
      When a GPIO is enabled as an output, the GPIO port outputs the corresponding DAT bit level from GPIO Data Register (0x075). The output level toggle remains on during any electrode charging. The level transition will occur after the ADC conversion takes place. It is important to note that reading this register returns the content of the GPIO Data Register, (not a level of the port). When a GPIO is configured as input, reading this register returns the latched input level of the corresponding port (not contents of the GPIO Data Register). Writing to the DAT changes content of the register, but does not effect the input function
      */
  get-gpio-pin --identifier --pin/int --get -> int:
    old-DAT-reg-value := devices_[identifier].read-u8 GPIO-DAT-REGISTER-ADDRESS_
    return  (old-DAT-reg-value & (1 << pin - 4))

  /** other GPIO functions:
      Writing “1” into the corresponding bits of GPIO Data Set Register, GPIO Data Clear Register, and GPIO Data Toggle Register will set/clear/toggle contents of the corresponding DAT bit in Data Register.  These functions are reproduced here.  (Writing “0” has no meaning.)  These registers allow any individual port(s) to be set, cleared, or toggled individually without affecting other ports. It is important to note that reading these registers returns the contents of the GPIO Data Register reading.
      */
  set-gpio-pin --identifier --pin/int --set -> none:
    old-SET-reg-value := devices_[identifier].read-u8 GPIO-SET-REGISTER-ADDRESS_
    new-SET-reg-value := (old-SET-reg-value |  (1 << pin - 4))                //1 in SET
    devices_[identifier].write-u8 GPIO-SET-REGISTER-ADDRESS_ new-SET-reg-value
    logger_.debug "set-gpio-pin (0x$(%02x identifier)):  set  pin $(pin) OUTPUT from=$(bits-16 old-SET-reg-value) to=$(bits-16 new-SET-reg-value)"

  clear-gpio-pin --identifier --pin/int --clear -> none:
    old-CLR-reg-value := devices_[identifier].read-u8 GPIO-CLR-REGISTER-ADDRESS_
    new-CLR-reg-value := (old-CLR-reg-value |  (1 << pin - 4))                //1 in CLR
    devices_[identifier].write-u8 GPIO-CLR-REGISTER-ADDRESS_ new-CLR-reg-value
    logger_.debug "clear-gpio-pin (0x$(%02x identifier)):  clear  pin $(pin) OUTPUT from=$(bits-16 old-CLR-reg-value) to=$(bits-16 new-CLR-reg-value)"

  toggle-gpio-pin --identifier --pin/int --toggle -> none:
    old-TOG-reg-value := devices_[identifier].read-u8 GPIO-TOG-REGISTER-ADDRESS_
    new-TOG-reg-value := (old-TOG-reg-value |  (1 << pin - 4))                //1 in TOG
    devices_[identifier].write-u8 GPIO-TOG-REGISTER-ADDRESS_ new-TOG-reg-value
    logger_.debug "toggle-gpio-pin (0x$(%02x identifier)):  toggle  pin $(pin) OUTPUT from=$(bits-16 old-TOG-reg-value) to=$(bits-16 new-TOG-reg-value)"

/*



  union AFE0Configuration
  {
    struct Fields
    {
      uint8_t charge_discharge_current : 6
      uint8_t first_filter_iterations : 2
    } fields
    uint8_t uint8
  }

  union AFE1Configuration
  {
    struct Fields
    {
      uint8_t sample_period : 3
      uint8_t second_filter_iterations : 2
      uint8_t charge_discharge_time : 3
    } fields
    uint8_t uint8
  }



*/


// HELPER / TROUBLESHOOTING / TESTING / DEBUGGING FUNCTIONS

  /** Displays bitmasks nicely */
  bits-16 x/int --display-bits/int=8 -> string:
    if (x > 255) or (display-bits > 8):
      outStr := "$(%b x)"
      outStr = outStr.pad --left 16 '0'
      outStr = "$(outStr[0..4]).$(outStr[4..8]).$(outStr[8..12]).$(outStr[12..16])"
      return outStr
    else:
      outStr := "$(%b x)"
      outStr = outStr.pad --left 8 '0'
      outStr = "$(outStr[0..4]).$(outStr[4..8])"
      return outStr

  // stop any running tasks
  stop-all-tasks -> none:
    tasks_.keys.do:
      tasks_[it].cancel
      logger_.info "stop-all: task '$(it)' is cleared"

  debug-touched -> none:
    stop-all-tasks
    tasks_["debug-touched"] = task:: debug-touched_
    logger_.info "debug-touched: task is set"
  
  debug-touched_ -> none:
    out/int := 0
    while true:
      devices_.keys.do: 
        out = touched --identifier=it
        if out > 0:
          logger_.debug "debug-touched: $(bits-16 out --display-bits=16)"
      sleep --ms=50

  read-afe1-register --identifier -> none:
    value := devices_[identifier].read-u8 FILTER-GLOBAL-CDC-REGISTER-ADDRESS_
    logger_.debug "read-afe1-register : read FILTER-GLOBAL-CDC-REGISTER-ADDRESS_[$(%02x identifier)]  has 0x$(%04x value) [$(bits-16 value)]"

  read-afe2-register --identifier -> none:
    value := devices_[identifier].read-u8 FILTER-GLOBAL-CDT-REGISTER-ADDRESS_
    logger_.debug "read-afe2-register : read FILTER-GLOBAL-CDT-REGISTER-ADDRESS_[$(%02x identifier)]  has 0x$(%04x value) [$(bits-16 value)]"

  read-accr0-register --identifier -> none:
    value := devices_[identifier].read-u8 ACCR0-REGISTER-ADDRESS_
    logger_.debug "read-accr0-register: read ACCR0-REGISTER-ADDRESS_[$(%02x identifier)] has 0x$(%04x value) [$(bits-16 value)]"

  read-accr1-register --identifier -> none:
    value := devices_[identifier].read-u8 ACCR1-REGISTER-ADDRESS_
    logger_.debug "read-accr1-register: read ACCR1-REGISTER-ADDRESS_[$(%02x identifier)] has 0x$(%04x value) [$(bits-16 value)]"
