
// Copyright (C) 2025 Toit Contributors
// This file includes both original work, and derivative work based on:
//   - A - https://github.com/janelia-arduino, licensed under the BSD 3-Clause License.
//         Retrieved from: https://github.com/janelia-arduino/MPR121
//   - B - https://github.com/BareConductive/mpr121, licensed under the MIT License.
//   - C - https://github.com/adafruit/Adafruit_MPR121/blob/master/Adafruit_MPR121.cpp
// See LICENSE-BSD and LICENSE-B-MIT for the original license texts.
//
//
// Use of this source code is governed by an MIT-style license that can be
// found in the package's LICENSE file.
//

import log
import binary
import serial.device as serial
import serial.registers as registers

/**
Toit Driver for MPR121 Touch Sensor.

Datasheet: https://www.nxp.com/docs/en/data-sheet/MPR121.pdf
*/

class Mpr121:
  // Device can be configured between 0x58 and 0x5d
  static PHYSICAL_CHANNELS_PER_DEVICE_ ::= 12
  static CHANNELS_PER_DEVICE_ ::= 13

  static I2C-ADDRESS       ::= 0x5a
  static I2C-ADDRESS-TOUCH ::= 0x5c
  static I2C-ADDRESS-5B    ::= 0x5b
  static I2C-ADDRESS-5D    ::= 0x5d

  static PROXIMITY-MODE-DISABLED        := 0 // default set by this driver
  static PROXIMITY-MODE-COMBINE_0_TO_1  := 1
  static PROXIMITY-MODE-COMBINE_0_TO_3  := 2
  static PROXIMITY-MODE-COMBINE_0_TO_11 := 3

  static BASELINE-TRACKING-DISABLED   := 0x01
  static BASELINE-TRACKING-INIT-0     := 0x00 // default in chipset
  static BASELINE-TRACKING-INIT-5BIT  := 0x02
  static BASELINE-TRACKING-INIT-10BIT := 0x03

  static CHARGE_DISCHARGE_TIME_DISABLED := 0x00
  static CHARGE_DISCHARGE_TIME_HALF_US  := 0x01 // default
  static CHARGE_DISCHARGE_TIME_1US      := 0x02
  static CHARGE_DISCHARGE_TIME_2US      := 0x03
  static CHARGE_DISCHARGE_TIME_4US      := 0x04
  static CHARGE_DISCHARGE_TIME_8US      := 0x05
  static CHARGE_DISCHARGE_TIME_16US     := 0x06
  static CHARGE_DISCHARGE_TIME_32US     := 0x07

  static FIRST_FILTER_ITERATIONS_6  := 0x00 // default
  static FIRST_FILTER_ITERATIONS_10 := 0x01
  static FIRST_FILTER_ITERATIONS_18 := 0x02
  static FIRST_FILTER_ITERATIONS_34 := 0x03

  static SECOND_FILTER_ITERATIONS_4  := 0x00 // default
  static SECOND_FILTER_ITERATIONS_6  := 0x01
  static SECOND_FILTER_ITERATIONS_10 := 0x02
  static SECOND_FILTER_ITERATIONS_18 := 0x03

/** Sample periods of the MPR121 - the time between capacitive readings.
    Higher values consume less power, but are less responsive. */
  static SAMPLE_PERIOD_1MS   := 0x00
  static SAMPLE_PERIOD_2MS   := 0x01
  static SAMPLE_PERIOD_4MS   := 0x02
  static SAMPLE_PERIOD_8MS   := 0x03
  static SAMPLE_PERIOD_16MS  := 0x04 // default
  static SAMPLE_PERIOD_32MS  := 0x05
  static SAMPLE_PERIOD_64MS  := 0x06
  static SAMPLE_PERIOD_128MS := 0x07

  // register addresses
  static REG-TOUCH-STATUS_        := 0x00  // = Low, high = 0x01
  static REG-OUT-OF-RANGE-STATUS_ := 0x02
  static REG-FILTERED-DATA0_      := 0x04  // = Low, high = 0x05
  static REG-BASELINE-DATA0_      := 0x1e

  // general electrode touch sense baseline filters
  // rising filter
  static REG-MHD-RISING_ ::= 0x2B
  static REG-NHD-RISING_ ::= 0x2c // Amount
  static REG-NCL-RISING_ ::= 0x2d
  static REG-FDL-RISING_ ::= 0x2e

  // falling filter
  static REG-MHD-FALLING_ ::= 0x2f
  static REG-NHD-FALLING_ ::= 0x30
  static REG-NCL-FALLING_ ::= 0x31
  static REG-FDL-FALLING_ ::= 0x32

  // touched filter
  static REG-NHD-TOUCHED_ ::= 0x33
  static REG-NCL-TOUCHED_ ::= 0x34
  static REG-FDL-TOUCHED_ ::= 0x35

  // proximity electrode touch sense baseline filters
  // rising filter
  static REG-PROX-MHD-RISING_ ::= 0x36
  static REG-PROX-NHD-RISING_ ::= 0x37
  static REG-PROX-NCL-RISING_ ::= 0x38
  static REG-PROX-FDL-RISING_ ::= 0x39

  // falling filter
  static REG-PROX-MHD-FALLING_ ::= 0x3a
  static REG-PROX-NHD-FALLING_ ::= 0x3b
  static REG-PROX-NCL-FALLING_ ::= 0x3c
  static REG-PROX-FDL-FALLING_ ::= 0x3d

  // touched filter
  static REG-PROX-NHD-TOUCHED_ ::= 0x3e
  static REG-PROX-NCL-TOUCHED_ ::= 0x3f
  static REG-PROX-FDL-TOUCHED_ ::= 0x40

  // electrode touch and release thresholds
  static REG-TOUCH-THRESHOLD0_      ::= 0x41
  static REG-RELEASE-THRESHOLD0_    ::= 0x42
  static TOUCH-THRESHOLD-DEFAULT_   ::= 40
  static RELEASE-THRESHOLD-DEFAULT_ ::= 20

  // debounce settings
  static REG-DEBOUNCE_            ::= 0x5B
  static DEBOUNCE-RELEASE-MASK_   ::= 0b0111_0000
  static DEBOUNCE-RELEASE-OFFSET_ ::= 4
  static DEBOUNCE-TOUCH-MASK_     ::= 0b0000_0111
  static DEBOUNCE-TOUCH-OFFSET_   ::= 0

  // channel charge-discharge CURRENTS
  static REG-CDC0_                     ::= 0x5f
  static CHARGE-DISCHARGE-CURRENT-MIN_ ::= 0     // 0 = defer to global
  static CHARGE-DISCHARGE-CURRENT-MAX_ ::= 63

  // channel charge-discharge TIMES
  static REG-CDT0_             ::= 0x6c
  static CDT-CHARGE-TIME-MASK_ ::= 0x07
  static CDT-FIELD-DIVISOR_    ::= 2
  static CDT-FIELD-OFFSET_     ::= 4

  // GPIO
  static REG_GPIO-CTL0_ ::= 0x73
  static REG_GPIO-CTL1_ ::= 0x74
  static REG_GPIO-DAT_  ::= 0x75
  static REG_GPIO-DIR_  ::= 0x76
  static REG_GPIO-EN_   ::= 0x77
  static REG_GPIO-SET_  ::= 0x78
  static REG_GPIO-CLR_  ::= 0x79
  static REG_GPIO-TOG_  ::= 0x7a

  // Configuration registers
  static REG-FILTER-GLOBAL-CDC_     ::= 0x5c
  static REG-FILTER-GLOBAL-CDT_     ::= 0x5d
  static FILTER-GLOBAL-CDC-DEFAULT_ ::= 0b0001_0000
  static FILTER-GLOBAL-CDT-DEFAULT_ ::= 0b0010_0100

  //auto-config
  static REG-ACCR0_ ::= 0x7b
  static REG-ACCR1_ ::= 0x7c
  static REG-USL_   ::= 0x7d  // Upper Limit
  static REG-LSL_   ::= 0x7e  // Lower Limit
  static REG-TL_    ::= 0x7f  // Target Limit

  //soft reset
  static REG-SOFT-RESET_   ::= 0x80
  static SOFT-RESET-VALUE_ ::= 0x63

  //PWM
  //static PWM0_ ::= 0x81
  //static PWM1_ ::= 0x82
  //static PWM2_ ::= 0x83
  //static PWM3_ ::= 0x84

  //ECR
  static REG-ECR_                      ::= 0x5e
  static ECR-ELECTRODE-MASK_           ::= 0b0000_1111
  static ECR-PROXIMITY-MASK_           ::= 0b0011_0000
  static ECR-CONFIGURATION-LOCK-MASK_  ::= 0b1100_0000

  static BASELINE-DATA-BIT-SHIFT_      ::= 2

  //Other Statics
  static OVER-CURRENT-REXT_                      ::= 0b1000_0000_0000_0000
  static TOUCH-STATUS-MASK-BASE_                 ::= 0x1FFF
  static OUT-OF-RANGE-STATUS-ACFF_               ::= 0x8000
  static OUT-OF-RANGE-STATUS-ARFF_               ::= 0x4000


  // Baseline Configuration
  static BASELINE-CONFIGURATION-JANELIA_/Map   ::= {
    REG-MHD-RISING_       : 0x01,
    REG-NHD-RISING_       : 0x01,
    REG-NCL-RISING_       : 0x10,
    REG-FDL-RISING_       : 0x20,
    REG-MHD-FALLING_      : 0x01,
    REG-NHD-FALLING_      : 0x01,
    REG-NCL-FALLING_      : 0x10,
    REG-FDL-FALLING_      : 0x20,
    REG-NHD-TOUCHED_      : 0x01,
    REG-NCL-TOUCHED_      : 0x10,
    REG-FDL-TOUCHED_      : 0xFF,
    REG-PROX-MHD-RISING_  : 0x0F,
    REG-PROX-NHD-RISING_  : 0x0F,
    REG-PROX-NCL-RISING_  : 0x00,
    REG-PROX-FDL-RISING_  : 0x00,
    REG-PROX-MHD-FALLING_ : 0x01,
    REG-PROX-NHD-FALLING_ : 0x01,
    REG-PROX-NCL-FALLING_ : 0xFF,
    REG-PROX-FDL-FALLING_ : 0xFF,
    REG-PROX-NHD-TOUCHED_ : 0x00,
    REG-PROX-NCL-TOUCHED_ : 0x00,
    REG-PROX-FDL-TOUCHED_ : 0x00,
    REG-DEBOUNCE_         : 0x11,
    REG-ACCR0_            : 0x00,
    REG-ACCR1_            : 0x00,
    REG-USL_              : 0x00,
    REG-LSL_              : 0x00,
    REG-TL_               : 0x00 }

  static DEFAULT-REGISTER-WIDTH_ ::= 8

  // Globals
  reg_/registers.Registers := ?
  logger_/log.Logger := ?
  touch-status-mask_/int := 0
  last-ecr-register_/int := 0
  tasks_/Map := {:}

  /** Class Constructor */
  constructor
      device/serial.Device
      --logger/log.Logger=log.default:
    logger_ = logger.with-name "mpr121"
    reg_ = device.registers
    touch-status-mask_ = TOUCH-STATUS-MASK-BASE_
    initialise-device_

  initialise-device_ -> none:
    soft-reset

    // Set baseline configuration
    BASELINE-CONFIGURATION-JANELIA_.keys.do:
      reg_.write-u8 it BASELINE-CONFIGURATION-JANELIA_[it]

    set-thresholds --touch=TOUCH-THRESHOLD-DEFAULT_ --release=RELEASE-THRESHOLD-DEFAULT_

    // Reset/disable GPIO mode for all pins
    write-register_ REG-GPIO-EN_ 0

    // DEFAULT CONFIGURATION: enable X electrodes = start MPR121
    // - CL Calibration Lock bits  : 0b10 = 5 bits for baseline tracking
    // - ELEPROX_EN proximity bits : disabled
    // - ELE_EN Electrode Enable   : amount of electrodes running (12)
    // Thus:
    initial-ecr-setting/int := 0b10000000 + 12
    write-register_ REG-ECR_ initial-ecr-setting
    logger_.debug "initialise-device_: Wrote REG-ECR_ with [$(bits-16_ initial-ecr-setting)]" --tags={"initial-ecr-setting" : initial-ecr-setting}

    // Testing
    show-afe1-register
    show-afe2-register

  /**
  Sets the number of touch pins enabled

  Must be set as a contiguous group, a number of consecutive pins from pin 0.
   Cannot enable/disable them individually.
  */
  touch-pins-enabled --number-of-pins/int -> none:
    assert: 0 <= number-of-pins <= 12
    old-value:= read-register_ REG-ECR_
    write-register_ REG-ECR_ number-of-pins
    logger_.debug "touch-pins-enabled enable pins $(number-of-pins) from=$(bits-16_ old-value) to=$(bits-16_ number-of-pins)" --tags={"pins-enabled" : number-of-pins}
    sleep --ms=1

  /**
  Set the touch and release thresholds for all channels.

  The threshold is defined as a deviation value from the baseline value, so it
   remains constant even if the baseline value changes. Typically the touch
   threshold is a little bigger than the release threshold to touch debounce and
   hysteresis. For typical touch application, the value can be in range
   0x05~0x30. The setting of the threshold is depended on the actual application.

  For operational details and how to set the threshold refer to application note
  AN3892 and MPR121 design guidelines.
   - touch: the touch threshold value from 0 to 255.
   - release: the release threshold from 0 to 255.
  For a specfic channel - higher values = less sensitive
  */
  set-thresholds --touch/int --release/int -> none:
    set-touch-threshold touch --channel=null
    set-release-threshold release --channel=null

  /**
  Set the touch threshold for a channel.

  See $set-thresholds.
  #TODO test for zero case
  */
  set-touch-threshold threshold --channel/int?=null -> none:
    if channel == null:
      PHYSICAL_CHANNELS_PER_DEVICE_.repeat:
        set-touch-threshold threshold --channel=(it)
    else:
      write-register_ (REG-TOUCH-THRESHOLD0_ + (2 * channel)) threshold

  /**
  Set the release threshold for a channel.

  See $set-thresholds.
  #TODO test for zero case
  */
  set-release-threshold threshold --channel/int?=null -> none:
    // #TODO measure and ensure release is greater than touch
    // #TODO test for zero case
    if channel == null:
      PHYSICAL_CHANNELS_PER_DEVICE_.repeat:
        set-release-threshold threshold --channel=(it)
    else:
      write-register_ (REG-RELEASE-THRESHOLD0_ + (2 * channel)) threshold

  soft-reset -> none:
    write-register_ REG-SOFT-RESET_ SOFT-RESET-VALUE_
    sleep --ms=1

  /**
  Reads the filtered data from specified channel.

  The ADC raw data outputs run through 3 levels of digital filtering to filter
   out the high frequency and low frequency noise encountered. For detailed
   information on this filtering see page 6 of the device datasheet.  Returns the
   filtered reading, as a 10 bit unsigned value.
  */
  read-filtered-channel-data --channel -> int:
    if (channel > 12): return 0
    return reg_.read-u16-le (REG-FILTERED-DATA0_ + (channel * 2))

  /**
  Returns the baseline value for the channel.

  The 3rd level filtered result is internally 10bit but only high 8 bits are
   readable from registers 0x1E~0x2A as the baseline value output for each
   channel.
  */
  read-baseline-channel-data --channel -> int:
    if (channel > 12): return 0
    value := reg_.read-u8 (REG-BASELINE-DATA0_ + channel)
    return (value << BASELINE-DATA-BIT-SHIFT_)

  /**
  Reads the touch status of all 13 channels as bit values in a 12 bit integer.

  Returns a 12 bit integer with each bit corresponding to the touch status of a
   sensor channel. For example, if bit 5 is equal to 1, then that channel of
   the device is currently deemed to be touched.
  */
  touched -> int:
    value := reg_.read-u16-le REG-TOUCH-STATUS_
    output := value & 0x0FFF
    return output

  stop-all-channels_ -> none:
    raw := reg_.read-u8 REG-ECR_
    last-ecr-register_ = raw
    write-register_ REG-ECR_ 0x0

  start-all-channels_ -> none:
    write-register_ REG-ECR_ last-ecr-register_
    logger_.debug "Write REG-ECR_ restored to 0x$(%02x last-ecr-register_) [$(bits-16_ last-ecr-register_)]"

  /**
  Clears the 'overcurrent' flag.

  When over current detected, the OVCF bit 0b1000_0000 is set on the pin status
   register, and MPR121 goes to Stop Mode immediately. The ExTS bits in status
   registers,output registers 0x04~0x2A, and bit D5~D0 in ECR will be also
   cleared on over current condition. When the bit is “1”, writing ECR register
   (to try to enter Run mode) will be discarded.  Write “1” to OVCF will clear
   this bit and MPR121 fault condition is cleared so that MPR121 can be
   configured into Run Mode again.
  */
  clear-overcurrent-flag -> none:
    reg_.write-u16-le REG-TOUCH-STATUS_ OVER-CURRENT-REXT_
    write-register_ REG-TOUCH-STATUS_ OVER-CURRENT-REXT_

  // looks like it will set the bit of the channel of the out of range status
  get-out-of-range-status -> int:
    value := reg_.read-u16-le REG-OUT-OF-RANGE-STATUS_
    logger_.debug "get-out-of-range-status: returned 0x$(%02x value) [$(bits-16_ value)]"
    return value

  auto-config-failed --out-of-range-status/int -> int:
    return get-out-of-range-status & OUT-OF-RANGE-STATUS-ACFF_

  auto-reconfig-failed --out-of-range-status/int -> int:
    return get-out-of-range-status & OUT-OF-RANGE-STATUS-ARFF_

  /**
  Sets release debounce on all channels for the device.

  A channels' status change will only take place after the number of consecutive
   touch or release detection meets the debounce number. If the number detected
   does not meet the debounce number, the status will not change.  Valid values
   are 0-7
  */
  set-release-debounce release/int -> none:
    assert: 0 <= release <= 7
    write-register_ REG-DEBOUNCE_ release --mask=DEBOUNCE-RELEASE-MASK_

  /**
  Sets touch debounce on all channels for the device.

  See $set-release-debounce.
  */
  set-touch-debounce touch/int -> none:
    assert: 0 <= touch <= 7
    write-register_ REG-DEBOUNCE_ touch --mask=DEBOUNCE-TOUCH-MASK_

  /**
  Sets the number of channels to consumed for virtual "13th" proximity
  channel.

  When configured, enables the use of a 13th channel, used for a single
  'Proximity Detection' channel.  This requires a number of the existing
  channels, and this configuration selects how many of them are to be used:
  - 0 - 0b00 - Proximity Detection is disabled (Default)
  - 1 - 0b01 - Run Mode with ELE0~ELE1 combined for proximity detection enabled
  - 2 - 0b10 - Run Mode with ELE0~ELE3 combined for proximity detection enabled
  - 3 - 0b11 - Run Mode with ELE0~ELE11 (all 12 channels) combined for the
               single proximity detection channel enabled
  See https://www.nxp.com/docs/en/application-note/AN3893.pdf
  */
  proximity-mode mode/int -> none:
    assert: 0 <= mode <= 3
    write-register_ REG-ECR_ mode --mask=ECR-PROXIMITY-MASK_

  /**
  Sets Configuration Lock register.

  Values:
  - 0 Baseline tracking enable (Default). The baseline values updates every {ESI
    x SFI} period by MPR121 per baseline filter operation. The initial value is
    ????.
  - 1 Calibration lock. Baseline tracking disabled. Baseline values are
    unchanged by MPR121.
  - 2 Baseline tracking and initialize enable. At the first {ESI x SFI}, MPR121
    copy 5MSBs of the 2nd filter output to 10bit baseline value (5LSBs become
    zero.  Subsequent update is per nominal baseline filter operation.
  - 3 Baseline tracking and initialize enable. At the first {ESI x SFI}, MPR121
    copy the 2nd filter output to 10bit baseline value. Subsequent update is per
    nominal baseline filter operation.
  */
  configuration-lock mode/int -> none:
    assert: 0 <= mode <= 3
    write-register_ REG-ECR_ mode --mask=ECR-CONFIGURATION-LOCK-MASK_

  /**
  Enable autoconfig option - enable / disable autoconfig.

  When enabled, the MPR121 automatically searches and sets the charging
   parameters for every enabled pad. This happens on each time the MPR121
   transitions from Stop Mode to Run Mode.
  */
  set-autoconfig_ --autoconfig/bool -> none:
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
      write-register_ REG-ACCR0_ 0b00001011
      logger_.debug "Wrote REG-ACCR0_ with    0x$(%04x 0b00001011) [$(bits-16_ 0b00001011)]"

      // details on values
      // https://www.nxp.com/docs/en/application-note/AN3889.pdf#page=7&zoom=310,-42,792
      // correct values for Vdd = 3.3V
      //reg_.write-u8 USL-REGISTER-ADDRESS_ 200                 // ((Vdd - 0.7)/Vdd) * 256
      write-register_ REG-USL_ 200     // ((Vdd - 0.7)/Vdd) * 256
      write-register_ REG-TL_  180     // UPLIMIT * 0.9
      write-register_ REG-LSL_ 130     // UPLIMIT * 0.65
    else:
      // really only disable ACE.
      write-register_ REG-ACCR0_ 0b00001010
      logger_.debug "Wrote REG-ACCR0_ with    0x$(%04x 0b00001010) [$(bits-16_ 0b00001010)]"

  afe1-register-reset -> none:
    value := read-register_ REG-FILTER-GLOBAL-CDC_
    write-register_ REG-FILTER-GLOBAL-CDC_ FILTER-GLOBAL-CDC-DEFAULT_

  afe2-register-reset -> none:
    value := reg_.read-u8 REG-FILTER-GLOBAL-CDT_
    write-register_ REG-FILTER-GLOBAL-CDT_ FILTER-GLOBAL-CDT-DEFAULT_


  /** charge-discharge-current: Sets individual Charge Discharge Current per
  channel if Global value is not used.

  The Individual Charge Discharge Current field selects the supply current
  to be used when charging and discharging a specific channel. Programmable
  in 1uA steps, up to 64uA.  When the CDCx is zero, the global value is
  used.  If auto configuration is enabled and run once, the individual CDC
  will be automatically updated by MPR121 internally after autoconfiguration
  is finished.
  */
  channel-charge-discharge-current --channel/int -> int:
    assert: 0 <= channel <= 12
    value := read-register_ (REG-CDC0_ + channel)
    return value

  channel-charge-discharge-current --channel/int --current/int -> none:
    assert: 0 <= channel <= 12
    assert: 0 <= current <= CHARGE-DISCHARGE-CURRENT-MAX_
    value := read-register_ (REG-CDC0_ + channel)
    write-register_ (REG-CDC0_ + channel) current
    logger_.debug "channel-charge-discharge-current: channel$(channel) register=$(bits-16_ (REG-CDC0_ + channel)) value=$(current)"

  /**
  Sets the charge time applied to each channel.

  Similar to the global CDT value, the range is 0~32 μS, from 0b000-0b111  When
   the CDTx is zero, the global CDT value is used for that channel.
  */
  set-channel-charge-discharge-time time/int --channel/int -> none:
    // 3 bits are held for 12 channels, across 6 registers, two channels per register, with one bit unused.
    assert: 0 <= time <= 7
    assert: 0 <= channel <= 12
    register := REG-CDT0_ + (channel / CDT-FIELD-DIVISOR_)
    shift    := (CDT-FIELD-OFFSET_ * (channel % CDT-FIELD-DIVISOR_))
    write-register_ register time --mask=CDT-CHARGE-TIME-MASK_ --offset=shift

  get-channel-charge-discharge-time --channel/int -> int:
    // 3 bits are held for 12 channels, across 6 registers, two channels per register, with one bit unused.
    register := REG-CDT0_ + (channel / CDT-FIELD-DIVISOR_)
    shift     := (CDT-FIELD-OFFSET_ * (channel % CDT-FIELD-DIVISOR_))
    return read-register_ register --mask=CDT-CHARGE-TIME-MASK_ --offset=shift

  /** GPIO and LED Driver function
      Among the 12 electrode inputs, 8 inputs are designed as multifunctional pins (D0 - D7 bits in the register correspond to GPIO and LED functions on ELE4 - ELE11). When these pins are not configured as electrodes, they may be used to drive LEDs or used for general purpose input or output. For more details on this feature, please refer to application note AN3894.
      https://www.nxp.com/docs/en/application-note/AN3894.pdf

      Note: The number of touch sensing electrodes, and therefore the number of GPIO ports left available is configured by the ECR (0x5E) and GPIO Enable Register (0x77). ECR has higher priority and overrides the GPIO enabled in 0x77 - that is, when a pin is enabled as GPIO but is also selected as electrode by ECR, the GPIO function is disabled immediately and it becomes an electrode during Run Mode.

      Note: Be aware that the channels/electrodes cannot be turned on or off without following the 'total number of touch channels enabled' sequence.  This means that using the first 6 channels for touch and wanting to use the second channel for GPIO, won't work.  This means that using the higher end channels for GPIO might need to be done first, and then to start reducing the number of channels for touch one by one.
      */
/*
  gpio-pin-function --pin/int --disable -> none:
    assert: 4 <= pin <= 11
    old-value := reg_.read-u8 GPIO-EN-REGISTER-ADDRESS_
    new-value       := (old-value &  ~(1 << pin - 4)) & 0xFF
    reg_.write-u8 GPIO-EN-REGISTER-ADDRESS_ new-value
    logger_.debug "gpio-pin-function  disable pin $(pin) from=$(bits-16_ old-value) to=$(bits-16_ new-value)"
    sleep --ms=1

  gpio-pin-function --pin/int --enable --input -> none:
    assert: 4 <= pin <= 11
    old-EN-reg-value := reg_.read-u8 GPIO-EN-REGISTER-ADDRESS_
    new-EN-reg-value := (old-EN-reg-value |  (1 << pin - 4)) & 0xFF                 //0 in EN
    reg_.write-u8 GPIO-EN-REGISTER-ADDRESS_ new-EN-reg-value
    logger_.debug "gpio-pin-function  enable  pin $(pin) INPUT from=$(bits-16_ old-EN-reg-value) to=$(bits-16_ new-EN-reg-value)"
    old-CTL0-reg-value := reg_.read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := reg_.read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL1
    reg_.write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    reg_.write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    old-DIR-reg-value := reg_.read-u8 GPIO-DIR-REGISTER-ADDRESS_
    new-DIR-reg-value := (old-DIR-reg-value &  ~(1 << pin - 4)) & 0xFF              //0 in DIR
    reg_.write-u8 GPIO-DIR-REGISTER-ADDRESS_ new-DIR-reg-value
    sleep --ms=1

  gpio-pin-function --pin/int --enable --input --pulldown -> none:
    assert: 4 <= pin <= 11
    gpio-pin-function --pin=pin --enable --input
    old-CTL0-reg-value := reg_.read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := reg_.read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL1
    reg_.write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    reg_.write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    logger_.debug "gpio-pin-function  enable internal pulldown on pin $(pin)" // from=$(bits-16_ old-value) to=$(bits-16_ new-value)"
    sleep --ms=1

  gpio-pin-function --pin/int --enable --input --pullup -> none:
    assert: 4 <= pin <= 11
    gpio-pin-function --pin=pin --enable --input
    old-CTL0-reg-value := reg_.read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := reg_.read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL1
    reg_.write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    reg_.write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    logger_.debug "gpio-pin-function  enable internal pullup on pin $(pin)" // from=$(bits-16_ old-value) to=$(bits-16_ new-value)"
    sleep --ms=1

  gpio-pin-function --pin/int --enable --output -> none:
    assert: 4 <= pin <= 11
    old-EN-reg-value := reg_.read-u8 GPIO-EN-REGISTER-ADDRESS_
    new-EN-reg-value := (old-EN-reg-value |  (1 << pin - 4)) & 0xFF                 //0 in EN
    reg_.write-u8 GPIO-EN-REGISTER-ADDRESS_ new-EN-reg-value
    logger_.debug "gpio-pin-function  enable  pin $(pin) OUTPUT from=$(bits-16_ old-EN-reg-value) to=$(bits-16_ new-EN-reg-value)"
    old-CTL0-reg-value := reg_.read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := reg_.read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL1
    reg_.write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    reg_.write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    old-DIR-reg-value := reg_.read-u8 GPIO-DIR-REGISTER-ADDRESS_
    new-DIR-reg-value := (old-DIR-reg-value  |  (1 << pin - 4)) & 0xFF              //1 in DIR
    sleep --ms=1

  gpio-pin-function --pin/int --enable --output --hi-side-open-drain-led -> none:
    assert: 4 <= pin <= 11
    gpio-pin-function --pin=pin --enable --output
    old-CTL0-reg-value := reg_.read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := reg_.read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL1
    reg_.write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    reg_.write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    logger_.debug "gpio-pin-function  enable hi-side-open-drain-led on pin $(pin)" // from=$(bits-16_ old-value) to=$(bits-16_ new-value)"
    sleep --ms=1

  gpio-pin-function --pin/int --enable --output --lo-side-open-drain -> none:
    assert: 4 <= pin <= 11
    gpio-pin-function --pin=pin --enable --output
    old-CTL0-reg-value := reg_.read-u8 GPIO-CTL0-REGISTER-ADDRESS_
    old-CTL1-reg-value := reg_.read-u8 GPIO-CTL1-REGISTER-ADDRESS_
    new-CTL0-reg-value := (old-CTL0-reg-value |  (1 << pin - 4)) & 0xFF             //1 in CTL0
    new-CTL1-reg-value := (old-CTL1-reg-value &  ~(1 << pin - 4)) & 0xFF            //0 in CTL1
    reg_.write-u8 GPIO-CTL0-REGISTER-ADDRESS_ new-CTL0-reg-value
    reg_.write-u8 GPIO-CTL1-REGISTER-ADDRESS_ new-CTL1-reg-value
    logger_.debug "gpio-pin-function  enable lo-side-open-drain on pin $(pin)" // from=$(bits-16_ old-value) to=$(bits-16_ new-value)"
    sleep --ms=1

  /** get-gpio-pin: returns DAT for the pin.
      When a GPIO is enabled as an output, the GPIO port outputs the corresponding DAT bit level from GPIO Data Register (0x075). The output level toggle remains on during any electrode charging. The level transition will occur after the ADC conversion takes place. It is important to note that reading this register returns the content of the GPIO Data Register, (not a level of the port). When a GPIO is configured as input, reading this register returns the latched input level of the corresponding port (not contents of the GPIO Data Register). Writing to the DAT changes content of the register, but does not effect the input function
      */
  get-gpio-pin --pin/int --get -> int:
    old-DAT-reg-value := reg_.read-u8 GPIO-DAT-REGISTER-ADDRESS_
    return  (old-DAT-reg-value & (1 << pin - 4))

  /** other GPIO functions:
      Writing “1” into the corresponding bits of GPIO Data Set Register, GPIO Data Clear Register, and GPIO Data Toggle Register will set/clear/toggle contents of the corresponding DAT bit in Data Register.  These functions are reproduced here.  (Writing “0” has no meaning.)  These registers allow any individual port(s) to be set, cleared, or toggled individually without affecting other ports. It is important to note that reading these registers returns the contents of the GPIO Data Register reading.
      */
  set-gpio-pin --pin/int --set -> none:
    old-SET-reg-value := reg_.read-u8 GPIO-SET-REGISTER-ADDRESS_
    new-SET-reg-value := (old-SET-reg-value |  (1 << pin - 4))                //1 in SET
    reg_.write-u8 GPIO-SET-REGISTER-ADDRESS_ new-SET-reg-value
    logger_.debug "set-gpio-pin  set  pin $(pin) OUTPUT from=$(bits-16_ old-SET-reg-value) to=$(bits-16_ new-SET-reg-value)"

  clear-gpio-pin --pin/int --clear -> none:
    old-CLR-reg-value := reg_.read-u8 GPIO-CLR-REGISTER-ADDRESS_
    new-CLR-reg-value := (old-CLR-reg-value |  (1 << pin - 4))                //1 in CLR
    reg_.write-u8 GPIO-CLR-REGISTER-ADDRESS_ new-CLR-reg-value
    logger_.debug "clear-gpio-pin  clear  pin $(pin) OUTPUT from=$(bits-16_ old-CLR-reg-value) to=$(bits-16_ new-CLR-reg-value)"

  toggle-gpio-pin --pin/int --toggle -> none:
    old-TOG-reg-value := reg_.read-u8 GPIO-TOG-REGISTER-ADDRESS_
    new-TOG-reg-value := (old-TOG-reg-value |  (1 << pin - 4))                //1 in TOG
    reg_.write-u8 GPIO-TOG-REGISTER-ADDRESS_ new-TOG-reg-value
    logger_.debug "toggle-gpio-pin  toggle  pin $(pin) OUTPUT from=$(bits-16_ old-TOG-reg-value) to=$(bits-16_ new-TOG-reg-value)"
*/

  read-register_
      register/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> any:
    assert: (width == 8) or (width == 16)
    if mask == null:
      mask = (width == 16) ? 0xFFFF : 0xFF
    if offset == null:
      offset = mask.count-trailing-zeros

    register-value/int? := null
    if width == 8:
      if signed:
        register-value = reg_.read-i8 register
      else:
        register-value = reg_.read-u8 register
    if width == 16:
      if signed:
        register-value = reg_.read-i16-be register
      else:
        register-value = reg_.read-u16-be register

    if register-value == null:
      logger_.error "read-register_: Read failed."
      throw "read-register_: Read failed."

    if ((mask == 0xFFFF) or (mask == 0xFF)) and (offset == 0):
      return register-value
    else:
      masked-value := (register-value & mask) >> offset
      return masked-value

  write-register_
      register/int
      value/any
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> none:
    assert: (width == 8) or (width == 16)
    if mask == null:
      mask = (width == 16) ? 0xFFFF : 0xFF
    if offset == null:
      offset = mask.count-trailing-zeros

    field-mask/int := (mask >> offset)
    assert: ((value & ~field-mask) == 0)  // fit check

    // Full-width direct write
    if ((width == 8)  and (mask == 0xFF)  and (offset == 0)) or
      ((width == 16) and (mask == 0xFFFF) and (offset == 0)):
      if width == 8:
        signed ? reg_.write-i8 register (value & 0xFF) : reg_.write-u8 register (value & 0xFF)
      else:
        signed ? reg_.write-i16-be register (value & 0xFFFF) : reg_.write-u16-be register (value & 0xFFFF)
      return

    // Read Reg for modification
    old-value/int? := null
    if width == 8:
      if signed :
        old-value = reg_.read-i8 register
      else:
        old-value = reg_.read-u8 register
    else:
      if signed :
        old-value = reg_.read-i16-be register
      else:
        old-value = reg_.read-u16-be register

    if old-value == null:
      logger_.error "write-register_: Read existing value (for modification) failed."
      throw "write-register_: Read failed."

    new-value/int := (old-value & ~mask) | ((value & field-mask) << offset)

    if width == 8:
      signed ? reg_.write-i8 register new-value : reg_.write-u8 register new-value
      return
    else:
      signed ? reg_.write-i16-be register new-value : reg_.write-u16-be register new-value
      return

    throw "write-register_: Unhandled Circumstance."

  /**
  Provides strings to display bitmasks nicely when testing.
  */
  bits-16_ x/int --min-display-bits/int=0 -> string:
    assert: (x >= 0) and (x <= 0xFFFF)
    if (x > 255) or (min-display-bits > 8):
      out-string := "$(%b x)"
      out-string = out-string.pad --left 16 '0'
      out-string = "$(out-string[0..4]).$(out-string[4..8]).$(out-string[8..12]).$(out-string[12..16])"
      return out-string
    else if (x > 15) or (min-display-bits > 4):
      out-string := "$(%b x)"
      out-string = out-string.pad --left 8 '0'
      out-string = "$(out-string[0..4]).$(out-string[4..8])"
      return out-string
    else:
      out-string := "$(%b x)"
      out-string = out-string.pad --left 4 '0'
      out-string = "$(out-string[0..4])"
      return out-string


  /**
  Removes all tasks, purpose of stopping debugging task.
  */
  stop-all-tasks -> none:
    tasks_.keys.do:
      tasks_[it].cancel
      logger_.info "stop-all: task '$(it)' is cleared"
    tasks_.clear

  /**
  Sets a task which prints the bitmask of keys touched, for dubugging purposes.
  */
  debug-touched -> none:
    stop-all-tasks
    tasks_["debug-touched"] = task:: debug-touched_
    logger_.info "debug-touched: task is set"

  /**
  Logs a debug whenever a channel is touched or released.
  */
  debug-touched_ -> none:
    out/int := 0
    while true:
      out = touched
      if out > 0:
        logger_.debug "debug-touched: $(bits-16_ out --min-display-bits=16)"
      sleep --ms=50

  /**
  Logs a bitmask of the AFE1 register.
  */
  show-afe1-register -> none:
    value := read-register_ REG-FILTER-GLOBAL-CDC_
    logger_.debug "read-afe1-register : read REG-FILTER-GLOBAL-CDC_  has 0x$(%04x value) [$(bits-16_ value)]"

  /**
  Logs a bitmask of the AFE2 register.
  */
  show-afe2-register -> none:
    value := read-register_ REG-FILTER-GLOBAL-CDT_
    logger_.debug "read-afe2-register : read REG-FILTER-GLOBAL-CDT_  has 0x$(%04x value) [$(bits-16_ value)]"

  /**
  Logs a bitmask of the ACCR0 register.
  */
  show-accr0-register -> none:
    value := read-register_ REG-ACCR0_
    logger_.debug "read-accr0-register: read REG-ACCR0_ has 0x$(%04x value) [$(bits-16_ value)]"

  /**
  Logs a bitmask of the ACCR1 register.
  */
  show-accr1-register -> none:
    value := read-register_ REG-ACCR1_
    logger_.debug "read-accr1-register: read REG-ACCR1_ has 0x$(%04x value) [$(bits-16_ value)]"
