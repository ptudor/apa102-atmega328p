
# apa102 controller

Arduino sketch. Program as Nano. Onboard USBASP.

The README for the PCB itself provides a background on the hardware.
Here we assume a general understanding of the various components
that we interact with (GPS, I2C, RTC, and so on) and look at the software.

***

### Overview of Libraries

First we bring in some standard AVR libraries: stdio.h, wdt.h, and the new time.h.

Next we do basic microcontroller things with EEPROM.h, Wire.h, and SPI.h.

Then we bring in the classic Time library, named TimeLib.h in 1.6+, and fetch time from the sky with TinyGPS++.h. The RTC is addressed with MCP79412RTC.h and MCP9804.h is for the temperature sensor.

Finally, we find FastLED.h for the core function of this board.

### arduino-style pin indentification on the microcontroller

| Pin | Macro           | Description            |
| ---:|:--------------- |:---------------------- |
|   0 | `SERIAL_RXD`    | TTL NMEA in from GPS   |
|   1 | `SERIAL_TXD`    | TTL Transmit from MCU  |
|   2 | `INT0_PPS`      | PPS interrupt from GPS |
|   3 | `INT1_SW`       | Switch interrupt       |
|   4 | `MCU_GPS_EN`    | Active high GPS power  |
|   5 | `LED_GREEN`     | PWM LED, green         |
|   6 | `LED_WHITE`     | PWM LED, white         |
|   7 | `DEBOUNCED_D7`  | Extra input            |
|   8 | `DEBOUNCED_D8`  | Extra input            |
|   9 | `LED_RED`       | PWM LED, red           |
|  10 | `DEBOUNCED_D10` | Extra input            |
|  11 | `MOSI`          | FastLED Data           |
|  12 | `MISO`          | MISO                   |
|  13 | `SCK`           | FastLED Clock          |
|  A0 | `ANALOG_A0`     | not connected          |
|  A1 | `TRIMPOT_A1`    | Trimpot input          |
|  A2 | `TRIMPOT_A2`    | Trimpot input          |
|  A3 | `ANALOG_A3`     | not connected          |
|  A4 | `SDA`           | I2C                    |
|  A5 | `SCL`           | I2C                    |


## A General Explanation of the Sample Code
An overview of what the program does in broad terms. Notes on the demonstration code.

### DEFINE

```cpp
#define NUM_LEDS 90
#define LED_TYPE APA102
#define COLOR_ORDER BGR
```

Here one must configure FastLED settings. In this example, I have three one-meter thirty-led strips connected, using the APA102 chipset with a color order of blue-green-red.

### interrupt()
The first interrupt is dedicated to time pulses from the navigation chip. It indicates the time should be correct and is used as a bit of a timer:

```cpp
if ( pps_loop > 53) pps_loop = 0;
```

One of the primary methods for user interaction is the momentary switch activated by depressing one of the front panel trimpots; a secondary onboard switch connected to INT1 assists development.

This button can be used to choose the current pattern by cycling through the predefined collection. To set the upper-bound and automatically wrap-around at the end, we use modulus:

```cpp
momentary_switch_loop = momentary_switch_loop % 6;
```

### setup()
Immediately we enable the watchdog timer: `wdt_enable(WDTO_8S);`. The corresponding `wdt_reset();` is the last instruction in the loop(). 

After that protection, we set the pin modes for the LEDs, interrupts, and trimpots. Next the hardware serial is enabled at 9600 and i2c is activated.

With core bootstrapping complete, we flash the red LED just to let everyone know power is okay or the reset button works. When your LEDs fail to illuminate, it is convenient to at least have confidence the code is running.

Next the code continues into secondary bootstrapping, where we can do things like set the resolution on the temperature sensor and attempt to fetch time from the RTC and set the MCU clock from it.

Now we initialize FastLED with `FastLED.addleds` and quickly display RGB calibration colors to ensure that is properly defined.

As the last step we begin listening for external interrupts and i2c requests.

### loop()

The key element of the loop is the `switch (momentary_switch_loop)` statement. It uses the current value derived from the momentary switch to display the configured pattern.

GPS related concerns are also monitored here: If we have enough NMEA to know there are less than three satellites in view, the red LED is illuminated. If an interrupt has incremented or after sufficient delay reset the pps_loop variable, the loop attempts to set the time on the RTC with time derived from GPS. Time and location are available so one can imagine automating power on and power off based on the sunrise equation or clock time.

The loop ends with a subtle sine-wave change to an LED on the PCB and uses TinyGPS++'s `smartDelay()` function to continue ingesting NMEA while slowing the rate of updates to `FastLED.show()`. 