/* 

LED Board for Arduino
MIT License

Copyright (c) 2017 Patrick Tudor
https://gitlab.com/apa102-controller

At its core, this board is an ATMega328p one can program like an Arduino Nano.
Its purpose is to drive APA102 (or whatever) LEDs on MOSI and SCK directly.

The mega328p has a standard ISP connector for programming. Other pins are:
D0: RXd from GPS 
D1: TXd to FT231
D2: INT0 from GPS PPS
D3: INT1 from trimpot switch, debounced
D4: GPS Enable
D5: LED (Green, PWM)
D6: LED (White, PWM)
D7: header, debounced
D8: header, debounced
D9: LED (Red, PWM)
D10: header, debounced
D11: MOSI (to LEDs)
D12: MISO
D13: SCK (to LEDs)
A0: header, analog
A1: trimpot, analog
A2: trimpot, analog
A3: NC
A4: SDA
A5: SCL

The USB provides four devices through an onboard hub:
uBlox GPS
USBASP Programmer
CP2112 I2C
FT230 Serial (transmit only)

There are several devices on the I2C bus. In order, from the front of the board, we have:
MCP9808 Temp Sensor
ATSHA
MCP79412 RTC (Same as MCP79408, but with UUID)
The primary I2C connects to USB with a CP2112. There is also an independent I2C eeprom for the GPS that can probably be left empty.

Primary power is provided by a DC jack that requires 5V. There is no onboard 5V regulator.
Laptop-style switched mode power supplies that offer several amps at 5V are fine.
There are actually five different powers:
Primary inbound 5V, via an 8A fuse.
The primary 5V connects to the LED 5V via a 6A fuse. The fuse could be replaced with a switch.
From this 5V we have a 3V3 RT9193 for the GPS.
USB power is connected via a 500mA fuse directly to the USBASP and FT230.
USB power also drives a 3V3 AMS117 for the CP2112 and TUSB.
There's a solder jumper to connect the mega328's AREF to 5V.

*/

#define DEBUG 0

#ifndef _HEADERS_JEMMA
#define _HEADERS_JEMMA
#include <stdio.h> /* for itoa(); */
#include <avr/wdt.h> // watchdog timer

#include <time.h>
// #include <util/usa_dst.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <Wire.h>  // http://arduino.cc/en/Reference/Wire
#include <SPI.h>
#include <MCP79412RTC.h>  //http://github.com/JChristensen/MCP79412RTC
#include <TinyGPS++.h>        // GPS Library
// https://github.com/JChristensen/MCP9808
// https://github.com/JChristensen/MCP79412RTC
#include "FastLED.h"
//#include <sha204_library.h>
#include <MCP9804.h>
// use time_t sun_rise and time_t sun_set someday instead. http://www.nongnu.org/avr-libc/user-manual/group__avr__time.html
// #include <sunMoon.h> // https://github.com/sfrwmaker/sunMoon
#include <SHA204.h> // https://github.com/nuskunetworks/arduino_sha204/
#include <SHA204Definitions.h>
#include <SHA204I2C.h>


#endif

#define PCB_REVISION_A 1

#if PCB_REVISION_A

#define SERIAL_RXD 0
#define SERIAL_TXD 1
#define INT0_PPS 2
#define INT1_SW 3
#define MCU_GPS_EN 4
#define LED_GREEN 5
#define LED_WHITE 6
#define DEBOUNCED_D7 7
#define DEBOUNCED_D8 8
#define LED_RED 9
#define DEBOUNCED_D10 10
#define MOSI 11
#define MISO 12
#define SCK 13
#define ANALOG_A0 A0
#define TRIMPOT_A1 A1
#define TRIMPOT_A2 A2
#define ANALOG_A3 A3
#define SDA A4
#define SCL A5

#endif

#define MCU_I2C_NODE_ADDRESS 8
#define GPS_ENABLED 1

#define NUM_LEDS 90
#define LED_TYPE    APA102
#define COLOR_ORDER BGR
#define BRIGHTNESS          128
#define FRAMES_PER_SECOND  120
CRGB leds[NUM_LEDS];

MCP9804 sensor9808(0x18); // default address 0x1f

TinyGPSPlus gps;

SHA204I2C sha204dev;

//atsha204Class sha204(sha204Pin);

namespace TinyGpsPlusPlus {
  // This version of delay() from TinyGPS ensures that the gps object is being "fed".
  static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do 
    {
      //while (ss.available()) //softwareserial
      //gps.encode(ss.read());  //softwareserial
      while (Serial.available())
        gps.encode(Serial.read());
    } while (millis() - start < ms);
  }
} // end namespace TinyGpsPlusPlus


static const uint32_t GPSBaud = 9600;     
unsigned int x = 0;
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

int tempCelcius;
byte gpsError = 0;
byte rtcError = 0;

volatile byte ffr_loop = 0;
void ffr_interrupt() {
  ffr_loop++;
  if ( ffr_loop > 3) {
      ffr_loop= 0;
    }
}

volatile time_t currentEpoch;
volatile byte pps_loop = 0;
volatile byte momentary_switch_loop = 0;

void momentary_switch_interrupt(){
    momentary_switch_loop++;
    // mod 4 gives wraparound.
    momentary_switch_loop = momentary_switch_loop % 6;
}

void pps_interrupt(){
    pps_loop++;
    if ( pps_loop > 53) {
        pps_loop = 0;
    }
}


time_t epochConverter(TinyGPSDate &d, TinyGPSTime &t) {
  // make the object we'll use
  tmElements_t tm;
  // if the TinyGPS time is invalid, we'll return past time to make it obvious.
  if (!t.isValid()) {
    return 0;
  } else {
    // GPS time is probably valid, let's construct seconds from it
    tm.Year =  CalendarYrToTm(d.year()); // TM offset is years from 1970, i.e. 2014 is 44.
    tm.Month =  d.month();
    tm.Day =  d.day();
    tm.Hour =  t.hour();
    tm.Minute =  t.minute();
    tm.Second =  t.second();
    time_t epochtime = makeTime (tm);
    setTime(epochtime);
    adjustTime(1);
    // we add one here because the time is from the NMEA sentence preceding the PPS signal
    return epochtime + 1 ; // + timeZoneOffset + daylightTimeOffset;
  }
  // nothing matched? Shouldn't arrive here.
  return 1320001666;
}


void setTimeFromGps(time_t currentEpoch) {
  // this is our time elements object,
  tmElements_t tm;
  RTC.write(tm); // or use the set below. write is tmElements and set is time_t
  // that we "break" into components, like Hour/Minute/Second
  breakTime(currentEpoch, tm);
  // because tm.Year is YYYY - 1970, and we want tmYY + 1970
  int year2k = tmYearToCalendar(tm.Year);
  Serial.println(year2k);   //set the system time to 23h31m30s on 13Feb2009

  setTime(tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, year2k);   //set the system time to 23h31m30s on 13Feb2009
  // this line commented out because of the RTC.write above.
  // RTC.set(now());                     //set the RTC from the system time

  rtcError = 0;
  if(timeStatus()!= timeSet)  {
     Serial.println("Unable to sync with the RTC");
     rtcError = 1;
  } else {
     Serial.println("Just set RTC from GPS okay.");
     rtcError = 0;
  }
}



// https://github.com/FastLED/FastLED/blob/master/examples/RGBCalibrate/RGBCalibrate.ino
void RGBCalibrate() {
   // "If the RGB ordering is correct, you should see 1 red led, 2 green leds, and 3 blue leds."
   leds[0] = CRGB(255,0,0); 
   leds[1] = CRGB(0,255,0);
   leds[2] = CRGB(0,255,0);
   leds[3] = CRGB(0,0,255);
   leds[4] = CRGB(0,0,255);
   leds[5] = CRGB(0,0,255);
   FastLED.show();
}

void setSystemClockFromRTC() {

    //setSyncProvider() causes the Time library to synchronize with the
    //external RTC by calling RTC.get() every five minutes by default.
    setSyncProvider(RTC.get);
    
    if(timeStatus()!= timeSet)  {
     Serial.println("Unable to sync with the RTC");
     rtcError = 1;
     setTime(12, 34, 30, 29, 2, 2016);   //set the system time to 12h34m30s on 29Feb2016
     //RTC.adjust(DateTime(__DATE__, __TIME__));
     RTC.set(now());                     //set the RTC from the system time
} else {
     Serial.println("RTC has set the system time");
     rtcError = 0;     
     //ptsat RTC.set(now());                     //set the RTC from the system time
}


    byte rtcID[8];
   RTC.idRead(rtcID);
    Serial.print(F("RTC ID = "));
    for (int i=0; i<8; ++i) {
        if (rtcID[i] < 16) Serial.print('0');
        Serial.print((rtcID[i]), HEX);
    }
    Serial.println("");
    Serial.print (F("Calibration Register = "));
    Serial.print (RTC.calibRead() );

}

byte wakeupSha204() {
  uint8_t response[SHA204_RSP_SIZE_MIN];
  byte returnValue;
  
  returnValue = sha204dev.resync(4, &response[0]);
  for (int i=0; i<SHA204_RSP_SIZE_MIN; i++) {
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  return returnValue;
}

byte serialNumberSha204() {
  uint8_t serialNumber[9];
  byte returnValue;
  
  returnValue = sha204dev.serialNumber(serialNumber);
  Serial.print("Serial: ");
  for (int i=0; i<9; i++) {
    Serial.print(serialNumber[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
 
  return returnValue;
}


void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void xmas() 
{
  // FastLED's built-in rainbow generator
 // fill_rainbow( leds, NUM_LEDS, gHue, 7);
 fill_solid(leds, NUM_LEDS, CRGB::DarkGreen);
  fill_gradient(leds,0,CHSV(0,255,255),19,CHSV(85,255,120),SHORTEST_HUES); 
  fill_gradient(leds,70,CHSV(85,255,120),89,CHSV(0,255,255),SHORTEST_HUES); 
}


void pink() 
{
  // FastLED's built-in rainbow generator
 // fill_rainbow( leds, NUM_LEDS, gHue, 7);
 fill_solid(leds, NUM_LEDS, CRGB::LightPink);
}


void january() 
{
  // FastLED's built-in rainbow generator
 // fill_rainbow( leds, NUM_LEDS, gHue, 7);
  fill_solid(leds, NUM_LEDS, CRGB::DarkBlue);
  fill_gradient(leds,0,CHSV(0,255,255),39,CHSV(85,255,120),SHORTEST_HUES); 
  fill_gradient(leds,50,CHSV(85,255,120),89,CHSV(0,255,255),SHORTEST_HUES); 
}

void januaryWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  january();
  addGlitter(10);
}

void xmasWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  xmas();
  addGlitter(10);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}


void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13,0,NUM_LEDS);
  leds[pos] += CHSV( gHue, 255, 192);
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void setup() {
  // Eight second watchdog timer
  wdt_enable(WDTO_8S);
  // These three pins are PWM LEDs
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_WHITE, OUTPUT);
  // This is high or low to enable or disable the voltage regulator GPS chip is on
  pinMode(MCU_GPS_EN, OUTPUT);
  // These two pins receive interrupts
  pinMode(INT0_PPS, INPUT);
  pinMode(INT1_SW, INPUT);
  // These pins have ALPS pots
  pinMode(TRIMPOT_A1, INPUT);
  pinMode(TRIMPOT_A2, INPUT);
  // Open hardware serial communication, GPS Rx and FTDI Tx.
  Serial.begin(9600);
  // init i2c for everything else
  Wire.begin(MCU_I2C_NODE_ADDRESS);
  // "Be sure to wake up device right as I2C goes up otherwise you'll have NACK issues"
  sha204dev.init();
  // power on the GPS, or power it off. Whatever.
  if (GPS_ENABLED == 1) {
    digitalWrite(MCU_GPS_EN, HIGH);
  } else {
    digitalWrite(MCU_GPS_EN, LOW);
  }
  // Blink the red LED to acknowledge reset/boot
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_RED, HIGH);
    delay(50);
    digitalWrite(LED_RED, LOW);
    delay(100);
  }
  // Temperature sensor
  sensor9808.setResolution(MCP9804::R_0_0625);
  int tempC_9808 = val_to_temp(sensor9808.readTemperature());


  setSystemClockFromRTC();
/*  
 * avr-libc-2.0 style  
 *  
  struct tm rtc_time;
  read_rtc(&rtc_time);
   rtc_time.tm_isdst = 0;
   set_system_time( mktime(&rtc_time) );
  set_position( 40.7142 * ONE_DEGREE, -74.0064 * ONE_DEGREE); 
  time_t sunRise = sun_rise(1);
*/

  // begin FastLED with values defined above for pins, colors, type, and quantity
  FastLED.addLeds<LED_TYPE,MOSI,SCK,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  // Dunno.
  set_max_power_in_volts_and_milliamps(5, 3000);               // FastLED Power management set at 5V, 1500mA.

  // This displays the RGB color order test so you can set the value correctly
  RGBCalibrate();
  delay(1500);
  
  // begin listening for PPS and button pushes
  attachInterrupt(0, pps_interrupt, RISING);
  attachInterrupt(1, momentary_switch_interrupt, RISING);
  // begin listening for i2c requests
  Wire.onRequest(requestEvent);

}

unsigned long previousMillis =  millis();
byte lastUniquePulse = 0;
byte noFix = 1;

//byte gpsError = 1;
// the loop function runs over and over again forever
void loop() {
  EVERY_N_MILLISECONDS( 53 ) { gHue++; } // slowly cycle the "base color" through the rainbow was 20

      noFix = 0; // reset it please
    gpsError = 0; // reset
  
     if (gps.satellites.value() <= 3 ) {
        noFix = 1;
        gpsError = 1;
     }
    digitalWrite(LED_RED, noFix);

      if (pps_loop == 1 ) {
    //convert GPS time to Epoch time
    currentEpoch =  epochConverter(gps.date, gps.time);
    // and display it on the LCD
    Serial.println("main loop trying to setTimeFromGps"); 
    setTimeFromGps(currentEpoch);
      pps_loop++; // otherwise it runs several times during the second

     }


  // This section is what makes colors show up. The value of the 
  // variable in this case/switch is set by the interrupt handler
  // attached to a physical momentary switch.
  switch (momentary_switch_loop) {
    case 1:
      januaryWithGlitter();
      break;
    case 2:
      xmasWithGlitter();
      break;
    case 3:
      sinelon();
      break;
    case 4:
      juggle();
      break;
    default: 
      pink();
    break;
  }



  int outVal = yforx(x);
  analogWrite(LED_WHITE, outVal);
   x++;
 

   FastLED.show();  
   // pause main loop 17ms
   TinyGpsPlusPlus::smartDelay(29);

    wdt_reset();
}


void requestEvent()
{
  //Wire.write(transmitCommands,MAX_SENT_BYTES);  
}

int yforx(int x) {
  return (-240*abs(sin(x*0.01)))+255; //sine wave
}


float val_to_temp(int val)
{
  float temp = (val & 0x3ff) / 16.0;
  if (val & 0x800) {
    temp *= -1;
  }
  return temp; 

}

