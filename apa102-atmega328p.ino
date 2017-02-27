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

*/

#define DEBUG 0
#define WRITE_TIMEZONE_EEPROM

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
#include <Timezone.h>    //https://github.com/JChristensen/Timezone
#include "FastLED.h"
//#include <sha204_library.h>
#include <MCP9804.h>
// use time_t sun_rise and time_t sun_set someday instead. http://www.nongnu.org/avr-libc/user-manual/group__avr__time.html
// #include <sunMoon.h> // https://github.com/sfrwmaker/sunMoon
#include <cryptoauth.h> // https://github.com/thiseldo/cryptoauth-arduino


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

static const uint8_t eepromSavedPattern = 32;

#define NUM_LEDS 90
#define LED_TYPE    APA102
#define COLOR_ORDER BGR
// default brightness at boot
#define BRIGHTNESS          128
// these are for if you don't actually want the full range of 0-255
#define BRIGHTNESS_MIN      32
#define BRIGHTNESS_MAX      192
uint8_t brightness = BRIGHTNESS;
uint8_t actualBrightness = BRIGHTNESS;
uint8_t previousBrightness = BRIGHTNESS;

int tempC_9808;

#define FRAMES_PER_SECOND  120
CRGB leds[NUM_LEDS];

MCP9804 sensor9808(0x18); // default address 0x1f

TinyGPSPlus gps;

//SHA204I2C sha204dev;

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
uint8_t sHue = 160; // static base color. 160 +-20 is blue
uint8_t sValue = 192; // static base color. 160 +-20 is blue
uint8_t cHue = 0; // trimpot controlled base color

int tempCelcius;
byte gpsError = 0;
byte rtcError = 0;

AtEccX08 ecc = AtEccX08();
//AtSha204 sha = AtSha204();

// the first time you upload the code, set this so it can be written to eeprom.
// then unset it and upload the code again. saves 64 bytes of RAM when you do.
#ifdef WRITE_TIMEZONE_EEPROM
//US Pacific Time Zone (Las Vegas, Los Angeles)
TimeChangeRule usPDT = {"PDT", Second, dowSunday, Mar, 2, -420};
TimeChangeRule usPST = {"PST", First, dowSunday, Nov, 2, -480};
Timezone usPT(usPDT, usPST);
/*
//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);
*/
/*
//US Eastern Time Zone (New York, Detroit)
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  //Eastern Daylight Time = UTC - 4 hours
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   //Eastern Standard Time = UTC - 5 hours
Timezone usET(usEDT, usEST);
*/
Timezone myTZ(usPDT, usPST);
#endif

//If TimeChangeRules are already stored in EEPROM...
#ifndef WRITE_TIMEZONE_EEPROM
Timezone myTZ(100);       //assumes rules stored at EEPROM address 100
#endif

TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
time_t utc, local;

volatile time_t currentEpoch;
volatile byte pps_loop = 0;
volatile byte momentary_switch_loop = 0;
byte previous_momentary_switch_loop = 0;

void momentary_switch_interrupt() {
  momentary_switch_loop++;
  // mod N gives wraparound.
  momentary_switch_loop = momentary_switch_loop % 8;
}

void pps_interrupt() {
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
  Serial.println(F("Attempting to update RTC from GPS."));
  // this is our time elements object,
  tmElements_t tm;
  // that we "break" into components, like Hour/Minute/Second
  breakTime(currentEpoch, tm);
  RTC.write(tm); // or use the set below. write is tmElements and set is time_t
  // because tm.Year is YYYY - 1970, and we want tmYY + 1970
  int year2k = tmYearToCalendar(tm.Year);
  Serial.println(year2k);   //set the system time to 23h31m30s on 13Feb2009

  setTime(tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, year2k);   //set the system time to 23h31m30s on 13Feb2009
  // this line commented out because of the RTC.write above.
  RTC.set(now());                     //set the RTC from the system time

  rtcError = 0;
  if(timeStatus()!= timeSet)  {
     Serial.println(F("Unable to set RTC from GPS."));
     rtcError = 1;
  } else {
     Serial.println(F("Set RTC from GPS okay."));
     rtcError = 0;
  }
}

void saveSettings(const int eepromAddress, const int newValue) {
  int eepromValue = EEPROM.read(eepromAddress);
  // only write if new value and saved value are different
  if  (eepromValue != newValue ) {
    EEPROM.write(eepromAddress, newValue);
    Serial.print(F("EEPROM Update: "));
    Serial.print(newValue);
  }
}

// this is the left pot sans switch
int readBrightnessFromPot() {
  int valueBrightness = analogRead(TRIMPOT_A1);
  valueBrightness = map(valueBrightness, 0, 1024, 0, 256);     // scale it to use value between 0 and 255
  valueBrightness = constrain(valueBrightness, 0, 255);
  return valueBrightness;
}

// this is the right pot con switch
int readColorFromPot() {
  int valueColor = analogRead(TRIMPOT_A2);
  valueColor = map(valueColor, 0, 1024, 0, 256);     // scale it to use value between 0 and 255
  valueColor = constrain(valueColor, 0, 255);
  return valueColor;
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
     Serial.println(F("Unable to set system clock from RTC"));
     rtcError = 1;
     setTime(12, 34, 30, 29, 2, 2016);   //set the system time to 12h34m30s on 29Feb2016
     //RTC.adjust(DateTime(__DATE__, __TIME__));
     RTC.set(now());                     //set the RTC from the system time
  } else {
     Serial.println(F("RTC has set the system time"));
     rtcError = 0;     
     //ptsat RTC.set(now());                     //set the RTC from the system time
  }
}

void extraRTC() {
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

byte runningTimedSleep = 0;
byte runningTimedWake = 0;

void checkCron() {
  //tmElements_t tm;
  utc = now();
  local = myTZ.toLocal(utc, &tcr);
  // wakeup
  if ( ( hour(local) == 17 ) && (minute(local) == 0 ) ) {
    runningTimedWake = 1;
  } 
  // sleep the first time
  if ( ( hour(local) == 23 ) && (minute(local) == 50 ) ) {
    runningTimedSleep = 1 ;
  }
  // sleep the second time if we interrupted the first
  if ( ( hour(local) == 2 ) && (minute(local) == 0 ) ) {
    runningTimedSleep = 1 ;
  }
}


// https://github.com/JChristensen/MCP79412RTC/blob/master/examples/rtcSet2/rtcSet2.pde
void printTimeFromRTC(time_t tm) {
    Serial.print(hour(tm), DEC);
    Serial.print(':');
    Serial.print(minute(tm),DEC);
    Serial.print(':');
    Serial.print(second(tm),DEC);
    Serial.print(' ');
    Serial.print(year(tm), DEC);
    Serial.print('-');
    Serial.print(month(tm), DEC);
    Serial.print('-');
    Serial.print(day(tm), DEC);
    Serial.println();
}


void atmelRandom() {
    /* If you haven't personalized your device yet, you will recieve
     * this on your serial terminal:
       ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000
       Success
       Otherwise, you'll get actual random bytes.
    */
    if (0 == ecc.getRandom(0)){
        Serial.println("Success with sha.getRandom:");
        ecc.rsp.dumpHex(&Serial);
        //random16_add_entropy(ecc.rsp);
    }
    else{
        Serial.println("Failure on sha.getRandom:");
    }
}

// Keep a record of key slot state for use in error reporting
uint16_t slotLockState = 0xFF;  // Same as SlotLocked, bit per slot, 0=Locked
uint16_t eccKeyState = 0x00;    // Bit per slot, 1 = ECC Key.

/** isSlotLocked - use previously saved flags to test if a key slot has been locked.
 * @param keyNum - Key nubmer to check
 * @return true/false indicating lock state of key
 */
boolean isSlotLocked( uint8_t keyNum ) {
  return slotLockState & (1 << keyNum ) ? false : true;
}

/** displayLockState - display the lock state for the specified zone
 * @param zone - the zone to display, 0 for Config Zone, 1 for Data Zone
 */
void displayLockState( uint8_t zone ) {
  Serial.print( ecc.is_locked( zone ) ? F("") : F(" not" ));
  Serial.println(F(" Locked"));
}
/** displayData - Display packet data in hex.
 *  @param rspPtr packet response pointer
 *  @param bufLen Length of data to display
 */
void displayData( const uint8_t *rspPtr, uint8_t bufLen ) {
  const uint8_t *bufPtr = rspPtr;
  // Display serial number and add to serialNum buffer
  for (int i = 0; i < bufLen; i++ ) {
    if ( bufPtr[i] < 16 ) Serial.print(F("0"));
    Serial.print(bufPtr[i], HEX);
  }
  Serial.println();
}
/** displayResponse - take a low I2C driver response code and display
 *  meaningfull message. Checks key slot flags if needed
 *  @param respCode - response code received from driver
 *  @param keyNum - Key number of the slot being worked on
 */
void displayResponse( uint8_t respCode, uint8_t keyNum ) {
  switch (respCode) {
    case 0xD2:
      Serial.print(F("CMD Fail - Parse Error"));
      break;
    case 0xD3:
      Serial.print(F("CMD Fail - "));
      Serial.print(isSlotLocked(keyNum) ? F("Slot locked") : F("No Private key") );
      break;
    case 0xE7:
      Serial.print(F("No Response"));
      break;
    default:
      Serial.print(respCode, HEX);
  }
  Serial.println();
}
// https://github.com/thiseldo/cryptoauth-arduino/blob/master/examples/Crypto_Examples/Crypto_Examples.ino

/** menuGetInfo - The Chip Info menu action, display info about the chip
 */
void eccGetInfo() {
  Serial.println(F("\n\rATECC Chip Info"));
  uint8_t serialNum[9];

  Serial.print(F("Serial Number: "));
  uint8_t ret = ecc.getSerialNumber();
  if ( ret == 0 ) {
    const uint8_t *bufPtr = ecc.rsp.getPointer();
    int bufLen = ecc.rsp.getLength();
    // Display serial number and add to serialNum buffer
    for (int i = 0; i < bufLen; i++ ) {
      if ( i < 9)
        serialNum[i] = bufPtr[i];
      if ( bufPtr[i] < 16 ) Serial.print(F("0"));
      Serial.print(bufPtr[i], HEX);
    }
    Serial.println();
  } else {
    Serial.print(F("Failed! "));
    Serial.println(ret, HEX);
  }

  // Chip revision
  Serial.print(F("Revision: "));
  ret = ecc.getInfo(0x00, 0);
  if ( ret == 0 ) {
    displayData(ecc.rsp.getPointer(), 4);
  } else {
    Serial.print(F("Failed! "));
    Serial.println(ret, HEX);
  }

  Serial.print(F("Config Zone is" ));
  displayLockState(0);

  Serial.print(F("Data Zone is" ));
  displayLockState(1);

  // Key validity check, E for ECC keys, - for anything else
  Serial.println(F("                111111"));
  Serial.println(F("      0123456789012345"));
  Serial.print(F("Type: "));
  for ( int k = 0; k < 16; k++ ) {
    ret = ecc.getInfo(0x01, k);
    if ( ret == 0 ) {
      const uint8_t *rPtr = ecc.rsp.getPointer();
      if ( *rPtr == 0x01 )  // Has a valid public or private ECC key
        Serial.print(F("E"));
      else
        Serial.print(F("-"));
    } else {
      Serial.print(F("-"));
    }
  }
  Serial.println();

  // Display key lock state, Y for locked, - unlocked. Onlt applied to ECC keys
  Serial.print(F("Lock: "));
  uint8_t respCode = ecc.getKeySlotConfig();
  if ( respCode != 0 ) {
    Serial.print(F("Fail getKeySlotConfig "));
    displayResponse(respCode, 0);
  }
  else
  {
    uint16_t lockState = 0xFFFF;
    memcpy (&lockState, ecc.rsp.getPointer(), 2);
    slotLockState = lockState;      // Save for later slot testing
    for ( int k = 0; k < 16; k++ ) {
      Serial.print(lockState & 0x01 ? F("-") : F("Y"));
      lockState = lockState >> 1;
    }
    Serial.println();
  }
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void xmas() {
  fill_solid(leds, NUM_LEDS, CRGB::DarkGreen);
  fill_gradient(leds,0,CHSV(0,255,255),19,CHSV(85,255,120),SHORTEST_HUES); 
  fill_gradient(leds,70,CHSV(85,255,120),89,CHSV(0,255,255),SHORTEST_HUES); 
}

CRGBPalette16 myPalette;
void tempAsColor() {
  // "Forest, Clouds, Lava, Ocean, Rainbow, and Rainbow Stripes."
  if ( tempC_9808 < 18 ) {
    //fill_solid(leds, NUM_LEDS, CRGB::DarkGreen);
    myPalette = OceanColors_p;
  } else if (tempC_9808 > 30 ) {
    myPalette = HeatColors_p;
  } else {
    //myPalette = RainbowStripesColors_p;
    myPalette = ForestColors_p;
   }
    static byte heat[NUM_LEDS];
    for( int j = 0; j < NUM_LEDS; j++) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8( heat[j], 240);
      CRGB color = ColorFromPalette( myPalette, colorindex);
      int pixelnumber;
      leds[pixelnumber] = color;
    }
}

void pink() {
 fill_solid(leds, NUM_LEDS, CRGB::LightPink);
}
 
byte hDelta = 0;
byte vDelta = 0;
void blueish() {
  sHue = 160; // blue
  sValue = 192; // 75%
  // fetch a random number between 0 and 15.
  hDelta = random8(16);
  vDelta = random8(64);
  // if result is odd number add; if even subtract
  if (hDelta % 2) { 
    sHue = sHue + hDelta;
  } else {
    sHue = sHue - hDelta;      
  }
  if (vDelta % 2) { 
    sValue = sValue + vDelta;
  } else {
    sValue = sValue - vDelta;      
  }
  if( hDelta == 0 ) {
    leds[NUM_LEDS] = CHSV( sHue, 192, 192);
  }
  //   
  leds[ random16(NUM_LEDS) ] =  CHSV( sHue, 192, sValue);
}

byte sDelta = 0;
void speckled() {
  // first fetch a random number between 0 and 31.
  sDelta = random8(32);
  // if result is odd number add; if even subtract
  if (sDelta % 2) { 
    sHue = (cHue + sDelta) & 255;
  } else {
    sHue = cHue - sDelta;      
  }
  leds[ random16(NUM_LEDS) ] =  CHSV( sHue, 192, 192);
}

void january() {
  fill_solid(leds, NUM_LEDS, CRGB::DarkBlue);
  fill_gradient(leds,0,CHSV(0,255,255),39,CHSV(85,255,120),SHORTEST_HUES); 
  fill_gradient(leds,50,CHSV(85,255,120),89,CHSV(0,255,255),SHORTEST_HUES); 
}

void januaryWithGlitter() {
  january();
  addGlitter(10);
}

void xmasWithGlitter() {
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

void sinelonColorPot(){
  // a colored dot sweeping back and forth, with fading trails
  //fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13,0,NUM_LEDS);
  leds[pos] += CHSV( cHue, 255, 192);
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

void getTempC() {
  // Temperature sensor
  Serial.print(F("Current Temperature: "));
  sensor9808.setResolution(MCP9804::R_0_0625);
  tempC_9808 = val_to_temp(sensor9808.readTemperature());
  Serial.println(tempC_9808);
}

void setup() {
  // Eight second watchdog timer to reset the controller if we get stuck
  wdt_enable(WDTO_8S);
  // These three pins are PWM (pulse-width modulation) ports with LEDs, ie intensity is software controlled
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_WHITE, OUTPUT);
  // This is high to enable or low to disable the voltage regulator GPS chip is on.
  pinMode(MCU_GPS_EN, OUTPUT);
  // These two pins receive interrupts, first from the GPS Pulse-Per-Second, second from the switch on the front
  pinMode(INT0_PPS, INPUT);
  pinMode(INT1_SW, INPUT);
  // These pins have ALPS pots, aka rotary knobs. 
  pinMode(TRIMPOT_A1, INPUT);
  pinMode(TRIMPOT_A2, INPUT);
  // Open hardware serial communication, GPS Rx and FTDI Tx. In Windows, connect with Tera Term.
  Serial.begin(9600);
  Serial.println(F("Hello World!"));
  // init i2c for everything else
  Wire.begin(MCU_I2C_NODE_ADDRESS); // Node Address is us, set to 8 by default above.
  // "Be sure to wake up device right as I2C goes up otherwise you'll have NACK issues"
  ecc.enableDebug(&Serial);
  // power on the GPS, or power it off. Whatever. 
  if (GPS_ENABLED == 1) {
    digitalWrite(MCU_GPS_EN, HIGH);
    Serial.println(F("GPS Enabled"));
  } else {
    digitalWrite(MCU_GPS_EN, LOW);
    Serial.println(F("GPS Disabled"));
  }
  // Blink the red LED to acknowledge reset/boot
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_RED, HIGH);
    delay(50);
    digitalWrite(LED_RED, LOW);
    delay(100);
  }

  getTempC();

#ifdef WRITE_TIMEZONE_EEPROM
    myTZ.writeRules(100);    //write rules to EEPROM address 100
#endif

  // ecc
  eccGetInfo();
  atmelRandom();

  if ( RTC.isRunning() ) {
    time_t powerDown, powerUp;    //power outage timestamps
    if ( RTC.powerFail(&powerDown, &powerUp) ) {
     Serial.print(F("Power Down: "));
     printTimeFromRTC(&powerDown);
     Serial.print(F("Power Up: "));
     printTimeFromRTC(&powerDown);
    }
    setSystemClockFromRTC();
  } else {
    Serial.println(F("RTC not running yet. Time?"));
  }
  extraRTC();
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
  Serial.println(F("Calibrating RGB: 1 Red, 2 Green, 3 Blue."));
  RGBCalibrate();
  delay(800);
  FastLED.clear();

  // begin listening for PPS and button pushes
  attachInterrupt(0, pps_interrupt, RISING);
  attachInterrupt(1, momentary_switch_interrupt, RISING);
  // begin listening for i2c requests
  Wire.onRequest(requestEvent);

  // fetch last pattern we displayed before power loss
  momentary_switch_loop = EEPROM.read(eepromSavedPattern);
  previous_momentary_switch_loop = momentary_switch_loop;

}

unsigned long previousMillis =  millis();
byte lastUniquePulse = 0;
byte noFix = 1;

byte previousColor = cHue;

//byte gpsError = 1;
// the loop function runs over and over again forever
void loop() {
  EVERY_N_MILLISECONDS( 53 ) { gHue++; } // slowly cycle the "base color" through the rainbow was 20
  EVERY_N_SECONDS( 60 ) { checkCron(); }
  
  gpsError, noFix = 0; // reset
  if (gps.satellites.value() <= 3 ) {
    noFix = 1;
    gpsError = 1;
  }
  digitalWrite(LED_RED, noFix);

  switch (pps_loop) {
    case 3:
      pps_loop++;
      setTimeFromGps(epochConverter(gps.date, gps.time));
      break;
    case 7:
      pps_loop++;
      printTimeFromRTC(now());
      break;
    case 15:
      pps_loop++;
      getTempC();
      break;
    default: 
      break;
  }
  
  // if the previous counter for button presses is different from present value
  if ( previous_momentary_switch_loop != momentary_switch_loop) {
    // write the new value to the eeprom for the next power loss
    saveSettings(eepromSavedPattern, momentary_switch_loop);
    // and reset our counter here
    previous_momentary_switch_loop = momentary_switch_loop;
    // and let's clear the old pattern.
    FastLED.clear();
    Serial.print(F("Detected a button press: "));
    Serial.println(momentary_switch_loop);
  }

  brightness = readBrightnessFromPot();
  if ( previousBrightness != brightness) {
    Serial.print(F("Detected brightness: "));
    Serial.println(brightness);
    previousBrightness = brightness;
    FastLED.setBrightness(constrain(brightness, BRIGHTNESS_MIN, BRIGHTNESS_MAX));
    // also disable the sleep timer if it was manually turned off just now
    actualBrightness = brightness;
    runningTimedSleep = 0 ;
    runningTimedWake = 0 ;
  }

  EVERY_N_SECONDS ( 9 ) {
    // if someone actually touches the trimpot, we exit these conditions.
    if ( runningTimedSleep ) {
      // we ignore the brightness setting from the pot with actualBrightness
      if (actualBrightness > 0) {
        actualBrightness--;
      }
      FastLED.setBrightness(actualBrightness);
    }

    if ( runningTimedWake ) {
      if (actualBrightness < brightness) {
        actualBrightness++;
      }
      FastLED.setBrightness(actualBrightness);
    }
  }

  cHue = readColorFromPot();
  if ( previousColor != cHue) {
    Serial.print(F("Detected cHue: "));
    Serial.println(cHue);
    previousColor = cHue;
  }

  // This section is what makes colors show up. The value of the 
  // variable in this case/switch is set by the interrupt handler
  // attached to a physical momentary switch.
  switch (momentary_switch_loop) {
    case 1:
      blueish();
      break;
    case 2:
      speckled();
      break;
    case 3:
      januaryWithGlitter();
      break;
    case 4:
      xmasWithGlitter();
      break;
    case 5:
      sinelon();
      break;
    case 6:
      pink();
      break;
    case 7:
      juggle();
      break;
    default: 
      sinelonColorPot();
      break;
  }

  // this is a kind of visible watchdog for the main loop
  int outVal = yforx(x);
  analogWrite(LED_WHITE, outVal);
  x++;
 
  FastLED.show();
  // pause main loop 29ms
  TinyGpsPlusPlus::smartDelay(29);

  wdt_reset();
}

void requestEvent() {
  // when someone asks us for bytes, send these.
  // for now, basically just respond with the current program
  byte responseArray[4];
  responseArray[0] = momentary_switch_loop;
  responseArray[1] = cHue;
  responseArray[2] = actualBrightness;
  responseArray[3] = 0;
  Wire.write(responseArray, 4);
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

