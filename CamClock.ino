//  CamClock.ino
 /* 
 * This file is part of the CamClock distribution (https://github.com/biocode3D/CamClock.git).
 * Copyright (c) 2026 John C. Silvia
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
This program is firmware for a ESP32-C3 Super Micro processor controling
a low-power, configurable clock.

It puts the controller in deep sleep using about 160 microamps; waking up
only once an hour for a few seconds.  That wake-up time could be reduced
by among other things, using a look-up table rather than calculating hours 
to position, and tuning the pauses which are currently very generous.
If my estimate is correct it should run over a year on rechargable AA 
batteries as is.
*/


#include "driver/gpio.h"
#include "RTClib.h"
#include <uEEPROMLib.h>
#include <WiFi.h>
#include "driver/rtc_io.h"


// Will be waking up from deep sleep repeatedly
// and starting at the begining, so need to
// flag the first time through the loop.
RTC_DATA_ATTR bool firstTime = true;

// optionally set network values here
String ssid = "";
String password = "";

// persistent storage for network
RTC_DATA_ATTR char sid[128];
RTC_DATA_ATTR char psk[128];

// eeprom location to store network
int sidAddr = 128;
int pskAddr = 256;

/* Configuration of NTP */
#define MY_NTP_SERVER "at.pool.ntp.org"
#define MY_TZ "CST6CDT,M3.2.0,M11.1.0"

// location for determining if there is usb connection
#define USB_SERIAL_JTAG_FRAM_NUM_REG 0x60043024

// Time keeping variables
time_t NTPnow;  // the seconds since Epoch (1970) - UTC
tm tm = { 0 };
DateTime now;
int hour = 0;

// servo control constants
const int minPulseWidth = 500;       // Minimum servi pulse width in microseconds
const int maxPulseWidth = 2500;      // Maximum servo pulse width in microseconds
const int pulseTrainDuration = 100;  // number of pulses (100 = 2 sec)

// IO pin definitions
const gpio_num_t GPIOhourly = GPIO_NUM_0;     // wake up pin
const gpio_num_t GPIObutton = GPIO_NUM_1;     // button press pin
const gpio_num_t servoPin = GPIO_NUM_4;       // servo control pin
const gpio_num_t servoPowerPin = GPIO_NUM_3;  // servo power pin

// USB connected flag
bool onUSB;

// positions for the hour hand in degrees
RTC_DATA_ATTR uint8_t hourPositions[12] = { 180, 140, 129, 115, 98, 74, 58, 46, 36, 28, 11, 165 };

// eeprom address
uEEPROMLib eeprom(0x57);


void setup() {
  esp_reset_reason_t resetReason;

  Wire.begin();
  Serial.begin(115200);
  RTC_DS3231 rtc;
  rtc.begin();

  // give Serial and rtc time to init
  delay(1000);

  resetReason = esp_reset_reason();

  // on first boot only, DS3231 RTC configuration
  if (firstTime) {
    firstTime = false;

    // SW restart resets everything, so test
    //if realy first time
    if (resetReason != ESP_RST_SW) {
      rtc.disable32K();
      rtc.disableAlarm(2);
      rtc.clearAlarm(1);
      rtc.clearAlarm(2);
      esp_restart();
    }
  }

  // Determine if we have a usb connection
  // if so, will be giving prompts and processing input
  onUSB = usbPowered();

  pinMode(GPIObutton, INPUT_PULLUP);
  pinMode(GPIOhourly, INPUT);

  rtc.clearAlarm(1);

  // if button still pressed, that was why wakeup
  if (digitalRead(GPIObutton) == LOW) {
    if (onUSB) {
      // if USB connected, adjust hour hand positions
      // (same as entering # for ssid)
      handAngleAdjust();
    } else {
      // force time update from NTP
      setClock(rtc);
    }
  }
  // enable wake up from RTC alarm
  esp_deep_sleep_enable_gpio_wakeup(1ULL << GPIOhourly, ESP_GPIO_WAKEUP_GPIO_LOW);
  // enable wake up because button pressed
  esp_deep_sleep_enable_gpio_wakeup(1ULL << GPIObutton, ESP_GPIO_WAKEUP_GPIO_LOW);

  switch (resetReason) {
    case ESP_RST_DEEPSLEEP:  // the usual option
      now = rtc.now();
      // reset RTC once a month or if power was lost
      if (((now.day() == 24) && (now.hour() == 6)) || rtc.lostPower()) {
        setClock(rtc);
        now = rtc.now();
      }
      break;
    case ESP_RST_SW:       // fall through for same action
    case ESP_RST_POWERON:  //
      if (onUSB) {
        // get values for network connection
        getCred(rtc);
      } else {
        // get hour hand angles for each hour
        // and set clock to current time
        readPositions();
        setClock(rtc);
      }
      // read current time
      now = rtc.now();
      break;
    default:
      now = rtc.now();
      break;
  }

  // display new hour
  int shiftHour = dst(rtc);  // ? daylight savings time currently
  if (onUSB) showDT(shiftHour);

  // move hour hand to current time
  hour = (now.hour() + shiftHour) % 12;
  toll(hour);

  // wait 2sec to be sure "interrupt" pin is high again
  // wake up on the hour
  rtc.setAlarm1(DateTime(0, 0, 0, 0, 0, 0), DS3231_A1_Minute);
  delay(2000);
  // shutdown and wait for signal to start all over again
  esp_deep_sleep_start();
}

// connects to internet and starts NTP
void getNTPtime() {
  int uncle = 0;

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  // get network values
  readNetSecrs();

  WiFi.begin(sid, psk);

  // connect to wifi
  while (WiFi.status() != WL_CONNECTED) {
    // dont wait forever
    if (++uncle > 60) {
      if (onUSB) Serial.println("Unable to connect");
      return;
    }
    delay(2000);
    if (onUSB) Serial.print(".");
  }
  configTzTime(MY_TZ, MY_NTP_SERVER);
  uncle = 0;

  // connect to NTP server
  while (time(nullptr) < 100000) {
    if (++uncle > 120) {
      if (onUSB) Serial.println("Unable to get time");
      return;
    }
    delay(500);
    if (onUSB) Serial.print(".");
  }
  if (onUSB) Serial.println("Time Synced");

  //disconnect WiFi it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}


// saves angles to eeprom
bool recordAngles() {
  uint8_t chksum = 0;
  uint8_t wrtchksum = 0;
  uint8_t buf = 0;

  // save values, avoiding unneccessary writes
  // and using XOR checksum
  for (uint i = 0; i < 12; i++) {
    buf = eeprom.eeprom_read(i);
    chksum = chksum ^ hourPositions[i];
    if (hourPositions[i] != buf) {
      eeprom.eeprom_write(i, hourPositions[i]);
      delay(5);
    }
  }
  buf = eeprom.eeprom_read(12);
  if (buf != chksum) {
    eeprom.eeprom_write(12, chksum);
    delay(5);
  }

  // test for success
  for (uint i = 0; i < 12; i++) {
    buf = eeprom.eeprom_read(i);
    wrtchksum = wrtchksum ^ buf;
  }
  if (wrtchksum != chksum) { return (false); }
  if (wrtchksum == eeprom.eeprom_read(12)) {
    return (true);
  } else {
    return (false);
  }
}

// retrieves network seetings from eeprom
void readNetSecrs() {
  int strLen = 0;

  strLen = eeprom.eeprom_read(sidAddr);
  eeprom.eeprom_read(sidAddr + 1, (byte *)sid, strLen);
  sid[strLen] = 0;
  strLen = eeprom.eeprom_read(pskAddr);
  eeprom.eeprom_read(pskAddr + 1, (byte *)psk, strLen);
  psk[strLen] = 0;
}


// runs thru hour angles and allow changes
void handAngleAdjust() {
  int adj = 0;
  bool changed = false;
  String advance;

  // enter # to end editing and save current values
  while (advance != "#") {
    for (int i = 0; i < 12; i++) {
      Serial.printf("Hour Position %i = %i     ", i, hourPositions[i]);
      showAngles();
      toll(i);
      advance = Serial.readStringUntil('\n');
      advance.trim();
      if (advance == "#") {
        break;
      }
      adj = advance.toInt();
      if (adj != 0) {
        hourPositions[i] = adj;
        changed = true;
        i--;
      }
    }
  }
  if (changed) {
    recordAngles();
  }
}

// inputs network access and list of angles
// if new values are given
void getAuth() {
  int strLen = 0;
  char esid[128];
  char epsk[128];

  Serial.setTimeout(50000);
  Serial.println("\nEnter SSID ");
  ssid = Serial.readStringUntil('\n');
  ssid.trim();

  // special escape char for test/debug
  if (ssid == "#") {
    handAngleAdjust();

    Serial.println("\nEnter SSID ");
    ssid = Serial.readStringUntil('\n');
    ssid.trim();
  }

  Serial.print(ssid);
  strLen = ssid.length();
  if (strLen) {
    eeprom.eeprom_write(sidAddr, (uint8_t)strLen);
    delay(500);
    ssid.toCharArray(esid, 128);
    Serial.print(" ESID = ");
    Serial.println(esid);
    delay(500);
    Serial.printf(
      "eeprom write says: %i\n",
      eeprom.eeprom_write(sidAddr + 1, (byte *)esid, strLen));
    delay(5);
  }

  Serial.println("\nEnter password ");
  password = Serial.readStringUntil('\n');
  password.trim();
  strLen = password.length();
  if (strLen) {
    eeprom.eeprom_write(pskAddr, (uint8_t)strLen);
    delay(5);
    password.toCharArray(epsk, 128);
    eeprom.eeprom_write(pskAddr + 1, (byte *)epsk, strLen);
    delay(5);
  }
}

// displays and updates stored angles
void getPositions() {
  char buf[128];
  char *token;
  uint8_t chksum = 0;
  String newAngles;

  readPositions();
  showAngles();
  Serial.println("\nTo change, enter comma separated list of 12 angles:\n");
  newAngles = Serial.readStringUntil('\n');
  newAngles.trim();
  if (!newAngles.isEmpty()) {
    newAngles.toCharArray(buf, sizeof(buf));
    token = strtok(buf, ",");
    hourPositions[0] = (uint8_t)atoi(token);
    chksum = hourPositions[0];
    for (int i = 1; i < 12; i++) {
      token = strtok(NULL, ",");
      hourPositions[i] = (uint8_t)atoi(token);
      chksum = chksum ^ hourPositions[i];
    }
    //    hourPositions[12] = chksum;
    Serial.println("Angles changed.");
    showAngles();
  }
}

// gets stored angles if valid
void readPositions() {
  char buf[16];
  uint8_t chksum = 0;

  for (uint i = 0; i < 13; i++) {
    buf[i] = eeprom.eeprom_read(i);
    chksum = chksum ^ buf[i];
  }
  if (chksum == 0) {
    for (uint i = 0; i < 12; i++) {
      hourPositions[i] = buf[i];
    }
  }
}

// prints simple list of hour to angle conversion
void showAngles() {
  Serial.print("\n Positions Now: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(hourPositions[i]);
    Serial.print(", ");
  }
  Serial.print("\n");
}


// calculate if DST now and returns 1 or 0 to add to hour
int dst(RTC_DS3231 rtc) {
  now = rtc.now();

  int thisYear = now.year();
  int thisMonth = now.month();
  int thisDay = now.day();
  int thisWeekday = now.dayOfTheWeek();
  int thisHour = now.hour();
  int thisMinute = now.minute();

  if (thisMonth == 11 && thisDay < 8 && thisDay < thisWeekday) return 1;

  if (thisMonth == 11 && thisDay < 8 && thisWeekday == 1 && thisHour < 1) return 1;

  if (thisMonth < 11 && thisMonth > 3) return 1;

  if (thisMonth == 3 && thisDay > 7 && thisDay >= (thisWeekday + 7)) {
    if (!(thisWeekday == 1 && thisHour < 2)) return 1;
  }
  return 0;
}

// given hour, displays it
void toll(int now) {
  uint8_t angle;

  pinMode(servoPin, OUTPUT);
  pinMode(servoPowerPin, OUTPUT);

  servoOn();

  angle = hourPositions[now];
  generatePWM(map(angle, 0, 180, minPulseWidth, maxPulseWidth));

  servoOff();
}

// update time to current NTP
void setClock(RTC_DS3231 rtc) {
  getNTPtime();
  delay(2000);
  // Set RTC to local time from NTP
  time(&NTPnow);

  localtime_r(&NTPnow, &tm);

  delay(1000);

  tm.tm_isdst = -1;
  if (onUSB) {
    Serial.print(tm.tm_year);
    Serial.print(tm.tm_mday);
    Serial.println(tm.tm_hour);
  }

  rtc.adjust(DateTime(tm.tm_year - 100, tm.tm_mon + 1, tm.tm_mday,
                      tm.tm_hour + tm.tm_isdst, tm.tm_min, tm.tm_sec));
  // adjust resets mode!
  rtc.writeSqwPinMode(DS3231_OFF);
  delay(2000);
}

// millisecond delay that blocks (which is fine)
// and doesnt require use of a timer
void servoDelay() {
  for (int i = 0; i < 15; i++) {
    delayMicroseconds(1000);
  }
}


void servoOn() {
  digitalWrite(servoPowerPin, HIGH);
}
void servoOff() {
  digitalWrite(servoPowerPin, LOW);
}


// Function to generate PWM servo signal
void generatePWM(int pulseWidth) {
  for (int k = 0; k < pulseTrainDuration; k++) {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000 - pulseWidth);  // 20ms period for the servo
  }
}

// get network variables
void getCred(RTC_DS3231 rtc) {

  // this is fossil code (no longer needed??)
  // but was necessary when this was a different function
  rtc.disable32K();
  delay(1000);

  getAuth();
  getPositions();
}

// output date-time on usb
void showDT(int shiftHour) {
  Serial.print(now.year());
  Serial.print('/');
  Serial.print(now.month() & 0x0F);
  Serial.print('/');
  Serial.print(now.day());
  Serial.print(" -- ");
  Serial.print(now.dayOfTheWeek());
  Serial.print(" -- ");
  Serial.print(now.hour() + shiftHour);
  Serial.print(':');
  Serial.print(now.minute());
  Serial.print(':');
  Serial.print(now.second());
  Serial.print('\n');
}

// determine if there is a usb connection
bool usbPowered() {
  int countA = usbActivity();
  // return countA != 0;  would work here except
  //if usb unplugged with battery connected
  delay(100);
  int countB = usbActivity();

  return (countA != countB);
}

int usbActivity() {
  uint32_t usbReg = *(volatile uint32_t *)USB_SERIAL_JTAG_FRAM_NUM_REG;
  return (int)(usbReg);
}

// never happens
void loop() {
}