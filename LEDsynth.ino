/*
    This file is part of LEDsynth
    LEDsynth - a dmx controller for control praradigm experiments with LED fixtures.
    Copyright (C) 2015  Ole Kristensen

    LEDsynth is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LEDsynth is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with LEDsynth.  If not, see <http://www.gnu.org/licenses/>.

    ole@kristensen.name
    olek@itu.dk
*/

// TODO: Setup menu
// TODO: Binary protocol for bluetooth
// TODO: Range pots
// TODO: Light sensor

#include <Wire.h>
#include <LiquidTWI.h>               // Display
#include <Adafruit_ADS1015.h>        // Analog Digital Converter
#include <Adafruit_MotorShield.h>    // Motor Shield
#include <Adafruit_TCS34725.h>       // Light Sensor
#include <PID_v1.h>
#include <RFduinoBLE.h>
#include <QueueList.h>
#include <String.h>
#include "tcs34725.h"
#include "Fader.h"
#include "flash.h"
#include "qdec.h"

// DEBUG

#define DEBUG_V 1
#include <DebugUtils.h>


// IDENTITY

const String identityString = "LEDSYNTH";
const int versionMajor = 0;
const int versionMinor = 4;


// QUAD ENCODER

const int quadButtonPin = 2;
const int quadPhasePinA = 3;
const int quadPhasePinB = 4;
const int quadPhasesPerTick = 4;
static volatile int woke;
int quadPos = 0;
qdec quad(quadPhasePinA, quadPhasePinB);

int quadButtonCallback(uint32_t ulPin)
{
  woke++;
  return 0;  // don't exit RFduino_ULPDelay
}


// DISPLAY

LiquidTWI lcd(0); // I2C 0x20

byte lcdCharBluetooth[8] = {0x4, 0x16, 0xd, 0x6, 0xd, 0x16, 0x4};
byte lcdCharFat1[8] = {0x2, 0x6, 0xe, 0x6, 0x6, 0x6, 0x6};
byte lcdCharFat2[8] = {0xe, 0x1b, 0x3, 0x6, 0xc, 0x18, 0x1f};
byte lcdCharFat3[8] = {0xe, 0x1b, 0x3, 0xe, 0x3, 0x1b, 0xe};
byte lcdCharFat4[8] = {0x3, 0x7, 0xf, 0x1b, 0x1f, 0x3, 0x3};
byte lcdCharFat5[8] = {0x1f, 0x18, 0x1e, 0x3, 0x3, 0x1b, 0xe};
byte lcdCharFat6[8] = {0xe, 0x1b, 0x18, 0x1e, 0x1b, 0x1b, 0xe};
byte lcdCharFat7[8] = {0x1f, 0x3, 0x6, 0xc, 0xc, 0xc, 0xc};
byte lcdCharFat8[8] = {0xe, 0x1b, 0x1b, 0xe, 0x1b, 0x1b, 0xe};
byte lcdCharFat9[8] = {0xe, 0x1b, 0x1b, 0xf, 0x3, 0x1b, 0xe};
byte lcdCharFat0[8] = {0xe, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0xe};
byte * lcdCharNumbers[10] = {lcdCharFat0, lcdCharFat1, lcdCharFat2, lcdCharFat3, lcdCharFat4, lcdCharFat5, lcdCharFat6, lcdCharFat7, lcdCharFat8, lcdCharFat9 };


// FADERS

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); // I2C 0x61

const double faderMotorSpeed = 200;
const int faderMotorHertz = 110 / 2;
const double faderPidP = .75;
const double faderPidI = 20;
const double faderPidD = 0.002;

const int faderPidSampleRate = 10; // Calling compute() every >10 ms.

Adafruit_ADS1115 faderIntensityPots(0x49);/*16-bit*/ // I2C 0x49
Fader faderIntensity(
  AFMS.getMotor(1),
  &faderIntensityPots, 0,
  faderPidP, faderPidI, faderPidD, faderPidSampleRate);

Adafruit_ADS1115 faderTemperaturePots(0x4A);/*16-bit*/ // I2C 0x4A
Fader faderTemperature(
  AFMS.getMotor(2),
  &faderTemperaturePots, 0,
  faderPidP, faderPidI, faderPidD, faderPidSampleRate);

// MENU




// LIGHT SENSOR

tcs34725 lightSensor; // I2C 0x29


// PWM

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// BLUETOOTH

enum bleInputStates { BIS_START, BIS_COMMAND, BIS_DATA, BIS_END };
int bleInputState = BIS_START;
char bleInputCommandBuf;
String bleInputHexstringBuf;

struct bleCommand {
  char command;
  String hexstring;
};

QueueList <bleCommand> bleCommandQueue;
long millisLastCommand = 0;

bool bleAdvertising = false;


// STATE

enum State { S_BOOT,
             S_MENU,
             S_FADER_CALIBRATE,
             S_PID_TEST,
             S_QDEC_TEST,
             S_BLE_SETUP,
             S_STANDALONE_SETUP,
             S_STANDALONE_LOOP,
             S_CONNECTED_SETUP,
             S_CONNECTED_LOOP
           };
int state = S_BOOT;

// TIME

long millisLastFrame = 0;
long frameCount = 0;

void setup() {

  Wire.speed = 400;

  if (DEBUG_V == 1) {
    Serial.begin(9600); // set up Serial library at 9600 bps
  }

  lightSensor.begin();

  lcd.begin(16, 2);

  if (!loadConf()) {
    eraseConf();
    DEBUG_PRINT("CONF: Not loaded, defaulting");
    struct flash_conf_t defaultConf = { 0, CONF_STATE_DEFAULT };
    if (!writeConf(defaultConf)) {
      DEBUG_PRINT("CONF: Could not save defaults");
      conf = new struct flash_conf_t;
      conf->id = 0;
      conf->state = CONF_STATE_ERROR;
    } else {
      loadConf();
      if (DEBUG_V == 1) {
        DEBUG_PRINT("CONF: Saved defaults");
        debug_conf( conf );
      }
    }
  }

  pinMode(quadButtonPin, INPUT_PULLUP);
  RFduino_pinWakeCallback(2, LOW, quadButtonCallback);

  //Print unit info on serial
  Serial.println(identityString);
  Serial.print("id:\t");
  Serial.println(conf->id);
  Serial.print("ver.\t");
  Serial.print(versionMajor);
  Serial.print(".");
  Serial.println(versionMinor);

  lcd.createChar(0, lcdCharBluetooth);   //Bluetooth Icon
  lcd.createChar(1, lcdCharNumbers[conf->id]); //Fat indentity number

  lcd.clear();
  //Print unit info on display
  lcd.write(byte(1));
  lcd.print(" ");
  if (conf->state == CONF_STATE_OK)
    lcd.print(identityString);
  else if (conf->state == CONF_STATE_DEFAULT)
    lcd.print("unconfigured");
  else if (conf->state == CONF_STATE_ERROR)
    lcd.print("conf error");
  lcd.setCursor(2, 1);
  lcd.print("v.");
  lcd.print(versionMajor);
  lcd.print(".");
  lcd.print(versionMinor);

  //Start BLE Advertisement
  RFduinoBLE.advertisementInterval = 200;
  RFduinoBLE.deviceName = identityString.cstr();
  RFduinoBLE.advertisementData = conf->id + "";
  RFduinoBLE.txPowerLevel = +4;

  AFMS.begin(faderMotorHertz);  // create with the default frequency 1.6KHz
  faderIntensity.setSpeedLimits(faderMotorSpeed);
  faderIntensity.setAdsGain(GAIN_TWOTHIRDS);
  faderIntensity.begin();
  faderTemperature.setSpeedLimits(faderMotorSpeed);
  faderTemperature.setAdsGain(GAIN_TWOTHIRDS);
  faderTemperature.begin();

  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

}

void loop() {
  long thisFrameMillis = millis();
  double t = thisFrameMillis * 0.001 * 0.5;
  int fadeSteps = 500;
  switch (state) {

    case S_BOOT :
      for (int i = 0; i < fadeSteps; i++) {
        double iNorm = i * 1.0 / fadeSteps * 1.0;
        // fade up from warm to cold
        setDisplayColorRGB(iNorm, iNorm * iNorm * iNorm, iNorm * iNorm * iNorm * iNorm);
        delay(10);
      }
      delay(250);
      state = S_FADER_CALIBRATE;
      break;

    case S_FADER_CALIBRATE :
      state = S_BLE_SETUP;
      lcd.clear();
      lcd.write(byte(1));
      lcd.print(" ");
      lcd.print("calibrating...");
      faderTemperature.calibrate();
      faderIntensity.calibrate();
      lcd.clear();
      if (digitalRead(quadButtonPin) == LOW) {
        state = S_MENU;
      }
      break;

    case S_MENU :
      lcd.clear();
      lcd.write(byte(1));


      break;

    case S_QDEC_TEST :

      quad.enable();

      lcd.setCursor(2, 0);
      lcd.print("Quad Encoder");
      lcd.setCursor(2, 1);
      quadPos += quad.readDelta();
      lcdPrintNumberPadded(quadPos / quadPhasesPerTick, 12, ' ');
      if (digitalRead(quadButtonPin) == LOW && quadPos != 0) {
        quadPos = 0;
        state = S_BLE_SETUP;
      }
      break;

    case S_PID_TEST :
      // FADER TEST

      faderIntensity.setAutomatic();
      faderTemperature.setAutomatic();

      //Intensity
      faderIntensity.update();
      if (fmod(t, 3.0) < 2.0) {
        faderIntensity.setSetpoint(
          mapFloat(sin(sin(t * 2.0) * fmod(t, 3.0)), -1.0, 1.0, 24.0, 1000.0)
        );
      }

      //Temperature
      faderTemperature.update();
      t += 0.5;
      if (fmod(t, 3.0) < 2.0) {
        faderTemperature.setSetpoint(
          mapFloat(sin(sin(t * 2.0) * fmod(t, 3.0)), -1.0, 1.0, 24.0, 1000.0)
        );
      }

      // Display
      lcd.setCursor(0, 0);
      lcdPrintNumberPadded(faderIntensity.getLastFaderAdsValue(), 5, ' ');
      lcdPrintNumberPadded(faderIntensity.getSetpoint(), 5, ' ');
      lcdPrintNumberPadded(faderIntensity.getSetpoint() - faderIntensity.getPos() , 5, ' ');
      lcd.setCursor(0, 1);
      lcdPrintNumberPadded(faderTemperature.getLastFaderAdsValue(), 5, ' ');
      lcdPrintNumberPadded(faderTemperature.getSetpoint(), 5, ' ');
      lcdPrintNumberPadded(faderTemperature.getSetpoint() - faderTemperature.getPos() , 5, ' ');
      break;


    case S_BLE_SETUP :
      state = S_STANDALONE_SETUP;
      RFduinoBLE.begin();
      break;

    case S_STANDALONE_SETUP :
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.write(byte(1)); // fat id number
      lcd.print(" ");
      lcd.print("no connection");
      for (int i = 0; i < fadeSteps; i++) {
        double iNorm = i * 1.0 / fadeSteps * 1.0;
        iNorm = 0.5 + (sin(2.0 * (iNorm + 0.25) * PI) / 2.0);
        // yellow flash
        setDisplayColorRGB(1.0, iNorm, iNorm * iNorm);
        delay(1);
      }
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.write(byte(1)); // fat id number
      lcd.print(" ");
      lcd.print("standalone");
      faderIntensity.setManual();
      faderTemperature.setManual();
      state = S_STANDALONE_LOOP;
      break;

    case S_STANDALONE_LOOP  :
      faderIntensity.update();
      faderTemperature.update();

      // Display
      lcd.setCursor(15, 0);
      if (thisFrameMillis % 2000 < 800 && thisFrameMillis % 400 < 200 ) {
        lcd.write(byte(0));
      } else {
        lcd.print(" ");
      }
      lcd.setCursor(0, 1);
      lcdPrintNumberPadded(faderIntensity.getSetpointNormalised() * 100, 5, ' ');
      lcd.print("%");
      lcdPrintNumberPadded(faderTemperature.getSetpointNormalised() * 100, 6, ' ');
      lcd.print("%");

      /*
      // millis per frame
      lcd.setCursor(0, 1);
      lcdPrintNumberPadded(thisFrameMillis - millisLastFrame, 2, ' ');
      */
      break;

    case S_CONNECTED_SETUP :
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.write(byte(1)); // fat id number
      lcd.print(" ");
      lcd.print("connected");
      lcd.setCursor(15, 0);
      lcd.write(byte(0)); // bluetooth icon
      for (int i = 0; i < fadeSteps; i++) {
        double iNorm = i * 1.0 / fadeSteps * 1.0;
        iNorm = 0.5 + (sin(2.0 * (iNorm + 0.25) * PI) / 2.0);
        // yellow flash
        setDisplayColorRGB(iNorm * iNorm, 1.0, iNorm);
        delay(1);
      }
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.write(byte(1)); // fat id number
      lcd.print(" ");
      lcd.print("connected");
      lcd.setCursor(15, 0);
      lcd.write(byte(0)); // bluetooth icon
      faderIntensity.setAutomatic();
      faderTemperature.setAutomatic();
      state = S_CONNECTED_LOOP;
      break;

    case S_CONNECTED_LOOP  :

      while (!bleCommandQueue.isEmpty()) {
        millisLastCommand = millis();
        /* DEBUG
        lcd.setCursor(0, 0);
        lcd.write(byte(1)); // fat id number
        lcd.print(" ");
        lcd.print(bleCommandQueue.count());
        lcd.print(" ");
        lcd.print(bleCommandQueue.peek().command);
        lcd.print(":");
        lcd.print(bleCommandQueue.peek().hexstring);
        */
        processCommand(bleCommandQueue.pop());
      }

      faderIntensity.update();
      faderTemperature.update();
      // Display
      lcd.setCursor(15, 0);
      if (thisFrameMillis > millisLastCommand + 50) {
        lcd.write(byte(0));
      } else {
        lcd.print(" ");
      }

      lcd.setCursor(0, 1);
      lcdPrintNumberPadded(faderIntensity.getSetpointNormalised() * 100, 5, ' ');
      lcd.print("%");
      lcdPrintNumberPadded(faderTemperature.getSetpointNormalised() * 100, 6, ' ');
      lcd.print("%");
      /*
      // millis per frame
      lcd.setCursor(0, 1);
      lcdPrintNumberPadded(thisFrameMillis - millisLastFrame, 2, ' ');
      */
      break;

  }

  frameCount++;
  millisLastFrame = thisFrameMillis;
}


// BLUETOOTH


void queueCommand (char command, String hexstring) {

  while (bleCommandQueue.count() > 10) {
    if (bleCommandQueue.peek().command == command)
      bleCommandQueue.pop();
  }
  bleCommand cmd = {command, hexstring};
  bleCommandQueue.push(cmd);
}

void processCommand (struct bleCommand cmd) {
  switch (cmd.command) {
    case 'I':
      faderIntensity.setSetpointNormalised(cmd.hexstring.toInt() / 65535.0);
      break;
    case 'T':
      faderTemperature.setSetpointNormalised(cmd.hexstring.toInt() / 65535.0);
      break;
  }
}


extern "C" {

  void RFduinoBLE_onConnect() {
    state = S_CONNECTED_SETUP;
    ;
  }

  void RFduinoBLE_onDisconnect() {
    state = S_STANDALONE_SETUP;
    ;
  }

  void RFduinoBLE_onReceive(char *data, int len) {
    for (int i = 0; i < len; i++) {
      bleInputStateMachine(data[i]);
    }
  }

  void RFduinoBLE_onAdvertisement(bool start)
  {
    bleAdvertising = start;
  }
}

void bleInputStateMachine(char data) {
  switch (bleInputState) {
    case BIS_START:                                           // wait for start byte
      if (data == 1) {
        bleInputState = BIS_COMMAND;
      }
      break;
    case BIS_COMMAND:                                           // wait for command
      if (data == 'I' || data == 'T' ) { // If we received a valid command
        bleInputCommandBuf = data;                             // store it
        bleInputHexstringBuf = "";                             // prepare to receive a hex string
        bleInputState = BIS_DATA;
      } else if (data != 1) {                        //Stay in state 2 if we received another 0x01
        state = BIS_START;
      }
      break;
    case  BIS_DATA:                                            // receive hex string
      if ((data >= 'a' && data <= 'z') || (data >= 'A' && data <= 'Z') || (data >= '0' && data <= '9')) {
        bleInputHexstringBuf = bleInputHexstringBuf + data;               // if we received a valid hex byte, add it to the end of the string
        if (bleInputHexstringBuf.length() == 6) {               // If we have received 6 characters (24 bits) move to state 4
          bleInputState = BIS_END;
        }
      } else if (data == 1) {                         // If we received another 0x01 back to state 2
        bleInputState = BIS_COMMAND;
      } else {
        bleInputState = BIS_START;                           // Anything else is invalid - back to look for 0x01
      }
      break;
    case BIS_END:
      if (data == 3)                                   // 0x03=valid terminator
      {
        queueCommand(bleInputCommandBuf, bleInputHexstringBuf);            // We have a valid command message - process it
        //while (! RFduinoBLE.send(bleInputCommandBuf))
          ;  // all tx buffers in use (can't send - try again later)
        bleInputState = BIS_START;
      } else if (data == 1) {                          // 0x01= start of new message, back to state 2
        bleInputState = BIS_COMMAND;
      } else {
        bleInputState = BIS_START;                             // anything else, back to look for 0x01
      }
      break;
  }
}

void setDisplayColorRGB(double r, double g, double b) {
  pwm.setPWM(0, 0, 0xFFF - round( 0xFFF * r ));
  pwm.setPWM(1, 0, 0xFFF - round( 0xFFF * g * 0.85));
  pwm.setPWM(2, 0, 0xFFF - round( 0xFFF * b * 0.7));
}

void lcdPrintNumberPadded(int number, int len, char padding) {
  for (int i = 0; i < len - numDigits(number); i++) {
    lcd.print(padding);
  }
  lcd.print(number);
}

int quadRead() {

}

void quadEnable() {

}

void quadDisable() {

}
