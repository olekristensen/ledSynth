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

// TODO: Setup menu?
// TODO: Connection and mixing
// TODO: Light sensor calibration

#include <Wire.h>                    // I2C
#include <LiquidTWI.h>               // Display
#include <Adafruit_ADS1015.h>        // Analog Digital Converter
#include <Adafruit_MotorShield.h>    // Motor Shield
#include <Adafruit_TCS34725.h>       // Light Sensor base
#include "tcs34725.h"                // Light Sensor autoranging
#include <RFduinoBLE.h>              // RFduino BLE
#include <String.h>                  // Text manipulation
#include <TinyQueue.h>               // Input char queue for EasyTransfer
#include <EasyTransferRfduinoBLE.h>  // Easy Transfer for bluetooth
#include <PID_v1.h>                  // PID control for faders
#include "Fader.h"                   // Fader class
#include "flash.h"                   // Flash memory
#include "qdec.h"                    // Quadrature Decoder
#include "i2cDmx.h"                  // DMX

// DEBUG

#define DEBUG_V 0
#include <DebugUtils.h>


// IDENTITY

const String identityString = "light node";
const int versionMajor = 0;
const int versionMinor = 6;
int newID = 0;


// CONNECTION

int connectionChannelID = 0;
const int maxChannelID = 9;
int connectionMixLevel = 0;
int mixLevelDisplayChars = 12;
int connectionMixMax = 5 * mixLevelDisplayChars;
bool showMixLevel = true;

const byte lcdCharBarGraph0[8] = {
  B00000,
  B11111,
  B00000,
  B00000,
  B00000,
  B11111,
  B00000
};
const byte lcdCharBarGraph1[8] = {
  B00000,
  B11111,
  B10000,
  B10000,
  B10000,
  B11111,
  B00000
};
const byte lcdCharBarGraph2[8] = {
  B00000,
  B11111,
  B11000,
  B11000,
  B11000,
  B11111,
  B00000
};
const byte lcdCharBarGraph3[8] = {
  B00000,
  B11111,
  B11100,
  B11100,
  B11100,
  B11111,
  B00000
};
const byte lcdCharBarGraph4[8] = {
  B00000,
  B11111,
  B11110,
  B11110,
  B11110,
  B11111,
  B00000
};
const byte lcdCharBarGraph5[8] = {
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B00000
};
const byte lcdCharBarGraphStart[8] = {
  B00000,
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
  B00000
};
const byte lcdCharBarGraphEnd[8] = {
  B00000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B00000
};

const byte * lcdCharBarGraphs[8] = {lcdCharBarGraph0, lcdCharBarGraph1, lcdCharBarGraph2, lcdCharBarGraph3, lcdCharBarGraph4, lcdCharBarGraph5, lcdCharBarGraphStart, lcdCharBarGraphEnd };



// RANGES

int useFaderRanges = 1;


// QUAD ENCODER

const int quadButtonPin = 2;
const int quadPhasePinA = 3;
const int quadPhasePinB = 4;
const int quadPhasesPerTick = 4;
long millisLastQuad = 0;
static volatile int quadButtonPushes;
int quadPos = 0;
qdec quad(quadPhasePinA, quadPhasePinB);

int quadButtonCallback(uint32_t ulPin)
{
  quadButtonPushes++;
  return 0;  // don't exit RFduino_ULPDelay
}


// DISPLAY

LiquidTWI lcd(0); // I2C 0x20

const byte lcdCharBluetooth[8] = {0x4, 0x16, 0xd, 0x6, 0xd, 0x16, 0x4};
const byte lcdCharFat1[8] = {0x2, 0x6, 0xe, 0x6, 0x6, 0x6, 0x6};
const byte lcdCharFat2[8] = {0xe, 0x1b, 0x3, 0x6, 0xc, 0x18, 0x1f};
const byte lcdCharFat3[8] = {0xe, 0x1b, 0x3, 0xe, 0x3, 0x1b, 0xe};
const byte lcdCharFat4[8] = {0x3, 0x7, 0xf, 0x1b, 0x1f, 0x3, 0x3};
const byte lcdCharFat5[8] = {0x1f, 0x18, 0x1e, 0x3, 0x3, 0x1b, 0xe};
const byte lcdCharFat6[8] = {0xe, 0x1b, 0x18, 0x1e, 0x1b, 0x1b, 0xe};
const byte lcdCharFat7[8] = {0x1f, 0x3, 0x6, 0xc, 0xc, 0xc, 0xc};
const byte lcdCharFat8[8] = {0xe, 0x1b, 0x1b, 0xe, 0x1b, 0x1b, 0xe};
const byte lcdCharFat9[8] = {0xe, 0x1b, 0x1b, 0xf, 0x3, 0x1b, 0xe};
const byte lcdCharFat0[8] = {0xe, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0xe};
const byte * lcdCharNumbers[10] = {lcdCharFat0, lcdCharFat1, lcdCharFat2, lcdCharFat3, lcdCharFat4, lcdCharFat5, lcdCharFat6, lcdCharFat7, lcdCharFat8, lcdCharFat9 };

double displayRed = 1.0;
double displayGreen = 1.0;
double displayBlue = 1.0;

// BATTERY

const byte lcdCharBatteryLevel6[8] = {0x6, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf};
const byte lcdCharBatteryLevel5[8] = {0x6, 0x9, 0xf, 0xf, 0xf, 0xf, 0xf};
const byte lcdCharBatteryLevel4[8] = {0x6, 0x9, 0x9, 0xf, 0xf, 0xf, 0xf};
const byte lcdCharBatteryLevel3[8] = {0x6, 0x9, 0x9, 0x9, 0xf, 0xf, 0xf};
const byte lcdCharBatteryLevel2[8] = {0x6, 0x9, 0x9, 0x9, 0x9, 0xf, 0xf};
const byte lcdCharBatteryLevel1[8] = {0x6, 0x9, 0x9, 0x9, 0x9, 0x9, 0xf};
const byte lcdCharBatteryLevel0[8] = {0x6, 0x9, 0x9, 0x9, 0x9, 0x9, 0xf};
const byte * lcdCharBatteryLevels[7] = {lcdCharBatteryLevel0, lcdCharBatteryLevel1, lcdCharBatteryLevel2, lcdCharBatteryLevel3, lcdCharBatteryLevel4, lcdCharBatteryLevel5, lcdCharBatteryLevel6 };

const int batteryLevels = 7;
int batteryLevel = 0;
float batteryLevelSmoothNormalised = 0.0;


// FADERS

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); // I2C 0x61

const double faderMotorSpeed = 200;
const int faderMotorHertz = 440 / 8; // max 1600
const double faderPidP = .75;
const double faderPidI = 20;
const double faderPidD = 0.002;

const int faderPidSampleRate = 10; // Calling compute() every >10 ms.

Adafruit_ADS1115 faderIntensityPots(0x49);/*16-bit*/ // I2C 0x49
Fader faderIntensity(
  AFMS.getMotor(1),
  &faderIntensityPots, 0,
  faderPidP, faderPidI, faderPidD, faderPidSampleRate);
int intensityPercent;

Adafruit_ADS1115 faderTemperaturePots(0x4A);/*16-bit*/ // I2C 0x4A
Fader faderTemperature(
  AFMS.getMotor(2),
  &faderTemperaturePots, 0,
  faderPidP, faderPidI, faderPidD, faderPidSampleRate);
int temperaturePercent;
int temperatureKelvin = 2700;

// LIGHT SENSOR

tcs34725 lightSensor; // I2C 0x29
int lightSensorLux;
int lightSensorCt;
int lightSensorR;
int lightSensorG;
int lightSensorB;
int lightSensorC;
bool lightSensorOnline = false;


// PWM

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// DMX

i2cDmx dmx;        //I2C 0x64


// BLUETOOTH

bool bleAdvertising = false;
TinyQueue<char> tq(20 * 10);
long millisLastCommand = 0;
int tqSize = 0;

// GUINO

int titleLabelId = 0;
int saveConfigButtonId = 0;
int calibrateButtonId = 0;
int statusMessageLabelId = 0;
int lightsensorButtonId = 0;
int idSliderId = 0;
int connectionChannelIdSliderId = 0;
int connectionMixLevelSliderId = 0;
String statusMessage = "";
long statusMessageMillisToClear = -1;
bool statusMessageCleared = false;
void setStatusMessage(char * _text, long millisToShow = -1);


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
             S_CONNECTED_LOOP,
           };
int state = S_BOOT;
void statefulLCDclear(int theState = -1);


// TIME

long millisLastFrame = 0;
long frameCount = 0;
int millisPerFrame = 0;



void setup() {

  Wire.speed = 400;

  if (DEBUG_V == 1) {
    Serial.begin(9600); // set up Serial library at 9600 bps
  }

  // BATTERY

  pinMode(1, INPUT);

  lightSensorOnline = lightSensor.begin();
  if (lightSensorOnline) lightSensor.getData();

  lcd.begin(16, 2);

  if (!loadConf()) {
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

  newID = conf->id;

  pinMode(quadButtonPin, INPUT_PULLUP);
  RFduino_pinWakeCallback(quadButtonPin, LOW, quadButtonCallback);

  if (DEBUG_V == 1) {
    //Print unit info on serial
    Serial.println(identityString);
    Serial.print("id:\t");
    Serial.println(conf->id);
    Serial.print("ver.\t");
    Serial.print(versionMajor);
    Serial.print(".");
    Serial.println(versionMinor);
  }

  createChar(lcd, 0, lcdCharBluetooth);   //Bluetooth Icon
  createChar(lcd, 1, lcdCharNumbers[min(9, conf->id)]); //Fat indentity number
  createChar(lcd, 2, lcdCharBatteryLevels[batteryLevel]);
  createChar(lcd, 3, lcdCharBarGraphs[6]);
  createChar(lcd, 4, lcdCharBarGraphs[7]);
  createChar(lcd, 5, lcdCharBarGraphs[0]);
  createChar(lcd, 6, lcdCharBarGraphs[5]);

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

  // FIXTURES

  dmx.addFixture(new TVL2000());

  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());
  dmx.addFixture(new DmxFixtureIT8bit());

  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());

  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());
  dmx.addFixture(new DmxFixtureTI8bit());

  createChar(lcd, 7, lcdCharBarGraphs[connectionMixLevel % 5]);

}

void loop() {
  long thisFrameMillis = millis();
  double t = thisFrameMillis * 0.001 * 0.5;
  millisPerFrame = thisFrameMillis - millisLastFrame;

  int fadeSteps = 500;
  switch (state) {

    case S_BOOT :
      statefulLCDclear();
      for (int i = 0; i < fadeSteps; i++) {
        double iNorm = i * 1.0 / fadeSteps * 1.0;
        temperatureToColor(round(mapFloat(iNorm, 0.0, 1.0, 1200.0, 6500.0)) , displayRed, displayGreen, displayBlue);
        setDisplayColorRGB(displayRed, displayGreen, displayBlue);
        delay(10);
      }
      delay(250);
      quad.enable();
      state = S_FADER_CALIBRATE;
      break;

    case S_FADER_CALIBRATE :
      statefulLCDclear();
      calibrateFaders();
      if (digitalRead(quadButtonPin) == LOW) {
        state = S_QDEC_TEST;
      } else {
        state = S_BLE_SETUP;
      }
      break;

    case S_MENU :
      statefulLCDclear();
      break;

    case S_QDEC_TEST :

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
        setDisplayColorRGB(displayRed, iNorm * displayGreen, iNorm * iNorm * displayBlue);
        delay(1);
      }
      statefulLCDclear();
      faderIntensity.setManual();
      faderTemperature.setManual();
      faderIntensity.setUseRanges(useFaderRanges > 0);
      faderTemperature.setUseRanges(useFaderRanges > 0);
      intensityPercent = faderIntensity.getSetpointPercent();
      temperaturePercent = faderTemperature.getSetpointPercent();
      state = S_STANDALONE_LOOP;
      break;

    case S_STANDALONE_LOOP  :
      faderIntensity.setUseRanges(useFaderRanges > 0);
      faderTemperature.setUseRanges(useFaderRanges > 0);
      faderIntensity.update();
      faderTemperature.update();
      intensityPercent = faderIntensity.getSetpointPercent();
      temperaturePercent = faderTemperature.getSetpointPercent();
      temperatureKelvin = round(mapFloat(faderTemperature.getSetpointNormalised(), 0.0, 1.0, 1.0 * dmx.getKelvinLow(), 1.0 * dmx.getKelvinHigh() ));

      dmx.setTemperatureKelvin(temperatureKelvin);
      dmx.setIntensity(faderIntensity.getSetpointNormalised());
      dmx.sendDmx();
      // Display
      lcd.setCursor(15, 1);
      if (thisFrameMillis % 2000 < 800 && thisFrameMillis % 400 < 200 ) {
        lcd.write(byte(0));
      } else {
        lcd.print(" ");
      }
      lcd.setCursor(1, 1);
      lcdPrintNumberPadded(faderIntensity.getSetpointNormalised() * 100, 4, ' ');
      lcd.print("%");
      /* temperature percent
      lcdPrintNumberPadded(faderTemperature.getSetpointNormalised() * 100, 6, ' ');
      lcd.print("%");
      */
      // temperature kelvin
      lcdPrintNumberPadded(temperatureKelvin, 6, ' ');
      lcd.print("k");

      temperatureToColor(temperatureKelvin, displayRed, displayGreen, displayBlue);
      setDisplayColorRGB(displayRed, displayGreen, displayBlue);

      /*
      // millis per frame
      lcd.setCursor(0, 1);
      lcdPrintNumberPadded(thisFrameMillis - millisLastFrame, 2, ' ');
      //*/
      break;

    case S_CONNECTED_SETUP :
      statefulLCDclear();
      lcd.setCursor(15, 1);
      lcd.write(byte(0)); // bluetooth icon
      quadPos = 0;
      for (int i = 0; i < fadeSteps; i++) {
        double iNorm = i * 1.0 / fadeSteps * 1.0;
        iNorm = 0.5 + (sin(2.0 * (iNorm + 0.25) * PI) / 2.0);
        // green flash
        setDisplayColorRGB(displayRed * iNorm * iNorm, displayGreen, displayBlue * iNorm);
        delay(1);
      }
      faderIntensity.setUseRanges(useFaderRanges > 0);
      faderTemperature.setUseRanges(useFaderRanges > 0);
      intensityPercent = faderIntensity.getSetpointPercent();
      temperaturePercent = faderTemperature.getSetpointPercent();
      faderIntensity.setAutomatic();
      faderTemperature.setAutomatic();
      gBegin();
      state = S_CONNECTED_LOOP;
      break;

    case S_CONNECTED_LOOP  :
      quadPos += quad.readDelta();
      if (quadButtonPushes > 0) {
        //TODO: Debouncing
        showMixLevel = !showMixLevel;
        if (!showMixLevel) millisLastQuad = thisFrameMillis;
        quadButtonPushes = 0;
      }
      if (abs(quadPos) >= quadPhasesPerTick) {
        if (showMixLevel) {
          connectionMixLevel = max(0, min(connectionMixLevel + ((quadPos / quadPhasesPerTick)), connectionMixMax));
          createChar(lcd, 7, lcdCharBarGraphs[connectionMixLevel % 5]);
          gUpdateValue(&connectionMixLevel);
        } else {
          connectionChannelID = max(0, min(maxChannelID, connectionChannelID + (quadPos / quadPhasesPerTick)));
          gUpdateValue(&connectionChannelID);
          if (connectionChannelID == conf->id) {
            faderIntensity.setManual();
            faderTemperature.setManual();
          } else {
            faderIntensity.setAutomatic();
            faderTemperature.setAutomatic();
          }
          millisLastQuad = thisFrameMillis;
        }
        quadPos = 0;
      }
      if (millisLastQuad > thisFrameMillis - 2000 && !showMixLevel) {
        lcd.setCursor(1, 0);
        if (connectionChannelID == 0) {
          lcd.print(" uses sensor  S");
        } else if (connectionChannelID == conf->id) {
          lcd.print(" is manual    M");
        } else {
          lcd.print(" connected to ");
          lcd.print(connectionChannelID);
        }
      } else {
        if (!showMixLevel) showMixLevel = true;
        lcd.setCursor(1, 0);
        if (connectionChannelID == conf->id) {
          lcd.print("              M");
        } else {
          lcd.write(byte(3));
          for (int i = 0; i < connectionMixLevel / 5; i++)
            lcd.write(byte(6));
          if (connectionMixLevel < connectionMixMax) {
            lcd.write(byte(7));
            for (int i = (connectionMixLevel / 5); i < (connectionMixMax / 5) - 1; i++)
              lcd.write(byte(5));
          }
          lcd.write(byte(4));
          if (connectionChannelID == 0) {
            lcd.print('S');
          } else {
            lcd.print(connectionChannelID);
          }
        }
      }

      if (guino_update()) {
        //millisLastCommand = thisFrameMillis;
      }
      // smoothing?
      // faderIntensity.setSetpointNormalised((0.2 * intensityPercent / 100.0) + (0.8 * faderIntensity.getSetpointNormalised()) );
      // faderTemperature.setSetpointNormalised((0.2 * temperaturePercent / 100.0) + (0.8 * faderTemperature.getSetpointNormalised()) );
      if (connectionChannelID != conf->id) {
        faderIntensity.setSetpointNormalised(intensityPercent / 100.0);
        faderTemperature.setSetpointNormalised(temperaturePercent / 100.0);
      }
      faderIntensity.setUseRanges(useFaderRanges > 0);
      faderTemperature.setUseRanges(useFaderRanges > 0);
      faderIntensity.update();
      faderTemperature.update();
      temperatureKelvin = round(mapFloat(faderTemperature.getSetpointNormalised(), 0.0, 1.0, 1.0 * dmx.getKelvinLow(), 1.0 * dmx.getKelvinHigh() ));
      if (connectionChannelID == conf->id) {
        intensityPercent = faderIntensity.getSetpointPercent();
        temperaturePercent = faderTemperature.getSetpointPercent();
        gUpdateValue(&intensityPercent);
        gUpdateValue(&temperaturePercent);
      }
      dmx.setTemperatureKelvin(temperatureKelvin);
      dmx.setIntensity(faderIntensity.getSetpointNormalised());
      dmx.sendDmx();

      // Display
      lcd.setCursor(15, 1);
      if (thisFrameMillis > millisLastCommand - 5) {
        lcd.write(byte(0));
      } else {
        lcd.print(" ");
      }

      lcd.setCursor(2, 1);
      lcdPrintNumberPadded(faderIntensity.getSetpointNormalised() * 100, 3, ' ');
      lcd.print("%");
      /* temperature percent
      lcdPrintNumberPadded(faderTemperature.getSetpointNormalised() * 100, 6, ' ');
      lcd.print("%");
      */
      // temperature kelvin
      lcdPrintNumberPadded(temperatureKelvin, 6, ' ');
      lcd.print("k");

      temperatureToColor(temperatureKelvin, displayRed, displayGreen, displayBlue);
      setDisplayColorRGB(displayRed, displayGreen, displayBlue);

      gUpdateValue(&(faderIntensity._potRangeTopValue));
      gUpdateValue(&(faderIntensity._potRangeBottomValue));
      gUpdateValue(&(faderTemperature._potRangeTopValue));
      gUpdateValue(&(faderTemperature._potRangeBottomValue));

      gUpdateValue(&millisPerFrame);
      //gUpdateValue(&batteryLevel);
      tqSize = tq.size();
      gUpdateValue(&tqSize);


      if (statusMessageMillisToClear < thisFrameMillis && !statusMessageCleared) {
        gUpdateLabel(statusMessageLabelId, "connected");
        statusMessageCleared = true;
      }

      break;

  }
  //  Battery level

  batteryLevelSmoothNormalised *= 0.99;
  batteryLevelSmoothNormalised += 0.01 * constrain(mapFloat(pow(getBatteryLevelNormalised(), 2), 0.75, 0.975, 0.0, 1.0), 0.0, 1.0);

  if (frameCount % 10 == 0) {
    int newBatteryLevel = round(batteryLevelSmoothNormalised * (batteryLevels - 1));
    if (newBatteryLevel != batteryLevel) {
      batteryLevel = newBatteryLevel;
      createChar(lcd, 2, lcdCharBatteryLevels[batteryLevel]);
    }
    lcd.setCursor(0, 1);
    lcd.write(byte(2));
  }

  frameCount++;
  millisLastFrame = thisFrameMillis;
}


// BLUETOOTH

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
      tq.enqueue(data[i]);
    }
    millisLastCommand = millisLastFrame;
  }

  void RFduinoBLE_onAdvertisement(bool start)
  {
    bleAdvertising = start;
  }
}

void setDisplayColorRGB(double r, double g, double b) {
  r = constrain(r, 0.0, 1.0);
  g = constrain(g, 0.0, 1.0);
  b = constrain(b, 0.0, 1.0);
  pwm.setPWM(0, 0, 0xFFF - round( 0xFFF * r ));
  pwm.setPWM(1, 0, 0xFFF - round( 0xFFF * g * 0.7));
  pwm.setPWM(2, 0, 0xFFF - round( 0xFFF * b * 0.55));
}

void lcdPrintNumberPadded(int number, int len, char padding) {
  for (int i = 0; i < len - numDigits(number); i++) {
    lcd.print(padding);
  }
  lcd.print(number);
}

// GUINO

// This is where you setup your interface
void gInit()
{
  gAddLabel(" ", 2);
  String identityWithNumber = identityString + " " + conf->id;
  char identityChars[identityWithNumber.length() + 1];
  identityWithNumber.toCharArray(identityChars, identityWithNumber.length() + 1);
  titleLabelId = gAddLabel(identityChars, 0);
  gAddLabel(" ", 2);
  gAddSpacer(1);

  gAddLabel("FADERS", 2);
  gAddSlider(0, 100, "intensity", &intensityPercent);
  gAddSlider(0, 100, "temperature", &temperaturePercent);
  calibrateButtonId = gAddButton("calibrate");
  gAddLabel("RANGES", 2);
  gAddSlider(0, 1023, "intensity top", &(faderIntensity._potRangeTopValue));
  gAddSlider(0, 1023, "intensity bottom", &(faderIntensity._potRangeBottomValue));
  gAddSlider(0, 1023, "temperature top", &(faderTemperature._potRangeTopValue));
  gAddSlider(0, 1023, "temperature bottom", &(faderTemperature._potRangeBottomValue));
  gAddToggle("USE RANGES", &useFaderRanges);

  gAddSpacer(1);
  if (lightSensorOnline) {
    gAddLabel("SENSOR", 2);
    gAddSlider(0, 10000, "lux", &lightSensorLux);
    gAddSlider(0, 10000, "ct", &lightSensorCt);
    lightsensorButtonId = gAddButton("measure");
    //gAddSlider(0, 1023, "r", &lightSensorR);
    //gAddSlider(0, 1023, "g", &lightSensorG);
    //gAddSlider(0, 1023, "b", &lightSensorB);
    //gAddSlider(0, 1023, "c", &lightSensorC);
  } else {
    gAddLabel("SENSOR OFFLINE", 2);
  }
  gAddSpacer(1);

  gAddLabel("MIXER", 2);
  connectionMixLevelSliderId = gAddSlider(0, connectionMixMax, "mixer", &connectionMixLevel);
  gAddSpacer(1);


  gAddLabel("IDENTITY", 2);
  idSliderId = gAddSlider(0, maxChannelID, "ID", &newID);
  saveConfigButtonId = gAddButton("save id");
  connectionChannelIdSliderId = gAddSlider(0, maxChannelID, "CHANNEL", &connectionChannelID);
  gAddSpacer(1);

  gAddMovingGraph("TQsize", 0, tq.buffersize(), &tqSize, 10);
  //  gAddMovingGraph("BATTERY", 0, batteryLevels, &batteryLevel, 10);

  gAddMovingGraph("MILLIS/FRAME", 0, 100, &millisPerFrame, 10);
  gAddSpacer(1);

  statusMessageLabelId = gAddLabel("connecting", 2);
  setStatusMessage("connected");
}

void updateTitle() {
  String identityWithNumber = identityString + " " +  conf->id;
  char identityChars[identityWithNumber.length() + 1];
  identityWithNumber.toCharArray(identityChars, identityWithNumber.length() + 1);
  gUpdateLabel(titleLabelId, identityChars);
}

void setStatusMessage(char * _text, long millisToShow) {
  statusMessage = _text;
  statusMessageCleared = false;
  if (millisToShow < 0) {
    statusMessageMillisToClear = millis() + 2000;
  } else {
    statusMessageMillisToClear = millis() + millisToShow;
  }
  gUpdateLabel(statusMessageLabelId, _text);
}

void gButtonPressed(int id, int value)
{
  if (saveConfigButtonId == id && value > 0)
  {
    if (newID != conf->id) {
      saveConf(newID);
      updateTitle();
      setStatusMessage("saved new id");
      //gUpdateLabel(saveConfigButtonId, " ");
    }
  }
  if (calibrateButtonId == id && value > 0)
  {
    setStatusMessage("calibrating faders");
    calibrateFaders();
    setStatusMessage("faders calibrated", 5000);
  }
  if (lightsensorButtonId == id && value > 0)
  {
    measureLight();
    gUpdateValue(&lightSensorR);
    gUpdateValue(&lightSensorG);
    gUpdateValue(&lightSensorB);
    gUpdateValue(&lightSensorC);
    gUpdateValue(&lightSensorLux);
    gUpdateValue(&lightSensorCt);
  }

}

void gItemUpdated(int id)
{
  if (connectionChannelIdSliderId == id) {
    if (connectionChannelID == conf->id) {
      faderIntensity.setManual();
      faderTemperature.setManual();
    } else {
      faderIntensity.setAutomatic();
      faderTemperature.setAutomatic();
    }
  }

  if (connectionMixLevelSliderId == id) {
    createChar(lcd, 7, lcdCharBarGraphs[connectionMixLevel % 5]);
  }

  /*  if(idSliderId == id){
      gUpdateLabel(saveConfigButtonId, "save id");
    }
  */
  /*
    if(rotaryRID == id || rotaryGID == id || rotaryBID == id)
    {
      gSetColor(r,g,b);
    }
    */
}

void saveConf(int id) {
  struct flash_conf_t newConf = { id, CONF_STATE_OK };
  while (!RFduinoBLE.radioActive) {} //wait until the radio is active, wastes time, but ensures we will get the most usage of non active cpu time
  delay(6);
  if (!writeConf(newConf)) {
    DEBUG_PRINT("CONF: Could not save new conf");
    conf = new struct flash_conf_t;
    conf->id = 0;
    conf->state = CONF_STATE_ERROR;
    newID = 0;
  } else {
    if (DEBUG_V == 1) {
      DEBUG_PRINT("CONF: Saved new id");
      debug_conf( conf );
    }
    createChar(lcd, 1, lcdCharNumbers[min(9, conf->id)]); //Fat indentity number
  }
}

void calibrateFaders() {
  statefulLCDclear(S_FADER_CALIBRATE);
  faderTemperature.calibrate();
  faderIntensity.calibrate();
  statefulLCDclear();
}
void measureLight() {
  /*  statefulLCDclear(-2);
    lcd.setCursor(2, 0);
    lcd.print("measuring light");
  */
  lightSensor.getData();
  lightSensorR = lightSensor.r_comp;
  lightSensorG = lightSensor.g_comp;
  lightSensorB = lightSensor.b_comp;
  lightSensorC = lightSensor.c_comp;
  lightSensorLux = lightSensor.lux;
  lightSensorCt = lightSensor.ct;
  //statefulLCDclear();
}

void statefulLCDclear(int theState) {
  if (theState == -1) {
    theState = state;
  }

  lcd.clear();
  lcd.write(byte(1));

  switch (theState) {

    case S_BOOT :
      lcd.setCursor(2, 0);
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
      break;

    case S_FADER_CALIBRATE :
      lcd.setCursor(2, 0);
      lcd.print("calibrating...");
      break;

    case S_MENU :
      break;

    case S_QDEC_TEST :
      break;

    case S_PID_TEST :
      break;

    case S_BLE_SETUP :
      break;

    case S_STANDALONE_SETUP :
    case S_STANDALONE_LOOP  :
      lcd.setCursor(2, 0);
      lcd.print("standalone");
      break;

    case S_CONNECTED_SETUP :
      lcd.setCursor(2, 0);
      lcd.print("connected");
    case S_CONNECTED_LOOP  :
      lcd.setCursor(2, 0);
      break;
    default  :
      break;
  }
}

float getBatteryLevelNormalised() {
  return constrain((analogRead(1) / 1023.0) * (3.33 / 2.366), 0.0, 1.0);
}
