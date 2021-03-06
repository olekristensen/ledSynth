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

#include <Wire.h>                    // I2C
#include <LiquidTWI.h>               // Display
#include <Adafruit_ADS1015.h>        // Analog Digital Converter
#include <Adafruit_MotorShield.h>    // Motor Shield
#include <Adafruit_TCS34725.h>       // Light Sensor base class
#include "tcs34725.h"                // Light Sensor autoranging
#include <RFduinoBLE.h>              // RFduino BLE
#include <String.h>                  // Text manipulation
#include <TinyQueue.h>               // Input char queue for EasyTransfer
#include <EasyTransferRfduinoBLE.h>  // Easy Transfer for bluetooth
#include <PID_v1.h>                  // PID control for faders
#include <Adafruit_MPR121.h>         // Capacitive touch for faders
#include "Fader.h"                   // Fader class
#include "flash.h"                   // Flash memory
#include "qdec.h"                    // Quadrature Decoder

// DEBUG

#define DEBUG_V 0
#include <DebugUtils.h>


// IDENTITY

const String identityString = "light node";
const int versionMajor = 2;
const int versionMinor = 1;
int newID = 0;


// REMOTE AND OUTPUT CALCULATION

int remoteChannel = 0;
const int maxRemoteChannels = 9;
int remoteMixLevel = 0;
bool mixLevelShown = true;
int mixLevelDisplayChars = 12;
int remoteMixMax = 5 * mixLevelDisplayChars;
int useFaderRanges = 1;
int remoteOverride = 0;
int remoteWasOverriding = 0;
int identify = 0;

int intensityPromilleManual;
int temperatureKelvinManual = 2700;

int intensityPromilleRemote;
int temperatureKelvinRemote;

int intensityPromilleOutput;
int temperatureKelvinOutput;

double intensityPosManual;
double temperaturePosManual;

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


// QUAD ENCODER

const int quadButtonPin = 2;
const int quadPhasePinA = 3;
const int quadPhasePinB = 4;
const int quadPhasesPerTick = 4;
unsigned long quadLastTickMillis = 0;
unsigned long quadLastClickMillis = 0;
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


/*
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
*/

// FADERS

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); // I2C 0x61

const double faderMotorSpeed = 255;
int faderMotorHertz = 440 / 8; // max 1600

int faderPidAint = 90; // divides with 100
int faderPidPint = 42; // divides with 100
int faderPidIint = 165; // divides with 100
int faderPidDint = 12; // divides with 10000

double faderPidA = faderPidAint / 100.0;
double faderPidP = (faderPidPint / 100.0) * faderPidA;
double faderPidI = (faderPidIint / 100.0) * faderPidA;
double faderPidD = (faderPidDint / 10000.0) * faderPidA;

const int faderPidSampleRate = 10; // Calling compute() every >10 ms.

Adafruit_ADS1115 faderIntensityPots(0x49);/*16-bit*/ // I2C 0x49
Fader faderIntensity(
  AFMS.getMotor(1),
  &faderIntensityPots, 0,
  faderPidP, faderPidI, faderPidD, faderPidSampleRate);
bool touchFaderIntensity = false;
int faderIntensityRangeTop = 1023;
int faderIntensityRangeBottom = 0;

Adafruit_ADS1115 faderTemperaturePots(0x4A);/*16-bit*/ // I2C 0x4A
Fader faderTemperature(
  AFMS.getMotor(2),
  &faderTemperaturePots, 0,
  faderPidP, faderPidI, faderPidD, faderPidSampleRate);
bool touchFaderTemperature = false;
int faderTemperatureRangeTop = 1023;
int faderTemperatureRangeBottom = 0;


// CAPACITIVE TOUCH SENSOR

Adafruit_MPR121 cap = Adafruit_MPR121();

unsigned long previousTouchIntensity = 0;
unsigned long previousTouchTemperature = 0;
unsigned long touchTimeout = 250;

// LIGHT SENSOR

tcs34725 lightSensor; // I2C 0x29
float lightSensorLux_raw;
float lightSensorCt_raw;
int lightSensorR_raw;
int lightSensorG_raw;
int lightSensorB_raw;
int lightSensorC_raw;
int lightSensorAgc_raw;

int lightSensorLux = 0;
int lightSensorCt = 0;
float lightSensorLuxFloat = 0.0;
float lightSensorCtFloat = 0.0;
int lightSensorR = 0;
int lightSensorG = 0;
int lightSensorB = 0;
int lightSensorC = 0;
int lightSensorAgc = 0;

int lightSensorCplPercent = 0;
float lightSensorStep = 0;
int lightSensorStepInt = 0;
float lightSensorLevelNormalised = 0.0; // human perception approximation from complete darkness (0 lux) to direct sunlight (1,000,000)
int lightSensorLevelPromille = 0; // x1000

bool lightSensorOnline = false;


// PWM

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// BLUETOOTH

bool bleAdvertising = false;
TinyQueue<char> tq(20 * 10);
unsigned long millisPreviousCommand = 0;
int tqSize = 0;

// GUINO

int titleLabelId = 0;
int saveConfigButtonId = 0;
int calibrateButtonId = 0;
int MessageLabelId = 0;
int lightsensorButtonId = 0;
int idSliderId = 0;
int remoteChannelSliderId = 0;
int remoteMixLevelSliderId = 0;
int pidAId = 0;
int pidPId = 0;
int pidIId = 0;
int pidDId = 0;

namespace hardware {
enum items {
  ownID,
  remoteID,
  versionMajor,
  versionMinor,
  mixRemote,
  mixNoise,
  intensityFader,
  temperatureFader,
  intensityRemote,
  temperatureRemote,
  intensityNoise,
  temperatureNoise,
  intensityOutput,
  temperatureOutput,
  intensityRangeTop,
  intensityRangeBottom,
  temperatureRangeTop,
  temperatureRangeBottom,
  useRanges,
  movementSensor,
  movementSensorLevel,
  movementSensorLedActive,
  lightSensorTemperature,
  lightSensorLux,
  lightSensorLightLevel,
  doFaderCalibration,
  doSaveId,
  identify,
  remoteOverride
};
};


// PIR (MOVEMENT) SENSOR

Adafruit_ADS1115 * pirADS = &faderIntensityPots;
const int pirADCChannel = 3;
const int pirLEDChannel = 3;
int pirLedActive = 1;
int pirReading = 0;
float pirReadingFiltered = 0.0;
float pirLevelNormalised = 0;
int pirLevelPromille = 0;

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

unsigned long millisLastFrame = 0;
unsigned long frameCount = 0;
unsigned long millisPerFrame = 0;


// DMX BOARD

int dmxBoardWireAddress = 100; // 0x64
int temperatureKelvinMax = 6500;
int temperatureKelvinMin = 2000;
unsigned long nextDMXSendMillis = 0;


void setup() {

  Wire.speed = 400;

  if (DEBUG_V == 1) {
    Serial.begin(9600); // set up Serial library at 9600 bps
  }

  // BATTERY
  // pinMode(1, INPUT);

  // LIGHT SENSOR
  lightSensorOnline = lightSensor.begin();
  if (lightSensorOnline) lightSensor.getData();

  //LCD
  lcd.begin(16, 2);

  // CONFIGURATION
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

  remoteChannel = conf->id;

  // QUAD ENCODER
  pinMode(quadButtonPin, INPUT_PULLUP);
  RFduino_pinWakeCallback(quadButtonPin, LOW, quadButtonCallback);

  // IDENTITY DEBUG
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

  // LCD CHARS
  createChar(lcd, 0, lcdCharBluetooth);   //Bluetooth Icon
  createChar(lcd, 1, lcdCharNumbers[min(9, conf->id)]); //Fat indentity number
  // createChar(lcd, 2, lcdCharBatteryLevels[batteryLevel]); // Battery icon
  createChar(lcd, 3, lcdCharBarGraphs[6]); // Bar chart
  createChar(lcd, 4, lcdCharBarGraphs[7]); // Bar chart
  createChar(lcd, 5, lcdCharBarGraphs[0]); // Bar chart
  createChar(lcd, 6, lcdCharBarGraphs[5]); // Bar chart
  createChar(lcd, 7, lcdCharBarGraphs[remoteMixLevel % 5]); // Bar chart

  DEBUG_PRINT("Display chars setup");

  //BLE Advertisement
  RFduinoBLE.advertisementInterval = 200;
  RFduinoBLE.deviceName = identityString.cstr();
  RFduinoBLE.advertisementData = conf->id + "";
  RFduinoBLE.txPowerLevel = +4;

  DEBUG_PRINT("BLE setup");

  // FADERS
  AFMS.begin(faderMotorHertz); // create with the default frequency 1.6KHz
  faderIntensity.setSpeedLimits(faderMotorSpeed);
  faderIntensity.setAdsGain(GAIN_TWOTHIRDS);
  faderIntensity.begin();
  faderTemperature.setSpeedLimits(faderMotorSpeed);
  faderTemperature.setAdsGain(GAIN_TWOTHIRDS);
  faderTemperature.begin();

  DEBUG_PRINT("Faders setup");

  //PWM
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  pwm.setPWM(pirLEDChannel, 0, 0);

  DEBUG_PRINT("PWM setup");

  // CAP SENSE
  cap.begin();
  cap.setThreshholds(1, 1);

  DEBUG_PRINT("CapSense setup");

}

void loop() {
  unsigned long thisFrameMillis = millis();
  double t = thisFrameMillis * 0.001 * 0.5;
  millisPerFrame = thisFrameMillis - millisLastFrame;

  int fadeSteps = 500;

  // TOUCH

  if ((cap.touched() & (1 << 1))) {
    previousTouchIntensity = thisFrameMillis;
  }
  touchFaderIntensity = (thisFrameMillis - previousTouchIntensity <  touchTimeout);

  if ((cap.touched() & (1 << 0))) {
    previousTouchTemperature = thisFrameMillis;
  }
  touchFaderTemperature = (thisFrameMillis - previousTouchTemperature < touchTimeout);

  switch (state) {

    case S_BOOT :
      statefulLCDclear();
      for (int i = 0; i < fadeSteps; i++) {
        double iNorm = i * 1.0 / fadeSteps * 1.0;
        temperatureToColor(round(mapFloat(iNorm, 0.0, 1.0, 1200.0, 6500.0)) , displayRed, displayGreen, displayBlue);
        setDisplayColorRGB(displayRed, displayGreen, displayBlue);
        delay(5);
      }
      delay(250);

      setTempRangesFromi2c(dmxBoardWireAddress);

      DEBUG_PRINT("Dmx Board setup");

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
      intensityPromilleOutput = intensityPromilleManual = faderIntensity.getSetpointPromille();
      temperatureKelvinOutput = temperatureKelvinManual = round(mapFloat(faderTemperature.getSetpointNormalised(), 0.0, 1.0, 1.0 * temperatureKelvinMin, 1.0 * temperatureKelvinMax ));
      state = S_STANDALONE_LOOP;
      break;

    case S_STANDALONE_LOOP  :
      faderIntensity.setUseRanges(useFaderRanges > 0);
      faderTemperature.setUseRanges(useFaderRanges > 0);
      faderIntensity.update();
      faderTemperature.update();
      intensityPromilleOutput = intensityPromilleManual = faderIntensity.getSetpointPromille();
      temperatureKelvinOutput = temperatureKelvinManual = round(mapFloat(faderTemperature.getSetpointNormalised(), 0.0, 1.0, 1.0 * temperatureKelvinMin, 1.0 * temperatureKelvinMax ));

      sendDMXi2c( intensityPromilleOutput,  temperatureKelvinOutput,  dmxBoardWireAddress);
      //sendDMXanalog( intensityPromilleOutput,  temperatureKelvinOutput );
      // Display
      lcd.setCursor(15, 1);
      if (thisFrameMillis % 2000 < 800 && thisFrameMillis % 400 < 200 ) {
        lcd.write(byte(0));
      } else {
        lcd.print(" ");
      }
      lcd.setCursor(1, 1);
      lcdPrintNumberPadded(intensityPromilleManual / 10, 4, ' ');
      lcd.print("%");
      lcdPrintNumberPadded(temperatureKelvinManual, 7, ' ');
      lcd.print("k");

      temperatureToColor(temperatureKelvinManual, displayRed, displayGreen, displayBlue);
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
      intensityPromilleManual = faderIntensity.getSetpointPromille();
      temperatureKelvinManual = round(mapFloat(faderTemperature.getSetpointNormalised(), 0.0, 1.0, 1.0 * temperatureKelvinMin, 1.0 * temperatureKelvinMax ));
      faderIntensity.setAutomatic();
      faderTemperature.setAutomatic();
      identify = 0;
      gBegin();
      state = S_CONNECTED_LOOP;
      break;

    case S_CONNECTED_LOOP  :


      if (!remoteWasOverriding > 0 && remoteOverride > 0) {
        for (int i = 0; i < fadeSteps; i++) {
          double iNorm = i * 1.0 / fadeSteps * 1.0;
          iNorm = 0.5 + (sin(2.0 * (iNorm + 0.25) * PI) / 2.0);
          // blue flash
          setDisplayColorRGB(displayRed * iNorm * iNorm, displayGreen * iNorm * iNorm, lerp(1.0, displayBlue, iNorm));
          delay(1);
        }
      }
      remoteWasOverriding = remoteOverride;

      if (!remoteOverride) {
        quadPos += quad.readDelta();
        if (quadButtonPushes > 0) {
          if (thisFrameMillis - quadLastClickMillis > 400) {
            mixLevelShown = !mixLevelShown;
            quadLastClickMillis = thisFrameMillis;
          }
          if (!mixLevelShown) quadLastTickMillis = thisFrameMillis;
          quadButtonPushes = 0;
        }
        if (abs(quadPos) >= quadPhasesPerTick) {
          if (mixLevelShown) {
            remoteMixLevel = max(0, min(remoteMixLevel + ((quadPos / quadPhasesPerTick)), remoteMixMax));
            createChar(lcd, 7, lcdCharBarGraphs[remoteMixLevel % 5]);
            gUpdateValue(&remoteMixLevel);
          } else {
            remoteChannel = max(0, min(maxRemoteChannels, remoteChannel + (quadPos / quadPhasesPerTick)));
            gUpdateValue(&remoteChannel);
            if (remoteChannel == conf->id) {
              faderIntensity.setManual();
              faderTemperature.setManual();
            } else {
              faderIntensity.setAutomatic();
              faderTemperature.setAutomatic();
            }
            quadLastTickMillis = thisFrameMillis;
          }
          quadPos = 0;
        }
        if (thisFrameMillis - quadLastTickMillis < 4000 && !mixLevelShown) {
          lcd.setCursor(1, 0);
          if (remoteChannel == 0) {
            lcd.print(" light sensor L");
          } else if (remoteChannel == conf->id) {
            lcd.print(" movement     M");
          } else {
            lcd.print(" remote node  ");
            lcd.print(remoteChannel);
          }
        } else {
          if (!mixLevelShown) mixLevelShown = true;
          lcd.setCursor(1, 0);
          lcd.write(byte(3));
          for (int i = 0; i < remoteMixLevel / 5; i++)
            lcd.write(byte(6));
          if (remoteMixLevel < remoteMixMax) {
            lcd.write(byte(7));
            for (int i = (remoteMixLevel / 5); i < (remoteMixMax / 5) - 1; i++)
              lcd.write(byte(5));
          }
          lcd.write(byte(4));
          if (remoteChannel == conf->id) {
            lcd.print('M');
          } else if (remoteChannel == 0) {
            lcd.print('L');
          } else {
            lcd.print(remoteChannel);
          }
        }
      } else {
        quad.readDelta();
        quadButtonPushes = 0;
        lcd.setCursor(1, 0);
        lcd.print(" override     O");
      }
      // GET REMOTE INPUT

      if (guino_update()) {
        //millisPreviousCommand = thisFrameMillis;
      }

      // GET FADER INPUTS IF TOUCHED

      // intensity input

      if (touchFaderIntensity) {
        if (faderIntensity._state == Fader::F_AUTOMATIC) {
          faderIntensity.setManual();
        } else {
          if (faderIntensity.wasMoved()) {
            intensityPosManual = faderIntensity.getPos();
            intensityPromilleManual = faderIntensity.getSetpointPromille();
            gUpdateValue(&intensityPromilleManual);
          }
        }
      }

      if (faderIntensityRangeTop != faderIntensity._potRangeTopValue) {
        gUpdateValue(&(faderIntensity._potRangeTopValue));
        faderIntensityRangeTop = faderIntensity._potRangeTopValue;
      }
      if (faderIntensityRangeBottom != faderIntensity._potRangeBottomValue) {
        gUpdateValue(&(faderIntensity._potRangeBottomValue));
        faderIntensityRangeBottom = faderIntensity._potRangeBottomValue;
      }

      int faderIntensityRangeBottomPromille = map(faderIntensityRangeBottom, 0, 1023, 0, 1000);
      int faderIntensityRangeTopPromille = map(faderIntensityRangeTop, 0, 1023, 0, 1000);

      int newIntensityPromilleManual = round(mapFloat(intensityPosManual * 1.0, 0.0, 1023.0, faderIntensityRangeBottomPromille * 1.0, faderIntensityRangeTopPromille * 1.0));
      if (newIntensityPromilleManual != temperatureKelvinManual) {
        intensityPromilleManual = newIntensityPromilleManual;
        gUpdateValue(&intensityPromilleManual);
      }

      // temperature input

      if (touchFaderTemperature) {
        if (faderTemperature._state == Fader::F_AUTOMATIC) {
          faderTemperature.setManual();
        } else {
          if (faderTemperature.wasMoved()) {
            temperaturePosManual = faderTemperature.getPos();
            temperatureKelvinManual = round(mapFloat(faderTemperature.getSetpointNormalised(), 0.0, 1.0, 1.0 * temperatureKelvinMin, 1.0 * temperatureKelvinMax ));
            gUpdateValue(&temperatureKelvinManual);
          }
        }
      }
      if (faderTemperatureRangeTop != faderTemperature._potRangeTopValue) {
        gUpdateValue(&(faderTemperature._potRangeTopValue));
        faderTemperatureRangeTop = faderTemperature._potRangeTopValue;
      }
      if (faderTemperatureRangeBottom != faderTemperature._potRangeBottomValue) {
        gUpdateValue(&(faderTemperature._potRangeBottomValue));
        faderTemperatureRangeBottom = faderTemperature._potRangeBottomValue;
      }

      int faderTemperatureRangeBottomKelvin = map(faderTemperatureRangeBottom, 0, 1023, temperatureKelvinMin, temperatureKelvinMax);
      int faderTemperatureRangeTopKelvin = map(faderTemperatureRangeTop, 0, 1023, temperatureKelvinMin, temperatureKelvinMax);

      int newTemperatureKelvinManual = round(mapFloat(temperaturePosManual * 1.0, 0.0, 1023.0, faderTemperatureRangeBottomKelvin * 1.0, faderTemperatureRangeTopKelvin * 1.0));
      if (newTemperatureKelvinManual != temperatureKelvinManual) {
        temperatureKelvinManual = newTemperatureKelvinManual;
        gUpdateValue(&temperatureKelvinManual);
      }

      // READ LIGHT SENSOR

      if (remoteChannel == 0 || remoteOverride) {
        // light sensor
        measureLight();
        gUpdateValue(&lightSensorLux);
        gUpdateValue(&lightSensorCt);
        gUpdateValue(&lightSensorLevelPromille);
      }

      // READ MOVEMENT SENSOR

      if (pirReading == 1) {
        pirLevelNormalised = min(pirLevelNormalised + 0.01, 2.0);
      } else {
        if (pirLevelNormalised < 0.25) {
          pirLevelNormalised = max(pirLevelNormalised - 0.0075, 0.0);
        } else {
          pirLevelNormalised *= 0.995;
        }
      }

      pirLevelPromille = min(round(pirLevelNormalised * 1000.0), 1000);
      gUpdateValue(&pirLevelPromille);

      // CALCULATE OUTPUT

      if (!remoteOverride) {
        if (remoteChannel == conf->id) {

          // movement

          int intensityPromilleSensorRanged = map(pirLevelPromille, 0, 1000, faderIntensityRangeBottomPromille, faderIntensityRangeTopPromille);
          int temperatureKelvinSensorRanged = map(pirLevelPromille, 0, 1000, faderTemperatureRangeBottomKelvin, faderTemperatureRangeTopKelvin);

          intensityPromilleOutput = round(lerp(intensityPromilleManual, intensityPromilleSensorRanged, mapFloat(remoteMixLevel, 0, remoteMixMax, 0.0, 1.0 )));
          temperatureKelvinOutput = round(lerp(temperatureKelvinManual, temperatureKelvinSensorRanged, mapFloat(remoteMixLevel, 0, remoteMixMax, 0.0, 1.0 )));

        } else if (remoteChannel != 0) {

          // connected

          int intensityPromilleRemoteRanged = map(intensityPromilleRemote, 0, 1000, faderIntensityRangeBottomPromille, faderIntensityRangeTopPromille);
          int temperatureKelvinRemoteRanged = map(temperatureKelvinRemote, temperatureKelvinMin, temperatureKelvinMax, faderTemperatureRangeBottomKelvin, faderTemperatureRangeTopKelvin);

          intensityPromilleOutput = round(lerp(intensityPromilleManual, intensityPromilleRemoteRanged, mapFloat(remoteMixLevel, 0, remoteMixMax, 0.0, 1.0 )));
          temperatureKelvinOutput = round(lerp(temperatureKelvinManual, temperatureKelvinRemoteRanged, mapFloat(remoteMixLevel, 0, remoteMixMax, 0.0, 1.0 )));
        } else if (remoteChannel == 0) {

          // light sensor

          int intensityPromilleSensorRanged = constrain(map(lightSensorLevelPromille, faderIntensityRangeBottomPromille , faderIntensityRangeTopPromille, 0, 1000), 0, 1000);
          int temperatureKelvinSensorRanged = map(lightSensorCt, temperatureKelvinMin, temperatureKelvinMax, faderTemperatureRangeBottomKelvin, faderTemperatureRangeTopKelvin);

          intensityPromilleOutput = round(lerp(intensityPromilleManual, intensityPromilleSensorRanged, mapFloat(remoteMixLevel, 0, remoteMixMax, 0.0, 1.0 )));
          temperatureKelvinOutput = round(lerp(temperatureKelvinManual, temperatureKelvinSensorRanged, mapFloat(remoteMixLevel, 0, remoteMixMax, 0.0, 1.0 )));

        }

        if (touchFaderIntensity) {
          intensityPromilleOutput = intensityPromilleManual;
        }

        if (touchFaderTemperature) {
          temperatureKelvinOutput = temperatureKelvinManual;
        }
      }

      gUpdateValue(&intensityPromilleOutput);
      gUpdateValue(&temperatureKelvinOutput);


      // SET FADER OUTPUTS IF NOT TOUCHED

      if (!touchFaderIntensity) {
        if (faderIntensity._state == Fader::F_MANUAL) {
          faderIntensity.setAutomatic();
        }
        if (faderIntensityRangeBottomPromille != faderIntensityRangeTopPromille)
          faderIntensity.setSetpointNormalised(max(min(1.0, mapFloat(1.0 * intensityPromilleOutput, 1.0 * faderIntensityRangeBottomPromille, 1.0 * faderIntensityRangeTopPromille, 0.0, 1.0 )), 0.0));
        else
          faderIntensity.setSetpointNormalised(max(min(1.0, mapFloat(1.0 * intensityPromilleOutput, 0.0, 1000.0, 0.0, 1.0 )), 0.0));
      }

      if (!touchFaderTemperature) {
        if (faderTemperature._state == Fader::F_MANUAL) {
          faderTemperature.setAutomatic();
        }
        if (faderTemperatureRangeBottomKelvin != faderTemperatureRangeTopKelvin)
          faderTemperature.setSetpointNormalised(max(min(1.0, mapFloat(1.0 * temperatureKelvinOutput, 1.0 * faderTemperatureRangeBottomKelvin, 1.0 * faderTemperatureRangeTopKelvin, 0.0, 1.0 )), 0.0));
        else
          faderTemperature.setSetpointNormalised(max(min(1.0, mapFloat(1.0 * temperatureKelvinOutput, 1.0 * temperatureKelvinMin, 1.0 * temperatureKelvinMax, 0.0, 1.0 )), 0.0));
      }

      faderIntensity.setUseRanges(useFaderRanges > 0 || !remoteChannel == 0); // light sensor has other logic
      faderTemperature.setUseRanges(useFaderRanges > 0);
      faderIntensity.update();
      faderTemperature.update();

      // SEND DMX

      sendDMXi2c( intensityPromilleOutput,  temperatureKelvinOutput,  dmxBoardWireAddress);
      //sendDMXanalog( intensityPromilleOutput,  temperatureKelvinOutput );

      // Display

      lcd.setCursor(15, 1);
      if (thisFrameMillis - millisPreviousCommand > 5) {
        lcd.write(byte(0)); // bluetooth icon
      } else {
        lcd.print(" ");
      }

      lcd.setCursor(1, 1);
      if (thisFrameMillis % 6000 > 3000 && remoteChannel == 0 && !touchFaderIntensity) {
        lcdPrintNumberPadded(lightSensorLux, 4, ' ');
        lcd.print("Lx");
      } else {
        if (faderIntensity._useRanges && faderIntensity._potRangeTopValue < 1023)
          lcd.write(byte(3)); // truncated range
        else if (touchFaderIntensity)
          lcd.print("[");
        else
          lcd.print(" ");
        if (touchFaderIntensity) {
          lcdPrintNumberPadded(intensityPromilleManual / 10, 3, ' ');
        } else {
          lcdPrintNumberPadded(intensityPromilleOutput / 10, 3, ' ');
        }
        lcd.print("%");
        if (faderIntensity._useRanges && faderIntensity._potRangeBottomValue > 0)
          lcd.write(byte(4)); // truncated range
        else if (touchFaderIntensity)
          lcd.print("]");
        else
          lcd.print(" ");

        lcd.print(" ");

      }
      lcd.setCursor(8, 1);
      if (faderTemperature._useRanges && faderTemperature._potRangeTopValue < 1023)
        lcd.write(byte(3)); // truncated range
      else if (touchFaderTemperature)
        lcd.print("[");
      else
        lcd.print(" ");
      if (touchFaderTemperature) {
        lcdPrintNumberPadded(temperatureKelvinManual, 4, ' ');
      } else {
        lcdPrintNumberPadded(temperatureKelvinOutput, 4, ' ');
      }
      lcd.print("k");
      if (faderTemperature._useRanges && faderTemperature._potRangeBottomValue > 0)
        lcd.write(byte(4)); // truncated range
      else if (touchFaderTemperature)
        lcd.print("]");
      else
        lcd.print(" ");
      if (touchFaderTemperature) {
        temperatureToColor(temperatureKelvinManual, displayRed, displayGreen, displayBlue);
      } else {
        temperatureToColor(temperatureKelvinOutput, displayRed, displayGreen, displayBlue);
      }

      if (identify > 0) {
        float sinusoidal = 0.5 + sin(millis() * 0.001 * TWO_PI) / 2.0;
        float sinusoidalHalf = 0.5 + sin(((millis() * 0.001) + TWO_PI) * PI) / 2.0;
        setDisplayColorRGB(lerp(displayRed, 1.0, sinusoidalHalf) * sinusoidal, lerp(displayGreen, 1.0, sinusoidalHalf) * sinusoidal, lerp(displayBlue, 1.0, sinusoidalHalf) * sinusoidal);
      } else if (remoteOverride > 0) {
        float sinusoidal = 0.5 + sin(millis() * 0.0005 * TWO_PI) / 2.0;
        setDisplayColorRGB(lerp(0.0, displayRed, sinusoidal), lerp(0.0, displayGreen, sinusoidal), lerp(1.0, displayBlue, sinusoidal));
      } else {
        setDisplayColorRGB(displayRed, displayGreen, displayBlue);
      }

      //gUpdateValue(&millisPerFrame);
      //gUpdateValue(&batteryLevel);
      tqSize = tq.size();
      //gUpdateValue(&tqSize);

      break;

  }

  /*
    //  Battery level

    batteryLevelSmoothNormalised *= 0.99;
    batteryLevelSmoothNormalised += 0.01 * constrain(mapFloat(pow(getBatteryLevelNormalised(), 2), 0.75, 0.975, 0.0, 1.0), 0.0, 1.0);

    if (frameCount % 20 == 0) {
    int newBatteryLevel = round(batteryLevelSmoothNormalised * (batteryLevels - 1));
    if (newBatteryLevel != batteryLevel) {
      batteryLevel = newBatteryLevel;
      createChar(lcd, 2, lcdCharBatteryLevels[batteryLevel]);
    }
    lcd.setCursor(0, 1);
    lcd.write(byte(2));
    }
  */

  int newPirReading = pirADS->readADC_SingleEnded(pirADCChannel);
  if (newPirReading > 10000) {
    pirReading = 1;
  } else {
    pirReading = 0;
  }
  if (pirLedActive > 0) {
    pirReadingFiltered *= 0.75;
    pirReadingFiltered += pirReading * 0.25;
    pwm.setPWM(3, 0, floor(pirReadingFiltered * 0xFFF));
  } else if (pirReadingFiltered > 0.0 && pirReading == 0) {
    pirReadingFiltered = 0.0;
    pwm.setPWM(pirLEDChannel, 0, 0);
  }
  gUpdateValue(&pirReading);


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
    millisPreviousCommand = millisLastFrame;
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

  gBindInt(0, maxRemoteChannels, hardware::ownID, &newID);
  gBindInt(0, maxRemoteChannels, hardware::remoteID, &remoteChannel);
  gUpdateConstValue(hardware::versionMajor, versionMajor);
  gUpdateConstValue(hardware::versionMinor, versionMinor);
  gBindInt(0, remoteMixMax, hardware::mixRemote, &remoteMixLevel);
  // mixNoise,

  gBindInt(0, 1000,                                      hardware::intensityFader,           &intensityPromilleManual);
  gBindInt(temperatureKelvinMin, temperatureKelvinMax,   hardware::temperatureFader,         &temperatureKelvinManual);
  gBindInt(0, 1000,                                      hardware::intensityRemote,          &intensityPromilleRemote);
  gBindInt(temperatureKelvinMin, temperatureKelvinMax,   hardware::temperatureRemote,        &temperatureKelvinRemote);

  // intensityNoise,
  // temperatureNoise,

  gBindInt(0, 1000,                                      hardware::intensityOutput,          &intensityPromilleOutput);
  gBindInt(temperatureKelvinMin, temperatureKelvinMax,   hardware::temperatureOutput,        &temperatureKelvinOutput);
  gBindInt(0, 1023,                                      hardware::intensityRangeTop,        &(faderIntensity._potRangeTopValue));
  gBindInt(0, 1023,                                      hardware::intensityRangeBottom,     &(faderIntensity._potRangeBottomValue));
  gBindInt(0, 1023,                                      hardware::temperatureRangeTop,      &(faderTemperature._potRangeTopValue));
  gBindInt(0, 1023,                                      hardware::temperatureRangeBottom,   &(faderTemperature._potRangeBottomValue));
  gBindInt(0, 1,                                         hardware::useRanges,                &useFaderRanges);
  gBindInt(0, 1,                                         hardware::movementSensor,           &pirReading);
  gBindInt(0, 1,                                         hardware::movementSensorLedActive,  &pirLedActive);
  gBindInt(0, 1000,                                      hardware::movementSensorLevel,      &pirLevelPromille);

  gBindInt(0, 30000,                                     hardware::lightSensorLux,           &lightSensorLux);
  gBindInt(0, 10000,                                     hardware::lightSensorTemperature,   &lightSensorCt);
  gBindInt(0, 1000,                                      hardware::lightSensorLightLevel,    &lightSensorLevelPromille);

  gBindInt(0, 1,                                         hardware::remoteOverride,           &remoteOverride);
  gBindInt(0, 1,                                         hardware::identify,                 &identify);

  // doFaderCalibration, // handled in update
  // doSaveId            // handled in update

}

void gItemUpdated(int id)
{

  if (hardware::mixRemote == id) {
    // update bar graph incremental char on remote mix setting
    createChar(lcd, 7, lcdCharBarGraphs[remoteMixLevel % 5]);
  }

  if (hardware::doSaveId == id)
  {
    if (newID != conf->id) {
      saveConf(newID);
      gUpdateConstValue(hardware::doSaveId, 0);
    }
  }

  if (hardware::doFaderCalibration == id)
  {
    calibrateFaders();
    gUpdateConstValue(hardware::doFaderCalibration, 0);
  }

  /*
    if (id == pidAId || id == pidPId || id == pidIId || id == pidDId) {

      faderPidA = faderPidAint / 100.0;
      faderPidP = (faderPidPint / 100.0) * faderPidA;
      faderPidI = (faderPidIint / 100.0) * faderPidA;
      faderPidD = (faderPidDint / 10000.0) * faderPidA;
      faderIntensity._pid->SetTunings(faderPidP, faderPidI, faderPidD);
      faderTemperature._pid->SetTunings(faderPidP, faderPidI, faderPidD);
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

boolean measureLight() {
  boolean newData = false;
  if (lightSensor.getDataAsync()) {
    //    lightSensorR_raw = lightSensorR = lightSensor.r_comp;
    //    lightSensorG_raw = lightSensorG = lightSensor.g_comp;
    //    lightSensorB_raw = lightSensorB = lightSensor.b_comp;
    //    lightSensorC_raw = lightSensorC = lightSensor.c_comp;
    lightSensorLux_raw = lightSensor.lux;
    lightSensorCt_raw = lightSensor.ct;
    //    lightSensorAgc_raw = lightSensorAgc = lightSensor.agc_cur;
    newData = true;
  }

  // smooth out lux and ct

  float smoothUpdateFactor = 0.15;

  lightSensorLuxFloat *= (1.0 - smoothUpdateFactor);
  lightSensorLuxFloat += lightSensorLux_raw * smoothUpdateFactor;
  lightSensorLux = round(lightSensorLuxFloat);

  lightSensorCtFloat *= (1.0 - smoothUpdateFactor);
  lightSensorCtFloat += lightSensorCt_raw * smoothUpdateFactor;
  lightSensorCt = round(lightSensorCtFloat);

  lightSensorStep = lightSensorStepFromLux(lightSensorLux);

  lightSensorStepInt = floor(lightSensorStep * 1000);
  lightSensorLevelNormalised = log10(lightSensorLux) / 5.0;
  lightSensorLevelPromille = floor(lightSensorLevelNormalised * 1000);
  return newData;
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

void sendDMXi2c(unsigned int iP, unsigned int tK, int addr) {

  //  if (nextDMXSendMillis < millis()) {
  //    nextDMXSendMillis = millis() + 100;
  Wire.beginTransmission( addr ); {
    byte ilow = lowByte(iP);
    byte ihi = highByte(iP);
    byte tlow = lowByte(tK);
    byte thi = highByte(tK);
    byte data[] = {ilow, ihi, tlow, thi};
    Wire.write(data, 4);
  }
  Wire.endTransmission();
  //  }
}

void sendDMXanalog(unsigned int iP, unsigned int tK) {
  float pwmIntensity = mapFloat(iP, 0.0, 1000.0, 0.0, 1.0);
  float pwmTemperature = mapFloat(tK, temperatureKelvinMin, temperatureKelvinMax, 0.0, 1.0);
  pwm.setPWM(4, 0, round( 0xFFF * pwmIntensity ));
  pwm.setPWM(5, 0, round( 0xFFF * pwmTemperature));
}


float getBatteryLevelNormalised() {
  return constrain((analogRead(1) / 1023.0) * (3.33 / 2.366), 0.0, 1.0);
}

void setTempRangesFromi2c(int addr) {

  byte hbMin;
  byte lbMin;
  byte hbMax;
  byte lbMax;

  Wire.requestFrom(addr, 4);    // request 4 bytes from slave device
  delay(100);
  if (Wire.available() == 4)   // if four bytes were received
  {
    lbMin = Wire.read();
    hbMin = Wire.read();
    lbMax = Wire.read();
    hbMax = Wire.read();

    int tMin = word(hbMin, lbMin);
    int tMax = word(hbMax, lbMax);

    temperatureKelvinMin = tMin;
    temperatureKelvinMax = tMax;
  }
}

float lightSensorStepFromLux(int lux) {
  // https://msdn.microsoft.com/en-us/library/windows/desktop/dd319008(v=vs.85).aspx
  // In this example, the expected values range from 0 lux to 1,000,000 lux

  if (lightSensorLux <= 10) {
    // Pitch Black
    return lightSensorLux / 10.0;
  } else if (lightSensorLux <= 50 ) {
    // Very Dark
    return ((lightSensorLux - 10) / 40.0)       + 1.0;
  } else if (lightSensorLux <= 200 ) {
    // Dark Indoors
    return ((lightSensorLux - 50) / 150.0)      + 2.0;
  } else if (lightSensorLux <= 400 ) {
    // Dim Indoors
    return ((lightSensorLux - 200) / 200.0)     + 3.0;
  } else if (lightSensorLux <= 1000 ) {
    // Normal Indoors
    return ((lightSensorLux - 400) / 600.0)     + 4.0;
  } else if (lightSensorLux <= 5000 ) {
    // Bright Indoors
    return ((lightSensorLux - 1000) / 4000.0)   + 5.0;
  } else if (lightSensorLux <= 10000 ) {
    // Dim Outdoors
    return ((lightSensorLux - 5000) / 5000.0)   + 6.0;
  } else if (lightSensorLux <= 30000 ) {
    // Cloudy Outdoors
    return ((lightSensorLux - 10000) / 20000.0) + 7.0;
  } else {
    // Direct Sunlight
    return min(((lightSensorLux - 30000) / 70000.0) + 8.0, 9.0);
  }

}
