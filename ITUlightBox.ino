// IDENTITY

const int id = 1;
const char *idChars = "1";
const String identityString = "LightSynth";
const char *identityChars = "LightSynth";

const int versionMajor = 0;
const int versionMinor = 3;

#include <RFduinoBLE.h>
#include <Wire.h>
#include <LiquidTWI.h>               // Display
#include <Adafruit_ADS1015.h>        // Analog Digital Converter
#include <Adafruit_MotorShield.h>    // Motor Shield
#include <PID_v1.h>
#include "Fader.h"


// DISPLAY

LiquidTWI lcd(0);                     //0x20

// FADERS

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

const double faderMotorSpeed = 156;
const int faderMotorHertz = 55;
const double faderPidP = .75;
const double faderPidI = 20;
const double faderPidD = 0.0002;

const int faderPidSampleRate = 10;                  // Calling compute() every 10 ms.

Adafruit_ADS1115 faderIntensityPots(0x49);/*16-bit*/ //0x49
Fader faderIntensity(
  AFMS.getMotor(1),
  &faderIntensityPots, 0,
  faderPidP, faderPidI, faderPidD, faderPidSampleRate);

Adafruit_ADS1115 faderTemperaturePots(0x4A);/*16-bit*/ //0x4A
Fader faderTemperature(
  AFMS.getMotor(2),
  &faderTemperaturePots, 0,
  faderPidP, faderPidI, faderPidD, faderPidSampleRate);

// PWM

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// CONTROL

enum State { S_SETUP, S_FADER_CALIBRATE, S_PID_TEST, S_RUNNING };
int state = S_SETUP;
long millisLastFrame = 0;
long frameCount = 0;

void setup() {

  Serial.begin(9600); // set up Serial library at 9600 bps
  Wire.speed = 400;

  //Print unit info on serial
  Serial.println(identityString);
  Serial.print("id:\t");
  Serial.println(id);
  Serial.print("ver.\t");
  Serial.print(versionMajor);
  Serial.print(".");
  Serial.print(versionMinor);

  lcd.begin(16, 2);
  lcd.clear();
  //Print unit info on display
  lcd.print(identityString);
  lcd.print(" ");
  lcd.print(id);
  lcd.setCursor(0, 1);
  lcd.print("ver. ");
  lcd.print(versionMajor);
  lcd.print(".");
  lcd.print(versionMinor);

  //Start BLE Advertisement
  RFduinoBLE.advertisementInterval = 675;
  RFduinoBLE.deviceName = identityChars;
  RFduinoBLE.advertisementData = idChars;
  RFduinoBLE.txPowerLevel = +4;
  RFduinoBLE.begin();

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
  switch (state) {
    case S_SETUP :
      delay(1000);
      state = S_FADER_CALIBRATE;
      break;
    case S_FADER_CALIBRATE :
      lcd.setCursor(0, 1);
      lcd.print("Calibrating...");
      faderTemperature.calibrate();
      faderIntensity.calibrate();
      state = S_PID_TEST;
      break;
    case S_PID_TEST :
      // FADER TEST

      //Intensity

      faderIntensity.update();

      if (fmod(t, 3.0) < 2.0) {
        faderIntensity.setSetpoint(
          mapFloat(sin(sin(t * 2.0) * fmod(t, 3.0)), -1.0, 1.0, 24.0, 1000.0)
        );
      }

      //Temperature

      faderTemperature.update();

      //t += 0.5;

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
    case S_RUNNING  :
      ;
      break;
  }
  frameCount++;
  millisLastFrame = thisFrameMillis;
}

// UTILS

int numDigits(int number)
{
  int digits = 0;
  if (number < 0) {
    digits = 1;
  }
  do {
    number /= 10;
    digits++;
  } while (number != 0);
  return digits;
}

void lcdPrintNumberPadded(int number, int len, char padding) {
  for (int i = 0; i < len - numDigits(number); i++) {
    lcd.print(padding);
  }
  lcd.print(number);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void comm_i2c(int address, int data) {
  Wire.beginTransmission(address);
  Wire.write(byte(data));
  Wire.endTransmission();
  delay(5);
}

void send_i2c(int address, int data, int val) {
  Wire.beginTransmission(address);
  Wire.write(byte(data));
  Wire.write(byte(val));
  Wire.endTransmission();
  delay(5);
}
