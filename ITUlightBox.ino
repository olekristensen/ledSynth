// IDENTITY

const int id = 1;
const char *idChars = "1";
const String identityString = "LightSynth";
const char *identityChars = "LightSynth";

const int versionMajor = 0;
const int versionMinor = 2;

#include <RFduinoBLE.h>
#include <Wire.h>
#include <LiquidTWI.h>               // Display
#include <Adafruit_ADS1015.h>        // Analog Digital Converter
#include <Adafruit_PWMServoDriver.h> // PWM
#include <OpenServo.h>               // Motorfaders

// DISPLAY

LiquidTWI lcd(0);                     //0x20

// FADERS

OpenServo faderIntensity(0x10);       //0x10
long millisLastFadeIntensity;
int faderIntensityDest = 0;
int faderIntensityVal = 0;

OpenServo faderTemperature(0x11);     //0x11
long millisLastFadeTemperature;
int faderTemperatureDest = 0;
int faderTemperatureVal = 0;

float pidP = 0.0;
float pidI = 0;
float pidD = 0;
float pidGain = 0;

// POTMETERS

Adafruit_ADS1115 faderRangePots(0x49);/*16-bit*/ //0x49
int16_t faderRangePot0, faderRangePot1, faderRangePot2, faderRangePot3;
double potIntensityRangeFrom;
double potIntensityRangeTo;
double potTemperatureRangeFrom;
double potTemperatureRangeTo;

// PWM

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// CONTROL

enum State { S_SETUP, S_ADDRESS_OPENSERVO, S_PID, S_RUNNING };
int state = S_SETUP;
long millisLastFrame = 0;
long frameCount = 0;

void setup() {

  Serial.begin(9600); // set up Serial library at 9600 bps

  //Print unit info on serial
  Serial.println(identityString);
  Serial.print("id:\t");
  Serial.println(id);
  Serial.print("ver.\t");
  Serial.print(versionMajor);
  Serial.print(".");
  Serial.print(versionMinor);

  Wire.speed = 400;

  //Start BLE Advertisement
  RFduinoBLE.advertisementInterval = 675;
  RFduinoBLE.deviceName = identityChars;
  RFduinoBLE.advertisementData = idChars;
  RFduinoBLE.txPowerLevel = +4;
  RFduinoBLE.begin();

  //Print unit info on display
  lcd.begin(16, 2);
  lcd.clear();
  // Print a message to the LCD.
  lcd.print(identityString);
  lcd.print(" ");
  lcd.print(id);
  lcd.setCursor(0, 1);
  lcd.print("ver. ");
  lcd.print(versionMajor);
  lcd.print(".");
  lcd.print(versionMinor);

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  faderRangePots.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  faderRangePots.begin();

  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

  state = S_PID;
  lcd.clear();
}

void loop() {
  long thisFrameMillis = millis();
  switch (state) {
    case S_SETUP :
      faderTemperature.enable();
      faderIntensity.enable();
      faderIntensity.disable();
      state = S_PID;
      ;
      break;
    case S_ADDRESS_OPENSERVO :
      lcd.clear();
      lcd.print("setting address");
      comm_i2c(0x10, 0x84);       // write enable
      send_i2c(0x10, 0x20, 0x11); // set address to 0x11
      comm_i2c(0x10, 0x86);       // save registers
      delay(2000);
      lcd.clear();
      lcd.print("testing");
      while (millis() < thisFrameMillis + 10000) {
        faderTemperatureVal = faderTemperature.getPosition();
        lcd.setCursor(0, 1);
        lcdPrintNumberPadded(faderTemperatureVal, 5, ' ');
      }
      state = S_SETUP;
      break;
    case S_PID :
      // faderRangePot0 = round(faderRangePot0 * 0.75 + faderRangePots.readADC_SingleEnded(0) * 0.25);
      faderRangePots.waitForConvertionComplete();
      faderRangePot0 = faderRangePots.readADC_SingleEnded(0);
      Serial.print("faderRangePot0:  ");
      Serial.print(faderRangePot0);
      Serial.print("\t");
      pidP = constrain(mapFloat(faderRangePot0, 20, 17100, 0.0, 2.0), 0.0, 2.0);
      faderRangePot1 = faderRangePots.readADC_SingleEnded(1);
      Serial.print("faderRangePot1:  ");
      Serial.print(faderRangePot1);
      Serial.print("\t");      
      pidI = constrain(mapFloat(faderRangePot1, 20, 17100,  0.0, 2.0), 0.0, 2.0);
      faderRangePot2 = round(faderRangePot2 * 0.75 + faderRangePots.readADC_SingleEnded(2) * 0.25);
      pidD = constrain(mapFloat(faderRangePot2, 20, 17100,  0.0, 2.0), 0.0, 2.0);
      faderRangePot3 = round(faderRangePot3 * 0.75 + faderRangePots.readADC_SingleEnded(3) * 0.25);
      pidGain = constrain(mapFloat(faderRangePot3, 20, 17100,  0.0, 2.0), 0.0, 2.0);
/*
      faderTemperature.setGainP(round((double)pidP * 1023.0 * pidGain));
      faderTemperature.setGainI(round((double)pidI * 1023.0 * pidGain));
      faderTemperature.setGainD(round((double)pidD * 1023.0 * pidGain));
*/
      faderIntensityVal = faderIntensity.getPosition();
      Serial.print("Int:  ");
      Serial.print(faderIntensityVal);
      Serial.print("\t");
      faderTemperatureVal = faderTemperature.getPosition();
      Serial.print("Temp: ");
      Serial.println(faderTemperatureVal);

      faderTemperatureDest = faderIntensityVal;
      faderTemperature.setPosition(faderTemperatureDest);

      if (thisFrameMillis % 2000 < 500) {
        lcd.setCursor(0, 0);
        lcdPrintNumberPadded(faderRangePot0, 5, ' ');
        lcdPrintNumberPadded(faderRangePot1, 6, ' ');
        lcd.setCursor(0, 1);
        lcdPrintNumberPadded(faderRangePot2, 5, ' ');
        lcdPrintNumberPadded(faderRangePot3, 6, ' ');
      } else if (thisFrameMillis % 2000 < 1000) {
        lcd.setCursor(0, 0);
        lcdPrintNumberPadded(pidP * 100.0, 5, ' ');
        lcdPrintNumberPadded(pidI * 100.0, 6, ' ');
        lcd.setCursor(0, 1);
        lcdPrintNumberPadded(pidD * 100.0, 5, ' ');
        lcdPrintNumberPadded(pidGain * 100.0, 6, ' ');
      }

      lcd.setCursor(11, 0);
      lcdPrintNumberPadded(faderIntensityVal, 5, ' ');
      lcd.setCursor(11, 1);
      lcdPrintNumberPadded(faderTemperatureDest - faderTemperatureVal, 5, ' ');

      ;
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
