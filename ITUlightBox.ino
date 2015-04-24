// TODO: Configuration struct with IDsa and Consts in EEPROM
// TODO: Binary protocol for bluetooth
// TODO: Range pots
// TODO: Rotary encoder
// TODO: Light sensor

// IDENTITY

const int id = 2;
const char *idChars = "2";
const String identityString = "LEDSYNTH";
const char *identityChars = "LEDSYNTH";

const int versionMajor = 0;
const int versionMinor = 3;

#include <Wire.h>
#include <LiquidTWI.h>               // Display
#include <Adafruit_ADS1015.h>        // Analog Digital Converter
#include <Adafruit_MotorShield.h>    // Motor Shield
#include <PID_v1.h>
#include <RFduinoBLE.h>
#include <QueueList.h>
#include <String.h>
#include "Fader.h"


// DISPLAY

LiquidTWI lcd(0);                     //0x20

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

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

const double faderMotorSpeed = 200;//156;
const int faderMotorHertz = 110 / 2;
const double faderPidP = .75;
const double faderPidI = 20;
const double faderPidD = 0.002;

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


// BLUETOOTH
enum bleInputStates { BIS_START, BIS_COMMAND, BIS_DATA, BIS_END };
int bleInputState = BIS_START;
char bleInputCommandBuf;
String bleInputHexstringBuf;

struct bleCommand{
  char command;
  String hexstring;
};

QueueList <bleCommand> bleCommandQueue;

bool bleAdvertising = false;

// CONTROL

enum State { S_SETUP, S_FADER_CALIBRATE, S_PID_TEST, S_STANDALONE_SETUP, S_STANDALONE_LOOP, S_CONNECTED_SETUP, S_CONNECTED_LOOP };
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

  lcd.createChar(0, lcdCharBluetooth);   //Bluetooth Icon
  lcd.createChar(1, lcdCharNumbers[id]); //Fat indentity number

  lcd.clear();
  //Print unit info on display
  lcd.write(byte(1));
  lcd.print(" ");
  lcd.print(identityString);
  lcd.setCursor(2, 1);
  lcd.print("v.");
  lcd.print(versionMajor);
  lcd.print(".");
  lcd.print(versionMinor);

  //Start BLE Advertisement
  RFduinoBLE.advertisementInterval = 200;
  RFduinoBLE.deviceName = identityChars;
  RFduinoBLE.advertisementData = idChars;
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
    case S_SETUP :
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
      state = S_STANDALONE_SETUP;
      lcd.clear();
      lcd.write(byte(1));
      lcd.print(" ");
      lcd.print("calibrating...");
      faderTemperature.calibrate();
      faderIntensity.calibrate();
      RFduinoBLE.begin();
      lcd.clear();
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
      
      while(!bleCommandQueue.isEmpty()){
        processCommand(bleCommandQueue.pop());
      }
      
      faderIntensity.update();
      faderTemperature.update();
      // Display
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


void queueCommand (char command, String hexstring){
  
  while(bleCommandQueue.count() > 20){
    if(bleCommandQueue.peek().command == command)
      bleCommandQueue.pop();
  }
  bleCommand cmd = {command, hexstring};
  bleCommandQueue.push(cmd);
}

void processCommand (struct bleCommand cmd){
    switch (cmd.command) {
        case 'I':                                          
        faderIntensity.setSetpointNormalised(cmd.hexstring.toInt()/65535.0);
        break;
        case 'T':                                          
        faderTemperature.setSetpointNormalised(cmd.hexstring.toInt()/65535.0);
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
            while (! RFduinoBLE.send(bleInputCommandBuf))
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

