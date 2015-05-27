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

#include "Arduino.h"
#include "utils.h"

class Fader {
  public:
    Adafruit_DCMotor *_motor;
    Adafruit_ADS1115 *_ads;
    int _adsFaderPosChannel;
    PID *_pid;
    long _millisLastFade = 0;
    double _setpoint = 0;
    double _pos = 0;
    double _speed = 0;
    double _outputLimit = 0;

    int16_t _faderAdsValue = 0;
    int16_t _faderCalibrationHigh = 17100;
    int16_t _faderCalibrationLow = 20;

    const int16_t faderPadding = 400;

    double potRangeFrom;
    double potIRangeTo;

    enum State { F_MANUAL, F_AUTOMATIC };
    int _state = F_AUTOMATIC;

    Fader(Adafruit_DCMotor *motor, Adafruit_ADS1115 *ads, int adsFaderPosChannel, double pidP, double pidI, double pidD, double sampleRate, double setpoint = 0, double spd = 0) {
      _motor = motor;
      _ads = ads;
      _adsFaderPosChannel = adsFaderPosChannel;
      _setpoint = setpoint;
      _speed = spd;
      _pid = new PID(&_pos, &_speed, &_setpoint, pidP, pidI, pidD, DIRECT);
      _pid->SetSampleTime(sampleRate); // Sets the sample rate
      _pid->SetMode(AUTOMATIC);
    }

    void setSpeedLimits(double outputLimit) {
      _outputLimit = outputLimit;
      _pid->SetOutputLimits(0.0 - outputLimit, outputLimit); // Set max speed for DC motors
    }

    void setAdsGain(adsGain_t gain) {
      _ads->setGain(gain);    // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    }

    void begin() {
      _ads->begin();
    }

    void setSetpoint(double setpoint) {
      _setpoint = setpoint;
    }

    void setSetpointNormalised(double setpoint) {
      setSetpoint(setpoint * 1023.0);
    }

    double getSetpoint() {
      return _setpoint;
    }

    double getSetpointNormalised() {
      return getSetpoint() / 1023.0;
    }

    double getPos() {
      return _pos;
    }

    int16_t getAdsValue(int adsChannel) {
      return _ads->readADC_SingleEnded(adsChannel);
    }

    int16_t getLastFaderAdsValue() {
      return _faderAdsValue;
    }

    void setManual() {
      if (_state != F_MANUAL) {
        _state = F_MANUAL;
        _pid->SetMode(MANUAL);
      }
    }

    void setAutomatic() {
      if (_state != F_AUTOMATIC) {
        _state = F_AUTOMATIC;
        _pid->SetMode(AUTOMATIC);
      }
    }

    void update() {
      _faderAdsValue = getAdsValue(_adsFaderPosChannel);
      _pos = constrain(mapFloat(_faderAdsValue, _faderCalibrationLow, _faderCalibrationHigh, 0.0, 1023.0), 0.0, 1023.0);
      if (_state == F_AUTOMATIC) {
        if (_pid->Compute()) {
          _motor->setSpeed(floor(abs(_speed)));
          if (abs(_pos - _setpoint) < 1.0 ) {
            _motor->run(RELEASE);
          } else if (_speed < 0.0) {
            if (_pos > 0.0) {
              _motor->run(BACKWARD);
            } else {
              _motor->run(RELEASE);
            }
          } else {
            if (_pos < 1023.0) {
              _motor->run(FORWARD);
            } else {
              _motor->run(RELEASE);
            }
          }
        }
      } else if (_state == F_MANUAL) {
        _motor->run(RELEASE);
        _setpoint = _pos;
      }
    }

    void calibrate() {
      _pid->SetMode(MANUAL);
      int16_t maxAdsValue = 0;
      int16_t minAdsValue = 17000;
      int16_t newAdsValue = 0;
      long lastTestMillis = 0;
      const int millisEndWindow = 150;
      const int minOutput = _outputLimit / 6;

      // find high ads value
      _motor->setSpeed(_outputLimit);
      _motor->run(FORWARD);
      lastTestMillis = millis();
      while (millis() - lastTestMillis < millisEndWindow) {
        newAdsValue = getAdsValue(_adsFaderPosChannel);
        if (newAdsValue > maxAdsValue) {
          maxAdsValue = max(maxAdsValue, newAdsValue);
          _motor->setSpeed(min(_outputLimit, map(maxAdsValue, minAdsValue, minAdsValue / 2, minOutput, _outputLimit)));
          lastTestMillis = millis();
        }
      }

      // find low ads value
      _motor->setSpeed(_outputLimit);
      _motor->run(BACKWARD);
      lastTestMillis = millis();
      while (millis() - lastTestMillis < millisEndWindow) {
        newAdsValue = getAdsValue(_adsFaderPosChannel);
        if (newAdsValue < minAdsValue) {
          minAdsValue = min(minAdsValue, newAdsValue);
          _motor->setSpeed(min(_outputLimit, map(minAdsValue, 0, maxAdsValue / 2, minOutput, _outputLimit)));
          lastTestMillis = millis();
        }
      }

      _faderCalibrationLow = minAdsValue + faderPadding;
      _faderCalibrationHigh = maxAdsValue - faderPadding;

      _motor->run(RELEASE);

      _pid->SetMode(AUTOMATIC);
    }

};
