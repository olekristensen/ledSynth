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

#define DMX_ADRESSES 200

class DmxFixture {
  public:
    DmxFixture(int kelvinLow, int kelvinHigh, int channelCount) {
      _kelvinLow = kelvinLow;
      _kelvinHigh = kelvinHigh;
      _channelCount = channelCount;
    };
    virtual int setChannels(byte data[], int startChannel, float normalisedIntensity, int temperatureKelvin);
    int _kelvinLow;
    int _kelvinHigh;
    int _channelCount;
    DmxFixture * next;
};

class DmxFixtureCWVW8bit : public DmxFixture {
  public:
    DmxFixtureCWVW8bit(int kelvinLow = 2700, int kelvinHigh = 6500, int channelCount = 2) : DmxFixture(kelvinLow, kelvinHigh, channelCount) {
      ;
    }
    float gamma = 1.2;
    float invGamma = 1.0/gamma;
    int setChannels(byte data[], int startChannel, float normalisedIntensity, int temperatureKelvin) {
      float temperatureNormalisedInRange = constrain(mapFloat(temperatureKelvin, _kelvinLow, _kelvinHigh, 0.0, 1.0), 0.0, 1.0);
      data[startChannel] = byte(pow(temperatureNormalisedInRange, invGamma) * normalisedIntensity * 255);
      data[startChannel + 1] = byte(pow((1.0 - temperatureNormalisedInRange), invGamma) * normalisedIntensity * 255);
      return _channelCount;
    };
};

class TVL2000 : public DmxFixtureCWVW8bit {
  public:
  TVL2000(int kelvinLow = 2700, int kelvinHigh = 6500, int channelCount = 2) : DmxFixtureCWVW8bit(kelvinLow, kelvinHigh, channelCount) {
    ;
  };
};

class i2cDmx {
  public:
    i2cDmx(void) {
      head = NULL;
      tail = NULL;
      needsUpdate = false;
      _kelvin = 0;
      _intensity = 0;
      _kelvinHigh = -1; 
      _kelvinLow = -1;
    };

    boolean begin(void);
    
    void setTemperatureKelvin(int kelvin) {
      if (abs(_kelvin-kelvin) > 1) {
        _kelvin = kelvin;
        needsUpdate = true;
      }
    };
    
    void setIntensity(float intensity) {
      if (abs(_intensity-intensity) > 1.0/1023.0) {
        _intensity = intensity;
        needsUpdate = true;
      }
    };
    
    int sendDmx() {
      int currentAddress = 0;
      if (needsUpdate) {
        DmxFixture * currentFixture = head;
        while (currentFixture != NULL) {
          currentAddress += currentFixture->setChannels(data, currentAddress, _intensity, _kelvin);
          currentFixture = currentFixture->next;
        }
      
        Wire.beginTransmission(100); // transmit to device #100 0x64
        for(int i = 0; i < currentAddress; i++){
          Wire.write(data[i]);
        }
        Wire.endTransmission();    // stop transmitting
        needsUpdate = false;
      }
      return currentAddress;
    };
    
    void addFixture(DmxFixture * newFixture) {
      newFixture->next = NULL;
      
      if(newFixture->_kelvinLow < _kelvinLow || _kelvinLow < 0)
        _kelvinLow = newFixture->_kelvinLow;
        
      if(newFixture->_kelvinHigh > _kelvinHigh || _kelvinHigh < 0)
        _kelvinHigh = newFixture->_kelvinHigh;
 
      if (head == NULL) {
        head = newFixture;
        tail = newFixture;
      } else {
        tail->next = newFixture;
        tail = newFixture;
      }
    };
    
    int getKelvinLow(){
      return _kelvinLow;
    }

    int getKelvinHigh(){
      return _kelvinHigh;
    }

  private:
    byte data[DMX_ADRESSES];
    DmxFixture * head;
    DmxFixture * tail;
    int _kelvin;
    int _kelvinLow;
    int _kelvinHigh;
    float _intensity;
    bool needsUpdate;
};

