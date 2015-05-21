/*  LEDsynth - a dmx controller for control praradigm experiments with LED fixtures.
    Copyright (C) 2015  Ole Kristensen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    ole@kristensen.name
    olek@itu.dk
*/

#include "Arduino.h"

class qdec {
  public:
    int phasePinA;
    int phasePinB;

    qdec(int pinA = 3, int pinB = 4) {

      pinMode(pinA, INPUT_PULLUP);
      pinMode(pinB, INPUT_PULLUP);
      NRF_QDEC->PSELA = pinA;
      NRF_QDEC->PSELB = pinB;

      // LED drive not used PSELLED default is off.
      NRF_QDEC->PSELLED = 0xFFFFFFFF; // Make sure it is OFF
      // LEDPRE is don't care if not using LED.
      // LEDPOL don't care about LED Polarity.
      // DBFEN default debounce disabled.
      NRF_QDEC->DBFEN = 0; // 0=disabled, 1=enabled
      // The SAMPLEPER default is 128 us.
      NRF_QDEC->SAMPLEPER = 3; // 0=128us, 3=1024us
      NRF_QDEC->REPORTPER = 1; //Not sure I need this if not using interrupts
      // Clear the accumulators with READCLRACC .. init I don't care
      NRF_QDEC->TASKS_READCLRACC = 1;
      NRF_QDEC->ACCREAD;
      NRF_QDEC->ACCDBLREAD;

    };

    void enable() {
      NRF_QDEC->ENABLE = 1;
      NRF_QDEC->TASKS_START = 1;
    };
    
    void disable() {
      NRF_QDEC->TASKS_STOP = 1;
      NRF_QDEC->ENABLE = 0;
    };
    
    int readDelta() {
      NRF_QDEC->TASKS_READCLRACC = 1;
      return NRF_QDEC->ACCREAD;
    };

};
