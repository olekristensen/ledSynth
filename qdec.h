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
