#include "Arduino.h"

//
// An experimental wrapper class that implements the improved lux and color temperature from
// TAOS and a basic autorange mechanism.
//
// Written by ductsoup, public domain
// Modified by ole kristensen

// RGB Color Sensor with IR filter and White LED - TCS34725
// I2C 7-bit address 0x29, 8-bit address 0x52
//
// http://www.adafruit.com/product/1334
// http://learn.adafruit.com/adafruit-color-sensors/overview
// http://www.adafruit.com/datasheets/TCS34725.pdf
// http://www.ams.com/eng/Products/Light-Sensors/Color-Sensor/TCS34725
// http://www.ams.com/eng/content/view/download/265215 <- DN40, calculations
// http://www.ams.com/eng/content/view/download/181895 <- DN39, some thoughts on autogain
// http://www.ams.com/eng/content/view/download/145158 <- DN25 (original Adafruit calculations)
//
// connect LED to digital 4 or GROUND for ambient light sensing
// connect SCL to analog 5
// connect SDA to analog 4
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

// some magic numbers for this device from the DN40 application note
#define TCS34725_R_Coef 0.136
#define TCS34725_G_Coef 1.000
#define TCS34725_B_Coef -0.444
#define TCS34725_GA 1.0869 // Clear acryllic (PLEXIGLAS® XT, SPMMA0030XRCL00, ~92% transmittance, UV Blocking) : 1/0.92
#define TCS34725_DF 310.0
#define TCS34725_CT_Coef 3810.0
#define TCS34725_CT_Offset 1391.0

// Autorange class for TCS34725
class tcs34725 {
  public:
    tcs34725(void);

    boolean begin(void);
    void getData(void);
    boolean getDataAsync(void);

    boolean isAvailable, isSaturated;
    uint16_t againx, atime, atime_ms;
    uint16_t r_raw, g_raw, b_raw, c_raw;
    uint16_t r, g, b, c;
    uint16_t ir;
    uint16_t r_comp, g_comp, b_comp, c_comp;
    uint16_t saturation, saturation75;
    float cratio, cpl, ct, lux, maxlux;
    uint16_t agc_cur;

  private:
    struct tcs_agc {
      tcs34725Gain_t ag;
      tcs34725IntegrationTime_t at;
      uint16_t mincnt;
      uint16_t maxcnt;
    };
    static const tcs_agc agc_lst[];

    uint16_t asyncState = 0;
    uint16_t asyncStateNextMillis = 0;

    void setGainTime(void);
    Adafruit_TCS34725 tcs;
};
//
// Gain/time combinations to use and the min/max limits for hysteresis
// that avoid saturation. They should be in order from dim to bright.
//
// Also set the first min count and the last max count to 0 to indicate
// the start and end of the list.
//
const tcs34725::tcs_agc tcs34725::agc_lst[] = {
  { TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_700MS,     0, 20000 },
  { TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_154MS,  4990, 63000 },
  { TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS, 16790, 63000 },
  { TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS, 15740, 63000 },
  { TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_154MS, 15740, 0 }
  //  { TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_50MS,  15740, 0 }
};
tcs34725::tcs34725() : agc_cur(0), isAvailable(0), isSaturated(0) {
}

// initialize the sensor
boolean tcs34725::begin(void) {
  tcs = Adafruit_TCS34725(agc_lst[agc_cur].at, agc_lst[agc_cur].ag);
  if ((isAvailable = tcs.begin()))
    setGainTime();
  return (isAvailable);
}

// Set the gain and integration time
void tcs34725::setGainTime(void) {
  tcs.setGain(agc_lst[agc_cur].ag);
  tcs.setIntegrationTime(agc_lst[agc_cur].at);
  atime = int(agc_lst[agc_cur].at);
  atime_ms = ((256 - atime) * 2.4);
  switch (agc_lst[agc_cur].ag) {
    case TCS34725_GAIN_1X:
      againx = 1;
      break;
    case TCS34725_GAIN_4X:
      againx = 4;
      break;
    case TCS34725_GAIN_16X:
      againx = 16;
      break;
    case TCS34725_GAIN_60X:
      againx = 60;
      break;
  }
}

// Retrieve data from the sensor and do the calculations
void tcs34725::getData(void) {
  // read the sensor and autorange if necessary
  tcs.getRawData(&r, &g, &b, &c);

  while (1) {
    if (agc_lst[agc_cur].maxcnt && c > agc_lst[agc_cur].maxcnt)
      agc_cur++;
    else if (agc_lst[agc_cur].mincnt && c < agc_lst[agc_cur].mincnt)
      agc_cur--;
    else break;
    setGainTime();
    delay((256 - atime) * 2.4 * 2); // shock absorber
    tcs.getRawData(&r, &g, &b, &c);
    break;
  }

  // DN40 calculations
  ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;
  r_comp = max(r - ir, 0);
  g_comp = max(g - ir, 0);
  b_comp = max(b - ir, 0);
  c_comp = max(c - ir, 0);
  cratio = float(ir) / float(c);

  saturation = ((256 - atime) > 63) ? 65535 : 1024 * (256 - atime);
  saturation75 = (atime_ms < 150) ? saturation75 = saturation - saturation / 4 : saturation;
  isSaturated = (atime_ms < 150 && c > saturation75) ? 1 : 0;
  cpl = (atime_ms * againx) / (TCS34725_GA * TCS34725_DF);
  maxlux = 65535 / (cpl * 3);

  lux = (TCS34725_R_Coef * float(r_comp) + TCS34725_G_Coef * float(g_comp) + TCS34725_B_Coef * float(b_comp)) / cpl;
  ct = TCS34725_CT_Coef * float(b_comp) / float(r_comp) + TCS34725_CT_Offset;
}

// Retrieve data from the sensor and do the calculations
boolean tcs34725::getDataAsync(void) {

  long currentMillis = millis();

  if (asyncStateNextMillis < currentMillis) {
    if (asyncState == 0) {
      tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);
      asyncStateNextMillis = currentMillis + atime_ms;
      asyncState = 1;
    } else if (asyncState == 1) {
      if (agc_lst[agc_cur].maxcnt && c_raw > agc_lst[agc_cur].maxcnt)
        agc_cur++;
      else if (agc_lst[agc_cur].mincnt && c_raw < agc_lst[agc_cur].mincnt)
        agc_cur--;
      else {
        asyncState = 3;
        return false;
      }
      setGainTime();
      asyncStateNextMillis = currentMillis + (atime_ms * 4);
      asyncState = 2;
    } else if (asyncState == 2) {
      tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);
      asyncStateNextMillis = currentMillis += atime_ms;
      asyncState = 3;
    } else if (asyncState == 3) {
      r = r_raw;
      g = g_raw;
      b = b_raw;
      c = c_raw;

      // DN40 calculations
      // TODO avoid negative ct values ....
      ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;
      long r_ir = long(r) - ir;
      long g_ir = long(g) - ir;
      long b_ir = long(b) - ir;
      long c_ir = long(c) - ir;
      r_comp = constrain(r_ir, 0, 2147483647L);
      g_comp = constrain(g_ir, 0, 2147483647L);
      b_comp = constrain(b_ir, 0, 2147483647L);
      c_comp = constrain(c_ir, 0, 2147483647L);
      cratio = float(ir) / float(c);

      saturation = ((256 - atime) > 63) ? 65535 : 1024 * (256 - atime);
      saturation75 = (atime_ms < 150) ? saturation75 = saturation - saturation / 4 : saturation;
      isSaturated = (atime_ms < 150 && c > saturation75) ? 1 : 0;
      cpl = (atime_ms * againx) / (TCS34725_GA * TCS34725_DF);
      maxlux = 65535 / (cpl * 3);

      lux = (TCS34725_R_Coef * float(r_comp) + TCS34725_G_Coef * float(g_comp) + TCS34725_B_Coef * float(b_comp)) / cpl;
      if (lux > 2) { // don't calculate temperature below 2 lux, math screws up...
        float ct_raw = TCS34725_CT_Coef * float(b_comp) / float(r_comp) + TCS34725_CT_Offset;
        ct = max(ct_raw, 0.0);
      }
      asyncState = 0;
      return true;

    }

  }

  return false;
}
