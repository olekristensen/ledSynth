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

// select a flash page that isn't in use (see Memory.h for more info)
#define  CONF_FLASH_PAGE  250

#define CONF_STATE_OK 'l'
#define CONF_STATE_DEFAULT 'd'
#define CONF_STATE_ERROR 'e'

struct flash_conf_t
{
  int id;
  char state;
};

static flash_conf_t *conf;

void debug_conf( struct flash_conf_t *p )
{
  Serial.print("  id = ");
  Serial.println(p->id);
  Serial.print("  state = ");
  Serial.println(p->state);
}

boolean loadConf() {
  // a flash page is 1K in length, so page 251 starts at address 251 * 1024 = 257024 = 3EC00 hex
  conf = (flash_conf_t*)ADDRESS_OF_PAGE(CONF_FLASH_PAGE);
  if (conf->id > 0 && conf->state == CONF_STATE_OK) {
    return true;
  } else {
    return false;
  }
}

boolean eraseConf() {
  int rc = flashPageErase(CONF_FLASH_PAGE);
  if (rc == 0)
    return true;
  else if (rc == 1)
    Serial.println("Error - the flash page is reserved");
  else if (rc == 2)
    Serial.println("Error - the flash page is used by the sketch");
  return false;
}

boolean writeConf(struct flash_conf_t value) {
  if (eraseConf()) {
    conf = (flash_conf_t*)ADDRESS_OF_PAGE(CONF_FLASH_PAGE);
    int rc = flashWriteBlock(conf, &value, sizeof(value));
    debug_conf(conf);
    if (rc == 0)
      return true;
    else if (rc == 1)
      Serial.println("Error - the flash page is reserved");
    else if (rc == 2)
      Serial.println("Error - the flash page is used by the sketch");
    return false;
  }
}

