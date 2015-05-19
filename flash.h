#include "Arduino.h"

// select a flash page that isn't in use (see Memory.h for more info)
#define  CONF_FLASH_PAGE  251

#define CONF_STATE_OK 'l'
#define CONF_STATE_DEFAULT 'd'
#define CONF_STATE_ERROR 'e'

struct flash_conf_t
{
  int id;
  char state;
};

flash_conf_t *conf;

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

boolean writeConf(struct flash_conf_t value) {
  conf = (flash_conf_t*)ADDRESS_OF_PAGE(CONF_FLASH_PAGE);
  int rc = flashWriteBlock(conf, &value, sizeof(value));
  if (rc == 0)
    return true;
  else if (rc == 1)
    Serial.println("Error - the flash page is reserved");
  else if (rc == 2)
    Serial.println("Error - the flash page is used by the sketch");
  return false;
}

boolean eraseConf() {
  conf = (flash_conf_t*)ADDRESS_OF_PAGE(CONF_FLASH_PAGE);
  int rc = flashPageErase(PAGE_FROM_ADDRESS(conf));
  if (rc == 0)
    return true;
  else if (rc == 1)
    Serial.println("Error - the flash page is reserved");
  else if (rc == 2)
    Serial.println("Error - the flash page is used by the sketch");
  return false;
}
