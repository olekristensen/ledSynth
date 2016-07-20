/*
  GUINO DASHBOARD TEMPLATE FOR THE ARDUINO.
 Done by Mads Hobye as a part of Instructables (AIR Program) & Medea (PhD Student).
 Licens: Creative Commons — Attribution-ShareAlike

 It should be used with the GUINO Dashboard app.

 More info can be found here: www.hobye.dk

 # This is the Guino Protocol Library should only be edited if you know what you are doing.
 */

#include <EasyTransferRfduinoBLE.h>

#define cmd_executed -1
#define cmd_init 0
#define cmd_ping 1
#define cmd_setValue 2
#define cmd_setMin 3
#define cmd_setMax 4
#define cmd_saveToBoard 9
#define cmd_disconnect 10
#define cmd_init_done 11


boolean guidino_initialized = false;

//create object
EasyTransferRfduinoBLE ET;

struct SEND_DATA_STRUCTURE
{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t cmd;
  int16_t item;
  int16_t value;
};

// Find a way to dynamically allocate memory
int guino_maxGUIItems = 100;
int guino_item_counter = 0;
void *guino_item_values[100];
int gTmpInt = 0; // temporary int for items without a variable
boolean internalInit = true; // boolean to initialize before connecting to serial

// COMMAND STRUCTURE

//give a name to the group of data
SEND_DATA_STRUCTURE cmd_data;

boolean guino_update()
{
  bool hadCommands = false;
  while (tq.size() > 0) {
    if (ET.receiveData())
    {
      /*
            Serial.print("CMD :");
            Serial.println(guino_data.cmd);
            Serial.print("ITEM:");
            Serial.println(guino_data.item);
            Serial.print("VAL :");
            Serial.println(guino_data.value);
      //*/
      switch (cmd_data.cmd)
      {
        case cmd_init:
          guino_item_counter = 0;
          gInit();
          guidino_initialized = true;
          gSendCommand(cmd_init_done, 0, 0);
          break;
        case cmd_setValue:
          *(int*)guino_item_values[cmd_data.item] = cmd_data.value;
          cmd_data.cmd = cmd_executed;
          gItemUpdated(cmd_data.item);
          break;
        case cmd_disconnect:

          break;
        case cmd_saveToBoard:
          /*      {

                    saveConf();
                    gInitEEprom();
                    for (int i =0; i < guino_item_counter;i++)
                    {
                        EEPROMWriteInt(i*2+2, *guino_item_values[i]);
                    }
                  }
          */
          break;
      }
      hadCommands = true;
    }
  }
  return hadCommands;
}

void gBegin()
{

  // Sets all pointers to a temporary value just to make sure no random memory pointers.
  for (int i = 0; i < guino_maxGUIItems; i++)
  {
    guino_item_values[i] = &gTmpInt;
  }
  guino_item_counter = 0;
  internalInit = true;
  guidino_initialized = false;
  gInit(); // this one needs to run twice only way to work without serial connection.
  guino_item_counter = 0;
  internalInit = false;
  ET.begin(details(cmd_data), &RFduinoBLE, &tq);
  gSendCommand(cmd_executed, 0, 0);
  gSendCommand(cmd_executed, 0, 0);
  gSendCommand(cmd_executed, 0, 0);
  gSendCommand(cmd_ping, 0, 0);

}

int gBindInt(int _min, int _max, int id, int * _variable) {
  if (guino_maxGUIItems > id)
  {
    guino_item_values[id] = _variable;
    guino_item_counter = max(guino_item_counter, id);
    //gGetSavedValue(guino_item_counter, _variable);
    gSendCommand(cmd_setMax, id, _max);
    gSendCommand(cmd_setMin, id, _min);
    gUpdateValue(id);
    return id;
  }
  return -1;
}

void gUpdateValue(int _item)
{
  gSendCommand(cmd_setValue, _item, *(int*)guino_item_values[_item]);
}

void gUpdateConstValue(int _item, int _value)
{
  gSendCommand(cmd_setValue, _item, _value);
}

void gUpdateValue(int * _variable)
{
  int current_id = -1;
  for (int i = 0; i <= guino_item_counter; i++)
  {

    if (guino_item_values[i] == _variable)
    {
      current_id = i;
      gUpdateValue(current_id);
    }
  }

}

void gSendCommand(int16_t _cmd, int16_t _item, int _value)
{
  if (!internalInit && (guidino_initialized || cmd_executed || _cmd == cmd_ping)  )
  {
    cmd_data.cmd = _cmd;
    cmd_data.item = _item;
    cmd_data.value = _value;
    ET.sendData();
  }

}


