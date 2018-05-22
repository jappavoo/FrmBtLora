//Created by Tusher Chakraborty on November 03, 2017.
#include "FarmBeats.h"

unsigned int _sleepTime = 900; 
byte _nodeID = 19;
boolean _startSaving = false;
boolean _batSaving = false;
int _batValue = 0;
byte _count = 0; 
boolean _sleepTimeAdjustDone = false;


void setup() {
  Serial.begin(9600);
  initPin();
  delay(5000);
}

void loop() {
   Serial.println("+");
  _batValue = vccValue();
  if(!_batSaving)
    {
      if(_batValue < 3250 && !_sleepTimeAdjustDone)
	{
	  _batSaving = true;
	  _sleepTime = 12*_sleepTime;
	  _sleepTimeAdjustDone = true;
	}
    }
  checkNHandleBTIncoming();

  // JA: SOMETHING SEEMS WRONG sleepTime is an int eg. 900 count is a byte
  // startSaving seems only to be triggered by a bluetooth start message
  if((_count == 0 || _count == _sleepTime) && _startSaving)
    {
      SBST();
      senseNsave();
      _count = 0;
    }

  
  if(_startSaving)
    {
      _count++;
    }
  else
    {
      _count = 0;
    }
#ifdef WITH_LORA
  handleLora();
#endif
  goSleep();
}


