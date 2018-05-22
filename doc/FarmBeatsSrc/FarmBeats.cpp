/*
  FarmBeats.cpp - An Arduino library for FarmBeats project.
  Created by Tusher Chakraborty on November 03, 2017.
*/

//#define WITH_LORA

#include<Arduino.h>
#include<math.h>
#include <SPI.h>
#include <SD.h>
#include <LowPower.h>
#include"FarmBeats.h"
#ifdef WITH_LORA
#include "FarmBeatsLora.h"
#endif


#include <pins_arduino.h>
extern "C" {
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
}

String _datalogFile  = "data.txt";
String _sbstlogFile  = "sbst.txt";

boolean _successWrite = true;
boolean _wrongPress = false;
boolean _wrongPressFlag = false;

unsigned long _datafilePos = 0;
unsigned long _sigfilePos = 0;
unsigned long _sbstfilePos = 0;
unsigned long _dataCounter = 0;

unsigned long _senseLocalTime = 0;

#ifdef WITH_LORA
FarmBeats::Id         id;
FarmBeats::LoraModule lm;
#endif

void initPin()
{
	pinMode(LORATRIGGER, OUTPUT);
	digitalWrite(SDTRIGGER, HIGH);
	pinMode(SDTRIGGER, OUTPUT);
	digitalWrite(SDTRIGGER, LOW);
	pinMode(ADC01TRIGGER, OUTPUT);
	digitalWrite(ADC01TRIGGER, HIGH);
	pinMode(ADC23TRIGGER, OUTPUT);
	digitalWrite(ADC23TRIGGER, HIGH);
	pinMode(ADC45TRIGGER, OUTPUT);
	digitalWrite(ADC45TRIGGER, HIGH);
	pinMode(BTENABLE, INPUT);
	
	attachInterrupt(digitalPinToInterrupt(BTENABLE), handleBT, FALLING);

#ifdef WITH_LORA
	lm.setId(id.value());
	lm.setup();
	lm.theSample_.setValues(); // reset values to zero
#endif
}

void handleBT()
{
    // Just a handler for the pin interrupt.
    detachInterrupt(digitalPinToInterrupt(BTENABLE));
    if(_wrongPress)
	{
		_wrongPress = false;
	}
	attachInterrupt(digitalPinToInterrupt(BTENABLE), handleBT, FALLING);
}

#ifdef WITH_LORA
void handleLora()
{
  // JA KLUDGE TEST: set 
  _senseLocalTime = millis();
  
    if (_senseLocalTime != lm.theSample_.getTimeStamp() ) {
      lm.theSample_.setTimeStamp(_senseLocalTime);
      lm.start();
	  
      lm.loopAction();  
//    lm.sendSample(millis(),
//		    0x0CCC, 0x0100, 0x0456, 0xCAFE, 0xFEED, 0xFACE);
      lm.stop();
    }
}
#endif

void enableLoRaOrSd(uint8_t decision)
{
	if(decision == 0)
	{
		digitalWrite(SDTRIGGER, LOW);
		digitalWrite(LORATRIGGER, LOW);
		delay(500);
	}
	
	else if (decision == 1)
	{
		digitalWrite(LORATRIGGER, HIGH);
		digitalWrite(SDTRIGGER, HIGH);
		delay(500);
	}
	else if (decision == 2)
	{
		digitalWrite(SDTRIGGER, LOW);
	}
	else if (decision == 3)
	{
		digitalWrite(SDTRIGGER, HIGH);
	}
}

boolean initFile()
{
	enableLoRaOrSd(1);
	if (!SD.begin(CS)) {
 //  Serial.println("initialization failed!");
    return false;
  }
 // Serial.println("initialization done.");
  return true;
}

boolean writeFile(String _fileName, String data)
{
	if (initFile())
	{
		File file = SD.open(_fileName, FILE_WRITE);
		if(file)
		{
		  file.println(data);
		  file.close();
		  enableLoRaOrSd(2);
		  return true;
		}
		enableLoRaOrSd(2);
		return false;
	}
	enableLoRaOrSd(2);
	return false;
}

/*String readNextLineFromFile(String _fileName, unsigned long _filePos)
{
	if (initFile())
	{
		File file = SD.open(_fileName);
		if(file.available())
		{
			file.seek(_filePos); 
			String buffer = file.readStringUntil('\n');
			_filePos = file.position();
			file.close();
			return buffer;
		}
		return "EOF";
	}
	return "Error";
}*/

unsigned long sendOverBTFromFile(String _fileName, unsigned long _filePos, boolean keep)
{
	//Send hi
	Serial.print(F("BOX ID: "));
	Serial.println(_nodeID);
	String _dataToSend = "";
	if (initFile())
	{
		File file = SD.open(_fileName);
		while(file.available()>0)
		{
			file.seek(_filePos); 
			_dataToSend = file.readStringUntil('\n');
			_filePos = file.position();
			Serial.println(_dataToSend);
			delayMicroseconds(20);
		}
		file.close();
		Serial.println(F("EOF"));
	}
	else
	{
		Serial.println(F("Error"));
	}
	
	//Notify battery status
	if(!_batSaving)
	{
		Serial.print(F("Battery health is Good.  "));
		Serial.print(_batValue);
		Serial.println(F("mV"));
	}
	else
	{
			Serial.print(F("LOW Battery Alert!!!  "));
			Serial.print(_batValue);
			Serial.println(F("mV"));
	}
	//Any failure in file writing
	if(_successWrite == false)
	{
		Serial.println(F("Encountered a problem in writing file."));
	}
	//Any failure in turning the switch off_type
	if(_wrongPressFlag)
	{
		Serial.println(F("Last time you forgot to turn Bluetooth off."));
		_wrongPressFlag = false;
	}
	
	Serial.flush();
	
	if (keep == false)
	{
		SD.remove(_fileName);
		_filePos = 0;
	}
	enableLoRaOrSd(2);
	return _filePos;
}

void sendOverBTInstant()
{
	//Send hi
	Serial.print(F("BOX ID: "));
	Serial.println(_nodeID);
	//sense data
	String outgoingData = sense(false);
	
	//send data 
	Serial.println(outgoingData);
	
	//Notify battery status
	if(!_batSaving)
	{
		Serial.print(F("Battery health is Good.  "));
		Serial.print(_batValue);
		Serial.println(F("mV"));
	}
	else
	{
			Serial.print(F("LOW Battery Alert!!!  "));
			Serial.print(_batValue);
			Serial.println(F("mV"));
	}
	//Any failure in file writing
	if(_successWrite == false)
	{
		Serial.println(F("Encountered a problem in writing file."));
	}
	//Any failure in turning the switch off_type
	if(_wrongPressFlag)
	{
		Serial.println(F("Last time you forgot to turn Bluetooth off."));
		_wrongPressFlag = false;
	}
	
	Serial.flush();
}

void senseNsave()
{
	//increase counter and add
	String outgoingData = String(_dataCounter);
	outgoingData+=",";
	//add time
	// JA:  Turned this into a global static to allow detection of
	//      new values in other parts of the code
	_senseLocalTime = millis()/1000 + _dataCounter*_sleepTime*8;
	outgoingData+=String(_senseLocalTime);
	outgoingData+=",";
	//sense data
	outgoingData += sense(true);
	//save data
	_successWrite = writeFile(_datalogFile, outgoingData);
	
	if(_successWrite)
	{
		_dataCounter++;
	}
}

void goSleep()
{
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

void checkNHandleBTIncoming()
{
	unsigned long _startLocalTime = millis()/1000;
	while((digitalRead(BTENABLE)==HIGH) && !_wrongPress)
	{
		
		if(Serial.available()>0)
		{
			String _in = Serial.readStringUntil('.');
			if(_in == "sense")
			{
				sendOverBTInstant();
				_startLocalTime = millis()/1000;
			}
			
			else if(_in == "data")
			{
				_datafilePos = sendOverBTFromFile(_datalogFile, _datafilePos, true);
				_startLocalTime = millis()/1000;
				
			}
			else if(_in == "datadel")
			{
				_datafilePos = sendOverBTFromFile(_datalogFile, _datafilePos, false);
				_startLocalTime = millis()/1000;
			}
			
			else if(_in == "start")
			{
				_startSaving = true;
				Serial.println(F("Saving in SD card enabled."));
				_startLocalTime = millis()/1000;
			}
			else if(_in == "stop")
			{
				_startSaving = false;
				Serial.println(F("Saving in SD card disabled."));
				_startLocalTime = millis()/1000;
			}
			else if(_in == "sampling")
			{
				Serial.println(F("Enter new sampling rate."));
				while(1)
				{	
					if(Serial.available()>0)
					{
						String _inSample = Serial.readStringUntil('.');
						_sleepTime = _inSample.toInt();
						Serial.println(F("Sampling rate has been changed."));
						break;
					}
				}
				_startLocalTime = millis()/1000;
			}
			else if(_in == "sig")
			{
				_sigfilePos = sendOverBTFromFile("sig.txt", _sigfilePos, true);
				_startLocalTime = millis()/1000;
			}
			else if(_in == "sigdel")
			{
				_sigfilePos = sendOverBTFromFile("sig.txt", _sigfilePos, false);
				_startLocalTime = millis()/1000;
			}
			else if(_in == "sbst")
			{
				_sbstfilePos = sendOverBTFromFile(_sbstlogFile, _sbstfilePos, true);
				_startLocalTime = millis()/1000;
				
			}
			else if(_in == "sbstdel")
			{
				_sbstfilePos = sendOverBTFromFile(_sbstlogFile, _sbstfilePos, false);
				_startLocalTime = millis()/1000;
			}
		}
			unsigned long _deltaLocalTime = millis()/1000;
			if(_deltaLocalTime - _startLocalTime > 180)
			{
				_wrongPress = true;
				_wrongPressFlag = true;
				break;
			}
	}
}

inline void triggerSensor(byte pin, boolean mode)
{
  if(mode == true)
  {
    digitalWrite(pin, LOW);
    delay(500);
  }
  if(mode == false)
  {
     digitalWrite(pin, HIGH);
  }
}

String sense(boolean _sigSave)
{
	String dataString ="";
	
	//soil moisture sense
	triggerSensor(ADC01TRIGGER, true);
	// JA: KLUDGE OPTIMIZATION :  We know endianess matches
	// so we by pass functional interface
	//  Add static assert to verify this fact at compile time
#ifdef WITH_LORA
	dataString+= _adcSense(A0, lm.theSample_.data.values.Data.values.ADC0);
#else
	dataString+= _adcSense(A0);
#endif
	dataString+=",";
	triggerSensor(ADC01TRIGGER, false);
	if(_sigSave)
	{
		SigSave(A0);
	}
	//soil temperature sense
	triggerSensor(ADC23TRIGGER, true);
#ifdef WITH_LORA
	dataString+= _adcSense(A2, lm.theSample_.data.values.Data.values.ADC1);
#else
	dataString+= _adcSense(A2);
#endif
	triggerSensor(ADC23TRIGGER, false);
	if(_sigSave)
	{
		SigSave(A2);
	}
	
	return dataString;
}

#ifdef WITH_LORA
String _adcSense(byte _adcPin, uint16_t &rval)
#else
  String _adcSense(byte _adcPin)
#endif
{
	double val = 0;
	
	pinMode(_adcPin, INPUT);
	
	for (byte i=0; i<100; i++)
	{
		val = val + analogRead(_adcPin);
	}
	
	val = val/100;
#ifdef WITH_LORA	
	rval=(uint16_t)val;
#endif
	return String(val);
}

int vccValue()
{
  analogRead(A6);
  bitSet(ADMUX, 3);
  delayMicroseconds(250);
  bitSet(ADCSRA, ADSC);
  while(bit_is_set(ADCSRA, ADSC));
  word x = ADC;
  return x? (1100L*1023)/x: -1;
}

void SigSave(byte _pin)
{
	double dataSet[100];
	for(byte j = 0; j<100; j++)
	  {
		  double val = 0;
		  for(byte i =0; i<100; i++)
		  {
			val = val + analogRead(_pin);
		  }
		val = val/100;
		dataSet[j] = val;
		delay(2);
	  }
	if (initFile())
	{
		File file = SD.open("sig.txt", FILE_WRITE);
		if(file)
		{
			//file.print(".........");
			file.print(_dataCounter);
			file.print(F(","));
			file.println(_pin);
			//file.println(".........");
			for(byte i=0; i<100; i++)
			{
				file.println(dataSet[i]);
			}
		}
		file.close();
	}
	enableLoRaOrSd(2);
}

String adcTest(byte _pin)
{
	String _result= "";
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, HIGH);
	_result += String(analogRead(_pin));
	_result += ",";
	digitalWrite(_pin, LOW);
	_result += String(analogRead(_pin));
	_result += ",";
	return _result;
}

int freeRam(void)
{
  extern unsigned int __heap_start;
  extern void *__brkval;

  int free_memory;
  int stack_here;

  if (__brkval == 0)
    free_memory = (int) &stack_here - (int) &__heap_start;
  else
    free_memory = (int) &stack_here - (int) __brkval;

  return (free_memory);
}

unsigned long sketchSize(void)
{
  extern int _etext;
  extern int _edata;

  return ((unsigned long)(&_etext) + ((unsigned long)(&_edata) - 256L));
}

void SBST()
{
	String _sbstResult = "";
	_sbstResult += String(_dataCounter);
	_sbstResult += ",";
	//ADC Test
	_sbstResult += adcTest(A3);
	_sbstResult += adcTest(A4);
	
	//sram status
	_sbstResult += String(freeRam());
	_sbstResult += ",";
	
	//sketch size
	_sbstResult += String(sketchSize());
	_sbstResult += ",";
	
	//battery
	_sbstResult += String(_batValue);
	
	writeFile(_sbstlogFile,_sbstResult);
}
  //////////////////////End/////////////////////////////
