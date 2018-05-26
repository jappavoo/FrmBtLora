/*
  FarmBeats.h - An Arduino library for FarmBeats project.
  Created by Tusher Chakraborty on November 03, 2017.
*/
#ifndef FarmBeats_h
#define FarmBeats_h

#define WITH_LORA

#include <Arduino.h>
#include <inttypes.h>
#include <SPI.h>
#include <SD.h>
#include <LowPower.h>

#define CS 4 //chip select for SD card
#define LORATRIGGER 10 // Trigger for LoRa module (PNP)
#define SDTRIGGER 3 // Trigger for SD crad (NPN)
#define ADC01TRIGGER 5 // Trigger for sensor on ADC0 and ADC1 (PNP)
#define ADC23TRIGGER 6 // Trigger for sensor on ADC2 and ADC3 (PNP)
#define ADC45TRIGGER 7 // Trigger for sensor on ADC4 and ADC5 (PNP)
#define BTENABLE 2 // HIGH when in BT communication mode


extern byte _nodeID;
extern boolean _startSaving;
extern boolean _batSaving;
extern int _batValue;
extern unsigned int _sleepTime;

void initPin();
void enableLoRaOrSd(uint8_t decision); //decision = 0 for LoRa and 1 for SD 
boolean initFile(); // returns true on successful initialization, else returns false
boolean writeFile(String _fileName, String data); // returns true on successful writing, else returns false

unsigned long sendOverBTFromFile(String _fileName, unsigned long _filePos, boolean keep); //sends all data lines from the file over bluetooth. If keep = false, file is deleted
void sendOverBTInstant(); //Instantly, senses and sends data over bluetooth.
void checkNHandleBTIncoming();

void goSleep();

#ifdef WITH_LORA
bool handleLora();
String _adcSense(byte _adcPin, uint16_t &rval);
#else
String _adcSense(byte _adcPin);
#endif

String sense(boolean _sigSave);
String dhtSense();
void senseNsave();
void handleBT();


int vccValue(); //returns the VCC voltage
void SigSave(byte _pin); //saves the signature of the sensor on port _pin


int freeRam(void);
String adcTest(byte _pin);
unsigned long sketchSize(void);
void SBST();
#endif
