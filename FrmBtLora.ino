#include "FarmBeatsLora.h"

namespace FarmBeats {
    // DUMMY Inteface to sensors
  class Sensors {
  public:
    void setup() {}
    uint32_t getTS()   { return 0xdeadbeef; }
    uint16_t getADC0() { return 0x0CCC; }
    uint16_t getADC1() { return 0x0100; }
    uint16_t getADC2() { return 0x0456; }
    uint16_t getADC3() { return 0xcafe; }
    uint16_t getADC4() { return 0xfeed; }
    uint16_t getADC5() { return 0xface; }
  };
};  


FarmBeats::Id         id;
FarmBeats::Sensors    sensors;
FarmBeats::LoraModule lm;

void setup() {
  Serial.begin(9600); // initialize serial
  while (!Serial);

  Serial.println("FrmBtLora version: " + String(VERSION));
  
  waitForKey("setup(): BEGIN: Send key to continue");

#ifdef DUMP_EEPROM
  FarmBeats::Id::dumpEEProm();
  waitForKey("EEProm dump done");
#endif

  lm.setId(id.value());

  sensors.setup();
  
#ifdef LORA_POWER_TEST
  lm.testPower();
  waitForKey("lm.testPower() done. Send key to continue");
#endif
  
  lm.setup();

#ifdef INDESIGN_PACKET_PROCESSING
  lm.theSample_.setValues(sensors.getTS(),
			  sensors.getADC0(), sensors.getADC1(), sensors.getADC2(),
			  sensors.getADC3(), sensors.getADC4(), sensors.getADC5());
#endif
  
  waitForKey("lm.setup() done. Send key to continue");

  
  if (!lm.start()) {
    Serial.println("ERROR: Failed to start Lora Module on.\r\n"
		   "    Check your connections.");
    while (true);                   
  }
  
  waitForKey("lm.start() done. Send key to continue");

#ifdef LORA_DUMP_REGISTERS
  Serial.println("LoRa Dump Registers");
  Serial.println("lm: STATUS:");
  lm.status();
  waitForKey("lm.status() done. Send key to continue");
#endif
  
  waitForKey("setup(): END: Send key to end");
}

void loop() {
   lm.loopAction();
}

