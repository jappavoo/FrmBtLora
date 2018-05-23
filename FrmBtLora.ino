// comment next line if you want to
// play with indesign packet processing
// rather than using it as lora sniffer
#define SERIAL_INPUT_PROCESSING
#define LORA_INFO
//#define DUMP_EEPROM
#include "FarmBeatsLora.h"

namespace FarmBeats {
    // DUMMY Inteface to sensors
  class Sensors {
  public:
    void setup() {}
    uint32_t getTS()   {
      unsigned long ts=millis();
      Serial.println("  sensors.getTS() = " + String(ts, HEX));
      return ts;
    }
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
  Serial.println("  sizeof(lm)=" + String(sizeof(lm)));
#ifdef INDESIGN_PACKET_PROCESSING  
  Serial.println("  sizeof(lm.theSample_)=" + String(sizeof(lm.theSample_)));
  Serial.println("  sizeof(lm.theSample_.data.raw)=" + String(sizeof(lm.theSample_.data.raw)));
  Serial.println("  sizeof(lm.theSample_.data.values)=" + String(sizeof(lm.theSample_.data.values)));
#endif
  Serial.println("  sizeof(LoRa)=" + String(sizeof(LoRa)));
  Serial.println("  sizeof(id)=" + String(sizeof(id)));
  
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
#ifdef INDESIGN_PACKET_PROCESSING
  Serial.println("Send a byte on serial line to trigger send of a packet");
#endif
}

void loop() {
#ifdef INDESIGN_PACKET_PROCESSING
  while (!Serial.available());
  while(Serial.available()) {
    Serial.read();
  }
  lm.sendSample(sensors.getTS(),
		sensors.getADC0(), sensors.getADC1(), sensors.getADC2(),
		sensors.getADC3(), sensors.getADC4(), sensors.getADC5());
#else
  lm.loopAction();
#endif  
}

