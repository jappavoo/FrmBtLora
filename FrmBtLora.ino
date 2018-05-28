// comment next line if you want to
// play with indesign packet processing
// rather than using it as lora sniffer
//#define SERIAL_INPUT_PROCESSING
#define TEST_SEND_ACK
#define DUMP_TX_PACKET
#define DUMP_RX_PACKET
#define LORA_INFO
#define IND_ACK_TIMEOUT_MS 10000
#define AUTO_ACK
//#define DUMP_EEPROM
#include "FarmBeatsLora.h"

namespace FarmBeats {
    // DUMMY Inteface to sensors
  class Sensors {
  public:
    void setup() {}
    uint32_t getTS()   {
      unsigned long ts=millis();
      Serial.println("  sensors.getTS() = " +
		     String(ts, DEC) + "(0x" +
		     String(ts, HEX) + ")");
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
  Serial.println("  sizeof(lm.theSample_.data.raw)=" +
		 String(sizeof(lm.theSample_.data.raw)));
  Serial.println("  sizeof(lm.theSample_.data.values)=" +
		 String(sizeof(lm.theSample_.data.values)));
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

#ifdef INDESIGN_PACKET_PROCESSING
      Serial.println("'s' to send of a Data Sample packet\r\n"
#ifdef TEST_SEND_ACK
		 "'a <id> <ts>' to send Ack Packet\r\n"
		 "'A' toggle Auto Ack mode\r\n"
#endif
		 );
#endif
  waitForKey("setup(): END: Send key to end");
}

bool doAck=false;
void loop() {
#ifdef INDESIGN_PACKET_PROCESSING

  if (Serial.available()) {
    switch (Serial.read()) {
    case 's':
      lm.sendSample(sensors.getTS(),
		    sensors.getADC0(), sensors.getADC1(), sensors.getADC2(),
		    sensors.getADC3(), sensors.getADC4(), sensors.getADC5());
      if (lm.isSampleAcked()) Serial.println("   * ACK: GOOD *   ");
      else Serial.println("   * ACK: NONE OR BAD*   ");
      
      break;
#ifdef TEST_SEND_ACK
    case 'a':
      uint32_t id;
      uint32_t ts;

      delay(100);
    
      Serial.read();
      ((char *)&id)[3] = (char)Serial.read();
      ((char *)&id)[2] = (char)Serial.read();
      ((char *)&id)[1] = (char)Serial.read();
      ((char *)&id)[0] = (char)Serial.read();
      ts = Serial.parseInt();
      Serial.println("Sending ack with id=0x" + String(id,HEX) + " and ts=0x" +
		     String(ts,HEX)+ "\r\n");
      lm.testSendAck(id, ts);
      break;
    case 'A':
      if (doAck) doAck=false; else doAck=true;
      Serial.println("AutoAck: " + String(doAck));
      break;
      
#endif
    default:
      while (Serial.available()) Serial.read();
    }
    Serial.println("'s' to send of a Data Sample packet\r\n"
#ifdef TEST_SEND_ACK
		 "'a <id> <ts>' to send Ack Packet\r\n"
#endif
		 );
  }
#ifdef DUMP_RX_PACKET
  lm.sniff(doAck);
#endif  
#else
  lm.loopAction();
#endif  
}

