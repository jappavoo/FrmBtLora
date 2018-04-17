/*
/*
  FarmBeats LoRa Arduino Code for Indian Sensor Box
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>

#define MYID 0x00000001
//#define MYID 0x00000002

#define DUMP_RX_PACKET true
#define DUMP_TX_PACKET true

//#define DEBUG 1

#ifndef DEBUG
#define waitForKey(...)
#else 
// DEBUG SETTING
#define WAIT false
#define MSG  true
#define LORA_POWER_TEST 1
#define LORA_DUMP_REGISTERS 1

void waitForKey(Stream &s, String str, bool msg=MSG, bool wait=WAIT)
{
  if (msg) s.println(str);
  if (wait) {
    while (!s.available());
    while (s.available()) s.read();
  }
}
#endif

  
namespace FarmBeats {

  
  enum ATMEGA328_MCU_TO_ARDUINO_PIN_MAPPINGS {
    // These names map the Ardunio functional IDE/toolchain pin names
    // to the ATMEGA328 MCU 28 pin DIP package chip pins
    // see: https://www.arduino.cc/en/Hacking/PinMapping168    
    // PIN1_PC6 = RESET,
    MCU_PIN2_PD0 = 0,
    MCU_PIN3_PD1 = 1,
    MCU_PIN4_PD2 = 2,
    MCU_PIN5_PD3 = 3,
    MCU_PIN6_PD4 = 4,
    // MCU_PIN7_VCC = VCC,
    // MCU_PIN8_GND = GND,
    // MCU_PIN9_PB6 = CRYSTAL,
    // MCU_PIN10_PB7 = CRYSTAL,
    MCU_PIN11_PD5 = 5,
    MCU_PIN12_PD6 = 6,
    MCU_PIN13_PD7 = 7,
    MCU_PIN14_PB0 = 8,
    MCU_PIN15_PB1 = 9,
    MCU_PIN16_PB2 = 10,
    MCU_PIN17_PB3 = 11,
    MCU_PIN18_PB4 = 12,
    MCU_PIN19_PB5 = 13,
    // MCU_PIN20_AVCC = VCC,
    // MCU_PIN21_AREF = AREF,
    // MCU_PIN22_GND = GND,
    MCU_PIN23_PC0 = A0,
    MCU_PIN24_PC1 = A1,
    MCU_PIN25_PC2 = A2,
    MCU_PIN26_PC3 = A3,
    MCU_PIN27_PC4 = A4,
    MCU_PIN28_PC5 = A5    
  };
  
  enum ATMEGA328_PIN_ASSIGNMENTS {
    // NOT WE MUST USE ATMEGA328 SPECIFIC
    // PIN NAMES NOT THE STANDARD ARDUINO BOARD PIN NAMES
    // SEE ABOVE MAPPING
    // For completeness the pins reserved for
    // SPI communication are here too
    SPI_MISI           = MCU_PIN17_PB3,
    SPI_MISO           = MCU_PIN18_PB4,
    SPI_SCK            = MCU_PIN19_PB5,
    
    LORA_L_NRESET      = MCU_PIN14_PB0,
    LORA_L_SS          = MCU_PIN6_PD4,
    LORA_R_DI00        = MCU_PIN15_PB1,
    LORA_R_PWR_DISABLE = MCU_PIN16_PB2
    // LORA_L_DI05=NOT_CONNECTED,
    // LORA_R_DI02=NOT_CONNECTED,
    // LORA_R_DI01=NOT_CONNECTED,
    // LORA_R_DI04=NOT_CONNECTED,
    // LORA_R_DI03=NOT_CONNECTED
  };
  // based on this document
  // https://www.thethingsnetwork.org/docs/lorawan/frequencies-by-country.html
  // default frequency to use in india is IN865-867
  const long LORA_FREQ = 868E6;
  
  class LoraModule {
  private:
    const int32_t  myId_         = MYID;
    const String   myIdStr_      = String(MYID);
    const long     freq_         = LORA_FREQ;
    const int      pwrDisable_   = LORA_R_PWR_DISABLE;
    const int      reset_        = LORA_L_NRESET;
    const int      ss_           = LORA_L_SS;
    const int      dio0_         = LORA_R_DI00;

    bool pwr_;
    unsigned char spcrIntialValue_;
    unsigned long lastTx_;
    unsigned long nextTxAfter_;
    int txCnt_;
    int rxCnt_;
    
    void sleep() { delay(100); }
    
    void disablePower() {
      // LORA Power is negative enable logic on MCU pin
      digitalWrite(pwrDisable_, HIGH);
      pwr_ = false;
    }
    
    void enablePower() {
      // LORA Power is negative enable logic on MCU pin
      digitalWrite(pwrDisable_, LOW);
      pwr_ = true;
    }
  public:
    LoraModule() {}

    void info(Stream &s) {
      s.println("LoraModule():");
      s.println("\tmyId_: " + String(myId_) + " myIdStr_: " + myIdStr_);
      s.print("\tPINS: pwrDisable_="); s.print(pwrDisable_, DEC);
      s.print(" reset_="); s.print(reset_, DEC);
      s.print(" ss_="); s.print(ss_, DEC);
      s.print(" dio0_="); s.print(dio0_, DEC);
      s.print("\n\tCONSTANTS: freq_=");
      s.println(freq_, DEC);
    }
    
    int start() {
      SPI.begin();
      pinMode(pwrDisable_, OUTPUT);
      enablePower();
      sleep(); // give module time to warm up???
      lastTx_ = 0;
      nextTxAfter_ = 2000;
      return LoRa.begin(freq_);
    }

    void onReceive(int packetSize, Stream &s, bool dumpPacket=DUMP_RX_PACKET) {
      if (packetSize == 0) return;
      String inStr = "";

      while (LoRa.available()) {
	inStr += (char)LoRa.read();
      }
      rxCnt_ +=  inStr.length();
      if (dumpPacket) {
        s.print(myIdStr_ + ":<\n\t");
	s.println(inStr);
        s.println("\tRSSI: " + String(LoRa.packetRssi()));
        s.println("\tSnr: " + String(LoRa.packetSnr()));
        s.println("\trxCnt_: " + String(rxCnt_));
      }
    }
    
    void loopAction(Stream &s, bool dumpPacket=DUMP_TX_PACKET) {
      int cnt=0;
      String outStr ="id:" + myIdStr_ + " : Hello: "; 
      if ((millis() - lastTx_) > nextTxAfter_) {
	// send packet	
	LoRa.beginPacket();
	cnt += LoRa.print(outStr);
	cnt += LoRa.print(txCnt_);
	LoRa.endPacket();
	txCnt_ += cnt;
	lastTx_ = millis();
	nextTxAfter_  = random(2000) + 1000;
	
	if (dumpPacket) {
	  s.print(myIdStr_ + ":>\n\t");
	  s.println(outStr);
	  s.println("\ttxCnt_: " + String(txCnt_));
	}
      }
      onReceive(LoRa.parsePacket(),s);
    }
    
    void stop() {
      // Initializes the SPI bus by setting SCK, MOSI, and SS to outputs,
      // pulling SCK and MOSI low, and SS high.
      LoRa.end();
      
      SPI.end();

      SPCR = spcrIntialValue_;
      pinMode(SCK, INPUT);
      pinMode(MOSI, INPUT);
      pinMode(MISO, INPUT);
      
      pinMode(pwrDisable_, OUTPUT);
      disablePower();
    }
    
    void status(Stream &out) {
      LoRa.dumpRegisters(out);
    }

    void setup() {
      spcrIntialValue_ = SPCR;
      txCnt_ = 0;
      rxCnt_ = 0;
      pinMode(pwrDisable_, OUTPUT);
      disablePower();
      LoRa.setPins(ss_, reset_, dio0_); 
    }

    bool isOn() { return pwr_; }

    void testPower(Stream &s) {
      // test power disable
      s.println("testPower(): BEGIN");
      pinMode(pwrDisable_, OUTPUT);
      
      disablePower();

      s.print("isOn(): ");
      s.println(isOn());

      waitForKey(s, "Send Key to Turn ON");

      enablePower();
      
      s.print("isOn(): ");
      s.println(isOn());
      s.println("\tPOWER ON: you should be able to measure 3.3v on LORA_PWR"
		" pin 4 of LORA_R");

      
      waitForKey(s, "Send key to Turn OFF");

      disablePower();

      
      s.print("isOn(): ");
      s.println(isOn());
      s.println("\t POWER OFF: you should be able to measure LOW v on LORA_PWR"
		" pin 4 of LORA_R");
      
      waitForKey(s, "Send key to end testPower");
      
      s.println("testPower(): END");
    }
 };
}


FarmBeats::LoraModule lm;

void setup() {
  Serial.begin(9600); // initialize serial
  while (!Serial);

  waitForKey(Serial, "setup(): BEGIN: Send key to continue");

  lm.info(Serial);
  
#ifdef LORA_POWER_TEST
  lm.testPower(Serial);
  waitForKey(Serial, "lm.testPower() done. Send key to continue");
#endif
  
  lm.setup();

  waitForKey(Serial, "lm.setup() done. Send key to continue");

  
  if (!lm.start()) {
    Serial.println("ERROR: Failed to start Lora Module on.\n"
		   "    Check your connections.");
    while (true);                   
  }
  
  waitForKey(Serial, "lm.start() done. Send key to continue");

#ifdef LORA_DUMP_REGISTERS
  Serial.println("LoRa Dump Registers");
  Serial.println("lm: STATUS:");
  lm.status(Serial);
  waitForKey(Serial, "lm.status() done. Send key to continue");
#endif
  
  waitForKey(Serial, "setup(): END: Send key to end");
}

void loop() {
  lm.loopAction(Serial);
}

