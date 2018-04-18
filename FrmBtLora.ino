/*
  FarmBeats LoRa Arduino Code for Indian Sensor Box
   
  Known Issues:
     1)  See FIXME in LoraModule::stop()
           I still see power on the LoRa Module after calling stop.
           Prior to starting the SPI the power tests works.
           However, after using SPI (eg. calling start()) asserting pin 16 high does
           cause R6 and the LORA_BJT to see the right value  but the VCC pin of the 
           LoRa stays high :-(

     2)  Need to optimize space, performance and power consumption
         I have only done some basic things keep footprint small.  
          eg. When DEBUG is commented out most of the debug code and data 
              overheads should be eliminated but not all. This needs to be 
              confirmed and likely improved.

     3)  No Duty cycle logic.
           See txDelay():  right now sender buffers data from an input
           stream and sends it based on a based delay and random delay
           since the last send.
           default values are based delay of 1 second with random delay
           of 2 seconds.  So next send window will be between 1 to 3 
           seconds after the last send (assuming there is data to send).

*/

#include <SPI.h>              // include libraries
#include <EEPROM.h>
#include <LoRa.h>

// uncomment next line to turn on debug (see below for how to customize
// the debug behaviour
// #define DEBUG

// set next line to true to reset EEPROM id back to null (-1)
#define EEPROM_RESET_ID false

// if eeprom id is null (-1) then we set the id the following value
//  get this value from somewhere else ;-)
//#define MYID 0x00000001
#define MYID 0x00000002

#define DUMP_RX_PACKET true
#define DUMP_TX_PACKET true


#ifndef DEBUG

#define waitForKey(...)

#else

// DEBUG SETTING
// use the following to customize debug behaviour
#define WAIT true
#define MSG  true
#define LORA_POWER_TEST 
#define LORA_DUMP_REGISTERS 
#define DUMP_EEPROM 

void waitForKey(Stream &s, String str, bool msg=MSG, bool wait=WAIT)
{
  if (msg) s.println(str);
  if (wait) {
    while (!s.available());
    while (s.available()) s.read();
  }
}

#endif

/************** CODE FOLLOWS ****************/
  
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
  const long LORA_FREQ                 = 868E6;
  const int  LORA_RANDOM_SEND_DELAY_MS = 2000;
  const int  LORAY_BASE_SEND_DELAY_MS  = 1000;

  
  class Id {
    const int EEPROM_ID_OFFSET_FROM_END=sizeof(uint32_t);
    const uint32_t NULLID = -1;
    uint32_t id_;

    unsigned int eeoffset() { return  EEPROM.length() - EEPROM_ID_OFFSET_FROM_END; }
    
    void setEEPromId(uint32_t val) {
      EEPROM.put(eeoffset(), val);
    }
    
    uint32_t getEEPromId() {
      uint32_t val;
      EEPROM.get(eeoffset(),val);
      return val;
    }

  public:
    Id() {
      if (EEPROM_RESET_ID) resetId();
      id_ = getEEPromId();
      if (isNull()) setEEPromId(MYID);
      id_ = getEEPromId();      
    }
    
    uint32_t value() { return id_; }
    
    void resetId() {
      setEEPromId(NULLID);
    }
    
    bool isNull() { return id_ == NULLID; }
    
#ifdef DUMP_EEPROM
    static void dumpEEProm(Stream &s) {
      unsigned int len=EEPROM.length();
      s.println("Id::dumpEEProm(): len=" + String(len));
      for (unsigned int i=0; i<len; i+=16) {
	s.print(String(i,HEX) + ":\t");
	for (int j=0; j<16; j++) {
	  s.print(EEPROM.read(i+j),HEX);
	  s.print(" ");
	}
	s.println();
      }
    }
#endif
  };
  
  class LoraModule {
  private:
    const long     freq_            = LORA_FREQ;
    const int      pwrDisable_      = LORA_R_PWR_DISABLE;
    const int      reset_           = LORA_L_NRESET;
    const int      ss_              = LORA_L_SS;
    const int      dio0_            = LORA_R_DI00;
    const int      baseSendDelay_   = LORAY_BASE_SEND_DELAY_MS;
    const int      randomSendDelay_ = LORA_RANDOM_SEND_DELAY_MS;

    
    class StreamBuffer {
#define INPUT_STREAM_BUFFER_SIZE 20
      const int len_ = INPUT_STREAM_BUFFER_SIZE;
      byte buf_[INPUT_STREAM_BUFFER_SIZE];      // can't use const member yet :-(
      int  end_;
    public:
      StreamBuffer() : end_(0) {}
      
      void reset()     { end_=0; }      
      int  spaceLeft() { return len_ - end_; }
      int  dataLen()   { return end_; }
      bool isFull()    { return end_ == len_; }
      bool isEmpty()   { return end_ == 0; }

      // if there is data on the stream copies into buffer space
      // aviable.  May leave data on stream if not enough space
      int buffer(Stream &s) {
        int copy=0;
	int len = s.available();
	int space = spaceLeft();
	
	if (len && space) {
	  if (space >= len) copy = len; else copy = space;
	  s.readBytes(&buf_[end_], copy);
	  end_ += copy;
	}
	return copy;
      }
      byte *data() { return buf_; }
      
    } streamBuf_;
    
    String    myIdStr_;
    uint32_t  myId_;
    unsigned long lastTx_;
    unsigned long nextTxAfter_;
    int txCnt_;
    int rxCnt_;
    unsigned char spcrIntialValue_;
    bool pwr_;

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

    // adds header info : just using ascii format for the moment
    // format: <id>,<len>,<data>
    int copyDataToStream(Stream &s, byte *data, int dataLen) {
      int cnt;
      
      cnt  = s.print(myIdStr_);
      cnt += s.print(",");
      cnt += s.print(dataLen);
      cnt += s.print(",");
      cnt += s.write(data, dataLen);
      
      return cnt;
    }

    // FIXME: JA harded coded delay eventually should be adapted to
    // correct duty cycle logic
    unsigned long txDelay() {
      return baseSendDelay_ + random(randomSendDelay_);
    }
    
  public:
    
    LoraModule() {}

    void setId(uint32_t id) { myId_ = id; myIdStr_ = String(id,HEX); }
    uint32_t getId() { return myId_; }
    
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
      nextTxAfter_ = txDelay();
      
      return LoRa.begin(freq_);
    }
    
    void sendStreamBuf(Stream &ds, bool dumpPacket=DUMP_TX_PACKET) {
      int dataLen = streamBuf_.dataLen();
      byte *data  = streamBuf_.data();
      
      if (dataLen) {
	// we have data to send
	int sentCnt;
	LoRa.beginPacket();
	sentCnt = copyDataToStream(LoRa, data, dataLen);
	LoRa.endPacket();
	
	txCnt_      += sentCnt;
	lastTx_      = millis();
	nextTxAfter_ = txDelay();
	
	if (dumpPacket) {
	  ds.print(myIdStr_ + ":>\n\t");
	  copyDataToStream(ds, data, dataLen);
	  ds.println("\tsentCnt:" + String(sentCnt) + " txCnt_: " + String(txCnt_));
	}
	streamBuf_.reset();
      }
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
    
    void loopAction(Stream &s) {
      // local input stream processing
      streamBuf_.buffer(s);
	
      // send stream data when and as necessary
      if ((millis() - lastTx_) > nextTxAfter_) {
        // time to send any buffered data
        sendStreamBuf(s);
      }

      // poll and process any data on LoRa
      onReceive(LoRa.parsePacket(),s);
    }

    // FIXME: JA: SOMETHING IS WRONG.
    //            I still see power on the LoRa Module after calling stop.
    //            Prior to starting the SPI the power tests works.
    //            However, after using SPI asserting pin 16 high does
    //            does cause R6 and the LORA_BJT to see the right value
    //            but the VCC pin of the LoRa stays high :-(
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


#ifdef LORA_POWER_TEST
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
#endif
 };
}


FarmBeats::Id id;
FarmBeats::LoraModule lm;

void setup() {
  Serial.begin(9600); // initialize serial
  while (!Serial);

  waitForKey(Serial, "setup(): BEGIN: Send key to continue");

#ifdef DUMP_EEPROM
  FarmBeats::Id::dumpEEProm(Serial);
  waitForKey(Serial, "EEProm dump done");
 #endif

  lm.setId(id.value());
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

