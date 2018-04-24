/*
  FarmBeats LoRa Arduino Code for Indian Sensor Box

  This code is hard coded around the Semtech sx1276 which is the radio module on the 
  LORA 868Mhz 8X1276 RF MODULE from ROBOKITS INDIA RKI-2726 

  NOTE THE MDOT uses sx1272 which is different from the sx1276 that the Indian Farm 
      Beats sensor box uses 

  Known Issues:
    -1)  Transmit power default set to 6 given testing in short ranges maybe more stable
         Expore this and set value for field as needed.

     0)  I have included a submodule checkout of the official semtech sx1276 code
         However I am only using it for the register headerfile.  This code
         provides a wealth of knowledge on how to properly configure and use the radio.
         One should carefully consider adopting its best practices for settup the
         radio in context of the FarmBeats depolyment scenario.  I have not done this!

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

         Having read the RFM95/96/97/98(W) manual there are many things we could
         do to optimize the performance and power by carefully customizing things to
         our environment.  
         Not sure it is worth this includes potentially using implicity header mode

     3)  No Duty cycle logic.
           See txDelay():  right now sender buffers data from an input
           stream and sends it based on a based delay and random delay
           since the last send.
           default values are based delay of 1 second with random delay
           of 2 seconds.  So next send window will be between 1 to 3 
           seconds after the last send (assuming there is data to send).

*/

#include <SPI.h>              
#include <EEPROM.h>
#include <LoRa.h>
#include "LoRaMac-Node/src/radio/sx1276/sx1276Regs-LoRa.h"

#define VERSION "$Id$ 0.2.5"

// uncomment next line to turn on debug (see below for how to customize
// the debug behaviour
#define DEBUG

// set next line to true to reset EEPROM id back to null (-1)
#define EEPROM_RESET_ID false

// if eeprom id is null (-1) then we set the id the following value
//  get this value from somewhere else ;-)
//#define MYID 0x00000001
#define MYID 0x00000002

// turn on/off packet dumping to serial with the following
#define DUMP_RX_PACKET 
#define DUMP_TX_PACKET

#define UNUSED(x) (void)(x)

#ifndef DEBUG

#define waitForKey(...)

#else

// DEBUG SETTING
// use the following to customize debug behaviour
#define PRINT_VERSION_STRING
#define WAIT false
#define MSG  false
#define LORA_INFO
//#define LORA_STATS
//#define LORA_POWER_TEST 
//#define LORA_DUMP_REGISTERS 
//#define DUMP_EEPROM 

void waitForKey(Stream &s, String str, bool msg=MSG, bool wait=WAIT)
{
  if (msg) s.println(str);
  if (wait) {
    while (!s.available());
    while (s.available()) s.read();
  }
}

#endif

#if defined(DUMP_RX_PACKET) || defined(DUMP_TX_PACKET)
int dumpHex(Stream &s, unsigned char *start, int bytes) 
{
  int j;
  int i;
  for (j=0; j<bytes; ) {
    if ((j%16) == 0) s.print("\t" + String(j,HEX) + ": ");
    for (i=0;(i<16) && ((j+i) < bytes);i++) {
      s.print(String(start[j+i],HEX) + " ");
    }
    s.print("|");
    for (i=0;(i<16) && ((j+i) < bytes);i++) {
      char c=start[j+i];
      if (c>=' ' && c<='~')  s.print(c);
      else s.print(".");
    }
    s.print("|\n");
    j+=i;
  }
  return j;
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

  // Map of sensor box pin assignments.
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
  

  class LoRaClass : public ::LoRaClass {
#ifdef LORA_STATS
    uint16_t numValidHdrsRx_;
    uint16_t numValidPktsRx_;
    uint8_t  numPayLoadBytesRx_;
    uint8_t  irqFlags_;
    
    uint8_t getIRQFlags() {
      return read_reg(REG_LR_IRQFLAGS);
    }
    
    uint8_t getPayLoadBytesRx() {
      return read_reg(REG_LR_RXNBBYTES);
    }
    
    uint16_t getNumValidPktsRx() {
      uint16_t val;
      val = (read_reg(REG_LR_RXPACKETCNTVALUEMSB) << 8) |
  	    (read_reg(REG_LR_RXPACKETCNTVALUELSB)     );  
      return val;
    }

    uint16_t getNumValidHdrRx() {
      uint16_t val;
      val = (read_reg(REG_LR_RXHEADERCNTVALUEMSB) << 8) |
  	    (read_reg(REG_LR_RXHEADERCNTVALUELSB)     );  
      return val;
    }
#endif
    unsigned char read_reg(uint8_t addr) { return readRegister(addr); }
  public:
    int getRssiPktValue() { return read_reg(REG_LR_PKTRSSIVALUE); }
    int getSnrPktValue() { return read_reg(REG_LR_PKTSNRVALUE); }
#ifdef LORA_INFO
    void info(Stream &s, long freq, long bw, int sf, int txPwr, int crd, long preLen,
	      int sw, bool crc, bool iMode) {
            s.print("\n\t\tversion=" + String(read_reg(REG_LR_VERSION),HEX));
      s.print("\n\t\tRegOpMode=" + String(read_reg(REG_LR_OPMODE),HEX));
      s.print("\n\t\tfreq_=" + String(freq, DEC) + " reg=" +
	      String(((uint32_t)read_reg(REG_LR_FRFMSB)) << 16 |
		     ((uint32_t)read_reg(REG_LR_FRFMID)) <<  8 |
		     ((uint32_t)read_reg(REG_LR_FRFLSB))       , DEC));
      s.print("\n\t\tbandwidth_=" + String(bw, DEC) + " reg=" +
	      String(((read_reg(REG_LR_MODEMCONFIG1)>>4)&0xf),DEC));
      s.print("\n\t\tspreadingFactor_=" + String(sf, DEC) + " reg=" +
	      String((read_reg(REG_LR_MODEMCONFIG2)>>4)&0xf,DEC));
      s.print("\n\t\ttxPower_=" + String(txPwr, DEC) + " REG_PA_CONFIG=" +
	      String(read_reg(REG_LR_PACONFIG),HEX));
      s.print("\n\t\tLNA=" + String(read_reg(REG_LR_LNA),HEX));
      s.print("\n\t\tcodingRateDenom_=" + String(crd, DEC) + " reg=" +
	      String((read_reg(REG_LR_MODEMCONFIG1) >> 1) & 0x3,DEC));
      s.print("\n\t\tpreambleLength_=" + String(preLen, DEC) + " reg=" +
	      String(read_reg(REG_LR_PREAMBLEMSB) << 8 |
		     read_reg(REG_LR_PREAMBLELSB)));
      s.print("\n\t\tsyncWord_=" + String(sw,HEX) + " reg=" +
	      String(read_reg(REG_LR_SYNCWORD),HEX));
      s.print("\n\t\tcrc_=" + String(crc) + " reg=" +
	      String((read_reg(REG_LR_MODEMCONFIG2)>>2)&0x1));
      s.println("\n\t\timplicitHeaderMode=" + String(iMode) + " reg=" +
		String(read_reg(REG_LR_MODEMCONFIG1)&0x1));
    }
#endif
#ifdef LORA_STATS
    void dumpStats(Stream &s) {
      s.println("\tirqFlags: RXTMTOUT:" +
		String((irqFlags_ & RFLR_IRQFLAGS_RXTIMEOUT)!=0)  +
		" RXDONE:" +
		String((irqFlags_ & RFLR_IRQFLAGS_RXDONE)!=0) +
		" PLCRCERR:" +
		String((irqFlags_ & RFLR_IRQFLAGS_PAYLOADCRCERROR)!=0) +
		" VALIDHDR:" +
		String((irqFlags_ & RFLR_IRQFLAGS_VALIDHEADER)!=0) +
		" TXDONE:" +
		String((irqFlags_ & RFLR_IRQFLAGS_TXDONE)!=0) +
		" CADDONE:" +
		String((irqFlags_ & RFLR_IRQFLAGS_CADDONE)!=0) +
		" FHSSCHGCH:" +
		String((irqFlags_ & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL)!=0) +
		" CADDET:" +
		String((irqFlags_ & RFLR_IRQFLAGS_CADDETECTED)!=0));
      s.println("\tnumValidHdrsRx:" + String(numValidHdrsRx_) + 
              " numValidPktsRx:" + String(numValidPktsRx_) +
		" numPayLoadBytesRx:" + String(numPayLoadBytesRx_));  
    }
    bool updateStats() {
      bool chg = false;
      uint16_t irqFlgs = getIRQFlags();
      uint16_t hdrCnt  = getNumValidHdrRx();
      uint16_t pktCnt  = getNumValidPktsRx();
      uint8_t  bytes   = getPayLoadBytesRx();
      
      if (irqFlgs != irqFlags_)          {
	// dont't payattention to changes in RXTIMEOUTS output is too noisy
	if ((irqFlags_ ^ irqFlgs) != RFLR_IRQFLAGS_RXTIMEOUT) chg = true;
	irqFlags_=irqFlgs;
      }
      if (hdrCnt  != numValidHdrsRx_)    { numValidHdrsRx_=hdrCnt;   chg=true; }
      if (pktCnt  != numValidPktsRx_)    { numValidPktsRx_=pktCnt;   chg=true; }
      if (bytes   != numPayLoadBytesRx_) { numPayLoadBytesRx_=bytes; chg=true; }
      
      return chg;      
    }
#endif
    static const int MAX_PKT_SIZE = 255;
  } theLoRa;


  
  // Default communication parameters
  //   based on this document
  //   https://www.thethingsnetwork.org/docs/lorawan/frequencies-by-country.html
  //   default frequency to use in india is IN865-867 but we seem to be using
  //   868.0
  const long LORA_FREQ                 = 868E6;
  const long LORA_BANDWIDTH            = 125E3;  
  const int  LORA_SPREADING_FACTOR     = 7;
  const int  LORA_TX_POWER_LEVEL       = 6;
  const int  LORA_CODING_RATE_DENOM    = 5;    // 4/5
  const long LORA_PREAMBLE_LENGTH      = 8;
  const int  LORA_SYNC_WORD            = 0x12;  // 0x34 is for LoRaWan using 0x12
  const bool LORA_CRC                  = true;
  // By default we operate in explicit header mode:
  // The packets include a hardware generated header
  // See page 26 of RFM95/96/97/98(W) manual for details. Brief summary is below
  const bool LORA_IMPLICIT_HEADER_MODE = false;

  // Explicity mode packet structure is as follows:
  // <preamble> <header,headr_crc> <payload> [16 bit payload_crc]
  // header: <payload len> <fwd error correcton code rate> <yes/no payload_crc present>
  // error correcton code rate for header is 4/8
  // as indicated the payload code rate is specified in the header
  
  // Default send timing parameters
  
  const int  LORA_RANDOM_SEND_DELAY_MS = 2000;
  const int  LORAY_BASE_SEND_DELAY_MS  = 1000;

  
  class Id {
    static const int EEPROM_ID_OFFSET_FROM_END=sizeof(uint32_t);
    static const uint32_t NULLID = -1;
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
    // radio configuration parameters
    const long     freq_               = LORA_FREQ;
    const long     bandwidth_          = LORA_BANDWIDTH;
    const long     preambleLength_     = LORA_PREAMBLE_LENGTH;
    const int      spreadingFactor_    = LORA_SPREADING_FACTOR;
    const int      txPower_            = LORA_TX_POWER_LEVEL;
    const int      codingRateDenom_    = LORA_CODING_RATE_DENOM;
    const int      syncWord_           = LORA_SYNC_WORD;
    const bool     crc_                = LORA_CRC;
    const bool     implicitHeaderMode_ = LORA_IMPLICIT_HEADER_MODE; 

    // pin/wiring parameters
    static const int      pwrDisable_         = LORA_R_PWR_DISABLE;
    static const int      reset_              = LORA_L_NRESET;
    static const int      ss_                 = LORA_L_SS;
    static const int      dio0_               = LORA_R_DI00;

    // send timing parameters
    const int      baseSendDelay_      = LORAY_BASE_SEND_DELAY_MS;
    const int      randomSendDelay_    = LORA_RANDOM_SEND_DELAY_MS;


    
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

    unsigned char pktBuffer[LoRaClass::MAX_PKT_SIZE];
    
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
      String hstr = myIdStr_ + "," + String(dataLen) + ","; 
      int hlen = hstr.length();
      int len = hlen + dataLen;
      
      hstr.getBytes(pktBuffer, LoRaClass::MAX_PKT_SIZE);
      memcpy(&pktBuffer[hlen], data, LoRaClass::MAX_PKT_SIZE - dataLen);
      s.write(pktBuffer, len);
      
      return len;
    }

    // FIXME: JA harded coded delay eventually should be adapted to
    // correct duty cycle logic
    unsigned long txDelay() {
      return baseSendDelay_ + random(randomSendDelay_);
    }

    void sendStreamBuf(Stream &ds) {
      UNUSED(ds); // suppress warning if dumping turned off
      int dataLen = streamBuf_.dataLen();
      byte *data  = streamBuf_.data();
      
      if (dataLen) {
	// we have data to send
	int sentCnt;
	theLoRa.beginPacket(implicitHeaderMode_);
	sentCnt = copyDataToStream(theLoRa, data, dataLen);
	theLoRa.endPacket();
	
	txCnt_      += sentCnt;
	lastTx_      = millis();
	nextTxAfter_ = txDelay();

#ifdef DUMP_TX_PACKET
	ds.print(myIdStr_ + ":>" + String(sentCnt) +"[" + String(txCnt_) + "]\n\t");
	dumpHex(ds, pktBuffer, sentCnt);
#endif
	streamBuf_.reset();
      }
    }

    void onReceive(int packetSize, Stream &s) {
      if (packetSize == 0) return;
      String inStr = "";
      int n=0;
      if ((unsigned int)packetSize > sizeof(pktBuffer)) {
	s.print("ERROR: packetSize:" + String(packetSize) + " > " +
		 String(sizeof(pktBuffer)));
      }
      
      while (theLoRa.available()) {
	pktBuffer[n] = (char)theLoRa.read();
	n++;
      }
      
      if (packetSize != n) { s.println("ERROR size mismatch!!!"); }
      rxCnt_ +=  packetSize;
      
#ifdef DUMP_RX_PACKET
        s.print(myIdStr_ + ":<" +
		String(packetSize) + "[" + String(rxCnt_) +
		"] rssi:" +
		String(theLoRa.packetRssi())+ "(" +
		String(theLoRa.getRssiPktValue()) + ") snr:" +
		String(theLoRa.packetSnr()) + "("
		+ String(theLoRa.getSnrPktValue()) + ")\n\t");
	dumpHex(s, pktBuffer, packetSize);
#endif	
    }

  public:
    
    LoraModule() {}

    void setId(uint32_t id) { myId_ = id; myIdStr_ = String(id,HEX); }
    uint32_t getId() { return myId_; }

#ifdef LORA_INFO
    void info(Stream &s) {
      s.println("LoraModule():");
      s.println("\tmyId_: " + String(myId_) + " myIdStr_: " + myIdStr_);
      s.print("\tPINS: pwrDisable_="); s.print(pwrDisable_, DEC);
      s.print(" reset_="); s.print(reset_, DEC);
      s.print(" ss_="); s.print(ss_, DEC);
      s.print(" dio0_="); s.print(dio0_, DEC);
      s.print("\n\tRADIO: ");
      theLoRa.info(s, freq_, bandwidth_, spreadingFactor_, txPower_, codingRateDenom_,
	     preambleLength_, syncWord_, crc_, implicitHeaderMode_);
    }
#endif
    
    int start() {
      SPI.begin();
      pinMode(pwrDisable_, OUTPUT);
      enablePower();
      sleep(); // give module time to warm up???
      lastTx_ = 0;
      nextTxAfter_ = txDelay();
      int rc = theLoRa.begin(freq_);
      if (rc) {
	// FIXME: JA probably only need to do this once unless changing
	// setup up the radio parameters;
	theLoRa.setTxPower(txPower_);
	theLoRa.setSpreadingFactor(spreadingFactor_);
	theLoRa.setSignalBandwidth(bandwidth_);
	theLoRa.setCodingRate4(codingRateDenom_);
	theLoRa.setPreambleLength(preambleLength_);
	theLoRa.setSyncWord(syncWord_);
	if (crc_) theLoRa.enableCrc(); else theLoRa.disableCrc();
#ifdef LORA_INFO
	info(Serial);
#endif
      }
      return rc;
    }
    	          
    void loopAction(Stream &s) {
      // local input stream processing
      streamBuf_.buffer(s);
	
      // send stream data when and as necessary
      if ((millis() - lastTx_) > nextTxAfter_) {
        // time to send any buffered data
        sendStreamBuf(s);
      }

#ifdef LORA_STATS
      if (theLoRa.updateStats()) { theLoRa.dumpStats(s); }
#endif
      
      // poll and process any data on theLoRa
      onReceive(theLoRa.parsePacket(),s);
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
      theLoRa.end();
      
      SPI.end();

      SPCR = spcrIntialValue_;
      pinMode(SCK, INPUT);
      pinMode(MOSI, INPUT);
      pinMode(MISO, INPUT);
      
      pinMode(pwrDisable_, OUTPUT);
      disablePower();
    }
    
    void status(Stream &out) {
      theLoRa.dumpRegisters(out);
    }

    void setup() {
      spcrIntialValue_ = SPCR;
      txCnt_ = 0;
      rxCnt_ = 0;
      pinMode(pwrDisable_, OUTPUT);
      disablePower();
      theLoRa.setPins(ss_, reset_, dio0_); 
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

#ifdef PRINT_VERSION_STRING
  Serial.println("FrmBtLora version: " + String(VERSION));
#endif
  
  waitForKey(Serial, "setup(): BEGIN: Send key to continue");

#ifdef DUMP_EEPROM
  FarmBeats::Id::dumpEEProm(Serial);
  waitForKey(Serial, "EEProm dump done");
#endif

  lm.setId(id.value());

  
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

