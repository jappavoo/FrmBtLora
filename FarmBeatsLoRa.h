#ifndef __FARM_BEATS_LORA_H__
#define __FARM_BEATS_LORA_H__
/*
  FarmBeats LoRa Arduino Code for Indian Sensor Box

  This code is hard coded around the Semtech sx1276 which is the radio module on the 
  LORA 868Mhz 8X1276 RF MODULE from ROBOKITS INDIA RKI-2726 

  NOTE THE MDOT uses sx1272 which is different from the sx1276 that the Indian Farm 
      Beats sensor box uses 
  Requirements:
       standard SPI library
       standar EEPROM library
       LoRa -- the repository include a version to use it link the libraries/LoRa directory
               to your arduino libraries directory

  Known Issues:

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
#include "FrmBtLoraPkt.h"

#define VNUM "0.3.2"
// uncomment next line to turn on debug (see below for how to customize
// the debug behaviour
//#define DEBUG

// To set the box define FARM_BEATS_ID on the compile line with -D
// eg. -DFARM_BREAT_ID={'1','0','N','I'}
// or in the file as
//#define  FARM_BEATS_ID {'1','0','N','I'}

#define RAW

#ifdef  SERIAL_INPUT_PROCESSING
#define MODE "Serial"
#define DUMP_RX_PACKET
#define DUMP_TX_PACKET
#else
#define INDESIGN_PACKET_PROCESSING
#define MODE "InDesignPkts"
#endif

#ifdef RAW
#define VERSION  VNUM " " MODE " raw"
#else
#define VERSION  VNUM " " MODE " LoRa V1.1: MAC"
#endif

#ifdef INDESIGN_PACKET_PROCESSING
// ATMEGA328 is little endian
inline __attribute__((always_inline)) uint32_t Host32ToBE32(const uint32_t x) {
  if (x==0) return 0;
  return x << 24U | x >> 24U |                    // xchg first and last byte
        (x & 0xFF00) << 8U |(x & 0xFF0000) >> 8U; // xchg first and last byte
}
inline __attribute__((always_inline)) uint32_t BE32ToHost32(const uint32_t x) {
  if (x==0) return 0;
    return x << 24U | x >> 24U |                  // xchg first and last byte
        (x & 0xFF00) << 8U |(x & 0xFF0000) >> 8U; // xchg first and last byte
}
inline __attribute__((always_inline)) uint32_t Host32ToLE32(const uint32_t x) { return x; }
inline __attribute__((always_inline)) uint32_t LE32ToHost32(const uint32_t x) { return x; }
inline __attribute__((always_inline)) uint32_t Host16ToLE16(const uint32_t x) { return x; }
inline __attribute__((always_inline)) uint32_t LE16ToHost16(const uint32_t x) { return x; }

#include "IDPkt.h"
#endif

// turn on/off packet dumping to serial with the following
//#define DUMP_RX_PACKET 
//#define DUMP_TX_PACKET

#define UNUSED(x) (void)(x)

#ifndef DEBUG
// data written to serial line is raw payload bytes
#define waitForKey(...)
#else

// DEBUG SETTING
// use the following to customize debug behaviour
#define DUMP_RX_PACKET 
#define DUMP_TX_PACKET
#define PRINT_VERSION_STRING
#define WAIT false
#define MSG  false
#define LORA_INFO
//#define LORA_STATS
//#define LORA_POWER_TEST 
//#define LORA_DUMP_REGISTERS 
//#define DUMP_EEPROM 

void waitForKey(String str, bool msg=MSG, bool wait=WAIT)
{
  if (msg) Serial.println(str);
  if (wait) {
    while (!Serial.available());
    while (Serial.available()) Serial.read();
  }
}

#endif

#if defined(DUMP_RX_PACKET) || defined(DUMP_TX_PACKET)
int dumpHex(unsigned char *start, int bytes) 
{
  int j;
  int i;
  for (j=0; j<bytes; ) {
    if ((j%16) == 0) Serial.print("\t" + String(j,HEX) + ": ");
    for (i=0;(i<16) && ((j+i) < bytes);i++) {
      Serial.print(String(start[j+i],HEX) + " ");
    }
    Serial.print("|");
    for (i=0;(i<16) && ((j+i) < bytes);i++) {
      char c=start[j+i];
      if (c>=' ' && c<='~')  Serial.print(c);
      else Serial.print(".");
    }
    Serial.print("|\r\n");
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

  // A bunch of extra LoRa functions to make debugging and optimization
  // easier.  Unfortunately the LoRa class from the library is not very
  // easy to extend so have had to create my own read_reg and use
  // our own copy of the register name constants from semtech
  class LoRaUtils {
  public:
#if defined(LORA_STATS) || defined(DUMP_RX_PACKET) || defined(LORA_INFO) || defined(LORA_RW)
    // duplicated here as we don't have access to the version
    // in the LoRa Class from the the library 
    static inline unsigned char read_reg(const uint8_t addr) {
      digitalWrite(LORA_L_SS, LOW);
      
        SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
          SPI.transfer(addr & 0x7f );
          uint8_t val = SPI.transfer(0x00);
        SPI.endTransaction();
      
      digitalWrite(LORA_L_SS, HIGH);

      return val;
      
    }
#endif
    
#ifdef LORA_STATS    
    static uint8_t getIRQFlags() {
      return read_reg(REG_LR_IRQFLAGS);
    }
    
    static uint8_t getPayLoadBytesRx() {
      return read_reg(REG_LR_RXNBBYTES);
    }
    
    static uint16_t getNumValidPktsRx() {
      uint16_t val;
      val = (read_reg(REG_LR_RXPACKETCNTVALUEMSB) << 8) |
  	    (read_reg(REG_LR_RXPACKETCNTVALUELSB)     );  
      return val;
    }

    static uint16_t getNumValidHdrRx() {
      uint16_t val;
      val = (read_reg(REG_LR_RXHEADERCNTVALUEMSB) << 8) |
  	    (read_reg(REG_LR_RXHEADERCNTVALUELSB)     );  
      return val;
    }
#endif

#ifdef DUMP_RX_PACKET
    static int getRssiPktValue() { return read_reg(REG_LR_PKTRSSIVALUE); }
    static int getSnrPktValue() { return read_reg(REG_LR_PKTSNRVALUE); }
#endif
#ifdef LORA_INFO
    static void info(long freq, long bw, int sf, int txPwr, int crd, long preLen,
	      int sw, bool crc, bool iMode) {
            Serial.print("\r\n\t\tversion=" + String(read_reg(REG_LR_VERSION),HEX));
      Serial.print("\r\n\t\tRegOpMode=" + String(read_reg(REG_LR_OPMODE),HEX));
      Serial.print("\r\n\t\tfreq_=" + String(freq, DEC) + " reg=" +
	      String(((uint32_t)read_reg(REG_LR_FRFMSB)) << 16 |
		     ((uint32_t)read_reg(REG_LR_FRFMID)) <<  8 |
		     ((uint32_t)read_reg(REG_LR_FRFLSB))       , DEC));
      Serial.print("\r\n\t\tbandwidth_=" + String(bw, DEC) + " reg=" +
	      String(((read_reg(REG_LR_MODEMCONFIG1)>>4)&0xf),DEC));
      Serial.print("\r\n\t\tspreadingFactor_=" + String(sf, DEC) + " reg=" +
	      String((read_reg(REG_LR_MODEMCONFIG2)>>4)&0xf,DEC));
      Serial.print("\r\n\t\ttxPower_=" + String(txPwr, DEC) + " REG_PA_CONFIG=" +
	      String(read_reg(REG_LR_PACONFIG),HEX));
      Serial.print("\r\n\t\tLNA=" + String(read_reg(REG_LR_LNA),HEX));
      Serial.print("\r\n\t\tcodingRateDenom_=" + String(crd, DEC) + " reg=" +
	      String((read_reg(REG_LR_MODEMCONFIG1) >> 1) & 0x3,DEC));
      Serial.print("\r\n\t\tpreambleLength_=" + String(preLen, DEC) + " reg=" +
	      String(read_reg(REG_LR_PREAMBLEMSB) << 8 |
		     read_reg(REG_LR_PREAMBLELSB)));
      Serial.print("\r\n\t\tsyncWord_=" + String(sw,HEX) + " reg=" +
	      String(read_reg(REG_LR_SYNCWORD),HEX));
      Serial.print("\r\n\t\tcrc_=" + String(crc) + " reg=" +
	      String((read_reg(REG_LR_MODEMCONFIG2)>>2)&0x1));
      Serial.println("\r\n\t\timplicitHeaderMode=" + String(iMode) + " reg=" +
		String(read_reg(REG_LR_MODEMCONFIG1)&0x1));
    }
#endif
#ifdef LORA_STATS
    static void dumpStats() {
      uint16_t irqFlgs = getIRQFlags();
      uint16_t hdrCnt  = getNumValidHdrRx();
      uint16_t pktCnt  = getNumValidPktsRx();
      uint8_t  bytes   = getPayLoadBytesRx();
      
      Serial.println("\tirqFlags: RXTMTOUT:" +
		String((irqFlgs & RFLR_IRQFLAGS_RXTIMEOUT)!=0)  +
		" RXDONE:" +
		String((irqFlgs & RFLR_IRQFLAGS_RXDONE)!=0) +
		" PLCRCERR:" +
		String((irqFlgs & RFLR_IRQFLAGS_PAYLOADCRCERROR)!=0) +
		" VALIDHDR:" +
		String((irqFlgs & RFLR_IRQFLAGS_VALIDHEADER)!=0) +
		" TXDONE:" +
		String((irqFlgs & RFLR_IRQFLAGS_TXDONE)!=0) +
		" CADDONE:" +
		String((irqFlgs & RFLR_IRQFLAGS_CADDONE)!=0) +
		" FHSSCHGCH:" +
		String((irqFlgs & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL)!=0) +
		" CADDET:" +
		String((irqFlgs & RFLR_IRQFLAGS_CADDETECTED)!=0));
      Serial.println("\tnumValidHdrsRx:" + String(hdrCnt) + 
              " numValidPktsRx:" + String(pktCnt) +
		" numPayLoadBytesRx:" + String(bytes));  
    }
#endif
#ifdef LORA_RW
    // FIXME: Might want to optimize for multiple transfers
    
    // duplicated here as we don't have access to the version
    // in the LoRa Class from the the library 
    static void write_reg(const uint8_t addr, const uint8_t v) {
      digitalWrite(LORA_L_SS, LOW);
      
        SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
          SPI.transfer(addr | 0x80);
          uint8_t val = SPI.transfer(v);
        SPI.endTransaction();
      
      digitalWrite(LORA_L_SS, HIGH);

      return val;
      
    }

    static void writePayLoadLength(uint8_t len) {
      write_reg(REG_LR_PAYLOADLENGTH, len);
    }
    static void writePayLoad8(uint8_t v) {
      write_reg(REG_LR_FIFO,v);
    }
    static void writePayLoad16LE(uint16_t v) {
      writePayLoad8(REG_LR_FIFO,(uint8_t)(v & 0xff));
      writePayLoad8(REG_LR_FIFO,(uint8_t)((v>>8) & 0xff)); 
    }
    static void writePayLoad32LE(uint32_t v) {
      writePayLoad8(REG_LR_FIFO,(uint8_t)(v & 0xff));
      writePayLoad8(REG_LR_FIFO,(uint8_t)((v>>8) & 0xff));
      writePayLoad8(REG_LR_FIFO,(uint8_t)((v>>16) & 0xff));
      writePayLoad8(REG_LR_FIFO,(uint8_t)((v>>24) & 0xff));
    }
    static void writePayLoad32BE(uint32_t v) {
      writePayLoad8(REG_LR_FIFO,(uint8_t)((v>>24) & 0xff));
      writePayLoad8(REG_LR_FIFO,(uint8_t)((v>>16) & 0xff));
      writePayLoad8(REG_LR_FIFO,(uint8_t)((v>>8) & 0xff));
      writePayLoad8(REG_LR_FIFO,(uint8_t)(v & 0xff));
    }
#endif
    static const int MAX_PKT_SIZE = 255;
  };

#define theLoRa LoRa
  
  // Default communication parameters

  //  I have consulted various documents here are some snipts that I found useful
  //  
  //   b
  //   https://www.thethingsnetwork.org/docs/lorawan/frequencies-by-country.html
  //   default frequency to use in india is IN865-867  -- spectrum IN44

  // ******************************************
  // The Lorawan resgional parameters_v1.1rb doc says that india paramaters for
  // LoRaWan are as follows:
  //
  // 1404 2.10.2 IN865-867 ISM Band channel frequencies
  // 1405 This section applies to the Indian sub-continent.
  // 1406 The network channels can be freely attributed by the network operator.
  //      However the three  
  // 1407 following default channels MUST be implemented in every
  //      India 865-867MHz end-device.
  // 1408 Those channels are the minimum set that all network gateways
  //      SHOULD always be listening
  // 1409 on.
  // 1410
  // 
  // Modulation Bandwidth    Channel      FSK Bitrate or      Nb
  //              [kHz]   Frequency [MHz] LoRa DR / Bitrate   Channels
  //    LoRa      125        865.0625       DR0 to DR5          3
  //                         865.4025       / 0.3-5 kbps
  //                         865.985
  // 1411 Table 68: IN865-867 default channels
  // 1412 End-devices SHALL be capable of operating in the 865 to 867 MHz
  //      frequency band and
  // 1413 should feature a channel data structure to store the parameters of
  //      at least 16 channels. A
  // 1414 channel data structure corresponds to a frequency and a set of data
  //      rates usable on this
  // 1415 frequency.
  // 1416 The first three channels correspond to 865.0625, 865.4025, and
  //      865.985 MHz / DR0 to DR5
  // 1417 and MUST be implemented in every end-device. Those default channels
  //      cannot be modified
  // 1418 through the NewChannelReq command and guarantee a minimal common
  //      channel set
  // 1419 between end-devices and network gateways.
  // 1420 The following table gives the list of frequencies that SHALL be
  //      used by end-devices to
  // 1421 broadcast the JoinReq message. The JoinReq message transmit duty-cycle
  //      SHALL follow the
  // 1422 rules described in chapter “Retransmissions back-off” of the LoRaWAN
  //      specification
  // 1423 document.
  // 1424
  // Modulation Bandwidth    Channel      FSK Bitrate or      Nb
  //              [kHz]   Frequency [MHz] LoRa DR / Bitrate   Channels
  //    LoRa      125        865.0625       DR0 to DR5          3
  //                         865.4025       / 0.3-5 kbps
  //                         865.9850
  // 1425 Table 69: IN865-867 JoinReq Channel List
  // 1426 2.10.3 IN865-867 Data Rate and End-device Output Power Encoding
  // 1427 There is no dwell time or duty-cycle limitation for the
  //      INDIA 865-867 PHY layer. The
  // 1428 TxParamSetupReq MAC command is not implemented by INDIA 865-867 devices.
  // 1429 The following encoding is used for Data Rate (DR) and End-device Output
  //      Power (TXPower)
  // 1430 in the INDIA 865-867 band:
  // 1431
  // DataRate Configuration          Indicative physical
  //                                   bit rate [bit/s]
  //     0     LoRa: SF12 / 125 kHz         250
  //     1     LoRa: SF11 / 125 kHz         440
  //     2     LoRa: SF10 / 125 kHz         980
  //     3     LoRa: SF9 / 125 kHz          1760
  //     4     LoRa: SF8 / 125 kHz          3125
  //     5     LoRa: SF7 / 125 kHz          5470
  //     6            RFU                    RFU
  //     7     FSK: 50 kbps                 50000
  //   8..14          RFU                    RFU 
  //     15     Defined in LoRaWAN
  // 1432      Table 70: IN865-867 TX Data rate table
  //
  //   This document has a lot more info in it about the LoRaWan parameters
  //   for India  Including power max payload and rxwindow's etc I will
  //   continue to look at once I understand how the frequency we are using
  //   relates to our need to be NOT LoRaWan compatible


  // ******************************************
  // mdot manual peer to peer says:
  //  "For Europe 868 models, use a fixed frequency, 869.85, with 7 dBm power
  // setting to allow 100% duty-cycle usage."


  // *******************************************
  // InDesign LoRa Peer-To-Peer Protocol Document specifies
  // AT+NJM=3
  // AT+ACK=0
  // AT+NA=4D734662
  // AT+NSK=89ABC89ABC89ABC89ABC89ABC89ABC34
  // AT+DSK=45678904567890456789045678904567
  // AT+TXDR=DR3
  // AT+TXP=11
  // AT+TXF=866300000
  // AT&W

  // So based the above we first off 
  const long LORA_FREQ                 = 866300000;
  const long LORA_BANDWIDTH            = 125E3;  
  const int  LORA_SPREADING_FACTOR     = 9;
  const int  LORA_TX_POWER_LEVEL       = 11;
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
  
  const int  LORA_RANDOM_SEND_DELAY_MS = 100;
  const int  LORAY_BASE_SEND_DELAY_MS  = 1000;

  
  class Id {
    static const int EEPROM_ID_OFFSET_FROM_END=sizeof(uint32_t);
    static const uint32_t NULLID = -1;
    union FM_ID {
      uint32_t raw;
      char     ascii[4];
    } id_;    
    static_assert(sizeof(FM_ID) == 4, "Bad FM_ID union size");

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
#ifdef FARM_BEATS_ID
      resetId();
      setEEPromId(((union FM_ID){ .ascii = FARM_BEATS_ID }).raw);
#endif
      id_.raw = getEEPromId();      
    }
    
    uint32_t value() { return id_.raw; }
    
    void resetId() {
      setEEPromId(NULLID);
    }
    
    bool isNull() { return id_.raw == NULLID; }
    
#ifdef DUMP_EEPROM
    static void dumpEEProm() {
      unsigned int len=EEPROM.length();
      Serial.println("Id::dumpEEProm(): len=" + String(len));
      for (unsigned int i=0; i<len; i+=16) { 
	Serial.print(String(i,HEX) + ":\t");
	for (int j=0; j<16; j++) {
	  Serial.print(EEPROM.read(i+j),HEX);
	  Serial.print(" ");  
	}
	Serial.println();
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


#ifdef SERIAL_INPUT_PROCESSING   
    class SerialBuffer {
#define INPUT_STREAM_BUFFER_SIZE 20
      const int len_ = INPUT_STREAM_BUFFER_SIZE;
      byte buf_[INPUT_STREAM_BUFFER_SIZE];      // can't use const member yet :-(
      int  end_;
    public:
      SerialBuffer() : end_(0) {}
      
      void reset()     { end_=0; }      
      int  spaceLeft() { return len_ - end_; }
      int  dataLen()   { return end_; }
      bool isFull()    { return end_ == len_; }
      bool isEmpty()   { return end_ == 0; }

      // if there is data on the stream copies into buffer space
      // aviable.  May leave data on stream if not enough space
      int buffer() {
        int copy=0;
	int len = Serial.available();
	int space = spaceLeft();
	
	if (len && space) {
	  if (space >= len) copy = len; else copy = space;
	  Serial.readBytes(&buf_[end_], copy);
	  end_ += copy;
	}
	return copy;
      }
      byte *data() { return buf_; }
      
    } serialBuf_;
 #endif

  public:
#ifdef INDESIGN_PACKET_PROCESSING
    FB2Srv1DataRecordPkt theSample_;
#else
    unsigned char pktBuffer[LoRaUtils::MAX_PKT_SIZE];
#endif
  private:
    unsigned long lastTx_;       // 4 bytes
    unsigned long nextTxAfter_;  // 4 bytes
#ifndef INDESIGN_PACKET_PROCESSING
    uint32_t  myId_;             // 4 bytes
    int txCnt_;                  // 2 bytes
    int rxCnt_;                  // 2 bytes  
#endif
    // We put this here to colocate all the data members
    // in one place to make reasoninb about size easier
#ifdef LORA_POWER_TEST           // 1 byte
    bool pwr_;
#endif
    
    // 7.2. Reset of the Chip
    // A power-on reset of the SX1276/77/78/79 is triggered at power up. Additionally, a manual reset can be issued by controlling pin 7.
    // 7.2.1. POR
    // If the application requires the disconnection of VDD from the SX1276/77/78/79, despite of the extremely low Sleep Mode current, the user should wait for 10 ms from of the end of the POR cycle before commencing communications over the SPI bus. Pin 7 (NRESET) should be left floating during the POR sequence.
    void sleepAfterPowerOn() { delay(10); }
    
    void disablePower() {
      // LORA Power is negative enable logic on MCU pin
      digitalWrite(pwrDisable_, HIGH);
#ifdef LORA_POWER_TEST
      pwr_ = false;
#endif
    }
    
    void enablePower() {
      // LORA Power is negative enable logic on MCU pin
      digitalWrite(pwrDisable_, LOW);
#ifdef LORA_POWER_TEST
      pwr_ = true;
#endif
    }

    // FIXME: JA harded coded delay eventually should be adapted to
    // correct duty cycle logic
    unsigned long txDelay() {
      return baseSendDelay_ + random(randomSendDelay_);
    }

#ifdef SERIAL_INPUT_PROCESSING
    int copyDataToStream(Stream &stm, byte *data, int dataLen) {
      int len = dataLen;
      memcpy(pktBuffer, data,
	     (dataLen <= (LoRaUtils::MAX_PKT_SIZE)) ? dataLen :
	     (LoRaUtils::MAX_PKT_SIZE));
      stm.write(pktBuffer, len);      
      return len;
    }

    void sendStreamBuf() {
      int dataLen = serialBuf_.dataLen();
      byte *data  = serialBuf_.data();
      
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
	Serial.print(">" + String(sentCnt) +"[" + String(txCnt_) + "]\r\n");
	dumpHex(pktBuffer, sentCnt);
#endif
	serialBuf_.reset();
      }
    }
#endif

#ifdef INDESIGN_PACKET_PROCESSING
    void sendSampleData() {
      waitForKey("Press key to send sample", true, true);
            
      theLoRa.beginPacket(implicitHeaderMode_);
#ifndef RAW
      theLoRa.write(FB_LORA_MHDR.raw);
#endif
      theLoRa.write(theSample_.data.raw, sizeof(theSample_.data.raw));
      theLoRa.endPacket();

      lastTx_ = millis();
      nextTxAfter_ = txDelay();
      
#ifdef DUMP_TX_PACKET
      Serial.print(">" +
		   String(sizeof(theSample_.data.raw
#ifndef RAW
				 + sizeof(FB_LORA_MHDR.raw)
#endif	// RAW				  
				 ))
		   +"\r\n");
#ifndef RAW
      Serial.print("LoRa v1.1 PHYHDR: " + String(FB_LORA_MHDR.raw, HEX) + "\r\n");
#endif  // RAW
      dumpHex(theSample_.data.raw, sizeof(theSample_.data.raw));
#endif  // DUMP_TX_PACKET
    }
#endif // INDESIGN_PACKET_PROCESSING
    
    void onReceive(int packetSize) {
      if (packetSize == 0) return;
#ifdef SERIAL_INPUT_PROCESSING 
      String inStr = "";
      int n=0;
      
      if ((unsigned int)packetSize > sizeof(pktBuffer)) {
	Serial.print("ERROR: packetSize:" + String(packetSize) + " > " +
		 String(sizeof(pktBuffer)));
      }
      
      while (theLoRa.available()) {
	pktBuffer[n] = (char)theLoRa.read();
	n++;
      }
      
      if (packetSize != n) { Serial.println("ERROR size mismatch!!!"); }
      rxCnt_ +=  packetSize;

            
#ifdef DUMP_RX_PACKET
        Serial.print("\r\n" + String(myId_,HEX) + ":<" +
		String(packetSize) + "[" + String(rxCnt_) +
		"] rssi:" +
		     String(theLoRa.packetRssi())+ "(" +
		     String(LoRaUtils::getRssiPktValue()) + ") snr:" +
		     String(theLoRa.packetSnr()) + "("
		     + String(LoRaUtils::getSnrPktValue()) + ")\r\n\t");
	dumpHex(pktBuffer, packetSize);
#endif	

      {
	int payloadIdx=0;
	if (payloadIdx < packetSize) {
	  Serial.write(&pktBuffer[payloadIdx],packetSize-payloadIdx);
	}
      }
#endif
    }

  public:
    LoraModule() {}

    inline void setId(uint32_t id) {
#ifdef INDESIGN_PACKET_PROCESSING
	theSample_.setSerNo(id);
#else
	myId_ = id;
#endif
      }
    inline uint32_t getId() {
#ifdef INDESIGN_PACKET_PROCESSING
	return theSample_.getSerNo();
#else
	return myId_;
#endif
    }
    //    uint32_t getId() { return myId_; }

#ifdef LORA_INFO
    void info() {
      Serial.println("LoraModule():");
      Serial.println("\tId: " + String(getId(),HEX));
      Serial.print("\tPINS: pwrDisable_="); Serial.print(pwrDisable_, DEC);
      Serial.print(" reset_="); Serial.print(reset_, DEC);
      Serial.print(" ss_="); Serial.print(ss_, DEC);
      Serial.print(" dio0_="); Serial.print(dio0_, DEC);
      Serial.print("\r\n\tRADIO: ");
      LoRaUtils::info(freq_, bandwidth_, spreadingFactor_, txPower_, codingRateDenom_,
		      preambleLength_, syncWord_, crc_, implicitHeaderMode_);
    }
#endif
    
    int start() {
      SPI.begin();
      pinMode(pwrDisable_, OUTPUT);
      enablePower();
      sleepAfterPowerOn(); // give module time to warm up???
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
	info();
#endif
      }
      return rc;
    }
    	          
    inline void loopAction() {
#ifdef SERIAL_INPUT_PROCESSING
      // local input stream processing
      serialBuf_.buffer();
#endif   
      // send data when and as necessary
      if ((millis() - lastTx_) > nextTxAfter_) {
        // time to send any buffered data
#ifdef SERIAL_INPUT_PROCESSING
        sendStreamBuf();
#endif
#ifdef INDESIGN_PACKET_PROCESSING
	sendSampleData();
#endif	
      }

#ifdef LORA_STATS
      LoRaUtils::dumpStats();
#endif
      
      // poll and process any data on theLoRa
      onReceive(theLoRa.parsePacket());
    }

#ifdef INDESIGN_PACKET_PROCESSING
    inline __attribute__((always_inline)) void sendSample(unsigned long ts,
							  uint16_t ADC0,
							  uint16_t ADC1,
							  uint16_t ADC2,
							  uint16_t ADC3,
							  uint16_t ADC4,
							  uint16_t ADC5) {
      // save the data 
      theSample_.setValues(ts,ADC0,ADC1,ADC2,ADC3,ADC4,ADC5);
      // attempt to schedule send if we are allowed to based on interval
      loopAction();
    }
#endif
    
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

      pinMode(SCK, INPUT);
      pinMode(MOSI, INPUT);
      pinMode(MISO, INPUT);
      
      pinMode(pwrDisable_, OUTPUT);
      disablePower();
    }
    
#ifdef LORA_DUMP_REGISTERS    
    void status() {
      theLoRa.dumpRegisters(Serial);
    }
#endif
    
    void setup() {
#ifndef INDESIGN_PACKET_PROCESSING
      txCnt_ = 0;
      rxCnt_ = 0;
#endif
      pinMode(pwrDisable_, OUTPUT);
      disablePower();
      theLoRa.setPins(ss_, reset_, dio0_); 
    }


#ifdef LORA_POWER_TEST
    bool isOn() { return pwr_; }
    
    void testPower() {
      // test power disable
      Serial.println("testPower(): BEGIN");
      pinMode(pwrDisable_, OUTPUT);
      
      disablePower();

      Serial.print("isOn(): ");
      Serial.println(isOn());

      waitForKey("Send Key to Turn ON");

      enablePower();
      
      Serial.print("isOn(): ");
      Serial.println(isOn());
      Serial.println("\tPOWER ON: you should be able to measure 3.3v on LORA_PWR"
		" pin 4 of LORA_R");

      
      waitForKey("Send key to Turn OFF");

      disablePower();

      
      Serial.print("isOn(): ");
      Serial.println(isOn());
      Serial.println("\t POWER OFF: you should be able to measure LOW v on LORA_PWR"
		" pin 4 of LORA_R");
      
      waitForKey("Send key to end testPower");
      
      Serial.println("testPower(): END");
    }
#endif
 };
}

#endif // __FARM_BEATS_LORA_H__
