#ifndef __ID_PKT_H__
#define __ID_PKT_H__

/************************************************************************************
This Header file translates the InDesign Lora Packet Protocol as per document

LORA PEER-TO-PEER PROTOCOL
Farmbeats
Document Number 2505F03 Revision 5
Prepared for Microsoft Date: 2-Jul-2017, 9:05 AM

To use the functions defined by this header file an implementation must provide
the Host[X]To<BE|LE>[X] and <BE|LE>[X]ToHostX functions to take care of byte
ordering.

ToDo:

1)  Add constructors
2)  Add set and get values

JA Q: I don't understand this statement 
TX Data Rate DR1 will be used. DR1 supports a maximum payload of 53 bytes, which is large 
enough to support the Farmbeats message sizes. It also uses a slower data rate, to help 
with long range operation.

************************************************************************************/
const int ID_PKT_MAX_MSG_SIZE = 53;

/*
const uint32_t Host32ToBE32(const uint32_t x);
const uint32_t BE32ToHost32(const uint32_t x);
const uint32_t Host32ToLE32(const uint32_t x);
const uint32_t LE32ToHost32(const uint32_t x);
const uint32_t Host16ToLE16(const uint32_t x);
const uint32_t LE16ToHost16(const uint32_t x);
*/

// 4.3 DATA FIELDS

// single byte types
typedef uint8_t BoolPkt_t;
typedef uint8_t MsgType_t;
typedef uint8_t STX_t;
typedef uint8_t ETX_t;
typedef uint8_t PwrCycCout_t;
typedef uint8_t WDRstCount_t;
typedef uint8_t OthRSTCount_t;

// 4.3.1 Serial Number Field
//  4 byte (big endian) data field indicating serial number of Farmbeats device. Example:
//  Serial Number: “123J”
//  Serial Number Field Values: 0x31 0x32 0x33 0x4A
typedef uint32_t SerNoPkt_t;
typedef uint32_t SerNoHost_t;
SerNoPkt_t  SerNoHost2Pkt(const SerNoHost_t sn) { return Host32ToBE32(sn); }
SerNoHost_t SerNoPkt2Host(const SerNoPkt_t sn)  { return BE32ToHost32(sn); }

// 4.3.2 Timestamp Field
//  4 byte (little endian) data field indicating timestamp of message. Example:
//  Unix time value 0x02 (1/1/1970 00:00:02)
//  Timestamp Field Values: 0x02 0x00 0x00 0x00
typedef uint32_t TimeStampPkt_t;
typedef uint32_t TimeStampHost_t;
TimeStampPkt_t  TimeStampHost2Pkt(const TimeStampHost_t ts) { return Host32ToLE32(ts); }
TimeStampHost_t TimeStampPkt2Host(const TimeStampPkt_t ts)  { return LE32ToHost32(ts); }

// 4.3.3 Sample Data Data Field
//  8 2-byte ADC values, where each ADC value is little endian.
//  Example:
//  ADC0 = 0x0CCC, ADC1=0x0100, ADC2 = 0x0456, ADC3-ADC7 = 0x0000
//  Sample Data Data Field = 0xCC 0x0C 0x00 0x01 0x56 0x04 0x00 0x00 ... 0x00 0x00
typedef uint16_t ADCValPkt_t;
typedef uint16_t ADCValHost_t;
ADCValPkt_t  ADCValHost2Pkt(const ADCValHost_t val) { return Host16ToLE16(val); }
ADCValHost_t ADCValPkt2Host(const ADCValPkt_t val)  { return LE16ToHost16(val); }

typedef union {
  uint8_t raw[16];
  struct {
    ADCValPkt_t ADC0;
    ADCValPkt_t ADC1;
    ADCValPkt_t ADC2;
    ADCValPkt_t ADC3;
    ADCValPkt_t ADC4;
    ADCValPkt_t ADC5;
    ADCValPkt_t ADC6;
    ADCValPkt_t ADC7;
  } __attribute__((packed)) values;
  static_assert(sizeof(raw) == sizeof(values), "SampleData_t Bad Size");
} __attribute__((packed)) SampleData_t;

// 4.3.4 Sample Data Acknowledgement Interval Value Field 16-bit (little endian) Interval value.
//  Example:
//  Interval = 600 (0x258)
//  Sample Data Ack Interval Value Field = 0x58 0x02
typedef uint16_t IntervalValuePkt_t;
typedef uint16_t IntervalValueHost_t;
IntervalValuePkt_t  IntervalValueHost2Pkt(const IntervalValueHost_t  iv) { return Host16ToLE16(iv); }
IntervalValueHost_t IntervalValuePkt2Host(const IntervalValuePkt_t iv)   { return LE16ToHost16(iv); }

// 4.3.5 Sample Data Acknowledgement Time/Date Value Field 4 byte (little endian) data
//  field indicating timestamp.
//  Example:
//  Unix time value 0x02 (1/1/1970 12:00:02) ????
//  Timestamp Field Values: 0x02 0x00 0x00 0x00

// reusing definitions from 4.3.2

// 4.3.6 Health Report Data Field
//  Data from Health Report structure. 16-bit values are little endian. The structure is defined as:
//  uint8_t powerCycleCount;
//  uint8_t watchdogResetCount;
//  uint8_t otherResetCount;
//  uint16_t loRaResendCount;
//  uint16_t loRaWatchdogCount;
//  uint16_t maximumMessageCount;
//  Example:
//  Power Cycle Count = 1
//  Watchdog Reset Count = 3
//  Other Reset Count = 0
//  LoRa Resend Count = 100 (0x64)
//  LoRa Watchdog Count = 50 (0x32)
//  Maximum Message Count = 10 (0x0A)
//  Health Report Data Field Values: 0x01 0x03 0x00 0x64 0x00 0x32 0x00 0x0A 0x00
typedef uint16_t LORAResendCountPkt_t;
typedef uint16_t LORAResendCountHost_t;
LORAResendCountPkt_t  LORAResendCountHost2Pkt(const LORAResendCountHost_t rc) { return Host16ToLE16(rc); }
LORAResendCountHost_t LORAResendCountPkt2Host(const LORAResendCountPkt_t rc)  { return LE16ToHost16(rc); }

typedef uint16_t LORAWatchDogCountPkt_t;
typedef uint16_t LORAWatchDogCountHost_t;
LORAWatchDogCountPkt_t  LORAWatchDogCountHost2Pkt(const LORAWatchDogCountHost_t  wdc) { return Host16ToLE16(wdc); }
LORAWatchDogCountHost_t LORAWatchDogCountPkt2Host(const LORAWatchDogCountPkt_t wdc) { return LE16ToHost16(wdc); }

typedef uint16_t MaxMsgCountPkt_t;
typedef uint16_t MaxMsgCountHost_t;
MaxMsgCountPkt_t  MaxMsgCountHost2Pkt(const MaxMsgCountHost_t mmc)  { return Host16ToLE16(mmc); }
MaxMsgCountHost_t MaxMsgCountPkt2Host(const MaxMsgCountPkt_t mmc) { return LE16ToHost16(mmc); }

typedef union {
    uint8_t  raw[9];
    struct  {
      PwrCycCout_t           powerCycleCount;
      WDRstCount_t           watchdogResetCount;
      OthRSTCount_t          otherResetCount;
      LORAResendCountPkt_t   loRaResendCount;
      LORAWatchDogCountPkt_t loRaWatchdogCount;
      MaxMsgCountPkt_t     maximumMessageCount;
    } __attribute__((packed)) values;
    static_assert(sizeof(raw) == sizeof(values), "HealthReportData_t Bad Size");
} __attribute__((packed)) HealthReportData_t;

/*****************************/

// Packet Formats

// 4.1.1 Transmission from Farmbeats to Server – One Data Record
// This message will send sample data after each set of samples is taken.
struct  FB2Srv1DataRecordPkt {
  const STX_t     STXVAL     = 0x02;
  const ETX_t     ETXVAL     = 0x03;
  const MsgType_t MSGTYPEVAL = 0xA1;
  union {
    uint8_t raw[sizeof(STX_t) +
		sizeof(SerNoPkt_t) +
		sizeof(MsgType_t) +
		sizeof(TimeStampPkt_t) +
		sizeof(SampleData_t) +
		sizeof(ETX_t)];
    struct {
      STX_t           STX;
      SerNoPkt_t      SerNo;
      MsgType_t       MsgType;
      TimeStampPkt_t  TimeStamp;
      SampleData_t    Data;
      ETX_t           ETX;
    } __attribute__((packed)) values;
    static_assert(sizeof(raw) == sizeof(values), "Bad Sizes");
  } __attribute__((packed)) data;

  // forced inline to ensure this does not require stack space to call
  inline __attribute__((always_inline)) void setValues(TimeStampHost_t ts=0,
		 ADCValHost_t adc0=0, ADCValHost_t adc1=0, ADCValHost_t adc2=0, ADCValHost_t adc3=0,
		 ADCValHost_t adc4=0, ADCValHost_t adc5=0, ADCValHost_t adc6=0, ADCValHost_t adc7=0) {
    data.values.TimeStamp = TimeStampHost2Pkt(ts);
    data.values.Data.values.ADC0 = ADCValHost2Pkt(adc0);
    data.values.Data.values.ADC1 = ADCValHost2Pkt(adc1);
    data.values.Data.values.ADC2 = ADCValHost2Pkt(adc2);
    data.values.Data.values.ADC3 = ADCValHost2Pkt(adc3);
    data.values.Data.values.ADC4 = ADCValHost2Pkt(adc4);
    data.values.Data.values.ADC5 = ADCValHost2Pkt(adc5);
    data.values.Data.values.ADC6 = ADCValHost2Pkt(adc6);
    data.values.Data.values.ADC7 = ADCValHost2Pkt(adc7);
  }

  inline void setSerNo(SerNoHost_t sn) {
    data.values.SerNo = SerNoHost2Pkt(sn);
  }
  
  inline FB2Srv1DataRecordPkt() {
    data.values.STX = STXVAL; data.values.MsgType = MSGTYPEVAL; data.values.ETX=ETXVAL;
  }
} __attribute__((packed)); 
static_assert(sizeof(FB2Srv1DataRecordPkt) <= ID_PKT_MAX_MSG_SIZE, "Exceeds Max Packet Size");
//Where:
// STX = Start of Message = 0x02
// SerNo = 4-byte Serial Number of Farmbeats device [see Section 4.3.1 for more detail]
// MsgType = 0xA1 for Sample Data
// Timestamp = 4-byte unix timestamp (little endian) [see Section 4.3.2 for more detail]
// Data = 8 2-byte ADC values (little endian) [see Section 4.3.3 for more detail]
// ETX = End of Message = 0x03

/*****************************/

// 4.1.2 Transmission from Farmbeats to Server – Two Data Records
// In the India implementation, the LoRa module imposes a duty cycle between transmissions.
// This means that the RF retry logic has need to be reworked. To support more robust
// operations in India, this new “two data records” message has been introduced.
// This message will send sample data after each set of samples is taken, if more than one
// data record is stored in the Farmbeats device.
struct  FB2Srv2DataRecordPkt {
  const STX_t     STXVAL     = 0x02;
  const ETX_t     ETXVAL     = 0x03;
  const MsgType_t MSGTYPEVAL = 0xA2;
  STX_t           STX;
  SerNoPkt_t      SerNo;
  MsgType_t       MsgType;
  TimeStampPkt_t  TimeStamp1;
  SampleData_t    Data1;
  TimeStampPkt_t  TimeStamp2;
  SampleData_t    Data2;
  ETX_t           ETX;
} __attribute__((packed));
static_assert(sizeof(FB2Srv2DataRecordPkt) <= ID_PKT_MAX_MSG_SIZE, "Exceeds Max Packet Size");
//Where:
// STX = Start of Message = 0x02
// SerNo = 4-byte Serial Number of Farmbeats device [see Section 4.3.1 for more detail]
// MsgType = 0xA2 for Two Data Records Sample Data
// TStamp1 = 4-byte unix timestamp of most recent record (little endian)
// Data1 = 8 2-byte ADC values of most recent record (little endian)
// TStamp2 = 4-byte unix timestamp of 2nd most recent record (little endian)
// Data2 = 8 2-byte ADC values of 2nd most recent record (little endian)
// ETX = End of Message = 0x03

/*****************************/

// 4.1.3 Transmission from Server to Farmbeats
// This message will acknowledge the receipt of the sample data from the Farmbeats device.
// Additionally, this message can update the Farmbeats time/date information, can change
// the Farmbeats reporting interval, or can reset the Farmbeats device.
struct Srv2FBAckAndConfigPkt {
  const STX_t        STXVAL = 0x02;
  const ETX_t        ETXVAL = 0x03;
  const MsgType_t    MSGTYPEVAL = 0xA2;
  STX_t              STX;
  SerNoPkt_t         SerNo;
  MsgType_t          MsgType;
  TimeStampPkt_t     TimeStamp;
  BoolPkt_t          Rst;
  BoolPkt_t          Hlth;
  BoolPkt_t          Intvl;
  IntervalValuePkt_t IntVal;
  BoolPkt_t          TD;
  TimeStampPkt_t     TDVal;
  ETX_t              ETX;
} __attribute__((packed));
static_assert(sizeof(Srv2FBAckAndConfigPkt) <= ID_PKT_MAX_MSG_SIZE, "Exceeds Max Packet Size");
//Where:
// STX = Start of Message = 0x02
// SerNo = 4-byte Serial Number of Farmbeats device from the sample data message [4.3.1 ]
// MT = 0xA2 for Sample Data acknowledgement
// TS = 4-byte unix timestamp from the sample data message (little endian) [see Section 4.3.2 ]
// Rst = 1 to instruct Farmbeats device to reset, 0 otherwise
// Hlth = 1 to request Health Report to be sent, 0 otherwise
// Intvl = 1 to instruct Farmbeats device to change interval value
// IntVal = new 2-byte interval value (in 10msec intervals) [see Section 4.3.4 for more detail]
// TD = 1 to instruct Farmbeats device to update time/date
// TDVal = new 4-byte unix timestamp [see Section 4.3.5 for more detail]
// ETX = End of Message = 0x03

/*****************************/

// 4.2.1 Transmission from Farmbeats to Server
// This message will send health report when requested by a sample data acknowledgement.
struct FB2SrvHealthReportPkt {
  const STX_t        STXVAL = 0x02;
  const ETX_t        ETXVAL = 0x03;
  const MsgType_t    MSGTYPEVAL = 0xA5;
  STX_t              STX;
  SerNoPkt_t         SerNo;
  MsgType_t          MsgType;
  TimeStampPkt_t     Timestamp;
  HealthReportData_t Data;
  ETX_t  ETX;
} __attribute__((packed));
static_assert(sizeof(FB2SrvHealthReportPkt) <= ID_PKT_MAX_MSG_SIZE, "Exceeds Max Packet Size");
//Where:
// STX = Start of Message = 0x02
// SerNo = 4-byte Serial Number of Farmbeats device [see Section 4.3.1 for more detail]
// MsgType = 0xA5 for Sample Data
// Timestamp = 4-byte unix timestamp (little endian) [see Section 4.3.2 for more detail]
// Data = 9 data bytes [see Section 4.3.6 for more detail]
// ETX = End of Message = 0x03

/*****************************/

// 4.2.2 Transmission from Server to Farmbeats
// This message will acknowledge the receipt of the health report from the Farmbeats device.
struct Srv2FBHealthReportAckPkt {
  const STX_t     STXVAL     = 0x02;
  const ETX_t     ETXVAL     = 0x03;
  const MsgType_t MSGTYPEVAL = 0xA6;
  uint8_t         STX;
  SerNoPkt_t      SerNo;
  uint8_t         MsgType;
  uint32_t        Timestamp;
  uint8_t         ETX;
} __attribute__((packed));
static_assert(sizeof(Srv2FBHealthReportAckPkt) <= ID_PKT_MAX_MSG_SIZE, "Exceeds Max Packet Size");
/*****************************/

#endif  // __ID_PKT_H__
