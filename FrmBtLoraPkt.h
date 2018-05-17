#ifndef __FRMBT_LORA_PKT_H__
#define __FRMBT_LORA_PKT_H__

//  This header file defines the FarmBeats India specific
//  Lora PHYPayload structure that we use.
//  To be compatible with LoRaWan 1.1  we comply to the MAC header word
//  that must be the first byte of the payload
//  specifically we use the Major value of 00 to indicate LoRaWan 1.1
//  but then use an MType value of Proprietary 111
//  which then allows us to use a custom format for the rest of the payload
//  Following is the docs used to make this decision

//  From LoRaWan specification v1.1
//  LoRaWan networks use a PHY payload structure as follows
//  
// 4 MAC Message Formats
//  All LoRa uplink and downlink messages carry a PHY payload (Payload)
//  starting with a single-octet MAC header (MHDR), followed by a MAC payload
//  (MACPayload)1, and ending with a 4-octet message integrity code (MIC).
//
//  In section 4.2 the HMDR is define to be a 1 byte word broken into
//  3 bit fields as follows
//  0:1  Major Type
//  4:2  Reserved
//  5:7  Message Type
//
//  4.2.2  Major version of data message (Major Type field)
//    says that 00 indicates LoRaWan R1
//    and that 01,10 and 11 are Reseved for future use
//
//  4.2.1 Message type (Mtype bit field)
// 451 The LoRaWAN distinguishes between 8 different MAC message types:
//     Join-request,
// 452 Rejoin-request, Join-accept, unconfirmed data up/down, and confirmed
//     data up/down
// 453 and proprietary protocol messages.
//   MType              Description
//    000               Join-request
//    001               Join-accept
//    010               Unconfirmed Data Up
//    011               Unconfirmed Data Down 
//    100               Confirmed Data Up 
//    101               Confirmed Data Down 
//    110               Rejoin-request
//    111               Proprietary

union LORA_PHY_MHDR {
    uint8_t raw;
    struct  {
      uint8_t MajorType:2,
	Reserved:3,
	MsgType:3;
    } __attribute__((packed)) bits;
} __attribute__((packed));  


const union LORA_PHY_MHDR FB_LORA_MHDR = { 
  .bits={
    .MajorType=0x0,
    .Reserved=0x0,
    .MsgType=0x7
  }
};
		
#endif
	       
