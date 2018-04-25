#include "sx127x_lora.h"

//#define DUMP_PACKETS
//                mosi,      miso,     sclk,       cs,        rst,      dio0,      dio1
// SX127x(PinName dio0, PinName dio_1, PinName cs, SPI&, PinName rst);
//SX127x radio(LORA_MOSI, LORA_MISO, LORA_SCK, LORA_NSS, LORA_RESET, LORA_DIO0, LORA_DIO1);
SPI spi(LORA_MOSI, LORA_MISO, LORA_SCK);
SX127x radio(LORA_DIO0, LORA_DIO1, LORA_NSS, spi, LORA_RESET);
SX127x_lora lora(radio);

DigitalOut txctl(LORA_TXCTL);
DigitalOut rxctl(LORA_RXCTL);

Serial pc(USBTX, USBRX);

void rfsw_callback()
{
    /* SKY13350 */
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {  // start of transmission
        txctl = 1;
        rxctl = 0;
    } else { // reception:
        txctl = 0;
        rxctl = 1;
    }
}

/**********************************************************************/

char waitForKey(char *msg) 
{
    printf("%s : \r\n\t Press any key to continue\r\n", msg);
    return getchar();
}

long sx1272_bandwidth(uint8_t bits) 
{
    switch(bits) {
        case 0: return 125E3;  
        case 1: return 250E3;
        case 2: return 500E3;
        case 3: return -1;
    }
    return -1;
}

int sx1272_codingRateDenom(uint8_t bits) {
    switch(bits) {
        case 1: return 5;
        case 2: return 6;
        case 3: return 7;
        case 4: return 8;
        default: return -1;
    }

}

void info()
{
   printf("INFO: radio.type=%d", radio.type);
   printf("\r\n\t\tversion=0x%02x", radio.read_reg(0x42));
   printf("\r\n\t\tRegOpMode=0x%02x", radio.read_reg(0x01));
   printf("\r\n\t\tfreq_=%f", radio.get_frf_MHz());
   printf("\r\n\t\tbandwidth_=%d reg=%d", sx1272_bandwidth(lora.getBw()), lora.getBw());
   printf("\r\n\t\tspreadingFactor_=%d", lora.getSf());
   printf("\r\n\t\ttxPower_=? REG_PA_CONFIG=0x%02x", radio.read_reg(0x09));
   printf("\r\n\t\tLNA=0x%02x", radio.read_reg(0x0c));
   printf("\r\n\t\tcodingRateDenom_=%d reg=%d", sx1272_codingRateDenom(lora.getCodingRate(false)), lora.getCodingRate(false));
   printf("\r\n\t\tpreambleLength_=%d",(radio.read_reg(0x20)<<8 | radio.read_reg(0x21)));
   printf("\r\n\t\tsyncWord_=0x%02x", radio.read_reg(0x39));
   printf("\r\n\t\tcrc_=%d", lora.getRxPayloadCrcOn());
   printf("\r\n\t\timplicitHeaderMode=%d\r\n", lora.getHeaderMode());
}

void setTxPower(int val) {
      radio.RegPaConfig.bits.OutputPower = 0xf & val;    
      radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
}

int 
hexdump(unsigned char *start, int bytes) 
{
  int j;
  int i;
  for (j=0; j<bytes; ) {
    if ((j%16) == 0) printf("\t%x: ", j);
    for (i=0;(i<16) && ((j+i) < bytes);i++) {
      printf("%x ", start[j+i]);
    }
    printf("|");
    for (i=0;(i<16) && ((j+i) < bytes);i++) {
      unsigned char c=start[j+i];
      if (c>=' ' && c<='~')  printf("%c", c);
      else  printf(".");
    }
    printf("|\r\n");
    j+=i;
  }
  return j;
}


int main()
{    
    pc.baud(115200);
    
    pc.printf("\r\nFrmBtmDot 0.2.8\r\n");
    
    radio.rf_switch.attach(rfsw_callback);
    
    radio.set_frf_MHz(868.85);
    lora.enable();
    lora.setBw_KHz(125);
    setTxPower(7);
    // lora.setCodingRate();
    /* Transmit power default set to 6 given testing in short ranges maybe 
       more stable.
       Expore this and set value for field as needed. */
    lora.setSf(7);
    lora.setRxPayloadCrcOn(true);

    /* RFO or PABOOST choice:     */
    radio.RegPaConfig.bits.PaSelect = 1;    // mDot connects PA_BOOST
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
                   
    info();
    
  
    char buf[20];
    int idx=0;
    int txCnt=0;
    int rxCnt=0;
    char hdr[80];
    int myId=0;
    int msNextSend=1000;
    Timer tmr;
    
    tmr.start();
    // start in continous receive mode
    lora.start_rx(RF_OPMODE_RECEIVER);
    
    for (;;) {
         
         if (lora.service() == SERVICE_READ_FIFO) {
            rxCnt += lora.RegRxNbBytes;
#ifdef DUMP_PACKETS            
            /* dump sent data */
            {
              printf("%d:<%d[%d] rssi:%d(%d) snr:%.f(%d)\r\n", myId, 
                     lora.RegRxNbBytes, rxCnt, 
                     lora.get_pkt_rssi(), lora.RegPktRssiValue, 
                     lora.RegPktSnrValue * 0.25, lora.RegPktSnrValue);
              hexdump((unsigned char *)radio.rx_buf, lora.RegRxNbBytes);
            } 
#endif            
            {
               char *pktBuffer=(char *)radio.rx_buf;
               int packetSize = lora.RegRxNbBytes;
               int payloadIdx=0;
               // skip header
               for (int i=0; i<2; i++) {
                 while (pktBuffer[payloadIdx]!=',' && payloadIdx < packetSize) { payloadIdx++;}
                 payloadIdx++;
               }
               if (payloadIdx < packetSize) {
                 write(1,&pktBuffer[payloadIdx],packetSize-payloadIdx);
               }
            }
        }
        
        if (pc.readable()) {
            buf[idx]=getchar();
            idx++;
        }
        
        if (idx && (tmr.read_ms() >= msNextSend)) {
          int hb = snprintf(hdr, sizeof(hdr), "%d,%d,", myId, idx);
          int n = hb + idx;
          if (n<=sizeof(radio.tx_buf)) {
            lora.RegPayloadLength = n;
            radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
            memcpy(radio.tx_buf, hdr, hb);
            memcpy(&radio.tx_buf[hb], buf, idx);
            
            /* begin transmission */    
            lora.start_tx(lora.RegPayloadLength);   
        
            /* wait for transmission to complete */
            while (lora.service() != SERVICE_TX_DONE); 
            
            /* done sending go back to continous receive mode */
            lora.start_rx(RF_OPMODE_RECEIVER);
            
            txCnt += lora.RegPayloadLength;
#ifdef DUMP_PACKETS            
            /* dump sent data */
            { 
              printf("%d:>%d[%d]\r\n", myId,lora.RegPayloadLength, txCnt);
              hexdump(radio.tx_buf, lora.RegPayloadLength);
            }
#endif            
            idx=0;
            msNextSend=1000;
            tmr.reset();
          }
       }
       
      
    }
}

