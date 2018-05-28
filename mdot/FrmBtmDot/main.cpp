#include "sx127x_lora.h"

#define DUMP_PACKETS
//                mosi,      miso,     sclk,       cs,        rst,      dio0,      dio1
// SX127x(PinName dio0, PinName dio_1, PinNa>me cs, SPI&, PinName rst);
//SX127x radio(LORA_MOSI, LORA_MISO, LORA_SCK, LORA_NSS, LORA_RESET, LORA_DIO0, LORA_DIO1);
SPI spi(LORA_MOSI, LORA_MISO, LORA_SCK);
SX127x radio(LORA_DIO0, LORA_DIO1, LORA_NSS, spi, LORA_RESET);
SX127x_lora lora(radio);

DigitalOut txctl(LORA_TXCTL);
DigitalOut rxctl(LORA_RXCTL);

Serial dbgSerial(USBTX, USBRX);

class MySerial : public Serial {
    public:
    void  handleWrite(char *buf, int len) {
       int i=0;
       //dbgSerial.printf("\r\nhandleWrite(buf=0x%p, len=%d)\r\n", buf, len);
       while (i<len)  {
        if (Serial::writeable()) {
            //attach(NULL, Serial::RxIrq);
            putc(buf[i]); 
            i++;
            //dbgSerial.printf("%d ",i);
            //attach(this, &ATSerial::handleRead, Serial::RxIrq);
        }
       }
     }
     MySerial(PinName tx, PinName rx) : Serial(tx,rx) {}
};

MySerial dataSerial(SERIAL_TX, SERIAL_RX);

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
   dbgSerial.printf("INFO: radio.type=%d", radio.type);
   dbgSerial.printf("\r\n\t\tversion=0x%02x", radio.read_reg(0x42));
   dbgSerial.printf("\r\n\t\tRegOpMode=0x%02x", radio.read_reg(0x01));
   dbgSerial.printf("\r\n\t\tfreq_=%f", radio.get_frf_MHz());
   dbgSerial.printf("\r\n\t\tbandwidth_=%d reg=%d", sx1272_bandwidth(lora.getBw()), lora.getBw());
   dbgSerial.printf("\r\n\t\tspreadingFactor_=%d", lora.getSf());
   dbgSerial.printf("\r\n\t\ttxPower_=? REG_PA_CONFIG=0x%02x", radio.read_reg(0x09));
   dbgSerial.printf("\r\n\t\tLNA=0x%02x", radio.read_reg(0x0c));
   dbgSerial.printf("\r\n\t\tcodingRateDenom_=%d reg=%d", sx1272_codingRateDenom(lora.getCodingRate(false)), lora.getCodingRate(false));
   dbgSerial.printf("\r\n\t\tpreambleLength_=%d",(radio.read_reg(0x20)<<8 | radio.read_reg(0x21)));
   dbgSerial.printf("\r\n\t\tsyncWord_=0x%02x", radio.read_reg(0x39));
   dbgSerial.printf("\r\n\t\tcrc_=%d", lora.getRxPayloadCrcOn());
   dbgSerial.printf("\r\n\t\timplicitHeaderMode=%d\r\n", lora.getHeaderMode());
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
      dbgSerial.printf("%x ", start[j+i]);
    }
    dbgSerial.printf("|");
    for (i=0;(i<16) && ((j+i) < bytes);i++) {
      unsigned char c=start[j+i];
      if (c>=' ' && c<='~')  dbgSerial.printf("%c", c);
      else  dbgSerial.printf(".");
    }
    dbgSerial.printf("|\r\n");
    j+=i;
  }
  return j;
}

event_callback_t cb;

#ifdef DUMP_PACKETS
int dump = 0;
#endif

int main()
{    
    dbgSerial.baud(115200);
    dataSerial.baud(115200);
       
    dbgSerial.printf("\r\nFrmBtmDot 0.3.0 raw with dump\r\n");
    
    radio.rf_switch.attach(rfsw_callback);
    
    radio.set_frf_MHz(866.3);
    lora.enable();
    lora.setBw_KHz(125);
    setTxPower(11);
    // lora.setCodingRate();
    /* Transmit power default set to 6 given testing in short ranges maybe 
       more stable.
       Expore this and set value for field as needed. */
    lora.setSf(9);
    lora.setRxPayloadCrcOn(true);

    /* RFO or PABOOST choice:     */
    radio.RegPaConfig.bits.PaSelect = 1;    // mDot connects PA_BOOST
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
                   
    info();
    
#ifdef DUMP_PACKETS
    dbgSerial.printf("press 'd' to toggle dumping of packets "
     "(DUMP OFF)\r\n");   
#endif 

    const int buflen=128;
    char buf[buflen];
    int idx=0;
    int txCnt=0;
    int rxCnt=0;

    int myId=0;    
    int msNextSend=1000;
    Timer tmr;
    
    tmr.start();
    // start in continous receive mode
    lora.start_rx(RF_OPMODE_RECEIVER);
    
    for (;;) {
         
         if (lora.service() == SERVICE_READ_FIFO) {
            rxCnt += lora.RegRxNbBytes;
            dataSerial.handleWrite((char *)radio.rx_buf, lora.RegRxNbBytes);
#ifdef DUMP_PACKETS            
            /* dump received data */
            if (dump) {
              dbgSerial.printf("%d:<%d[%d] rssi:%d(%d) snr:%.f(%d)\r\n", myId,  
                     lora.RegRxNbBytes, rxCnt, 
                     lora.get_pkt_rssi(), lora.RegPktRssiValue, 
                     lora.RegPktSnrValue * 0.25, lora.RegPktSnrValue);
              hexdump((unsigned char *)radio.rx_buf, lora.RegRxNbBytes);
            } 
#endif            
        }
        
        if (dataSerial.readable()) {
            // copy to buffer if there is space
            if (idx < buflen) {
                buf[idx]=dataSerial.getc();
                idx++;
            } else {
                dbgSerial.printf("WARNING: Serial Data avaiable but buffer"
                " full\n");
            }
        }
        
        if (idx && (tmr.read_ms() >= msNextSend)) {
          int n = idx;
                
          if (n<=sizeof(radio.tx_buf)) {
            lora.RegPayloadLength = n;
            radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);

            memcpy(radio.tx_buf, buf, idx);
                       
            /* begin transmission */    
            lora.start_tx(lora.RegPayloadLength);   
        
            /* wait for transmission to complete */
            while (lora.service() != SERVICE_TX_DONE); 
            
            /* done sending go back to continous receive mode */
            lora.start_rx(RF_OPMODE_RECEIVER);
            
            txCnt += lora.RegPayloadLength;
#ifdef DUMP_PACKETS            
            /* dump sent data */
            if (dump) { 
              dbgSerial.printf("%d:>%d[%d]\r\n", myId, lora.RegPayloadLength, txCnt);
              hexdump(radio.tx_buf, lora.RegPayloadLength);
            }
#endif            
            idx=0;
            msNextSend=100;
            tmr.reset();
          }
       }
       
#ifdef DUMP_PACKETS  
      if (dbgSerial.readable()) {
          char c=dbgSerial.getc();
          if (c=='d') { 
            if (dump) {
              dump=0; 
              dbgSerial.printf("DUMP OFF\r\n");
            } else {
              dump=1;
              dbgSerial.printf("DUMP ON\r\n");
            }  
         }
      }
#endif      
    }
}
