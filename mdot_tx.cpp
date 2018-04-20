/*  NOTE THE MDOT uses sx1272 which is different from the sx1278 that the Indian Farm Beats sensor box uses */


/* 
This code was derived from https://os.mbed.com/users/dudmuck/code/sx127x_simple_RX_mDot/
which uses the following library
https://os.mbed.com/users/dudmuck/code/SX127x/
*/

#include "sx127x_lora.h"

//                mosi,      miso,     sclk,       cs,        rst,      dio0,      dio1
SX127x radio(LORA_MOSI, LORA_MISO, LORA_SCK, LORA_NSS, LORA_RESET, LORA_DIO0, LORA_DIO1);
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
int main()
{
    uint8_t seq = 'a';
    
    pc.baud(115200);
    
    pc.printf("\r\nsx127x_simple_TX_mDot!! 0.1.0\r\n");
    
    radio.rf_switch.attach(rfsw_callback);
    
    radio.set_frf_MHz(868.0);
    lora.enable();
    lora.setBw_KHz(125);
    // lora.setCodingRate();
    lora.setSf(7);
    lora.setRxPayloadCrcOn(false);

    /* RFO or PABOOST choice:     */
    radio.RegPaConfig.bits.PaSelect = 1;    // mDot connects PA_BOOST
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
                
    /* constant payload length of one byte */
    lora.RegPayloadLength = 1;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
    
    info();
    waitForKey("info done");
    
    char key=0;
    int count=0;
    for (;;) {
        if (key != 'c') {
          key=waitForKey("Send loop top: press c to do continues sends");
        }       
        printf("Sending: %c\r\n", seq);
        radio.tx_buf[0] = seq;  /* set payload */
        lora.start_tx(lora.RegPayloadLength);   /* begin transmission */
        
        while (lora.service() != SERVICE_TX_DONE)   /* wait for transmission to complete */
            ;
        count++;
        printf("sent: %c count: %d bytes: %d\r\n", seq, count, count*lora.RegPayloadLength);
        radio.set_opmode(RF_OPMODE_STANDBY);
        wait(1.0);  /* throttle sending rate */
        seq++;  /* change payload */
        if (seq > 'z') seq = 'a';
    }
}

