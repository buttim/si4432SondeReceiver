#include <SPI.h>
//#include <RadioLib.h>
#include <CRC.h>
#include <navduino.h>
#include "Si4432.h"

const pin_size_t cs=1, sdn=5, irq=4, miso=0, mosi=3, sck=2, button=14;
const int PACKET_LENGTH=316, CHUNK_SIZE=53;

SPIClassRP2040 spi(spi0,miso,cs,sck,mosi);
SPISettings spiSettings(1e6,MSBFIRST,SPI_MODE0);
//Module m(cs,irq,-1,-1,spi,spiSettings);
//Si4432 radio(&m);
int nBytes=0;
uint8_t buf[PACKET_LENGTH]={};
uint8_t mask[] = {
    0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98,
    0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
    0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
    0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
    0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
    0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
    0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
    0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1 };

uint8_t Si44xxWriteReg(int i,uint8_t val) {
  spi.beginTransaction(SPISettings(8e6,MSBFIRST,SPI_MODE0));
  digitalWrite(cs,LOW);
  spi.transfer(0x80|i);
  int res=spi.transfer(val);
  digitalWrite(cs,HIGH);
  spi.endTransaction();
  return res;
}

uint8_t Si44xxReadReg(int i) {
  spi.beginTransaction(SPISettings(8e6,MSBFIRST,SPI_MODE0));
  digitalWrite(cs,LOW);
  spi.transfer(i);
  int res=spi.transfer(0);
  digitalWrite(cs,HIGH);
  spi.endTransaction();
  return res;
}

void dumpRegisters() {
  for (int i=0;i<0x7F;i++) {
    if (i%16==0) Serial.printf("\n%02X: ",i);
    else if (i%16==8) Serial.print("-");
    Serial.printf("%02X ",Si44xxReadReg(i));
  }
  Serial.println();
}

bool dump=false;

void f(void) {
  dump=true;
}

void radioIRQ(void) {
  //Serial.println("***");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("VIA");

  pinMode(button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button),f,FALLING);

  pinMode(irq,INPUT);
  pinMode(cs,OUTPUT);
  pinMode(sdn,OUTPUT);
  digitalWrite(sdn,LOW);

  spi.begin();
  /*int res=radio.begin(405.95);
  if (res!=RADIOLIB_ERR_NONE) {
    while (true) {
      Serial.println("Errore inizializzazione Si4432");
      delay(1000);
    }
  }*/
  Serial.printf("radio ok (type: %02X,version: %02X)\n",Si44xxReadReg(0),Si44xxReadReg(1));

  Si44xxWriteReg(0x07, 0x80); //software reset
  delay(100);

  uint8_t syncWord[]= { 0x10, 0xB6, 0xCA, 0x11 };
  for (int i=0;i<4;i++)
    Si44xxWriteReg(0x36+i,syncWord[i]);
  Si44xxWriteReg(0x0D,SI443X_GPIOX_SYNC_WORD_DETECTED_OUT);//GPIO2: Sync word detected
  Si44xxWriteReg(0x1C,0x96);//IF filter bandpass (come da radiolib)
  Si44xxWriteReg(0x20,0xE2);//(come da radiolib)
  Si44xxWriteReg(0x21,0x80);//(come da radiolib)
  Si44xxWriteReg(0x22,0x1A);//(come da radiolib)
  Si44xxWriteReg(0x23,0x36);//(come da radiolib)
  Si44xxWriteReg(0x24,0x00);//(come da radiolib)
  Si44xxWriteReg(0x25,0x21);//(come da radiolib)
  Si44xxWriteReg(0x2A,0x50);//(come da radiolib)
  Si44xxWriteReg(0x69,0x60);//(come da radiolib)
  Si44xxWriteReg(0x72,0x08);//(come da radiolib)
  Si44xxWriteReg(0x30,SI443X_LSB_FIRST_ON);//disabilita CRC e packet handling
  Si44xxWriteReg(0x32,0x00);//check 0 address bytes
  Si44xxWriteReg(0x33,0b00001110);//header length=0,no packet length in header,sync word length=4
  Si44xxWriteReg(0x34,80);//preamble length: 80 nibbles
  Si44xxWriteReg(0x35,(15<<3)|0b010);//preamble length threshold (rssi_offset=8dB, default)
  Si44xxWriteReg(0x70,0b00100000);//txdtrtscale, no data whitening
  Si44xxWriteReg(0x71,0b00100011);//Modulation GFSK (forse 0x22?)

  Si44xxWriteReg(0x75,0x50);//Frequency band select
  Si44xxWriteReg(0x76,0x94);//Nominal carrier frequency 1 (405.95)
  Si44xxWriteReg(0x77,0xC0);//Nominal carrier frequency 0 (405.95)
  
  Si44xxWriteReg(0x7E,CHUNK_SIZE-1);//RX FIFO almost full threshold
  
  //dumpRegisters();

  attachInterrupt(digitalPinToInterrupt(irq),radioIRQ,FALLING);

  startReceive();
}

void startReceive() {
  Si44xxWriteReg(SI443X_REG_OP_FUNC_CONTROL_1, SI443X_XTAL_ON);
  Si44xxWriteReg(SI443X_REG_OP_FUNC_CONTROL_2, SI443X_RX_FIFO_RESET);
  Si44xxWriteReg(SI443X_REG_OP_FUNC_CONTROL_2, SI443X_RX_FIFO_CLEAR);
  Si44xxReadReg(SI443X_REG_INTERRUPT_STATUS_1);
  Si44xxReadReg(SI443X_REG_INTERRUPT_STATUS_2);
  Si44xxWriteReg(SI443X_REG_INTERRUPT_ENABLE_1, SI443X_RX_FIFO_ALMOST_FULL_ENABLED);
  Si44xxWriteReg(SI443X_REG_INTERRUPT_ENABLE_2, SI443X_SYNC_WORD_DETECTED_ENABLED);
  Si44xxWriteReg(SI443X_REG_OP_FUNC_CONTROL_1, SI443X_RX_ON | SI443X_XTAL_ON);
}

void loop() {
  int i;
  static uint32_t tLast=0;

  //delay(100);
  if (dump) {
    dump=false;
    Serial.print("nBytes=");
    Serial.print(nBytes);
    Serial.print(" reg3=");
    Serial.print(Si44xxReadReg(3),HEX);
    Serial.print(" reg4=");
    Serial.print(Si44xxReadReg(4),HEX);
    for (int i=0;i<128;i++) {
      if (i%16==0) Serial.printf("\n0x%03X: ",i);
      Serial.printf("%02X ",buf[i]^mask[(i+4)%sizeof mask]);
    }
    Serial.println();
  }

  if (millis()-tLast>1200) {
    nBytes=0;
    startReceive();
    tLast=millis();
    Serial.println("*****************");
    return;
  }

  int is2=Si44xxReadReg(4), //interrupt/status 2
    is1=Si44xxReadReg(3); //interrupt/status 1

  if (0!=(is2&0b10000000)) nBytes=0;  //sync word detected
  if (0==(is1&0b00010000)) return;
  for (i=0;i<CHUNK_SIZE;i++)
    buf[nBytes++]=Si44xxReadReg(0x7F);//FIFO access
  if (nBytes<PACKET_LENGTH) return;
  nBytes=0;
  tLast=millis();

  startReceive();

  for (i=0;i<PACKET_LENGTH;i++) buf[i]^=mask[(i+4)%sizeof mask];  //whitening
  /*Serial.print("\nRSEC: ");
  for (i=4;i<4+48;i++) 
    Serial.printf("%02X ",buf[4+i]);*/
  Serial.printf("\nFrame: %02X ",buf[4+48]);
  i=4+48+1;
  while (i<PACKET_LENGTH) {
    int blockType=buf[i++],
      len=buf[i++];

    Vector3f v;
    double x, y, z;

    uint16_t crc=calcCRC16(buf+i,len,CRC16_CCITT_FALSE_POLYNOME,CRC16_CCITT_FALSE_INITIAL,CRC16_CCITT_FALSE_XOR_OUT,CRC16_CCITT_FALSE_REV_IN,CRC16_CCITT_FALSE_REV_OUT);
    if (buf[i+len]+256*buf[i+len+1]!=crc)
      Serial.printf("CRC err %02X%02X %04X\n",buf[i+1],buf[i],crc);
    else
      switch (blockType) {
        case 0x79://status
          Serial.printf("%.8s Frame# %d\n",buf+i+2,buf[i]+256*buf[i+1]);
          break;
        case 0x7B:
          x=buf[i  ]+256*(buf[i+1]+256*(buf[i+ 2]+256*buf[i+ 3]))/100.0;
          y=buf[i+4]+256*(buf[i+5]+256*(buf[i+ 6]+256*buf[i+ 7]))/100.0;
          z=buf[i+8]+256*(buf[i+9]+256*(buf[i+10]+256*buf[i+11]))/100.0;
          v=Vector3f(x,y,z);
          v=ecef2lla(v);
          Serial.printf("lat:%f lon:%f h:%f\n",v.x(),v.y(),v.z());
          break;
        default:
          Serial.printf("block %02X (%d bytes) ",blockType,len);
          for (int j=0;j<len;j++)
            Serial.printf("%02X ",buf[i+j]);
          Serial.println();
          break;
      }
    i+=len+2;
  }
}
