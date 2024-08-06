#include "ci24r1.hpp"
#include <Arduino.h>

sdf
#define CI24R1_MODE 1

uint8_t TX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01};

uint8_t RX_ADDRESS[5]={0x01,0x43,0x10,0x10,0x34};
char temp1[100];

uint8_t tmp[] = {
        0x1F, 0x80, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
        0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
        0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
        0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x37, 0x48};

        
extern uint8_t *xbuf_data;
extern uint8_t xbuf[CI24R1_PLOAD_MAX_WIDTH + 1];

void setup() {
  Serial.begin(9600);
   pinMode(CI24R1_SCK, OUTPUT);
  pinMode(CI24R1_CSN, OUTPUT);

  uint8_t i, j, status;

   while (CI24R1_SPI_Test() == 1)
   {
       Serial.println(" - check failed\r\n");
   }

    Serial.println(" - check passed\r\n");
    delay(2000);
    CI24R1_Init();

    CI24R1_SetChannel(60);
  CI24R1_SetRxMode();
  CI24R1_SetTxAddress(RX_ADDRESS);
  CI24R1_SetRxAddress(TX_ADDRESS);
  Serial.println("CI24R1 RX Initialized\r\n");
}

void loop() {
  
  

  
  //CI24R1_PrintStatus();
  CI24R1_Rx();
  sprintf(temp1,"recieved-buff[0] = %d",xbuf[0]);
  Serial.println("\r\n");
  Serial.println(temp1);
  delay(100);
        
}
