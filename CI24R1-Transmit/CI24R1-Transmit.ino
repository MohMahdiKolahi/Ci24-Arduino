#include "ci24r1.hpp"
#include <Arduino.h>

// 0:TX, 1:RX
#define CI24R1_MODE 0
uint8_t TX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01};

uint8_t RX_ADDRESS[5]={0x01,0x43,0x10,0x10,0x34};


uint8_t tmp[] = {
        0x1F, 0x80, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
        0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
        0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
        0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x37, 0x48};

        
extern uint8_t *xbuf_data;

void setup() {
  Serial.begin(9600);
  pinMode(CI24R1_SCK, OUTPUT);
  pinMode(CI24R1_CSN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
    uint8_t i, j, status;

   while (CI24R1_SPI_Test() == 1)
   {
       Serial.println(" - check failed\r\n");
   }

    Serial.println(" - check passed\r\n");
    delay(2000);
    CI24R1_Init();
    
}

void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  // delay(1000);                       // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  // delay(1000);  
  
if (CI24R1_MODE == 0)
    {
        // TX
        CI24R1_SetChannel(60);
        CI24R1_SetTxMode();
        CI24R1_SetTxAddress(TX_ADDRESS);
        CI24R1_SetRxAddress(RX_ADDRESS);
        Serial.println("CI24R1 TX Initialized\r\n");

        
            CI24R1_PrintStatus();
            CI24R1_Tx(tmp, CI24R1_PLOAD_WIDTH);
            delay(500);
        
    }
    
    
}
