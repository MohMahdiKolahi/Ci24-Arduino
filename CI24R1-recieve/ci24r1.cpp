// Copyright 2021 IOsetting <iosetting(at)outlook.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ci24r1.hpp"
#include <Arduino.h>


uint8_t cbuf[2], xbuf[CI24R1_PLOAD_MAX_WIDTH + 1];
uint8_t *xbuf_data = xbuf + 1;

uint8_t HAL_ERROR=1;
uint8_t HAL_OK=0;
char temp[100];

void CI24R1_WriteByte(uint8_t value)
{
    uint8_t i = 0;
    CI24R1_CLK_LOW();
    CI24R1_DATA_OUT();
    for (i = 0; i < 8; i++)
    {
        CI24R1_CLK_LOW();
        if (value & 0x80)
        {
            CI24R1_DATA_HIGH();
        }
        else
        {
            CI24R1_DATA_LOW();
        }
        CI24R1_CLK_HIGH();
        value = value << 1;
    }
    CI24R1_CLK_LOW();
}

uint8_t CI24R1_ReadByte(void)
{
    uint8_t i = 0, RxData;

    CI24R1_DATA_IN();
    CI24R1_CLK_LOW();
    for (i = 0; i < 8; i++)
    {
        RxData = RxData << 1;
        CI24R1_CLK_HIGH();
        if (CI24R1_DATA_READ())
        {
            RxData |= 0x01;
        }
        else
        {
            RxData &= 0xfe;
        }
        CI24R1_CLK_LOW();
    }
    CI24R1_CLK_LOW();
    return RxData;
}

void CI24R1_WriteReg(uint8_t reg,uint8_t value)
{
	CI24R1_NSS_LOW();					
	CI24R1_WriteByte(reg);
	CI24R1_WriteByte(value);
	CI24R1_NSS_HIGH();
}

uint8_t CI24R1_ReadReg(uint8_t reg)
{
    uint8_t reg_val;
    CI24R1_NSS_LOW();
    CI24R1_WriteByte(reg);
    reg_val = CI24R1_ReadByte();
    CI24R1_NSS_HIGH();
    return reg_val;
}

void CI24R1_WriteCmd(uint8_t cmd)
{
	CI24R1_NSS_LOW();					
	CI24R1_WriteByte(cmd);
	CI24R1_NSS_HIGH();
}

void CI24R1_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    CI24R1_NSS_LOW();
    CI24R1_WriteByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        CI24R1_WriteByte(*pBuf++);
    }
    CI24R1_NSS_HIGH();
}

void CI24R1_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    CI24R1_NSS_LOW();
    CI24R1_WriteByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        pBuf[ctr] = CI24R1_ReadByte();
    }
    CI24R1_NSS_HIGH();

}

void CI24R1_SetTxMode(void)
{
    uint8_t value;
    value = CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_CONFIG);
    value &= 0xFE;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_CONFIG, value);
}

void CI24R1_SetRxMode(void)
{
    uint8_t value;
    value = CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_CONFIG);
    value |= 0x01;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_CONFIG, value);
}

uint8_t CI24R1_SPI_Test(void)
{
    
    uint8_t i, *ptr = (uint8_t *)CI24R1_TEST_ADDR;
    CI24R1_CE_LOW();
    CI24R1_WriteReg(CI24R1_CMD_SELSPI, CI24R1_CMD_NOP);
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_TX_ADDR, ptr, 5);
    CI24R1_ReadToBuf(CI24R1_CMD_R_REGISTER | CI24R1_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++) {
        Serial.println(*(xbuf + i));
        if (*(xbuf + i) != *ptr++) return HAL_ERROR;
    }
    return HAL_OK;
}

void CI24R1_SetTxAddress(uint8_t *address)
{
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_TX_ADDR, address, 5);
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_ADDR_P0, address, 5);
}

void CI24R1_SetRxAddress(uint8_t *address)
{
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_ADDR_P1, address, 5);
}

void CI24R1_SetChannel(uint8_t channel)
{
    if (channel > 125) channel = 125;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RF_CH, channel);
}

void CI24R1_Init(void)
{
    CI24R1_CE_LOW();
#if (CI24R1_PLOAD_WIDTH == 0)
    // Enable dynamic payload length on pipe 0 and pipe 1
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_DYNPD, 0x03);
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_FEATURE, 0x07);
#else
    // Fixed payload length
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_DYNPD, 0x00);
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_FEATURE, 0x03);
    // Length of pipe 0
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_PW_P0, CI24R1_PLOAD_WIDTH);
    // Length of pipe 1
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_PW_P1, CI24R1_PLOAD_WIDTH);
#endif
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_CONFIG, 0x0E);
    // Enable auto ack all pipes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_AA, 0x3F);
    // Enable all pipes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_RXADDR, 0x3F);
    // Address width, 0x1:3bytes, 0x02:4bytes, 0x3:5bytes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_SETUP_AW, 0x03);
    // Resend 500us and 3 times. interval: 250us * ([0, 15] + 1), retries: [0, 15]
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_SETUP_RETR, (0x01 << 4) | 0x03);
    // RF Data Rate 250K 11db
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RF_SETUP, CI24R1_RF_SETUP_1M | CI24R1_RF_SETUP_11DB);
    CI24R1_CE_HIGH();
}

uint8_t CI24R1_Tx(uint8_t *ucPayload, uint8_t length)
{
    uint8_t status;
#if (CI24R1_PLOAD_WIDTH == 0)
    CI24R1_WriteFromBuf(CI24R1_CMD_W_TX_PAYLOAD, ucPayload, length);
#else
    CI24R1_WriteFromBuf(CI24R1_CMD_W_TX_PAYLOAD, ucPayload, CI24R1_PLOAD_WIDTH);
#endif
    CI24R1_CE_HIGH();
    CI24R1_WriteCmd(CI24R1_CMD_SELIRQ);
    CI24R1_DATA_IN();
    while (CI24R1_DATA_READ());
    CI24R1_DATA_OUT();
    CI24R1_WriteCmd(CI24R1_CMD_SELSPI);
    status = CI24R1_ReadStatus();
    if (status & CI24R1_FLAG_MAX_RT)
    {
        CI24R1_WriteReg(CI24R1_CMD_FLUSH_TX, CI24R1_CMD_NOP);
    }
    // Clear status flags
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_STATUS, status);
    return status;
}

uint8_t CI24R1_Rx(void)
{
    uint8_t i, status, rxplWidth;
    CI24R1_WriteReg(CI24R1_CMD_FLUSH_RX, CI24R1_CMD_NOP);
    CI24R1_WriteReg(CI24R1_CMD_SELIRQ, CI24R1_CMD_NOP);
    CI24R1_DATA_IN();
    while(CI24R1_DATA_READ());
    
    CI24R1_DATA_OUT();
    CI24R1_WriteReg(CI24R1_CMD_SELSPI, CI24R1_CMD_NOP);
    status = CI24R1_ReadStatus();
    
    Serial.println('>');
    Serial.println(status);
    if (status & CI24R1_FLAG_RX_READY)
    {
#if CI24R1_PLOAD_WIDTH == 0
        rxplWidth = CI24R1_ReadReg(CI24R1_CMD_R_RX_PL_WID);
#else
        rxplWidth = CI24R1_PLOAD_WIDTH;
#endif
        // Read RX to buffer
        CI24R1_ReadToBuf(CI24R1_CMD_R_RX_PAYLOAD, xbuf, rxplWidth);
        // Clear status flags
        CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_STATUS, status);
        Serial.println('>');
        for (i = 0; i < rxplWidth; i++)
        {
          sprintf(temp,"xbuf[%d] ",i);
          Serial.print(temp);
            Serial.println(*(xbuf_data + i));
        }
    }
    return status;
}

uint8_t CI24R1_ReadStatus(void)
{
    return CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_STATUS);
}

void CI24R1_Switch1F_AF(uint8_t af)
{
    uint8_t val;

    val = CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_EN_AA);
    val &= 0x3F;
    val |= (af & 0x03) << 6;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_AA, val);

    val = CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_EN_RXADDR);
    val &= 0x3F;
    val |= (af & 0x0C) << 4;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_RXADDR, val);
}

uint8_t CI24R1_PrintStatus(void)
{
    uint8_t i, status;

    Serial.println("[Config]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_CONFIG));

    Serial.println("  [EN_AA]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_EN_AA));

    Serial.println("  [EN_RxAddr]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_EN_RXADDR));

    Serial.println("  [AddrWidth]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_SETUP_AW));

    Serial.println("  [Retry]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_SETUP_RETR));

    Serial.println("\r\n[RF_Channel]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RF_CH));

    Serial.println("  [RF_Setup]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RF_SETUP));

    Serial.println("  [Observe_Tx]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_OBSERVE_TX));

    Serial.println("  [RSSI]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RSSI));

    Serial.println("\r\n[TxAddr]  ");
    CI24R1_ReadToBuf(CI24R1_CMD_R_REGISTER | CI24R1_REG_TX_ADDR, xbuf_data, 5);
    for (i = 0; i < 5; i++) {
        Serial.println(*(xbuf_data + i));
    }

    Serial.println("\r\n[RxAddrP0]");
    CI24R1_ReadToBuf(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P0, xbuf_data, 5);
    for (i = 0; i < 5; i++) {
        Serial.println(*(xbuf_data + i));
    }
    Serial.println(" [RxAddrP1]");
    CI24R1_ReadToBuf(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P1, xbuf_data, 5);
    for (i = 0; i < 5; i++) {
        Serial.println(*(xbuf_data + i));
    }
    Serial.println(" [RxAddrP2]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P2));
    Serial.println(" [RxAddrP3]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P3));
    Serial.println(" [RxAddrP4]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P4));
    Serial.println(" [RxAddrP5]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    Serial.println("\r\n[0F_CRC]");
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_CRC);
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    Serial.println(" [0F_OSC_C]");
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_OSC_C);
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    Serial.println(" [0F_BT]");
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_BT);
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    Serial.println(" [0F_BT_CRC_L/M/H]");
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_BT_CRC_L);
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_BT_CRC_M);
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_BT_CRC_H);
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    Serial.println("\r\n[RX_PW_P0]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P0));
    Serial.println(" [RX_PW_P1]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P1));
    Serial.println(" [RX_PW_P2]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P2));
    Serial.println(" [RX_PW_P3]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P3));
    Serial.println(" [RX_PW_P4]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P4));
    Serial.println(" [RX_PW_P5]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P5));

    Serial.println("\r\n[FIFO_Status]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_FIFO_STATUS));

    Serial.println("  [DynPloadWidth]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_DYNPD));

    Serial.println("  [Feature]");
    Serial.println(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_FEATURE));

    status = CI24R1_ReadStatus();
    Serial.println("\r\n[Status]");
    Serial.println(status);
    Serial.println("\r\n\r\n");
    return status;
}
