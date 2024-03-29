/*
 * max9860_i2c.c
 *
 *  Created on: 28 ���. 2018 �.
 *      Author: CIT_007
 */

#include "max9860_i2c.h"
#include "GeneralDef.h"

static I2C_Handle      i2c;
static I2C_Params      i2cParams;
static I2C_Transaction i2cTransaction;
static uint8_t         i2cRxBuffer[17];


void max9860_I2C_Init(void){
    uint8_t         i2cTxBuffer[15];
    uint8_t         i2cRxBuffer[18];


    I2C_init();
    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C0, &i2cParams);
    if (i2c == NULL)
    {
        while (1);
    }

    /* Point to the T ambient register and read its 2 bytes */
    i2cTxBuffer[0]  = 0x03;//!addr reg
    i2cTxBuffer[1]  = 0x13;//!system clock                       //clock control            0x03
    i2cTxBuffer[2]  = 0x80;//!stereo audio clock control high                               0x04
    i2cTxBuffer[3]  = 0x00;//!stereo audio clock control low                                0x05
    i2cTxBuffer[4]  = 0x40;//!interface                          //digital audio interface  0x06
    i2cTxBuffer[5]  = 0x00;//!interface                                                     0x07
    i2cTxBuffer[6]  = 0x11;//!voice filter                       //digital filtering        0x08
    i2cTxBuffer[7]  = INIT_GAIN;//!DAC att                       //digital level control    0x09
    i2cTxBuffer[8]  = 0x00;//!ADC output levels                                             0x0A
    i2cTxBuffer[9]  = 0x00;//!DAC gain and sidetone                                         0x0B
    i2cTxBuffer[10] = 0x40;//!microphone gain                    //MIC level control        0x0C
    i2cTxBuffer[11] = 0x00;                                     //RESERVED                  0x0D
    i2cTxBuffer[12] = 0x3f;//!microphone AGC                   //MIC automatic gain control 0x0E //norm - 0x3f, test - 0x00
    i2cTxBuffer[13] = 0x4F;//!Noise gate, mic AGC  0x4F                                     0x0F //norm - 0x4F, test - 0x00
    i2cTxBuffer[14] = 0x00;//!System shutdown                    //POWER MANAGEMENT         0x10

    i2cTransaction.slaveAddress = 0x10;
    i2cTransaction.writeBuf = i2cTxBuffer;
    i2cTransaction.readBuf = i2cRxBuffer;//rxBuffer;

    i2cTransaction.writeCount = 15;
    i2cTransaction.readCount = 0;
    I2C_transfer(i2c, &i2cTransaction);

    volatile static uint32_t delay = 4800000;
    while(delay>0)
    {
        delay--;
    }
}

void max9860_I2C_Read_Status(void){

    i2cTransaction.slaveAddress = 0x10;
    i2cTransaction.readBuf = i2cRxBuffer;//rxBuffer;

    i2cTransaction.writeCount = 0;
    i2cTransaction.readCount = 17;
    I2C_transfer(i2c, &i2cTransaction);
}

void max9860_I2C_Volume_update(uint8_t vol){
    uint8_t         i2cTxBuffer[3];
    i2cTxBuffer[0]  = 0x09;//!addr reg "DAC att register"
    i2cTransaction.slaveAddress = 0x10;
    i2cTransaction.writeBuf = i2cTxBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;
    i2cTxBuffer[1]  = vol;//!DAC att  digital level control    0x09
    I2C_transfer(i2c, &i2cTransaction);
}



void max9860_I2C_Shutdown_state(uint8_t state){
    uint8_t         i2cTxBuffer[3];
    i2cTxBuffer[0]  = 0x10;//!addr reg "System shutdown register"
    i2cTransaction.slaveAddress = 0x10;
    i2cTransaction.writeBuf = i2cTxBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;
    if(state){
        i2cTxBuffer[1]  = 0x00;//!DAC att  digital level control    0x09
    }else{
#ifdef SECOND_MICROPHONE
        i2cTxBuffer[1]  = 0x8B;
#else
        i2cTxBuffer[1]  = 0x8A;
#endif
    }
    I2C_transfer(i2c, &i2cTransaction);
}





