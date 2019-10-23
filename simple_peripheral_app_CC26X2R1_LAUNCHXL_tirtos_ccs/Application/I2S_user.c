/*
 * I2S_user.c
 *
 *  Created on: 6 окт. 2019 г.
 *      Author: CIT_007
 */


#include <stdint.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2S.h>
#include <ti/sysbios/BIOS.h>
/* Example/Board Header files */
#include "Board.h"
#include "project_zero.h"
#include "HandsFree.h"


static I2S_Handle i2sHandle;
static sem_t semDataReadyForTreatment;

// These buffers will successively be written, treated and sent out
int16_t readBuf1[I2S_SAMP_PER_FRAME];
int16_t readBuf2[I2S_SAMP_PER_FRAME];
int16_t readBuf3[I2S_SAMP_PER_FRAME];
int16_t readBuf4[I2S_SAMP_PER_FRAME];
int16_t writeBuf1[I2S_SAMP_PER_FRAME];
int16_t writeBuf2[I2S_SAMP_PER_FRAME];
int16_t writeBuf3[I2S_SAMP_PER_FRAME];
int16_t writeBuf4[I2S_SAMP_PER_FRAME];

// These transactions will successively be part of the
// i2sReadList and the i2sWriteList
I2S_Transaction i2sRead1;
I2S_Transaction i2sRead2;
I2S_Transaction i2sRead3;
I2S_Transaction i2sRead4;
I2S_Transaction i2sWrite1;
I2S_Transaction i2sWrite2;
I2S_Transaction i2sWrite3;
I2S_Transaction i2sWrite4;

List_List i2sReadList;
List_List i2sWriteList;

extern uint32_t measure_tim_value;
extern uint8_t stream_on;
extern int16_t raw_data_received[I2S_SAMP_PER_FRAME];
extern int16_t mic_data_1ch[I2S_SAMP_PER_FRAME];
uint32 write_timestamps[100];
uint32 read_timestamps[100];

I2S_Transaction *write_transactionFinished;
I2S_Transaction *read_transactionFinished;
uint32 *write_but_pointer;

static void writeCallbackFxn(I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr)
{
    static uint32 I2S_write_timestamp = 0;
    static uint32 I2S_write_counter = 0;

    write_timestamps[I2S_write_counter % 100] = measure_tim_value - I2S_write_timestamp;

    // We must remove the previous transaction (the current one is not over)
    write_transactionFinished = (I2S_Transaction*)List_prev(&transactionPtr->queueElement);
    if(write_transactionFinished != NULL)
    {
        ProjectZero_enqueueMsg(PZ_WRITE_DATA_TO_I2S, NULL);
        memcpy(write_transactionFinished->bufPtr  ,raw_data_received, sizeof(raw_data_received));

        // Remove the finished transaction from the write queue
        List_remove(&i2sWriteList, (List_Elem*)write_transactionFinished);
        write_transactionFinished->queueElement.next = NULL;
        List_put(&i2sWriteList, (List_Elem*)write_transactionFinished);
    }
    I2S_write_timestamp = measure_tim_value;
    I2S_write_counter++;
}

static void readCallbackFxn(I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr)
{
    static uint32 I2S_read_timestamp = 0;
    static uint32 I2S_read_counter = 0;

    read_timestamps[I2S_read_counter % 100] = measure_tim_value - I2S_read_timestamp;

    // We must remove the previous transaction (the current one is not over)
    I2S_Transaction *read_transactionFinished = (I2S_Transaction*)List_prev(&transactionPtr->queueElement);

    if(read_transactionFinished != NULL){
        ProjectZero_enqueueMsg(PZ_READ_DATA_FROM_I2S, NULL);
        memcpy(mic_data_1ch, read_transactionFinished->bufPtr, sizeof(mic_data_1ch));
        // The finished transaction contains data that must be treated
        List_remove(&i2sReadList, (List_Elem*)read_transactionFinished);
        read_transactionFinished->queueElement.next = NULL;
        List_put(&i2sReadList, (List_Elem*)read_transactionFinished);

        // Start the treatment of the data
        //Semaphore_post(dataReadyForTreatment);

        // We do not need to queue transaction here: writeCallbackFxn takes care of this :)
    }

    I2S_read_timestamp = measure_tim_value;
    I2S_read_counter++;
}

static void errCallbackFxn(I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr)
{
    // Handle the I2S error
}


void I2S_user_init (void)
{
    I2S_Params i2sParams = {
                        .samplingFrequency    = 16000,
                        .memorySlotLength     = I2S_MEMORY_LENGTH_16BITS,
                        .moduleRole             = I2S_MASTER,
                        .trueI2sFormat        = (bool)true,
                        .invertWS             = (bool)true,
                        .isMSBFirst           = (bool)true,
                        .isDMAUnused          = (bool)false,
                        .samplingEdge         = I2S_SAMPLING_EDGE_RISING,
                        .beforeWordPadding    = 0,
                        .bitsPerWord          = 16,
                        .afterWordPadding     = 0,
                        .fixedBufferLength    = 1,
                        .SD0Use               = I2S_SD0_OUTPUT ,
                        .SD1Use               = I2S_SD1_INPUT,
                        .SD0Channels          = I2S_CHANNELS_STEREO,
                        .SD1Channels          = I2S_CHANNELS_STEREO,
                        .phaseType            = I2S_PHASE_TYPE_DUAL,
                        .startUpDelay         = 0,
                        .MCLKDivider          = 4,
                        .readCallback         = NULL,
                        .writeCallback        = NULL,
                        .errorCallback        = NULL,
                        .custom               = NULL
    };

    I2S_init();

    //Initialize I2S opening parameters
    I2S_Params_init(&i2sParams);
    i2sParams.fixedBufferLength     = I2S_SAMP_PER_FRAME * 2;
    i2sParams.samplingFrequency     = 16000;
    i2sParams.memorySlotLength      = I2S_MEMORY_LENGTH_16BITS;
    i2sParams.beforeWordPadding     = 0;
    i2sParams.afterWordPadding      = 0;
    i2sParams.isMSBFirst            = (bool)true;
    i2sParams.writeCallback         = writeCallbackFxn ;
    i2sParams.readCallback          = readCallbackFxn ;
    i2sParams.errorCallback         = errCallbackFxn;
    i2sParams.MCLKDivider           = 4;
    i2sParams.bitsPerWord           = 16;
    i2sParams.SD0Channels           = I2S_CHANNELS_MONO_INV;
    i2sParams.SD1Channels           = I2S_CHANNELS_MONO_INV;
    i2sParams.moduleRole            = I2S_MASTER;
    i2sParams.phaseType             = I2S_PHASE_TYPE_DUAL;
    i2sParams.SD0Use                = I2S_SD0_OUTPUT;
    i2sParams.SD1Use                = I2S_SD1_INPUT;
    i2sParams.trueI2sFormat         = (bool)false;
    i2sParams.samplingEdge          = I2S_SAMPLING_EDGE_RISING;

    i2sHandle = I2S_open(Board_I2S0, &i2sParams);

    // Initialize the read-transactions
    I2S_Transaction_init(&i2sRead1);
    I2S_Transaction_init(&i2sRead2);
    I2S_Transaction_init(&i2sRead3);
    I2S_Transaction_init(&i2sRead4);
    i2sRead1.bufPtr            = readBuf1;
    i2sRead2.bufPtr            = readBuf2;
    i2sRead3.bufPtr            = readBuf3;
    i2sRead4.bufPtr            = readBuf4;
    i2sRead1.bufSize           = sizeof(readBuf1);
    i2sRead2.bufSize           = sizeof(readBuf2);
    i2sRead3.bufSize           = sizeof(readBuf3);
    i2sRead4.bufSize           = sizeof(readBuf4);
    List_clearList(&i2sReadList);
    List_put(&i2sReadList, (List_Elem*)&i2sRead1);
    List_put(&i2sReadList, (List_Elem*)&i2sRead2);
    List_put(&i2sReadList, (List_Elem*)&i2sRead3);
    List_put(&i2sReadList, (List_Elem*)&i2sRead4);

    I2S_setReadQueueHead(i2sHandle, &i2sRead1);

    // Initialize the write-transactions
    I2S_Transaction_init(&i2sWrite1);
    I2S_Transaction_init(&i2sWrite2);
    I2S_Transaction_init(&i2sWrite3);
    I2S_Transaction_init(&i2sWrite4);
    i2sWrite1.bufPtr           = writeBuf1;
    i2sWrite2.bufPtr           = writeBuf2;
    i2sWrite3.bufPtr           = writeBuf3;
    i2sWrite4.bufPtr           = writeBuf4;
    i2sWrite1.bufSize          = sizeof(writeBuf1);
    i2sWrite2.bufSize          = sizeof(writeBuf2);
    i2sWrite3.bufSize          = sizeof(writeBuf3);
    i2sWrite4.bufSize          = sizeof(writeBuf4);
    List_clearList(&i2sWriteList);
    List_put(&i2sWriteList, (List_Elem*)&i2sWrite1);
    List_put(&i2sWriteList, (List_Elem*)&i2sWrite2);
    List_put(&i2sWriteList, (List_Elem*)&i2sWrite3);
    List_put(&i2sWriteList, (List_Elem*)&i2sWrite4);

    I2S_setWriteQueueHead(i2sHandle, &i2sWrite1);

//    for(int16_t i = 0 ; i < 500; i++)
//    {
//        writeBuf1[i] = (int16_t)(32767*sin(2*M_PI*i/20));
//        writeBuf2[i] = (int16_t)(32767*sin(2*M_PI*i/20));
//        writeBuf3[i] = (int16_t)(32767*sin(2*M_PI*i/20));
//        writeBuf4[i] = (int16_t)(32767*sin(2*M_PI*i/20));
//    }
}


void I2S_start_transfers(void)
{
    I2S_startClocks(i2sHandle);
    I2S_startWrite(i2sHandle);
    I2S_startRead(i2sHandle);
}


void I2S_stop_transfers(void)
{
    if(stream_on == 1)
    {
        I2S_stopWrite(i2sHandle);
        I2S_stopRead(i2sHandle);

        volatile static uint32_t delay_i2S = 9600 * 2; //20ms * 2
        while(delay_i2S>0)
        {
            delay_i2S--;
        }

        I2S_stopClocks(i2sHandle);
    }
}

