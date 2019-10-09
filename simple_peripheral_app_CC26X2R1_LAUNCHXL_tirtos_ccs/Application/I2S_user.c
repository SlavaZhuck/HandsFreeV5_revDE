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


   static I2S_Handle i2sHandle;
    static I2S_Config i2sConfig;
    static sem_t semDataReadyForTreatment;

      // This buffer will be continuously re-written
      static int16_t readBuf[500];
      // This data will be continuously sent out
      static int16_t writeBuf[500] = {0};

      static I2S_Transaction i2sRead;
      static I2S_Transaction i2sWrite;

      List_List i2sReadList;
      List_List i2sWriteList;

    static void writeCallbackFxn(I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr)
    {

          // Nothing to do here: the buffer(s) are queued in a ring list, the transfers are
          // executed without any action from the application.

          // We must consider the previous transaction (ok, when you have only one transaction it's the same)
          I2S_Transaction *transactionFinished = (I2S_Transaction*)List_prev(&transactionPtr->queueElement);

          if(transactionFinished != NULL)
          {
              // After an arbitrary number of completion of the transaction, we will stop writting
              if(transactionFinished->numberOfCompletions >= 10)
              {

                  // Note: You cannot use I2S_stopRead() / I2S_stopWrite() in the callback.
                  // The execution of these functions is potentially blocking and can mess up the
                  // system.

                 // writeFinished = (bool)true;
              }
          }
    }

    static void readCallbackFxn(I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr)
    {


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

        // Initialize the treatmentList (this list is initially empty)

        //Initialize I2S opening parameters
        I2S_Params_init(&i2sParams);
        i2sParams.fixedBufferLength     = 1000;
        i2sParams.samplingFrequency     = 16000,
        i2sParams.writeCallback         = writeCallbackFxn ;
        i2sParams.readCallback          = readCallbackFxn ;
        i2sParams.errorCallback         = errCallbackFxn;
        i2sParams.MCLKDivider           = 4;
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
        I2S_Transaction_init(&i2sRead);
        i2sRead.bufPtr            = readBuf;
        i2sRead.bufSize           = sizeof(readBuf);
        List_put(&i2sReadList, (List_Elem*)&i2sRead);
        List_tail(&i2sReadList)->next = List_head(&i2sReadList);// Read buffers are queued in a ring-list
        List_head(&i2sReadList)->prev = List_tail(&i2sReadList);

        I2S_setReadQueueHead(i2sHandle, &i2sRead);

        // Initialize the write-transactions
        I2S_Transaction_init(&i2sWrite);
        i2sWrite.bufPtr           = writeBuf;
        i2sWrite.bufSize          = sizeof(writeBuf);
        List_put(&i2sWriteList, (List_Elem*)&i2sWrite);
        List_tail(&i2sWriteList)->next = List_head(&i2sWriteList); // Write buffers are queued in a ring-list
        List_head(&i2sWriteList)->prev = List_tail(&i2sWriteList);

        I2S_setWriteQueueHead(i2sHandle, &i2sWrite);
        for(int16_t i = 0 ; i < 500; i++)
        {
            writeBuf[i] = (int16_t)(32767*sin(2*M_PI*i/20));
        }


        /*
         * Prepare the semaphore
         */
              I2S_startClocks(i2sHandle);
              I2S_startWrite(i2sHandle);
              I2S_startRead(i2sHandle);
//        int retc = sem_init(&semDataReadyForTreatment, 0, 0);
//        if (retc == -1) {
//            while (1);
//        }



    int k;

//    while(1)
//    {
//
//    }
 }
