/*
 * Uart_Parser.h
 *
 *  Created on: 29 янв. 2018 г.
 *      Author: CIT_007
 */

#ifndef APPLICATION_UART_PARSER_H_
#define APPLICATION_UART_PARSER_H_

#include <stdint.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>

#define ADR_TX(x)           (x)<<4
#define ADR_REC(x)          (x)
#define ADR_LZO             0x01
#define ADR_PC              0x03

#define GET_STATUS          0x01 //запрос статуса устройства на линии
#define SEND_DATA           0x02 //передача массива данных
#define SEND_FH_PARAM       0x03 //передача текущих параметров гарнитуры
#define SEND_FH_KEY         0x04 //передача ключа шифрования
#define SEND_FH_CR_TP       0x05 //передача типа шифрования

#define GET_FH_PARAM        0x23 //запрос текущих параметров гарнитуры
#define GET_FH_KEY          0x24 //запрос ключа шифрования
#define GET_FH_CR_TP        0x25 //запрос типа шифрования
#define ENABLE_NG           0x26 //ENABLE_NG
#define DISABLE_NG          0x27 //DISABLE_NG
#define ENABLE_LPF          0x28 //ENABLE_LPF
#define DISABLE_LPF         0x29 //DISABLE_LPF
#define ENABLE_UART_DEBUG   0x30 //ENABLE_UART_DEBUG
#define DISABLE_UART_DEBUG  0x31 //DISABLE_UART_DEBUG
#define ENABLE_EC           0x32 //ENABLE_EC
#define DISABLE_EC          0x33 //DISABLE_EC
#define LPF_TYPE            0x34 //LPF type

#define STATUS_OK           0x10 //устройство работает в штатном режиме
#define STATUS_BAD          0x11 //устройство работает некорректно
#define REC_OK              0x12 //подтверждение безошибочного приёма данных
#define REC_ERROR           0x13 //в процессе приёма возникли ошибки
#define NO_COMAND           0x14
#define BAD_CRC             0x15

#define UART_BYTE_DELAY_TIME 5000000
typedef struct {
    uint8_t header;

    uint8_t addr;

    uint8_t data_lenght ;
    uint8_t command;
    uint8_t data[256];
    unsigned short CRC;
} Serial_Data_Packet;

typedef enum {
    stHEADER = 0,
    stAddr,
    stDL,
    stCtrl,
    stDATA,
    stCRC
}State ;


/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
unsigned short Crc16(unsigned char*pcBlock, unsigned short len);
 void OnRxByte(unsigned char Chr);
 uint16_t PackProcessing(void);
 void parser_init(void);
// void set_Myaddr(unsigned char addr);
// void set_Masteraddr(unsigned char addr);

#endif /* APPLICATION_UART_PARSER_H_ */
