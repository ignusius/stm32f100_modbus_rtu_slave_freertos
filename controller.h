#include "stm32f10x.h"

#define OBJ_SZ 123
#define SETUP 4 //setup elements

//PARAMETERRS ARRAY 0 PARAMETER = MODBUS ADDRESS
uint8_t SET_PAR[SETUP];

//OBJECT ARRAY WHERE READING AND WRITING OCCURS
union {
int16_t regs[OBJ_SZ];
int8_t bytes[OBJ_SZ*2];
}res_table;

//buffer uart
#define BUF_SZ 256
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //max quantity of words in responce

//uart structure
typedef struct {
uint8_t buffer[BUF_SZ];
uint16_t rxtimer;
uint8_t rxcnt;
uint8_t txcnt;
uint8_t txlen;
uint8_t rxgap;
uint8_t protocol;
uint8_t delay;
} UART_DATA;

UART_DATA uart1;

void MODBUS_SLAVE(UART_DATA *MODBUS);



