#ifndef UART_LIB
#define UART_LIB

#include <stdio.h>
#include <stdint.h>

// UART REGISTERS
#define UART_0_BASE_ADDR  0x40007800  
#define UART_BRR(UART_X) *((uint16_t *) UART_X) //Baud Rate Register
#define UART_CR1(UART_X) *((uint16_t *) ((uint8_t *)UART_X + 0x04))    //Control Register 1
#define UART_CR2(UART_X) *((uint16_t *) ((uint8_t *)UART_X + 0x08))    //Control Register 2

// UART Peripheral Registers
typedef enum
{
    UART_0 = UART_0_BASE_ADDR
}UART_INSTANCE;

// UART Baudrate Configurations
typedef enum
{
    UART_BAUDRATE_110 = 0x00,
    UART_BAUDRATE_150 = 0x01,
    UART_BAUDRATE_300 = 0x02,
    UART_BAUDRATE_1200 = 0x03,
    UART_BAUDRATE_2400 = 0x04,
    UART_BAUDRATE_4800 = 0x05,
    UART_BAUDRATE_9600 = 0x07,
    UART_BAUDRATE_19200 = 0x08,
    UART_BAUDRATE_38400 = 0x09,
    UART_BAUDRATE_57600 = 0x0A,
    UART_BAUDRATE_115200 = 0x0B,
    UART_BAUDRATE_230400 = 0x0C,
    UART_BAUDRATE_460800 = 0x0D,
    UART_BAUDRATE_921600 = 0x0F
}UART_BAUDRATE;

// UART Parity Control
typedef enum
{
    UART_PARITY_NONE = 0x00,
    UART_PARITY_ODD = 0x01,
    UART_PARITY_EVEN = 0x02,
}UART_PARITY;

// UART Stopbits Control
typedef enum
{
    UART_STOPBITS_1 = 0x00,     //1 stop-bit
    UART_STOPBITS_05 = 0x01,    //0.5 stop-bit    
    UART_STOPBITS_2 = 0x02,     //2 stop-bit
    UART_STOPBITS_15 = 0x03     //1.5 stop-bit
}UART_STOPBITS;

// UART Flow Control
typedef enum
{
    UART_FLOW_NONE = 0x00,
    UART_FLOW_RTS = 0x01,
    UART_FLOW_CTS = 0x02,
    UART_FLOW_RTS_CTS = 0x03
}UART_FLOW;


// String representation of configs. For DEBUG purposes only.
char* str_baudrate[] = {"110","150","300","1200","2400","4800","N/A","9600","19200","38400","57600","115200","230400","460800","N/A","921600"};
char* str_parity[] = {"None","Odd","Even"};
char* str_stopbits[] = {"1 stop-bit","0.5 stop-bit","2 stop-bit","1.5 stop-bit"};
char* str_flow[] = {"None","RTS","CTS","RTS and CTS"};

#endif