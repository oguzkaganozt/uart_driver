#include <stdio.h>
#include <stdint.h>
#include "uart.h"

/* UART driver
 *
 * Write a the initialization for a UART device. In this case it is a fictional
 * device as described in more detail below, since there is no access to an actual
 * microcontroller available for this task.
 *
 * Write an initialization routine for the UART peripheral in C. The initialization
 * should be implement in the 'init_uart' function.
 *
 * Since there is no access to a real microcontroller device, the memory accesses
 * needs to be stubbed. A small test case should be written to show the memory
 * values are correct. The code should be compilable with a GCC compiler. Please
 * mention the used compiler version in your answer.
 *
 * The required UART configuration is:
 *
 * 	Baud rate: 230400
 * 	Stop bits: 1
 * 	Parity: none
 * 	Flow control: Hardware flow control RTS/CTS
 *
 * The UART peripheral is memory mapped to address 0x4000 7800. The UART peripheral
 * is controlled by the registers as described below. It is only a very small subset
 * of a real UART peripheral setup.
 *
 * Register description:
 * ---------------------
 * BRR: Baud Rate Register
 * 	Address offset: 0x00
 * 	Reset value: 0x0000 0000
 *
 * 	Bits 3:0 BRR[3:0]: Baud rate selection
 * 	Values:
 * 		0000: 110
 * 		0001: 150
 * 		0010: 300
 * 		0011: 1200
 * 		0100: 2400
 * 		0101: 4800
 * 		0111: 9600
 * 		1000: 19200
 * 		1001: 38400
 * 		1010: 57600
 * 		1011: 115200
 * 		1100: 230400
 * 		1101: 460800
 * 		1111: 921600
 *
 * CR1: Control register 1
 * 	Address offset: 0x04
 * 	Reset value: 0x0000 0000
 *
 * 	Bit 12 UE: UART enable
 * 		When this bit is cleared, the UART will stop sending and receiving data.
 * 		This bit is set and cleared by software.
 *
 * 		0: UART disabled
 *		1: UART enabled
 *
 * 	Bit 10 PCE: Parity control enable
 * 		This bit selects the hardware parity control (generation and detection).
 * 		This bit is set and cleared by software.
 *
 * 		0: Parity control disabled
 *		1: Parity control enabled
 *
 *	Bit 3 PS: Parity selection
 *		This bit selects the odd or even parity when the parity
 *		generation/detection is enabled (PCE bit set). It is set and
 *              cleared by software.
 *
 *		0: Even parity
 *		1: Odd parity
 *
 *	Bit 2 TE: Transmitter enable
 *		This bit enables the transmitter. It is set and cleared by software.
 *
 *		0: Transmitter is disabled
 *		1: Transmitter is enabled
 *
 *	Bit 1 RE: Receiver enable
 *		This bit enables the receiver. It is set and cleared by software.
 *
 *		0: Receiver is disabled
 *		1: Receiver is enabled and begins searching for a start bit
 *
 * CR2: Control register 2
 * 	Address offset: 0x08
 * 	Reset value: 0x0000 0000
 *
 * 	Bits 9:8 STOP: STOP bits
 *		These bits are used for programming the stop bits.
 *
 *		00: 1 Stop bit
 *		01: 0.5 Stop bit
 *		10: 2 Stop bits
 *		11: 1.5 Stop bit
 *
 *	Bit 3 CTSE: CTS enable
 *		0: CTS hardware flow control disabled
 *		1: CTS mode enabled.
 *
 *	Bit 2 RTSE: RTS enable
 *		0: RTS hardware flow control disabled
 *		1: RTS interrupt enabled, data is only requested when there is
 *		   space in the receive buffer.
 */

typedef struct uart_handle {
    uint16_t* uart_instance;    //this is a stub normally it is UART_INSTANCE type.
    UART_BAUDRATE baudrate;
    UART_PARITY parity;
    UART_STOPBITS stopbits_control;
    UART_FLOW flow_control;
} uart;

// Fill this function to initialize the UART peripheral
void init_uart(uart *handle)
{
    //localize device registers
    uint16_t uart_brr = UART_BRR(handle->uart_instance);
    uint16_t uart_cr1 = UART_CR1(handle->uart_instance);
    uint16_t uart_cr2 = UART_CR2(handle->uart_instance);

    //set baudrate
    uart_brr = handle->baudrate;

    //set parity
    if(handle->parity == UART_PARITY_EVEN)
    {
        uart_cr1 |= (0X01 << 10);
        uart_cr1 &= ~(0X01 << 3);
    }
    else if(handle->parity == UART_PARITY_ODD)
    {
        uart_cr1 |= (0X01 << 10);
        uart_cr1 |= (0X01 << 3);
    }
    else
    {
        uart_cr1 &= ~(0X01 << 10);
    }
    
    //set stop bits
    if(handle->stopbits_control == UART_STOPBITS_1)
    {
        uart_cr2 &= ~(0X01 << 8);
        uart_cr2 &= ~(0X01 << 9);
    }
    else if(handle->stopbits_control == UART_STOPBITS_05)
    {
        uart_cr2 |= (0X01 << 8);
        uart_cr2 &= ~(0X01 << 9);
    }
    else if(handle->stopbits_control == UART_STOPBITS_2)
    {
        uart_cr2 &= ~(0X01 << 8);
        uart_cr2 |= (0X01 << 9);
    }
    else
    {
        uart_cr2 |= (0X01 << 8);
        uart_cr2 |= (0X01 << 9);
    }
    
    //set flow control
    if(handle->flow_control & UART_FLOW_RTS)
        uart_cr2 |= (0X01 << 2);
    else
        uart_cr2 &= ~(0X01 << 2);
    
    if(handle->flow_control & UART_FLOW_CTS)
        uart_cr2 |= (0X01 << 3);
    else
        uart_cr2 &= ~(0X01 << 3);
    
    //map changes to register
     UART_BRR(handle->uart_instance) = uart_brr;
     UART_CR1(handle->uart_instance) = uart_cr1;
     UART_CR2(handle->uart_instance) = uart_cr2;
}

// Enable UART both RX-TX
void uart_enable_rx_tx(uint16_t* uart_inst)
{
    UART_CR1(uart_inst) |= (0X01 << 12);
    UART_CR1(uart_inst) |= (0X01 << 2);
    UART_CR1(uart_inst) |= (0X01 << 1);
}

// Enable UART only RX
void uart_enable_rx(uint16_t* uart_inst)
{
    UART_CR1(uart_inst) |= (0X01 << 12);
    UART_CR1(uart_inst) |= (0X01 << 1);
    UART_CR1(uart_inst) &= ~(0X01 << 2);
}

// Enable UART only TX
void uart_enable_tx(uint16_t* uart_inst)
{
    UART_CR1(uart_inst) |= (0X01 << 12);
    UART_CR1(uart_inst) |= (0X01 << 2);
    UART_CR1(uart_inst) &= ~(0X01 << 1);
}

// Get handle from specified uart instance
void uart_get_handler(uint16_t* uart_inst,uart *handle)
{
    //get registers
    uint16_t uart_brr = UART_BRR(uart_inst);
    uint16_t uart_cr1 = UART_CR1(uart_inst);
    uint16_t uart_cr2 = UART_CR2(uart_inst);

    //get instance
    handle->uart_instance = uart_inst;

    //get baudrate
    handle->baudrate = uart_brr;

    //get flow control
    if(uart_cr2 & (0x01 << 3))
        handle->flow_control |= UART_FLOW_CTS;
    else
        handle->flow_control &= ~(UART_FLOW_CTS);

    if(uart_cr2 & (0x01 << 2))
        handle->flow_control |= UART_FLOW_RTS;
    else
        handle->flow_control &= ~(UART_FLOW_RTS);

    //get parity
    if(uart_cr1 & (0x01 << 10))
    {
        if(uart_cr1 & (0x01 << 3))
            handle->parity = UART_PARITY_ODD;
        else
            handle->parity = UART_PARITY_EVEN;
    }
    else
        handle->parity = UART_PARITY_NONE;

    //get stopbits
    if((uart_cr2 & (0x01 << 8)) && (uart_cr2 & (0x01 << 9)))
        handle->stopbits_control = UART_STOPBITS_15;
    else if((uart_cr2 & (0x01 << 8)) && !(uart_cr2 & (0x01 << 9)))
        handle->stopbits_control = UART_STOPBITS_05;
    else if(!(uart_cr2 & (0x01 << 8)) && (uart_cr2 & (0x01 << 9)))
        handle->stopbits_control = UART_STOPBITS_2;
    else
        handle->stopbits_control = UART_STOPBITS_1;
    
}

// Dump handle to stdout. For DEBUG purposes only.
void dump_uart_handler(uart *handle)
{
    printf("\n--------------------");
    printf("\nbaudrate: %s",str_baudrate[handle->baudrate]);
    printf("\nstopbits_control: %s",str_stopbits[handle->stopbits_control]);
    printf("\nparity: %s",str_parity[handle->parity]);
    printf("\nflow_control: %s",str_flow[handle->flow_control]);
    printf("\n--------------------");
}

// Initialize the UART and provide a test case to verify the register values.
int main(int argc, char *argv)
{   
    uint16_t base_address[16];   //stub address allocated at stack to simulate device registers

    //INIT
    //init uart1
    uart uart_handler;
    uart_handler.uart_instance = base_address;  //this is a stub for UART device address normally user can use UART_0 
    uart_handler.baudrate = UART_BAUDRATE_230400;
    uart_handler.parity = UART_PARITY_NONE;
    uart_handler.stopbits_control = UART_STOPBITS_1;
    uart_handler.flow_control = UART_FLOW_RTS_CTS;
    init_uart(&uart_handler);
    
    //TEST
    //read uart handle from uart register
    uart uart_handler_read;
    uart_get_handler(base_address,&uart_handler_read);

    //prompt inited and read configurations
    dump_uart_handler(&uart_handler);
    dump_uart_handler(&uart_handler_read);

    //check whether the init configs are the same as read config
    if (uart_handler.baudrate == uart_handler_read.baudrate &&
        uart_handler.flow_control == uart_handler_read.flow_control &&
        uart_handler.parity == uart_handler_read.parity &&
        uart_handler.stopbits_control == uart_handler_read.stopbits_control)
        {
            printf("\nUART configuration validated.\n");
        }

	return 0;
}