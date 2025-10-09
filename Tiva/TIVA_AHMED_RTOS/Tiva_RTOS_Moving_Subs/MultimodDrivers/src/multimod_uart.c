// multimod_uart.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for UART functions

/************************************Includes***************************************/

#include "../multimod_uart.h"

#include <stdint.h>
#include <stdbool.h>

#include <inc/tm4c123gh6pm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>

#include <driverlib/uartstdio.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>

/************************************Includes***************************************/

/********************************Public Functions***********************************/

// UART_Init
// Initializes UART serial communication with PC
// Return: void
void UART_Init() {

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}

        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){}

        UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));

        UARTStdioConfig(0, 115200, SysCtlClockGet());
    // This should have been done in lab 0, so it's just copy & paste.
    // Enable port A

    // Enable UART0 module


    // Configure UART0 pins on port A



    // Set UART clock source

    // Configure UART baud rate

}

void UART_ESP32_Init(void) {
    // Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1));

    // Configure UART1 pins
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure UART1 (115200 baud, 8N1)
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    UARTEnable(UART1_BASE);
}

// UART_ESP32_ReadChar
int UART_ESP32_ReadChar(void) {
    if (UARTCharsAvail(UART1_BASE))
        return UARTCharGetNonBlocking(UART1_BASE);
    return -1;
}

// Beaglebone UART
//void UART_BeagleBone_Init(void) {
//    // Enable peripherals
//
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
//    GPIOPinConfigure(GPIO_PC4_U1RX);
//    GPIOPinConfigure(GPIO_PC5_U1TX);
//    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
//
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1));
//
//    // Configure UART4 pins
//
//    // Configure UART4 settings
//
//    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
//                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                             UART_CONFIG_PAR_NONE));
//
//    // Enable UART4 RX interrupts
//    UARTIntEnable(UART1_BASE, UART_INT_RX);
//
//    UARTFIFOEnable(UART1_BASE);
//
//    // change to whatever
//    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
//
//    UARTEnable(UART1_BASE);
//
//}

void UART_BeagleBone_Init(void) {
    // Enable peripheral for GPIOC (PC4 = U4RX, PC5 = U4TX)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    // Configure pin mux for UART4 on PC4/PC5
    GPIOPinConfigure(GPIO_PC4_U4RX);
    GPIOPinConfigure(GPIO_PC5_U4TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Enable UART4 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART4));

    // Configure UART4: 115200, 8-N-1
    UARTConfigSetExpClk(UART4_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    // Enable UART4 RX interrupts (masked)
    UARTIntEnable(UART4_BASE, UART_INT_RX);

    // Enable FIFOs and set FIFO trigger levels
    UARTFIFOEnable(UART4_BASE);
    UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    // Enable the UART (TX & RX)
    UARTEnable(UART4_BASE);

    // NOTE: make sure you have an ISR named `UART4_Handler` (C linkage)
    // or register a handler with UARTIntRegister(UART4_BASE, YourHandler).
    // Also enable the interrupt in the NVIC if you haven't:
    // IntEnable(INT_UART4);
}

/********************************Public Functions***********************************/

