// multimod_uart.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for UART functions

/************************************Includes***************************************/

#include "../multimod_uart.h"
#include "../multimod_i2c.h"
#include "../multimod_PCA9555.h"

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
    // Enable port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable UART0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure UART0 pins on port A
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTFIFODisable(UART0_BASE);

    // Set UART clock source
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    // Configure UART baud rate

    UARTStdioConfig(0, 115200, SysCtlClockGet());

}

// UART_BeagleBone_Init
// Initializes UART serial communication with Beaglebone
// Return: void
void UART_BeagleBone_Init(void) {
    // finish this function

    // Enable port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    // Enable UART0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);

    // Configure UART0 pins on port A
    GPIOPinConfigure(GPIO_PC4_U4RX);
    GPIOPinConfigure(GPIO_PC5_U4TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Set UART clock source
    UARTClockSourceSet(UART4_BASE, UART_CLOCK_SYSTEM);
    
    // Configure UART baud rate
    UARTConfigSetExpClk(UART4_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE);

}

void UART_Bluetooth_init(void) {
    //BLUETOOTH PC6/U3RX - bluetooth tx
    //BLUETOOTH PC7/U3TX - bluetooth rx
    // finish this function
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    // Enable UART0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

    // Configure UART0 pins on port A
    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    UARTFIFODisable(UART3_BASE);

    // Set UART clock source
    UARTClockSourceSet(UART3_BASE, UART_CLOCK_SYSTEM);
    // Configure UART baud rate
    //by default 8 bits, 9600, no parity, one stop
    UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 1000000, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //UARTStdioConfig(0, 115200, SysCtlClockGet());
    UARTEnable(UART3_BASE);

    // Enable UART RX interrupts
    // UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT); 
    UARTIntEnable(UART3_BASE, UART_INT_RX); 


    //perform reset
    I2C_Init(I2C_B_BASE);
    PCA9555_SetPinDir(I2C_B_BASE, 0x21, 0xFE);
    PCA9555_SetOutput(I2C_B_BASE, 0x21, 0);
    SysCtlDelay(100000);
    PCA9555_SetOutput(I2C_B_BASE, 0x21, 1);

}

// UART_ESP32_Init
// Initializes UART1 (PB0 = RX, PB1 = TX) for ESP32 communication
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




/********************************Public Functions***********************************/

