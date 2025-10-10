// Hackathon file: UART bridge between ESP32 and TIVA
// Created: 2025-10-07
// Author: Sanat Konda

#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"

int main(void) {
    // Set system clock to 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    multimod_init();

    UARTprintf("\n--- ESP32 <-> TIVA UART Bridge Started ---\n");

    while (1) {
        int c = UART_ESP32_ReadChar();
        if (c != -1) {
            //UARTprintf("peen");
            // Print received character to terminal
            UARTprintf("%c", (char)c);
        }
    }
}
