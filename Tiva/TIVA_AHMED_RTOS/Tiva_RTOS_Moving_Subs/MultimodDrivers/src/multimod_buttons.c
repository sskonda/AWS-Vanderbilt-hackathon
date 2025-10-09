// multimod_buttons.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for button functions

/************************************Includes***************************************/

#include "../multimod_buttons.h"

#include "../multimod_i2c.h"

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>

#include <inc/tm4c123gh6pm.h>
#include <inc/hw_i2c.h>

/********************************Public Functions***********************************/

// Buttons_Init
// Initializes buttons on the multimod by configuring the I2C module and
// relevant interrupt pin.
// Return: void
void MultimodButtons_Init() {
    // Initialize this function & the relevant interrupt pin
    // INT on PE4
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // Wait for the GPIO module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    // Configure PF4 as input
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);

    // Set interrupt on Falling edge for PF4 (active low)
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);


}

// MultimodButtons_Get
// Gets the input to GPIO bank 0, [0..7].
// Return: uint8_t 
uint8_t MultimodButtons_Get() {
    // complete this function
    I2C_WriteSingle(I2C1_BASE, BUTTONS_PCA9555_GPIO_ADDR, 0x00, 1);
    uint8_t data = I2C_ReadSingle(I2C1_BASE, BUTTONS_PCA9555_GPIO_ADDR, 1);
    return data;
}

