// multimod_PCA9956b.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for PCA9956b functions

/************************************Includes***************************************/

#include "../multimod_PCA9956b.h"


#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>

#include <stdint.h>
#include "../multimod_i2c.h"

/********************************Public Functions***********************************/

// PCA9956b_Init
// Initializes the PCA9956b, initializes the relevant output enable pins
// Return: void
void PCA9956b_Init() {
    I2C_Init(I2C_A_BASE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, OE_PIN);
    GPIOPinWrite(GPIO_PORTD_BASE, OE_PIN, 1);

    PCA9956b_SetAllOff();

    return;
}

// PCA9956b_SetAllMax
// Writes to the IREFALL, PWMALL registers, sets LEDs to maximum.
// Return: void
void PCA9956b_SetAllMax() {
    uint8_t bytes[] = {(~(AI_BIT) & IREFALL), 0xFF};
    I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

    bytes[0] = (~(AI_BIT) & PWMALL);
    I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
}

// PCA9956b_SetAllOff
// Writes to the IREFALL, PWMALL registers, sets LEDs to off.
// Return: void
void PCA9956b_SetAllOff() {
    uint8_t bytes[] = {(~(AI_BIT) & IREFALL), 0x00};
    I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

    bytes[0] = (~(AI_BIT) & PWMALL);
    I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
}

// PCA9956b_EnableOutput
// Sets output enable pin to true.
// Return: void
void PCA9956b_EnableOutput() {
    GPIOPinWrite(GPIO_PORTD_BASE, OE_PIN, 0);
}

// PCA9956b_EnableOutput
// Sets output enable pin to false.
// Return: void
void PCA9956b_DisableOutput() {
    GPIOPinWrite(GPIO_PORTD_BASE, OE_PIN, 1);
}

void PCA9956b_SetLED(int16_t x, int16_t y){

    uint8_t bytes[] = {(~(AI_BIT) & IREF9), 0xFF};
    PCA9956b_SetAllOff();

    if(x <= -2){

        // collumn 1
        if(y <= -2){
            // row 4

            bytes[0] = (~(AI_BIT) & IREF7);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM7);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == -1){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF6);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM6);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 0){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF6);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM6);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 1){
            // row 2
            bytes[0] = (~(AI_BIT) & IREF5);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM5);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else{
            // row1
            bytes[0] = (~(AI_BIT) & IREF4);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM4);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }

    }
    else if(x == -1){
       // collumn 2
        if(y <= -2){
            // row 4

            bytes[0] = (~(AI_BIT) & IREF11);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM11);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == -1){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF10);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM10);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 0){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF10);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM10);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 1){
            // row 2
            bytes[0] = (~(AI_BIT) & IREF9);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM9);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }else{
            // row1
            bytes[0] = (~(AI_BIT) & IREF8);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM8);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }

    }
    else if(x == 0){
        // collumn 2
        if(y <= -2){
            // row 4

            bytes[0] = (~(AI_BIT) & IREF11);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM11);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == -1){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF10);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM10);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 0){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF10);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM10);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 1){
            // row 2
            bytes[0] = (~(AI_BIT) & IREF9);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM9);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }else{
            // row1
            bytes[0] = (~(AI_BIT) & IREF8);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM8);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }


    }
    else if(x == 1){
        // collumn 3
        if(y <= -2){
            // row 4

            bytes[0] = (~(AI_BIT) & IREF15);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM15);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == -1){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF14);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM14);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 0){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF14);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM14);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 1){
            // row 2
            bytes[0] = (~(AI_BIT) & IREF13);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM13);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }else{
            // row1
            bytes[0] = (~(AI_BIT) & IREF12);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM12);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }

    }
    else{
        // collumn 4
        if(y <= -2){
            // row 4

            bytes[0] = (~(AI_BIT) & IREF19);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM19);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == -1){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF18);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM18);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 0){
            // row 3
            bytes[0] = (~(AI_BIT) & IREF18);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM18);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }
        else if(y == 1){
            // row 2
            bytes[0] = (~(AI_BIT) & IREF17);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM17);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }else{
            // row1
            bytes[0] = (~(AI_BIT) & IREF16);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);
            bytes[0] = (~(AI_BIT) & PWM16);
            I2C_WriteMultiple(I2C_A_BASE, PCA9956B_ADDR, bytes, 2);

        }

    }

}

uint8_t PCA9956b_GetChipID() {
    I2C_WriteSingle(I2C_A_BASE, PCA9956B_ADDR, 0x01,1);
    return I2C_ReadSingle(I2C_A_BASE, PCA9956B_ADDR,1);
}
