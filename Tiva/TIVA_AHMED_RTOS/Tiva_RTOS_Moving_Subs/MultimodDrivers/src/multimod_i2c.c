// multimod_i2c.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for I2C functions

/************************************Includes***************************************/

#include "../multimod_i2c.h"

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>

#include <inc/tm4c123gh6pm.h>
#include <inc/hw_i2c.h>

/************************************Includes***************************************/

/********************************Public Functions***********************************/

// I2C_Init
// Initializes specified I2C module
// Param uint32_t "mod": base address of module
// Return: void
void I2C_Init(uint32_t mod) {
    // Note:    The multimod board uses multiple I2C modules
//              to communicate with different devices. You can use
    //          the 'mod' parameter to choose which module to initialize
    //          and use.

     //Enable clock to relevant I2C and GPIO modules
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1)){}

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}

    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    I2CMasterInitExpClk(mod, SysCtlClockGet(), false);

    I2CMasterEnable(mod);




    // Configure pins for I2C module

    // Configure I2C SCL speed, set as master

}

// I2C_WriteSingle
// Writes a single byte to an address.
// Param uint32_t "mod": base address of module
// Param uint8_t "addr": address to device
// Param uint8_t "byte": byte to send
// Return: void
void I2C_WriteSingle(uint32_t mod, uint8_t addr, uint8_t byte, uint8_t stop) {
    // Set the address in the slave address register
    I2C1_MSA_R = ((addr << 1) & 0xFE);
    // Input data into I2C module
    I2C1_MDR_R = byte;
    // Trigger I2C module send
    if (stop){
        I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_STOP;
    }
    else{
        I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    }
    // Wait until I2C module is no longer busy
    while(I2C1_MCS_R & I2C_MCS_BUSY);

    return;
}

// I2C_ReadSingle
// Reads a single byte from address.
// Param uint32_t "mod": base address of module
// Param uint8_t "addr": address to device
// Return: uint8_t
uint8_t I2C_ReadSingle(uint32_t mod, uint8_t addr, uint8_t stop) {

    uint8_t data;

    // Set the address in the slave address register
    I2C1_MSA_R = ((addr << 1) & 0xFE) | 0x1;
    // Start Receive
    if (stop){
        I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_STOP;
    }
    else{
        I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    }
    // Wait until I2C module is no longer busy
    while(I2C1_MCS_R & I2C_MCS_BUSY);
    // get data
    data = I2C1_MDR_R;

    return data;
}

// I2C_WriteMultiple
// Write multiple bytes to a device.
// Param uint32_t "mod": base address of module
// Param uint8_t "addr": address to device
// Param uint8_t* "data": pointer to an array of bytes
// Param uint8_t "num_bytes": number of bytes to transmit
// Return: void
void I2C_WriteMultiple(uint32_t mod, uint8_t addr, uint8_t* data, uint8_t num_bytes) {
    // Set the address in the slave address register
    I2C1_MSA_R = ((addr << 1) & 0xFE);
    // Input data into I2C module
    I2C1_MDR_R = data[0];
    // Trigger I2C module send
    I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    // Wait until I2C module is no longer busy
    while(I2C1_MCS_R & I2C_MCS_BUSY);
    // While num_bytes > 1
        // Input data into I2C module
        // Trigger I2C module send
        // Wait until I2C module is no longer busy

    for (int i = 1; i < num_bytes - 1; i++){

        I2C1_MDR_R = data[i];
        I2C1_MCS_R = I2C_MCS_RUN;
        while(I2C1_MCS_R & I2C_MCS_BUSY);

    }

    // Input last byte into I2C module
    I2C1_MDR_R = data[num_bytes-1];
    // Trigger I2C module send
    I2C1_MCS_R = I2C_MCS_RUN | I2C_MCS_STOP;
    // Wait until I2C module is no longer busy
    while(I2C1_MCS_R & I2C_MCS_BUSY);

    return;
}

// I2C_ReadMultiple
// Read multiple bytes from a device.
// Param uint32_t "mod": base address of module
// Param uint8_t "addr": address to device
// Param uint8_t* "data": pointer to an array of bytes
// Param uint8_t "num_bytes": number of bytes to read
// Return: void
void I2C_ReadMultiple(uint32_t mod, uint8_t addr, uint8_t* data, uint8_t num_bytes) {

    // Receive 1 case
    if (num_bytes == 1){
        I2C1_MSA_R = ((addr << 1) & 0xFE) | 0x1;
        // Trigger I2C module receive
        I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_STOP;
        // Wait until I2C module is no longer busy
        while(I2C1_MCS_R & I2C_MCS_BUSY);
        // Read received data
        data[0] = I2C1_MDR_R;

        return;
    }

    I2C1_MSA_R = ((addr << 1) & 0xFE) | 0x1;
    // Trigger I2C module receive
    I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_ACK;
    // Wait until I2C module is no longer busy
    while(I2C1_MCS_R & I2C_MCS_BUSY);
    // Read received data
    data[0] = I2C1_MDR_R;
    // While num_bytes > 1
        // Trigger I2C module receive
        // Wait until I2C module is no longer busy
        // Read received data

    for(int i = 1; i < num_bytes - 1; i++){
        I2C1_MCS_R = I2C_MCS_RUN | I2C_MCS_ACK;
        while(I2C1_MCS_R & I2C_MCS_BUSY);
        data[i] = I2C1_MDR_R;
    }

    // Trigger I2C module receive
    I2C1_MCS_R = I2C_MCS_RUN | I2C_MCS_STOP;
    // Wait until I2C module is no longer busy
    while(I2C1_MCS_R & I2C_MCS_BUSY);
    // Read last byte
    data[num_bytes - 1] = I2C1_MDR_R;


    return;
}

/********************************Public Functions***********************************/

