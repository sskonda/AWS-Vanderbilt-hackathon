// multimod_OPT3001.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for OPT3001 functions

/************************************Includes***************************************/

#include "../multimod_OPT3001.h"

#include <stdint.h>
#include "../multimod_i2c.h"

/************************************Includes***************************************/

#define OPT3001_CONFIG_ADDR     0x01

/********************************Public Functions***********************************/

// OPT3001_Init
// Initializes OPT3001, configures it to continuous conversion mode.
//0x47
// Return: void
void OPT3001_Init(void) {
    // Initialize I2C module
    // DOES IT IN MAIN
    I2C_Init(I2C1_BASE);
    // Set the correct configuration byte for continuous conversions
    uint16_t temp = OPT3001_ReadRegister(OPT3001_CONFIG_ADDR);
    OPT3001_WriteRegister(OPT3001_CONFIG_ADDR,(temp | 0b0000011000000000));
    return;
}

// OPT3001_WriteRegister
// Writes to a register in the OPT3001.
// Param uint8_t "addr": Register address of the OPT3001.
// Param uint16_t "data": 16-bit data to write to the register.
// Return: void
void OPT3001_WriteRegister(uint8_t addr, uint16_t data) {
    // Read the datasheet!
    // Complete this function

    uint8_t temp[3] = {addr, (uint8_t)(data >> 8), (uint8_t)(data)};

    I2C_WriteMultiple(I2C1_BASE, 0x47, temp, 3);

    return;

}

// OPT3001_ReadRegister
// Reads from a register in the OPT3001.
// Param uint8_t "addr": Register address of the OPT3001.
// Return: uint16_t
uint16_t OPT3001_ReadRegister(uint8_t addr) {

    uint8_t temp[2] = {addr, 0};
    uint16_t data;

    I2C_WriteSingle(I2C1_BASE, 0x47, addr, 1);
    I2C_ReadMultiple(I2C1_BASE, 0x47, temp, 2);

    data = (temp[0] << 8) | temp[1];

    return data;

    // Complete this function
}

// OPT3001_GetResult
// Gets conversion result, calculates byte result based on datasheet
// and configuration settings.
// Return: uint32_t
uint32_t OPT3001_GetResult(void) {
    // Check if data is ready first
    //uint16_t potato = OPT3001_ReadRegister(OPT3001_CONFIG_ADDR);
    //while(!(potato & 0x80)){

        //UARTprintf("OPT3001: 0x%04X\n", potato);

    //}

    uint16_t result = OPT3001_ReadRegister(OPT3001_RESULT_ADDR);

    result = LUX((result >> 12 & 0xF), (result & 0x0FFF));

    return result;
}

// OPT3001_SetLowLimit
// Sets the low limit register.
// Param uint16_t "exp": Exponential bound
// Param uint16_t "result": Conversion bound
// Return: void
void OPT3001_SetLowLimit(uint16_t exp, uint16_t result) {
    OPT3001_WriteRegister(OPT3001_LOWLIMIT_ADDR, (exp << OPT3001_RESULT_E_S | (result & 0xFFF)));

    return;
}

// OPT3001_SetHighLimit
// Sets the high limit register.
// Param uint16_t "exp": Exponential bound
// Param uint16_t "result": Conversion bound
// Return: void
void OPT3001_SetHighLimit(uint16_t exp, uint16_t result) {
    OPT3001_WriteRegister(OPT3001_HIGHLIMIT_ADDR, (exp << OPT3001_RESULT_E_S | (result & 0xFFF)));

    return;
}

// OPT3001_GetChipID
// Gets the chip ID of the OPT3001.
// Return: uint16_t
uint16_t OPT3001_GetChipID(void) {
    return OPT3001_ReadRegister(OPT3001_DEVICEID_ADDR);
}

/********************************Public Functions***********************************/
