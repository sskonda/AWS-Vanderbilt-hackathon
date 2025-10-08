// multimod_PCA9956b.h
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// LED driver header file

#ifndef MULTIMOD_PCA9956B_H_
#define MULTIMOD_PCA9956B_H_

/************************************Includes***************************************/

#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>

#include <driverlib/i2c.h>
#include <driverlib/gpio.h>

/************************************Includes***************************************/

/*************************************Defines***************************************/

#define PCA9956B_ADDR   0x01
#define I2C_SCL_PIN     GPIO_PIN_6
#define I2C_SDA_PIN     GPIO_PIN_7
#define OE_PIN          GPIO_PIN_2
#define LEDOUT0         0x02
#define LEDOUT5         0x07

#define PWM0            0x0A
#define PWM1            0x0B
#define PWM2            0x0C
#define PWM3            0x0D
#define PWM4            0x0E
#define PWM5            0x0F
#define PWM6            0x10
#define PWM7            0x11
#define PWM8            0x12
#define PWM9            0x13
#define PWM10           0x14
#define PWM11           0x15
#define PWM12           0x16
#define PWM13           0x17
#define PWM14           0x18
#define PWM15           0x19
#define PWM16           0x1A
#define PWM17           0x1B
#define PWM18           0x1C
#define PWM19           0x1D
#define PWM20           0x1E
#define PWM21           0x1F

#define IREF0           0x22
#define IREF1           0x23
#define IREF2           0x24
#define IREF3           0x25
#define IREF4           0x26
#define IREF5           0x27
#define IREF6           0x28
#define IREF7           0x29
#define IREF8           0x2A
#define IREF9           0x2B
#define IREF10          0x2C
#define IREF11          0x2D
#define IREF12          0x2E
#define IREF13          0x2F
#define IREF14          0x30
#define IREF15          0x31
#define IREF16          0x32
#define IREF17          0x33
#define IREF18          0x34
#define IREF19          0x35
#define IREF20          0x36
#define IREF21          0x37

#define MODE2           0x01
#define EFLAG0          0x41
#define EFLAG2          0x43
#define PWMALL          0x3F
#define IREFALL         0x40

#define AI_BIT          0x80

/*************************************Defines***************************************/

/******************************Data Type Definitions********************************/
/******************************Data Type Definitions********************************/

/****************************Data Structure Definitions*****************************/
/****************************Data Structure Definitions*****************************/

/***********************************Externs*****************************************/
/***********************************Externs*****************************************/

/********************************Public Variables***********************************/
/********************************Public Variables***********************************/

/********************************Public Functions***********************************/

void PCA9956b_Init(void);
void PCA9956b_SetAllMax(void);
void PCA9956b_SetAllOff(void);
void PCA9956b_EnableOutput(void);
void PCA9956b_DisableOutput(void);
void PCA9956b_SetLED(int16_t x, int16_t y);

uint8_t PCA9956b_GetChipID(void);

/********************************Public Functions***********************************/

/*******************************Private Variables***********************************/
/*******************************Private Variables***********************************/

/*******************************Private Functions***********************************/
/*******************************Private Functions***********************************/

#endif /* MULTIMOD_PCA9956B_H_ */


