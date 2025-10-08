#pragma once

#include "./G8RTOS/G8RTOS.h"

semaphore_t sem_GPIOE;
semaphore_t sem_SPI;

void Idle_Thread(void);

void GPIOE_Handler(void);

void Read_Buttons(void);
void Draw_Subs(void);
void Draw_Depth(void);

void Read_ESP32(void);
