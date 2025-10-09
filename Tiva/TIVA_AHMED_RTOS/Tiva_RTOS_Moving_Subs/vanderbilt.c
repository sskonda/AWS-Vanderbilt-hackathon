/************************************Includes***************************************/

#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"

/************************************MAIN*******************************************/

#include "general_drivers.h"
#include "vanderbilt_threads.h"

int main(void)
{
    // Sets clock speed to 80 MHz. You'll need it!
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    G8RTOS_Init();
    multimod_init();

    // Semaphore
    G8RTOS_InitSemaphore(&sem_GPIOE, 1);
    G8RTOS_InitSemaphore(&sem_SPI, 1);
    G8RTOS_InitSemaphore(&sem_UART, 1);
    G8RTOS_InitSemaphore(&sem_UART4, 1);
    G8RTOS_InitSemaphore(&sem_I2C, 1);

    // FIFO
    G8RTOS_InitFIFO(0);
    G8RTOS_InitFIFO(1);

    // Threads
    G8RTOS_AddThread(Idle_Thread, 254, "idle thread", 0); // Thread 0
    G8RTOS_AddThread(Read_Buttons, 1, "read buttons", 1); // Thread 1
    G8RTOS_AddThread(Draw_Subs, 2, "draw subs", 2);       // Thread 2
    G8RTOS_AddThread(Read_ESP32, 2, "read esp", 3);       // Thread 3
    G8RTOS_AddThread(Draw_Data, 2, "draw data", 4);     // Thread 4
    G8RTOS_AddThread(BeagleBone_Do, 2, "handle beagle", 5);// Thread 5
    G8RTOS_AddThread(Get_Data, 2, "get data", 6);       // Thread 6
    G8RTOS_AddThread(Draw_Position, 2, "draw position", 7); // Thread 7
    G8RTOS_AddThread(Send_PO_Data, 2, "send PO data", 8); // Thread 8

    // Periodic Threads
    G8RTOS_Add_PeriodicEvent(Get_Joystick, 100, 1);

    // Interrupt Threads
    G8RTOS_Add_APeriodicEvent(GPIOE_Handler, 4, INT_GPIOE);
    G8RTOS_Add_APeriodicEvent(UART4_Handler, 4, INT_UART4);

    G8RTOS_Launch();




    while(1);
}
