
/************************************Includes***************************************/

#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"
#include "final_threads.h"
#include <time.h>

/************************************MAIN*******************************************/




int main(void)
{
    // Sets clock speed to 80 MHz. You'll need it!
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    G8RTOS_Init();
    multimod_init();

    // Add threads, semaphores, here
    G8RTOS_InitSemaphore(&sem_UART, 1);
    G8RTOS_InitSemaphore(&sem_GPIOE, 1);
    G8RTOS_InitSemaphore(&sem_GPIOD, 1);
    G8RTOS_InitSemaphore(&sem_SPI, 1);

    // Init FIFO
    G8RTOS_InitFIFO(0);
    G8RTOS_InitFIFO(1);

    // init Threads
    G8RTOS_AddThread(Idle_Thread, 254, "idle thread", 0); // Thread 0
    G8RTOS_AddThread(Draw_Border, 2, "draw border", 1);
    G8RTOS_AddThread(Draw_Player, 2, "draw player", 2);
    G8RTOS_AddThread(Enemy, 2, "enemy", 3);
    G8RTOS_AddThread(Move_Player, 2, "move player", 4);
    G8RTOS_AddThread(Read_Buttons, 1, "button", 5);
    G8RTOS_AddThread(Draw_Obstacles, 2, "draw_obs", 6);
    G8RTOS_AddThread(Animation, 0, "ani", 7);
    G8RTOS_AddThread(State, 2, "state", 8);




    G8RTOS_Add_PeriodicEvent(Get_Joystick, 100, 1);
    G8RTOS_Add_PeriodicEvent(Add_Enemy, 5000, 5000);
    G8RTOS_Add_PeriodicEvent(Fix_Screen, 1000, 200);

    G8RTOS_Add_APeriodicEvent(GPIOE_Handler, 4, INT_GPIOE);
    //G8RTOS_Add_APeriodicEvent(GPIOE_Handler, 4, INT_GPIOE);
    //G8RTOS_Add_APeriodicEvent(GPIOD_Handler, 4, INT_GPIOD);


    G8RTOS_Launch();
    while (1);
}




