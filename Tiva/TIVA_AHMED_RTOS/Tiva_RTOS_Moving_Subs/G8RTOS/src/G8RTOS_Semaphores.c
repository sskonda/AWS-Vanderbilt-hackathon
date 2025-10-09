// G8RTOS_Semaphores.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for semaphore functions

#include "../G8RTOS_Semaphores.h"

/************************************Includes***************************************/

#include "../G8RTOS_CriticalSection.h"
#include "../G8RTOS_Scheduler.h"
#include <inc/tm4c123gh6pm.h>

/************************************Includes***************************************/

/******************************Data Type Definitions********************************/
/******************************Data Type Definitions********************************/

/****************************Data Structure Definitions*****************************/
/****************************Data Structure Definitions*****************************/

/***********************************Externs*****************************************/
/***********************************Externs*****************************************/

/********************************Public Variables***********************************/




/********************************Public Variables***********************************/

/********************************Public Functions***********************************/

// G8RTOS_InitSemaphore
// Initializes semaphore to a value. This is a critical section!
// Param "s": Pointer to semaphore
// Param "value": Value to initialize semaphore to
// Return: void
void G8RTOS_InitSemaphore(semaphore_t* s, int32_t value) {
    IBit_State = StartCriticalSection();
    *s = value;
    EndCriticalSection(IBit_State);
}

// G8RTOS_WaitSemaphore
// Waits on the semaphore to become available, decrements value by 1. This is a
// critical section!
// Currently spin-locks.
// Param "s": Pointer to semaphore
// Return: void
void G8RTOS_WaitSemaphore(semaphore_t* s) {
    IBit_State = StartCriticalSection();
    (*s)--;
    if(*s < 0){

        CurrentlyRunningThread->blocked = s;

//        NVIC_ST_CURRENT_R = 0;
//        NVIC_INT_CTRL_R = 0x04000000;
        IntPendSet(14);

        EndCriticalSection(IBit_State);

    }

    EndCriticalSection(IBit_State);
}

// G8RTOS_SignalSemaphore
// Signals that the semaphore has been released by incrementing the value by 1.
// This is a critical section!
// Param "s": Pointer to semaphore
// Return: void
void G8RTOS_SignalSemaphore(semaphore_t* s) {
    IBit_State = StartCriticalSection();
    (*s)++;
    if((*s) <= 0){
        tcb_t *pt = CurrentlyRunningThread->nextTCB;
        while(pt->blocked != s){
            pt = pt->nextTCB;
        }
        pt->blocked = 0;
    }
    EndCriticalSection(IBit_State);

}

/********************************Public Functions***********************************/
