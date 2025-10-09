// G8RTOS_Scheduler.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for scheduler functions

#include "../G8RTOS_Scheduler.h"

/************************************Includes***************************************/

#include <stdint.h>
#include <stdbool.h>

#include "../G8RTOS_CriticalSection.h"

#include <inc/hw_memmap.h>
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include <inc/tm4c123gh6pm.h>
#include "../G8RTOS_Semaphores.h"

/********************************Private Variables**********************************/

// Thread Control Blocks - array to hold information for each thread
static tcb_t threadControlBlocks[MAX_THREADS];

// Thread Stacks - array of arrays for individual stacks of each thread
static uint32_t threadStacks[MAX_THREADS][STACKSIZE];

// Periodic Event Threads - array to hold pertinent information for each thread
static ptcb_t pthreadControlBlocks[MAX_PTHREADS];

// Current Number of Threads currently in the scheduler
static uint32_t NumberOfThreads;

// Current Number of Periodic Threads currently in the scheduler
static uint32_t NumberOfPThreads;

static uint32_t threadCounter = 0;


/*******************************Private Functions***********************************/

// Occurs every 1 ms.
static void InitSysTick(void)
{
    // Replace with code from lab 3
    SysTickDisable();
    // hint: use SysCtlClockGet() to get the clock speed without having to hardcode it!
    SysTickPeriodSet((SysCtlClockGet()/1000));

    // Set systick period to overflow every 1 ms.
    // Set systick interrupt handler
    IntRegister(FAULT_SYSTICK, SysTick_Handler);
    // Set pendsv handler
    IntRegister(FAULT_PENDSV, PendSV_Handler);
    // Enable systick interrupt
    SysTickIntEnable();
    // Enable systick
    SysTickEnable();
}


/********************************Public Variables***********************************/

uint32_t SystemTime;

tcb_t* CurrentlyRunningThread;

tcb_t* CurrentlySleepingThread;



/********************************Public Functions***********************************/

// SysTick_Handler
// Increments system time, sets PendSV flag to start scheduler.
// Return: void
void SysTick_Handler() {
    SystemTime++;

    if(NumberOfPThreads > 0){

        ptcb_t *Pptr = pthreadControlBlocks[0].nextPTCB;
        while (Pptr != &pthreadControlBlocks[0]){
            if (SystemTime >= Pptr->executeTime){
                Pptr->handler();
                Pptr->currentTime = SystemTime;
                Pptr->executeTime = SystemTime + Pptr->period;
            }
            Pptr = Pptr->nextPTCB;
        }
        if (SystemTime == Pptr->executeTime){
            Pptr->handler();
            Pptr->currentTime = SystemTime;
            Pptr->executeTime = SystemTime + Pptr->period;
        }

    }

    tcb_t *ptr = CurrentlyRunningThread->nextTCB;

    tcb_t *compare = CurrentlyRunningThread;
    while (!(compare->isAlive)){
        // shift compare and ptr with each other to fulfill condition in the next while loop
        compare = compare->nextTCB;
        ptr = ptr->nextTCB;
    }

    // Wake up sleeping threads
//    if(CurrentlySleepingThread != 0){
//
//        if(CurrentlySleepingThread->nextTCB == CurrentlySleepingThread){
//
//            if(CurrentlySleepingThread->sleepCount == SystemTime){
//
//                CurrentlySleepingThread->asleep = 0;
//
//                // remove from linked list
//                CurrentlyRunningThread->nextTCB->previousTCB = CurrentlySleepingThread;
//                CurrentlySleepingThread->nextTCB = CurrentlyRunningThread->nextTCB;
//
//                // create a holder
//                tcb_t *cstPtr = CurrentlySleepingThread;
//
//                // add to linked list
//                cstPtr->nextTCB = CurrentlyRunningThread->nextTCB;
//                cstPtr->previousTCB = CurrentlyRunningThread;
//                CurrentlyRunningThread->nextTCB = cstPtr;
//                cstPtr->nextTCB->previousTCB = cstPtr;
//
//                CurrentlySleepingThread = 0;
//
//
//            }
//
//        }
//        else{
//
//            if(CurrentlySleepingThread->sleepCount == SystemTime){
//                CurrentlySleepingThread->asleep = 0;
//
//                // remove from linked list
//                CurrentlySleepingThread->previousTCB->nextTCB = CurrentlySleepingThread->nextTCB;
//                CurrentlySleepingThread->nextTCB->previousTCB = CurrentlySleepingThread->previousTCB;
//
//                // create a holder
//                tcb_t *cstPtr = CurrentlySleepingThread;
//
//                // change CurrentlySleepingThread
//                CurrentlySleepingThread = CurrentlySleepingThread->nextTCB;
//
//                // add cst to linked list
//                cstPtr->nextTCB = CurrentlyRunningThread->nextTCB;
//                cstPtr->previousTCB = CurrentlyRunningThread;
//                CurrentlyRunningThread->nextTCB = cstPtr;
//                cstPtr->nextTCB->previousTCB = cstPtr;
//
//
//            }
//
//        }
//
//    }


    while(ptr != compare){
        if((ptr->asleep) && (ptr->sleepCount == SystemTime)){
            ptr->asleep = 0;
        }
        ptr = ptr->nextTCB;
    }
    // edge case
    if((ptr->asleep) && (ptr->sleepCount == SystemTime)){
        ptr->asleep = 0;
    }

    IntPendSet(FAULT_PENDSV);
    // Traverse the linked-list to find which threads should be awake.
    // Traverse the periodic linked list to run which functions need to be run.
}

// G8RTOS_Init
// Initializes the RTOS by initializing system time.
// Return: void
void G8RTOS_Init() {
    uint32_t newVTORTable = 0x20000000;
    uint32_t* newTable = (uint32_t*)newVTORTable;
    uint32_t* oldTable = (uint32_t*) 0;

    for (int i = 0; i < 155; i++) {
        newTable[i] = oldTable[i];
    }

    HWREG(NVIC_VTABLE) = newVTORTable;

    SystemTime = 0;
    NumberOfThreads = 0;
    NumberOfPThreads = 0;
}

// G8RTOS_Launch
// Launches the RTOS.
// Return: error codes, 0 if none
int32_t G8RTOS_Launch() {

    // Initialize system tick
    InitSysTick();
    // Set currently running thread to the first control block
    CurrentlyRunningThread = &threadControlBlocks[0];
    // Set interrupt priorities
       // Pendsv
       // Systick
    // Call G8RTOS_Start()
    G8RTOS_Start();

    return 0;
}

// G8RTOS_Scheduler
// Chooses next thread in the TCB. This time uses priority scheduling.
// Return: void
void G8RTOS_Scheduler() {
    // Using priority, determine the most eligible thread to run that
    // is not blocked or asleep. Set current thread to this thread's TCB.



    while(!(CurrentlyRunningThread->isAlive)){
        CurrentlyRunningThread = CurrentlyRunningThread->nextTCB;
    }

    if(CurrentlyRunningThread->asleep){
        CurrentlyRunningThread = &threadControlBlocks[0];
    }

    tcb_t *ptr = CurrentlyRunningThread->nextTCB;
    tcb_t *execute = NULL;
    uint8_t minPrio = 255;

    while(ptr != CurrentlyRunningThread){
        if((ptr->blocked) || (ptr->asleep)){
            ptr = ptr->nextTCB;
            continue;
        }
        if(ptr->priority <= minPrio){
            minPrio = ptr->priority;
            execute = ptr;
        }

        ptr = ptr->nextTCB;
    }
    // edge case
    if(!((ptr->blocked) || (ptr->asleep))){
        if(ptr->priority < minPrio){
            execute = ptr;
        }
    }

    // If there is nothing to execute
    if(execute == NULL){
        while(1);
    }

    CurrentlyRunningThread = execute;


}

// G8RTOS_AddThread
// Adds a thread. This is now in a critical section to support dynamic threads.
// It also now should initalize priority and account for live or dead threads.
// Param void* "threadToAdd": pointer to thread function address
// Param uint8_t "priority": priority from 0, 255.
// Param char* "name": character array containing the thread name.
// Return: sched_ErrCode_t
sched_ErrCode_t G8RTOS_AddThread(void (*threadToAdd)(void), uint8_t priority, char *name, threadID_t threadID) {
    // Your code here

    // This should be in a critical section!

    IBit_State = StartCriticalSection();

    uint8_t index = 0;

    if (threadCounter >= MAX_THREADS){
        EndCriticalSection(IBit_State);
        return THREAD_LIMIT_REACHED;

    }
    else if (threadCounter == 0){
        //new thread next = first one
        threadControlBlocks[0].nextTCB = &threadControlBlocks[0];
        // new thread previous = previous thread
        threadControlBlocks[0].previousTCB = &threadControlBlocks[0];
    }
    else{
        //new thread next = first one

        for(int i = 0; i < MAX_THREADS; i++){
            if(threadControlBlocks[i].isAlive == 0){

                index = i;
                NumberOfThreads -= 1;
                break;

            }
        }

        threadControlBlocks[index].nextTCB = &threadControlBlocks[0];

        // new thread previous = previous thread
        threadControlBlocks[index].previousTCB = threadControlBlocks[0].previousTCB;

        // previous thread next = new thread
        threadControlBlocks[0].previousTCB = &threadControlBlocks[index];

        // first thread previous = new thread
        threadControlBlocks[index].previousTCB->nextTCB = &threadControlBlocks[index];
    }

    // set TCB
    threadControlBlocks[index].stackPointer = &threadStacks[index][STACKSIZE-16];
    threadControlBlocks[index].asleep = 0;
    threadControlBlocks[index].priority = priority;
    threadControlBlocks[index].ThreadID = threadID;
    threadControlBlocks[index].isAlive = 1;

    if (name != NULL) {
        int i = 0;
        while (i < 16 && name[i] != '\0') {
            threadControlBlocks[index].threadName[i] = name[i];
            i++;
        }
    }

    // PC (Pointer to the function)
    threadStacks[index][STACKSIZE-2] = (uint32_t)threadToAdd;
    // fake context (Status Register)
    threadStacks[index][STACKSIZE-1] = THUMBBIT;

    NumberOfThreads += 1;
    threadCounter += 1;

    EndCriticalSection(IBit_State);
    return NO_ERROR;
}

// G8RTOS_Add_APeriodicEvent


// Param void* "AthreadToAdd": pointer to thread function address
// Param int32_t "IRQn": Interrupt request number that references the vector table. [0..155].
// Return: sched_ErrCode_t
sched_ErrCode_t G8RTOS_Add_APeriodicEvent(void (*AthreadToAdd)(void), uint8_t priority, int32_t IRQn) {
    // Disable interrupts
    IBit_State = StartCriticalSection();
    // Check if IRQn is valid
    if (IRQn > 155 || IRQn < 0){
        EndCriticalSection(IBit_State);
        return;
    }
    // Check if priority is valid
    if (priority > 6 || priority < 0){
        EndCriticalSection(IBit_State);
        return;
    }
    // Set corresponding index in interrupt vector table to handler.
    IntRegister(IRQn, AthreadToAdd);
    // Set priority.
    IntPrioritySet(IRQn, priority);
    // Enable the interrupt.
    IntEnable(IRQn);

    IntPendClear(IRQn);
    // End the critical section.
    EndCriticalSection(IBit_State);
    return;

}

// G8RTOS_Add_PeriodicEvent
// Adds periodic threads to G8RTOS Scheduler
// Function will initialize a periodic event struct to represent event.
// The struct will be added to a linked list of periodic events
// Param void* "PThreadToAdd": void-void function for P thread handler
// Param uint32_t "period": period of P thread to add
// Param uint32_t "execution": When to execute the periodic thread
// Return: sched_ErrCode_t
sched_ErrCode_t G8RTOS_Add_PeriodicEvent(void (*PThreadToAdd)(void), uint32_t period, uint32_t execution) {
    // your code
    // Make sure that the number of PThreads is not greater than max PThreads.
        // Check if there is no PThread. Initialize and set the first PThread.
        // Subsequent PThreads should be added, inserted similarly to a doubly-linked linked list
            // last PTCB should point to first, first PTCB should point to last.
        // Set function
        // Set period
        // Set execute time
        // Increment number of PThreads

    if(NumberOfPThreads >= MAX_PTHREADS){
        return;
    }
    else if(NumberOfPThreads == 0){
        pthreadControlBlocks[NumberOfPThreads].handler = PThreadToAdd;
        pthreadControlBlocks[NumberOfPThreads].previousPTCB = &pthreadControlBlocks[NumberOfPThreads];
        pthreadControlBlocks[NumberOfPThreads].nextPTCB = &pthreadControlBlocks[NumberOfPThreads];
        pthreadControlBlocks[NumberOfPThreads].period = period;
        pthreadControlBlocks[NumberOfPThreads].currentTime = SystemTime;
        pthreadControlBlocks[NumberOfPThreads].executeTime = execution;

    }
    else{
        pthreadControlBlocks[NumberOfPThreads].handler = PThreadToAdd;
        pthreadControlBlocks[NumberOfPThreads].previousPTCB = &pthreadControlBlocks[NumberOfPThreads-1];
        pthreadControlBlocks[NumberOfPThreads].nextPTCB = &pthreadControlBlocks[0];

        pthreadControlBlocks[NumberOfPThreads - 1].nextPTCB = &pthreadControlBlocks[NumberOfPThreads];
        pthreadControlBlocks[0].previousPTCB = &pthreadControlBlocks[NumberOfPThreads];

        pthreadControlBlocks[NumberOfPThreads].period = period;
        pthreadControlBlocks[NumberOfPThreads].currentTime = SystemTime;
        pthreadControlBlocks[NumberOfPThreads].executeTime = execution;
    }


    NumberOfPThreads += 1;

    return NO_ERROR;
}

// G8RTOS_KillThread
// Param uint32_t "threadID": ID of thread to kill
// Return: sched_ErrCode_t

// IF A KILLED THREAD GRABS A SEMAPHORE, THE SEMAPHORE NEEDS TO BE RELEASED OR THE THREAD SHOULDN'T BE ALLOWED TO GRAB IT

sched_ErrCode_t G8RTOS_KillThread(threadID_t threadID) {
    // Start critical section
    IBit_State = StartCriticalSection();
    // Check if there is only one thread, return if so
    if (threadCounter == 1){

        EndCriticalSection(IBit_State);
        return CANNOT_KILL_LAST_THREAD;
    }
    // Traverse linked list, find thread to kill
        // Update the next tcb and prev tcb pointers if found
            // mark as not alive, release the semaphore it is blocked on
        // Otherwise, thread does not exist.
    for(int i = 0; i < NumberOfThreads; i++){
        if(threadControlBlocks[i].ThreadID == threadID){

            if(threadControlBlocks[i].blocked != 0){
                G8RTOS_SignalSemaphore(threadControlBlocks[i].blocked);
                threadControlBlocks[i].blocked = 0;
            }

            threadControlBlocks[i].isAlive = 0;
            threadControlBlocks[i].nextTCB->previousTCB = threadControlBlocks[i].previousTCB;
            threadControlBlocks[i].previousTCB->nextTCB = threadControlBlocks[i].nextTCB;
            break;
        }
    }

    threadCounter -=1;

    EndCriticalSection(IBit_State);

    return NO_ERROR;

}

// G8RTOS_KillSelf
// Kills currently running thread.
// Return: sched_ErrCode_t
sched_ErrCode_t G8RTOS_KillSelf() {
    // your code
    IBit_State = StartCriticalSection();
/*(
    if (threadCounter == 1){
        EndCriticalSection(IBit_State);
        return CANNOT_KILL_LAST_THREAD;
    }*/
    CurrentlyRunningThread->isAlive = 0;
    CurrentlyRunningThread->nextTCB->previousTCB = CurrentlyRunningThread->previousTCB;
    CurrentlyRunningThread->previousTCB->nextTCB = CurrentlyRunningThread->nextTCB;

    CurrentlyRunningThread->stackPointer = (uint32_t*)(0x00);
    //CurrentlyRunningThread = CurrentlyRunningThread->nextTCB;

    threadCounter -= 1;

//    NVIC_ST_CURRENT_R = 0;
//    NVIC_INT_CTRL_R = 0x04000000;

    IntPendSet(FAULT_PENDSV);

    EndCriticalSection(IBit_State);

    return NO_ERROR;

    // Check if there is only one thread
    // Else, mark this thread as not alive.

}

// sleep
// Puts current thread to sleep
// Param uint32_t "durationMS": how many systicks to sleep for
void sleep(uint32_t durationMS) {
    // Update time to sleep to
    // Set thread as asleep

//    IBit_State = StartCriticalSection();
//
//    tcb_t *ptr = CurrentlySleepingThread;
//
//    CurrentlyRunningThread->previousTCB->nextTCB = CurrentlyRunningThread->nextTCB;
//    CurrentlyRunningThread->nextTCB->previousTCB = CurrentlyRunningThread->previousTCB;
//
//    if(CurrentlySleepingThread == 0){
//        CurrentlySleepingThread = CurrentlyRunningThread;
//        CurrentlySleepingThread->nextTCB = CurrentlySleepingThread;
//        CurrentlySleepingThread->previousTCB = CurrentlySleepingThread;
//        CurrentlySleepingThread->sleepCount = SystemTime + durationMS;
//        CurrentlySleepingThread->asleep = 1;
//
//    }
//    else{
//
//        do{
//            CurrentlyRunningThread->sleepCount = SystemTime + durationMS;
//            CurrentlyRunningThread->asleep = 1;
//
//            if(CurrentlyRunningThread->sleepCount < ptr->sleepCount){
//
//                ptr->previousTCB->nextTCB = CurrentlyRunningThread;
//                CurrentlyRunningThread->previousTCB = ptr->previousTCB;
//                ptr->previousTCB = CurrentlyRunningThread;
//                CurrentlyRunningThread->nextTCB = ptr;
//                if(ptr == CurrentlySleepingThread){
//                    CurrentlySleepingThread = CurrentlyRunningThread;
//                }
//
//                IntPendSet(FAULT_PENDSV);
//
//                EndCriticalSection(IBit_State);
//
//                return;
//
//            }
//            ptr = ptr->nextTCB;
//        }while(ptr != CurrentlySleepingThread);
//    }
//
//    CurrentlySleepingThread->previousTCB->nextTCB = CurrentlyRunningThread;
//    CurrentlyRunningThread->nextTCB = CurrentlySleepingThread;
//    CurrentlyRunningThread->previousTCB = CurrentlySleepingThread->previousTCB;
//    CurrentlySleepingThread->previousTCB = CurrentlyRunningThread;
//
//
//    IntPendSet(FAULT_PENDSV);
//
//    EndCriticalSection(IBit_State);
//    return;

    IBit_State = StartCriticalSection();

    CurrentlyRunningThread->asleep = 1;
    CurrentlyRunningThread->sleepCount = SystemTime + durationMS;

    IntPendSet(FAULT_PENDSV);

    EndCriticalSection(IBit_State);

    return;

}

// G8RTOS_GetThreadID
// Gets current thread ID.
// Return: threadID_t
threadID_t G8RTOS_GetThreadID(void) {
    return CurrentlyRunningThread->ThreadID;        //Returns the thread ID
}

// G8RTOS_GetNumberOfThreads
// Gets number of threads.
// Return: uint32_t
uint32_t G8RTOS_GetNumberOfThreads(void) {
    return NumberOfThreads;         //Returns the number of threads
}
