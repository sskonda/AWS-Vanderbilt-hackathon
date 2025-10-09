; G8RTOS_SchedulerASM.s
; Created: 2022-07-26
; Updated: 2022-07-26
; Contains assembly functions for scheduler.

	; Functions Defined
	.def G8RTOS_Start, PendSV_Handler

	; Dependencies
	.ref CurrentlyRunningThread, G8RTOS_Scheduler

	.thumb		; Set to thumb mode
	.align 2	; Align by 2 bytes (thumb mode uses allignment by 2 or 4)
	.text		; Text section

; Need to have the address defined in file
; (label needs to be close enough to asm code to be reached with PC relative addressing)
RunningPtr: .field CurrentlyRunningThread, 32

; G8RTOS_Start
;	Sets the first thread to be the currently running thread
;	Starts the currently running thread by setting Link Register to tcb's Program Counter
G8RTOS_Start:

	.asmfunc

	; Disable interrupts
	CPSID I

	; Load the address of RunningPtr
	ldr r4, RunningPtr ; r4 = runningptr

	; Load the address of the thread control block of the currently running pointer
	ldr r5, [r4] ; r5 = ptr to stack ptr

	; Load the first thread's stack pointer
	ldr r6, [r5] ; r6 = stack ptr
	ADD r6, #60 ; moves stack ptr to THUMBBIT
	str r6, [r5] ; stores r6 (stack ptr) into memory location pointed to by r5 (stack ptr)

	;Move stack ptr to beginning of stack
	mov sp, r6 ; place stack ptr into the actual stack ptr


	; Load LR with the first thread's PC
	ldr lr, [r6, #-4] ; load lr (link register) with the address of the function ptr (PC)

	; Enables Interrupts
	CPSIE I

	BX LR				;Branches to the first thread

	.endasmfunc

; PendSV_Handler
; - Performs a context switch in G8RTOS
; 	- Saves remaining registers into thread stack
;	- Saves current stack pointer to tcb
;	- Calls G8RTOS_Scheduler to get new tcb
;	- Set stack pointer to new stack pointer from new tcb
;	- Pops registers from thread stack
PendSV_Handler:

	.asmfunc

	; Disable Interrupts
	CPSID I

	; push r4 - r11 to stack
	push {R4 - R11}

	; Load the address of RunningPtr
	ldr r0, RunningPtr ; r0 = runningptr

	; Load the address of the thread control block of the currently running pointer
	ldr r1, [r0] ; r1 = ptr to stack ptr

	str sp, [r1] ; store stack ptr into tcb (save context)

	push {r0, LR}

	BL G8RTOS_Scheduler

	pop{r0, LR}

	ldr r1, [r0] ;	r1 = RunPt, new thread

	ldr sp, [r1] ;  store new context stack ptr into the stack ptr

	; pop r4-r11
	pop {R4-R11}

	;enable interrupts
	CPSIE I

	; Branch to new thread
	BX LR


	.endasmfunc

	; end of the asm file
	.align
	.end
