/* Dalton Banks - EXTERN gave an error */
.global SVC_Handler
.global nrk_start_high_ready_task 
.global _nrk_spin_wait_us

nrk_start_high_ready_task:

    /* Use the NVIC offset register to locate the stack. */
    ldr r0, =0xE000ED08
    ldr r0, [r0]
    ldr r0, [r0]
    /* Set the msp back to the start of the stack. */
    msr msp, r0
    /* Call SVC to start the first task. */
    cpsie i
    svc 0

_nrk_spin_wait_us:

    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop

SVC_Handler:

    /* Restore the context. */
    ldr	r3, =nrk_high_ready_TCB
    /* Use nrk_high_ready_TCB to get the nrk_high_ready_TCB address. */
    ldr r1, [r3]
    /* The first item in nrk_high_ready_TCB is the task top of stack. */
    ldr r0, [r1]
    /* Pop the registers that are not automatically saved on exception entry. */
    /* Dalton Banks - not working */
    /* ldmia r0!, {r4-r11} */
    /* Restore the task stack pointer. */
    msr psp, r0
    /* Dalton Banks - not working */
	/* mov r0, #0 */
    msr	basepri, r0						
    /* Dalton Banks - not working */
    /* orr r14, #0xd */
    bx r14																	
    	
