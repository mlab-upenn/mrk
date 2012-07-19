.section .text, "x"
.syntax unified
.global nrk_start_high_ready_task
.global ENABLE_GLOBAL_INT
.global DISABLE_GLOBAL_INT
.global _nrk_spin_wait_us
.global SVC_Handler
.global led
.global print_test
.equ STACK_OFFSET, 4
    
ENABLE_GLOBAL_INT:
    cpsie i 
DISABLE_GLOBAL_INT:
    cpsid i 

led:
    LDR     R1, =0x2009C020 
    LDR   R2, =0x00040000  
    CMP     R0, #0       
    IT EQ
    STREQ   R2, [R1,#0x1C]  
    IT NE
    STRNE   R2, [R1,#0x18] 
    BX      LR
 

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


SVC_Handler:
    /* Restore the context. */
    ldr	r3, =nrk_high_ready_TCB
    /* Use nrk_high_ready_TCB to get the nrk_high_ready_TCB address. */
    ldr r1, [r3]
    /* The first item in nrk_high_ready_TCB is the task top of stack. */
    ldr r0, [r1]
    /* Pop the registers that are not automatically saved on exception entry. */
    ldmia r0!, {r4-r11} 
    /* Restore the task stack pointer. */
    msr psp, r0
	mov r0, #0 
    msr	basepri, r0						
    orr r14,r14, #0xd 
    bx r14

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
       