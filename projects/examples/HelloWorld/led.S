.section .text, "x"
.syntax unified
.global test

test:
LDR     R1, =0x2009C020 
LDR     R2, =0x00040000  
CMP     R0, #0       
IT EQ
STREQ   R2, [R1,#0x1C]  
IT NE
STRNE   R2, [R1,#0x18] 
BX      LR
