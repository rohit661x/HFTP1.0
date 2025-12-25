/* entry.S: protected-mode entrypoint stub */

.section .text
.global _start
.extern main

_start:
    cli
    xor %ebp, %ebp
    mov $0x9FC00, %esp    /* set up a simple 32-bit stack */
    call main             /* hand off to your C entrypoint */

1:  hlt
    jmp 1b