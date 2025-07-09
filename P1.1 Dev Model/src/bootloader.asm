[BITS 16]
[ORG 0x7C00]

start:
    cli                 ; Disable interrupts
    xor ax, ax          ; registrer zeroing idiom 2 bytes vs mov ax, 0 which is 3 bytes
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x7BFF      ; Set stack just below bootloader

    lgdt [gdt_descriptor] ; Load GDT

    mov eax, cr0
    or eax, 1
    mov cr0, eax
    
    jmp 0x08:protected_mode_start ; Far jump to clear pipeline and load CS

; 
; GDT definition
;

gdt_start:
gdt_null:
    dq 0x0000000000000000       ; Null descriptor

gdt_code:                       ; Offset = 0x08
    dq 0x00CF9A000000FFFF       ; Base=0x0, Limit=0xFFFFF, Code Segment, 4GB

gdt_data:                       ; Offset = 0x10
    dq 0x00CF92000000FFFF       ; Base=0x0, Limit=0xFFFFF, Data Segment, 4GB

gdt_end:

gdt_descriptor:
    dw gdt_end - gdt_start - 1
    dd gdt_start

; 
; Protected Mode 
; 

[BITS 32]
protected_mode_start:
    mov ax, 0x10                ; Load Data Segment Selector (GDT offset 0x10)
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax
    mov esp, 0x9FC00            ; Set stack in protected mode (safe area in RAM)

    call kernel            ; Call kernel 

.halt:
    cli
    hlt
    jmp .halt

; 
; Pad to 512 bytes
; 
times 510 - ($ - $$) db 0
dw 0xAA55                      ; Boot signature