    .text
    .globl unpack11

    # void unpack11(uint8_t *payload_ptr, hw_msg_small *dst_ptr)
unpack11:
    # Load msg_type (1 byte) into AL
    movb    (%rdi), %al

    # Load stock_locate (2 bytes little-endian) into DX
    movzwl  1(%rdi), %edx

    # Load price (4 bytes little-endian) into EBX
    movl    3(%rdi), %ebx

    # Load shares (4 bytes little-endian) into EDI
    movl    7(%rdi), %edi

    # Store msg_type
    movb    %al,    (%rsi)
    # Store stock_locate
    movw    %dx,    1(%rsi)
    # Store price
    movl    %ebx,   3(%rsi)
    # Store shares
    movl    %edi,   7(%rsi)

    ret
