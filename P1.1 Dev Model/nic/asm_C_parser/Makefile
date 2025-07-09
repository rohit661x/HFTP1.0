# Makefile for ASM stub assembly

# Source directory for ASM stub
ASM_DIR := ../asm
ASM_SRCS := unpack11.S
ASM_OBJS := $(ASM_SRCS:.S=.o)

# All targets
all: $(ASM_OBJS)

# Rule to assemble stub
%.o: $(ASM_DIR)/%.S
	gcc -arch x86_64 -x assembler-with-cpp -c $< -o $@

clean:
	rm -f $(ASM_OBJS)