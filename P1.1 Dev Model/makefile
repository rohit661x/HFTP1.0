# P1.0/Makefile
# ---------------------------------------------------
# cross‐tools for 32-bit freestanding build
CC      := gcc -m32 -ffreestanding -std=gnu11 -O2 -Wall
LD      := ld -m elf_i386
NASM    := nasm
QEMU    := qemu-system-i386

# -------------------------------------------------------------------
# 1) Bootloader → bin
bootloader/bootloader.bin: bootloader/bootloader.asm
	$(NASM) -f bin $< -o $@

# -------------------------------------------------------------------
# 2) Kernel objects
KERN_SRCS  := $(wildcard src/*.c src/orderlogic/*.c)
KERN_OBJS  := $(KERN_SRCS:.c=.o)

$(KERN_OBJS): %.o: %.c
	$(CC) -c $< -o $@

# -------------------------------------------------------------------
# 3) Link kernel at 1 MiB
kernel.bin: $(KERN_OBJS) linker.ld
	$(LD) -T linker.ld -o $@ $(KERN_OBJS)

# -------------------------------------------------------------------
# 4) Bundle OS image
os-image.bin: bootloader/bootloader.bin kernel.bin
	cat $^ > $@

# -------------------------------------------------------------------
# 5) Launch under QEMU
qemu: os-image.bin
	$(QEMU) -drive format=raw,file=os-image.bin

# -------------------------------------------------------------------
# 6) Simulation harness (in sim/)
SIM_VSRCS  := sim/moldudp64generator.v \
              sim/moldudp64parser.v    \
              sim/header.v dispatch.v   \
              sim/endian_flip.v ...     # etc., list all your .v files
SIM_TB     := sim/moldudp64_tb.v
SIM_VVP    := sim/tb.vvp

sim: $(SIM_VSRCS) $(SIM_TB)
	#  1) compile Verilog TB → sim/tb.vvp
	iverilog -g2005-sv -o $(SIM_VVP) \
	  sim/moldudp64generator.v \
	  sim/moldudp64parser.v    \
	  sim/header.v dispatch.v   \
	  sim/endian_flip.v ...     

	#  2) compile your C “orderlogic” hand-off into an executable
	$(CC) -std=c11 -m32 -O2 -o sim/orderlogic_dbg src/orderlogic/orderlogic.c

sim-run: sim
	# run the Verilog testbench, which will emit sim/in.bin
	vvp sim/tb.vvp

	# feed that flat binary into your C logic, producing sim/out.bin
	sim/orderlogic_dbg 0 30000 sim/in.bin sim/out.bin 2> sim/debug.log

	# sanity-check the debug.log
	@grep "DBG" sim/debug.log

# -------------------------------------------------------------------
# clean up everything
clean:
	rm -f $(SIM_VVP) sim/orderlogic_dbg sim/*.bin sim/debug.log
	rm -f bootloader/*.bin *.o kernel.bin os-image.bin