#include <stdint.h>
#include "memmap.h"
#include <stdbool.h>     // for bool, true, false

#define assert(expr) ((expr) ? (void)0 : __assert_func(__FILE__, __LINE__, __func__, #expr))

#// Forward declaration for serial output used in assert handler
void serial_write(const char *str);

// Minimal freestanding memset
#include <stddef.h>
void *memset(void *dest, int c, size_t n) {
    unsigned char *p = dest;
    for (size_t i = 0; i < n; i++) {
        p[i] = (unsigned char)c;
    }
    return dest;
}

// Freestanding assert failure handler
void __assert_func(const char *file, int line, const char *func, const char *expr) {
    serial_write("Assertion failed: ");
    serial_write(expr);
    serial_write(" in ");
    serial_write(func);
    serial_write(" at ");
    serial_write(file);
    serial_write("\r\n");
    // Halt
    for (;;)
        __asm__ volatile ("hlt");
}
_Static_assert(sizeof(hw_msg_small) == PKT_BUF_SIZE,
    "hw_msg_small must be exactly one packet buffer in size");

_Static_assert((RX_DESC_BASE & (64 - 1)) == 0, "RX_DESC_BASE must be 64-byte aligned");
_Static_assert((TX_DESC_BASE & (64 - 1)) == 0, "TX_DESC_BASE must be 64-byte aligned");

_Static_assert((NIC_MMIO_BASE & 0xFFF) == 0, "NIC_MMIO_BASE must be 4KiB-aligned");

// RX ring mask for power-of-two size
#define RX_RING_MASK (RX_RING_SIZE - 1)

//using static to avoid symbol collision
//using inline to save stack setup and teardown, therefore less latency


static inline uint8_t inb(uint16_t port) {
    uint8_t ret;
    __asm__ volatile ("inb %1, %0" : "=a"(ret) : "d"(port));
    return ret;
}

static inline void outb(uint16_t port, uint8_t val) {
    __asm__ volatile ("outb %0, %1" : : "a"(val), "d"(port));
}

static inline uint32_t inl(uint16_t port) {
    uint32_t ret;
    __asm__ volatile ("inl %1, %0" : "=a"(ret) : "d"(port));
    return ret;
}

static inline void outl(uint16_t port, uint32_t data) {
    __asm__ volatile ("outl %0, %1" : : "a"(data), "d"(port));
}

//same logic as with inb and outb but now for 32 bits



void serial_init() {
    outb(0x3F8 + 1, 0x00);    // write 0x00 to Interrupt Enable Register (IER), disable all interrupts
    outb(0x3F8 + 3, 0x80);    // write to LCR, enable DLAB
    outb(0x3F8 + 0, 0x03);    // Divisor low byte (3), not portable!
    outb(0x3F8 + 1, 0x00);    // Divisor high byte
    outb(0x3F8 + 3, 0x03);    // configure 8 bits, no parity (for performance), 1 stop bit
    outb(0x3F8 + 2, 0xC7);    // Enable FIFO, clearing it with 14-byte threshold, using for diagnostics will not be used in final model for latency 
    outb(0x3F8 + 4, 0x0B);    // IRQs enabled, RTS/DSR set, needed due to legacy 
}

void serial_write(const char* str) {             // for debugging will remove in performance model
    while (*str) {
        while ((inb(0x3F8 + 5) & 0x20) == 0);    //polling bit 5 of LSR, waiting for empty TX buffer, ensuring no overwrites on the UART buffer
        outb(0x3F8, *str++);                     //UART is ready, move to next character
    }
}
//"Hello" -> ['H'] -> poll UART -> UART ready? -> send 'H'
//        -> ['e'] -> poll UART -> UART ready? -> send 'e'
//        -> ['l'] ...



static inline uint64_t read_tsc() {                    // funtion returning the value of TSC (64-bit unassigned integer)
    uint32_t lo, hi;                                   // we are in 32-bit mode, so we must split the 64 bits of raw data into two pieces: lower 32 bits and higher 32 bits 
    __asm__ volatile ("rdtsc" : "=a"(lo), "=d"(hi));   // maps assembly output into lo and hi, EAX -> lo, EDX -> hi
    return ((uint64_t)hi << 32) | lo;                  // build the 64-bit interger by parts using variables lo and hi
}



// Global storage for the NIC's MMIO base address
static uintptr_t nic_mmio_base;

// PCI configuration address is built with these components
#define PCI_CONFIG_ADDRESS 0xCF8          //address selector, what to look at
#define PCI_CONFIG_DATA    0xCFC          //data register, what you get back
#define PCI_ENABLE_BIT     0x80000000     //enable bit set 
#define PCI_BUS_SHIFT      16             //bus number is stored from bits 23-16
#define PCI_DEVICE_SHIFT   11             //device number is stored from bits 15-11
#define PCI_FUNCTION_SHIFT 8              //function number is stored from bits 10-8






uint32_t pci_config_read(uint8_t bus, uint8_t device, uint8_t function, uint8_t offset) {     // reads 32-bit value from PCI bus using bus/device/function/offset tuple
    uint32_t address = PCI_ENABLE_BIT |                                                       // temporary varaiable set to PCI address defined earlier using bitwise OR: A|B
                       ((uint32_t)bus << PCI_BUS_SHIFT) |                                     // left shift of 8-bit bus value by 16 bits 
                       ((uint32_t)device << PCI_DEVICE_SHIFT) |                               // left shift of 5-bit device value by 11 bits 
                       ((uint32_t)function << PCI_FUNCTION_SHIFT) |                           // left shift of 3-bit function value by 8 bits
                       (offset & 0xFC);                                                       // offset must be 4-byte aligned - bits 0-1 are always zero
    outl(PCI_CONFIG_ADDRESS, address);                                                        // sends the message to the PCI address port
    return inl(PCI_CONFIG_DATA);                                                              // reads back the 32-bit value from the PCI data port
}


// Read 8/16‐bit helpers as needed
static uint16_t __attribute__((unused)) pci_config_read_word(uint8_t bus, uint8_t device, uint8_t function, uint8_t offset) {
    uint32_t val = pci_config_read(bus, device, function, offset & 0xFC);
    return (uint16_t)((val >> ((offset & 2) * 8)) & 0xFFFF);
}
static uint8_t  __attribute__((unused)) pci_config_read_byte(uint8_t bus, uint8_t device, uint8_t function, uint8_t offset) {
    uint32_t val = pci_config_read(bus, device, function, offset & 0xFC);
    return (uint8_t)((val >> ((offset & 3) * 8)) & 0xFF);
}


// Simple hex‐printer for diagnostics
void print_hex(uint32_t v);

// Print a 32-bit value in hexadecimal (8 uppercase digits)
void print_hex(uint32_t val) {
    const char hex_chars[] = "0123456789ABCDEF";
    char buf[9];
    for (int i = 0; i < 8; i++) {
        buf[7 - i] = hex_chars[(val >> (i * 4)) & 0xF];
    }
    buf[8] = '\0';
    serial_write(buf);
}


void pci_enumerate() {                                                             // scan every possible PCI bus (0-255), device slot (0-31), and function number (0-7)
    for (uint16_t bus = 0; bus < 256; bus++) {                                     // for loop for bus
        for (uint8_t device = 0; device < 32; device++) {                          // for loop for device
            for (uint8_t function = 0; function < 8; function++) {                 // for loop for function number 

                                                                                   // Step 1: Read Vender / Device ID
                uint32_t vd = pci_config_read(bus, device, function, 0x00);
                uint16_t vendor_id = vd & 0xFFFF;
                if (vendor_id == 0xFFFF) 
                    continue;                                                      // no device here

                                                                                   // Step 2: Class Code / Subclass
                uint16_t device_id = (vd >> 16) & 0xFFFF;
                uint32_t ci = pci_config_read(bus, device, function, 0x08);
                uint8_t base_class = (ci >> 24) & 0xFF;
                uint8_t sub_class  = (ci >> 16) & 0xFF;

                if (base_class != 0x02 || sub_class != 0x00)                      // Only Ethernet controllers (class 2, subclass 0)
                    continue;                                                     // not an Ethernet NIC

                                                                                  // Step 3: Found NIC and report
                serial_write("Ethernet NIC Found: Vendor=0x");                    // for debugging will remove in performance model
                print_hex(vendor_id);
                serial_write(", Device=0x");                                      // for debugging will remove in performance model
                print_hex(device_id);
                serial_write("\r\n");                                             // for debugging will remove in performance model

                                                                                  // Step 4: Read BAR0 for its MMIO base
                uint32_t bar0 = pci_config_read(bus, device, function, 0x10);
                if (bar0 & 0x1) {
                    serial_write("BAR0 is I/O space—unsupported\r\n");           // for debugging will remove in performance model
                } else {
                    nic_mmio_base = bar0 & ~0xF;
                    serial_write("NIC MMIO Base = 0x");                          // for debugging will remove in performance model
                    print_hex(nic_mmio_base);
                    serial_write("\r\n");                                        // for debugging will remove in performance model
                }

                // Step 5: Stop after first matching NIC
                return;  

            }
        }
    }
    // Fallback for emulation: no NIC found
    if (nic_mmio_base == 0) {
        serial_write("No NIC found; using default NIC_MMIO_BASE\r\n");
        nic_mmio_base = NIC_MMIO_BASE;
    }
}


// MMIO access primitives
static inline void nic_write32(uint32_t offset, uint32_t val) {           // writes a 32-bit word to NIC's MMIO register at (nic_mmio_base + offset)
    *(volatile uint32_t*)(nic_mmio_base + (uintptr_t)offset) = val;
}
static inline uint32_t nic_read32(uint32_t offset) {                      // reads a 32-bit word from NIC's MMIO register at (nic_mmio_base + offset)
    return *(volatile uint32_t*)(nic_mmio_base + (uintptr_t)offset);
}


//Descriptor-Ring & Control Definitions:

//Control-Register Bits
#define CTRL_RST               0x04000000U      // Global reset
#define CTRL_RX_EN             0x00000002U      // Receive enable
#define CTRL_TX_EN             0x00000004U      // Transmit enable

//Rx-Descriptor Status Bits (status field)
#define RX_DESC_STAT_DD        0x01U            // Descriptor Done

//Tx-Descriptor Command Bits (cmd field)
#define TX_DESC_CMD_EOP        0x01U            // End of Packet
#define TX_DESC_CMD_IFCS       0x02U            // Insert FCS (Frame Check Sequence)
#define TX_DESC_CMD_RS         0x08U            // Report Status (set status bit on completion)

//Tx-Descriptor Status Bits (status field)
#define TX_DESC_STAT_DD        0x01U            // Descriptor Done



// TX descriptor layout
typedef struct {
    uint64_t addr;              // pointer to packet buffer, 64-bit physical address where the driver placed the TX packet in RAM
    uint32_t length;            // size of the packet for NIC to know how long of the msg is 
    uint32_t cmd;               // command flags (EOP, IFCS, RS)
    uint8_t  status;            // After the NIC completes sending the packet, it sets DD=1
    uint8_t  pad[3];            // pad to 16 bytes, for easy math
} tx_desc_t;

// Output interface, to hand to the NIC for transmission
typedef struct {
    char     msg_type;                         // identfies message subtype ex. 'O' for OUCH Enter
    char     version;                          // format version or filler
    char     reserved[2];                      // alignment padding
    uint32_t stock_locate;                     // identfies security, ex.AAPL_LOCATE
    uint32_t order_id;                         // unique ID for order entry 
    uint32_t price;                            // price in ticks for execution
    uint32_t shares;                           // share quantity of order
    //ADD FIELDS AS NEEDED TO MATCH OUTCH
} OuchEnter;



// Define the stock locate code for AAPL ("AAPL")
#define AAPL_LOCATE 0x0042U //defined variable from generator, later will make this dynamic with an intial stock directory message to define



// Initialize the RX descriptor ring and tell the NIC about it
void init_rx_ring(void) {
    memset((void*)RX_DESC_BASE, 0, RX_RING_SIZE * DESC_SIZE);        // Zero out the all the descriptors 

    rx_desc_t *rx_desc = (rx_desc_t*)RX_DESC_BASE;                   // Populate each descriptor’s fields for array mapping
    for (uint32_t i = 0; i < RX_RING_SIZE; i++) {                    // Iterates through every descriptor slot in the ring
        rx_desc[i].addr   = RX_PKT_BASE + i * PKT_BUF_SIZE;          // Points each descriptor at its dedicated packet buffer in RAM
        rx_desc[i].length = PKT_BUF_SIZE;                            // Prevents buffer overrun
        rx_desc[i].status = 0;                                       // Clear DD bit to reset 
        // the remaining pad bytes can stay zero
    }

    // Program the NIC’s MMIO registers for RX
    nic_write32(NIC_REG_RXDESC_LO,   (uint32_t)(RX_DESC_BASE & 0xFFFFFFFFU));       // Lower 32 bits if the ring base address
    nic_write32(NIC_REG_RXDESC_HI,   (uint32_t)((uint64_t)RX_DESC_BASE >> 32));     // Upper 32 bits if the ring base address
    nic_write32(NIC_REG_RXDESC_LEN,  RX_RING_SIZE * DESC_SIZE);                     // Total byte length of the ring (n descriptors x size)
    nic_write32(NIC_REG_RXDESC_HEAD, 0);                                            // Inital Consumer Index, where CPU will look next for packets
    nic_write32(NIC_REG_RXDESC_TAIL, 0);                                            // Inital Producer Index, where NIC will place the first packet

    // Enable the RX engine
    uint32_t ctrl = nic_read32(NIC_REG_CTRL);                   // read the 32-bit CTRL register for other flags
    ctrl |= CTRL_RX_EN;                                         // enable DMA of received packets by OR function
    nic_write32(NIC_REG_CTRL, ctrl);                            // write updated control value back

    // Prefetch all RX descriptors into L1 cache to avoid misses in the spin loop
    {
        rx_desc_t *rx_desc = (rx_desc_t*)RX_DESC_BASE;
        for (uint32_t i = 0; i < RX_RING_SIZE; i++) {
            // Touch each descriptor’s status, length, and addr fields
            volatile uint8_t  s = rx_desc[i].status;
            volatile uint32_t l = rx_desc[i].length;
            volatile uint64_t a = rx_desc[i].addr;
            (void)s; (void)l; (void)a;
        }
    }

    serial_write("RX ring initialized\r\n");                    // for debugging will remove in performance model
}


// Initialize the TX descriptor ring and tell the NIC about it
void init_tx_ring(void) {
    memset((void*)TX_DESC_BASE, 0, TX_RING_SIZE * DESC_SIZE);                       // Zero out the all the descriptors

    // Program the NIC’s MMIO registers for TX:
    nic_write32(NIC_REG_TXDESC_LO,   (uint32_t)(TX_DESC_BASE & 0xFFFFFFFFU));       // Extracts bits 31-0 of TX_DESC_BASE so NIC knows where descriptor memory starts
    nic_write32(NIC_REG_TXDESC_HI,   (uint32_t)((uint64_t)TX_DESC_BASE >> 32));     // Shifts right 32 bits, to get the full 64-bit message
    nic_write32(NIC_REG_TXDESC_LEN,  TX_RING_SIZE * DESC_SIZE);                     // Calculates how many bytes the NIC should consider part of the TX ring
    nic_write32(NIC_REG_TXDESC_HEAD, 0);                                            // Inital Consumer Index, where NIC will read next for packets
    nic_write32(NIC_REG_TXDESC_TAIL, 0);                                            // Inital Producer Index, CPU will bump when placing packets for tramission 

    // Enable TX engine
    uint32_t ctrl = nic_read32(NIC_REG_CTRL);                                       
    ctrl |= CTRL_TX_EN;                                                             // “switch on” the TX engine bit (0x0004)
    nic_write32(NIC_REG_CTRL, ctrl);                                                // writes back the updated value to the same MMIO register 

    serial_write("TX ring initialized\r\n");                                        // for debugging will remove in performance model
}


// RX Polling & Order Logic Loop

// Align each index to its own 64-byte cache line to prevent false sharing
static uint32_t rx_head       __attribute__((aligned(64))) = 0; // software consumer index for RX ring
static uint32_t tx_idx        __attribute__((aligned(64))) = 0; // software producer index for TX ring
static uint32_t tx_head       __attribute__((aligned(64))) = 0; // NIC consumer index for TX ring
static uint32_t next_order_id __attribute__((aligned(64))) = 1; // counter for unique OUCH order IDs

// Align best-bid state to avoid sharing with index variables
static uint32_t best_bid_price  __attribute__((aligned(64))) = 0;
static uint32_t best_bid_shares __attribute__((aligned(64))) = 0;

// Packet‐arrival callback: parses and acts on each incoming hw_msg_small
static inline void handle_msg(hw_msg_small *m);

void reclaim_tx_descriptors(void) {                                // to avoid overwrites   
    uint32_t nic_head = nic_read32(NIC_REG_TXDESC_HEAD);           // free up any descriptors the NIC has finished with
    
    while (tx_head != nic_head) {
      // Optionally: clear descriptor or perform accounting
      // tx_desc[tx_head].status = 0; 
      tx_head = (tx_head + 1) % TX_RING_SIZE;
    }
}


// Main Polling Loop:
void poll_rx_loop(void) {
    serial_write("Entering RX poll loop\r\n");                              // for debugging will remove in performance model
    while (true) {                                                          // entering infinite spin loop
        uint64_t trx_start = read_tsc();  // for debugging, remove in perf model
        reclaim_tx_descriptors();                                           // reclaim any completed TX descriptors before processing new RX packets

        // 1) Read NIC’s tail (where the hardware last DMA’d a packet)
        uint32_t rx_tail = nic_read32(NIC_REG_RXDESC_TAIL);                 // how many new descriptors are waiting processing
        uint64_t trx_read = read_tsc();  // for debugging, remove in perf model
        serial_write("RXREAD cycles: "); // for debugging, remove in perf model
        print_hex((uint32_t)((trx_read - trx_start) >> 32));
        print_hex((uint32_t)(trx_read - trx_start));
        serial_write("\r\n");            // for debugging, remove in perf model

        uint32_t next_rx_tail = (rx_tail + 1) & RX_RING_MASK;               // compute the next tail index if NIC wrote one more packet
        if (next_rx_tail == rx_head) {                                      // compare software_rx_head, if true then no free descriptor
            // RX ring is full
            serial_write("RX ring full; dropping oldest packet\r\n");       // for debugging will remove in performance model
            
            rx_desc_t *desc = (rx_desc_t*)RX_DESC_BASE;                     // clear DD bit on oldest slot so NIC can reuse
            desc[rx_head].status = 0;
            rx_head = next_rx_tail;                                         // dropping oldest packet to free up space
        }

        // 2) Process each new descriptor
        rx_desc_t *desc = (rx_desc_t*)RX_DESC_BASE;
        while (rx_head != rx_tail) {                                                // loop from head up to but not including tail, processing each descriptor only once
            if (__builtin_expect(desc[rx_head].status & RX_DESC_STAT_DD, 0)) {      // check DD bit of the descriptor
                uint32_t pkt_len = desc[rx_head].length;                            // read length of packet
                if (pkt_len > PKT_BUF_SIZE) {                                       // if packet is larger than buffer size then drop it
                    serial_write("Dropping oversize RX packet\r\n");                // for debugging will remove in performance model
                    desc[rx_head].status = 0;                                       // clear DD for this slot
                    rx_head = (rx_head + 1) & RX_RING_MASK;                         // bump rx_head and continue to skip handeling
                    continue;
                }
                assert(pkt_len == sizeof(hw_msg_small) && "Parsed message length mismatch");

                // Prefetch next message buffer
                uint32_t next_head = (rx_head + 1) & RX_RING_MASK;
                hw_msg_small *next_msg = (hw_msg_small*)(RX_PKT_BASE + next_head * PKT_BUF_SIZE);
                __builtin_prefetch(next_msg, 0, 3);

                desc[rx_head].status = 0;                                   // clear DD bit so NIC can reuse this entry

                hw_msg_small *msg = (hw_msg_small*)(uintptr_t)desc[rx_head].addr;      // call message handler for valid packet
                handle_msg(msg);                                            // calls handle_msg, thus we begin processing
            }

            rx_head = (rx_head + 1) & RX_RING_MASK;                         // Move to next descriptor, increment rx_head so each descriptor is visited once per pass
        }
       
        __asm__ volatile ("pause");                                         // lets the CPU know we're in a spin-wait, reduces power use and improves hyperthread efficiency
    }
}


// Build an OUCH Enter packet and queue it in the TX ring
static inline void send_ouch(uint32_t order_id, uint32_t price, uint32_t shares) {
    uint64_t tout_start = read_tsc();    // for debugging, remove in perf model
    uint32_t next_tx_idx = (tx_idx + 1) % TX_RING_SIZE;                     // ensure there is a free TX descriptor slot
    while (__builtin_expect(next_tx_idx == tx_head, 0)) {                   // detect full tx ring condition
        reclaim_tx_descriptors();                                           // keep calling until theres room 
        // Optional small pause to avoid busy spin
        __asm__ volatile ("pause");
    }

    // Construct OUCH packet in packet buffer
    OuchEnter *o = (OuchEnter*)(TX_PKT_BASE + tx_idx * PKT_BUF_SIZE);       // Cast the TX packet‐buffer region into an OuchEnter
    o->msg_type     = 'O';                                                  // 'O' tells the recipient “this is an OUCH Enter message
    o->version      = 0;                                                    // Spec compliance and zeros for alignment
    o->reserved[0]  = 0;                                                    
    o->reserved[1]  = 0;                                                    
    o->stock_locate = AAPL_LOCATE;                                          // Order Info
    o->order_id     = order_id;                                             
    o->price        = price;                                                
    o->shares       = shares;                                               

    // Populate TX descriptor
    tx_desc_t *tx_desc = (tx_desc_t*)TX_DESC_BASE;                                       // Cast the TX‐descriptor ring base to a tx_desc_t
    tx_desc[tx_idx].addr   = TX_PKT_BASE + tx_idx * PKT_BUF_SIZE;                        // Where the NIC should fetch the OUCH packet
    tx_desc[tx_idx].length = sizeof(OuchEnter);                                          // Exact byte count (sizeof(OuchEnter)
    tx_desc[tx_idx].cmd    = TX_DESC_CMD_EOP | TX_DESC_CMD_IFCS | TX_DESC_CMD_RS;        // Bitwise-OR of EOP, IFCS, RS

    tx_desc[tx_idx].status = 0;                                                          // Cleared to 0 so the NIC will later set bit-0 on completion

    // Notify NIC by bumping TX tail
    nic_write32(NIC_REG_TXDESC_TAIL, tx_idx);
    uint64_t tout_end = read_tsc();       // for debugging, remove in perf model
    serial_write("OUCH+TX cycles: ");    // for debugging, remove in perf model
    print_hex((uint32_t)((tout_end - tout_start) >> 32));
    print_hex((uint32_t)(tout_end - tout_start));
    serial_write("\r\n");                // for debugging, remove in perf model

    // 4) Advance software index
    tx_idx = (tx_idx + 1) % TX_RING_SIZE;
}





static inline void handle_msg(hw_msg_small *m) {                                 // Gets invoked once per packet in RX loop
    __builtin_prefetch((uint8_t*)m, 0, 3);
    uint64_t t_logic_start = read_tsc();  // for debugging, remove in perf model
    if (__builtin_expect(m->msg_type == 'A' && m->stock_locate == AAPL_LOCATE, 1)) { // Filter out everything but “Add Order” messages for AAPL 
        // 'A' ensures you only process “Add Order” messages (ITCH code 'A')
        // AAPL_LOCATE further narrows it to orders for AAPL
        
        // On first access we get cache line miss, m points at RX packet buffer region 
        uint32_t price  = m->price;                                                                              
        uint32_t shares = m->shares;                                
        
        if (price > best_bid_price) {                                            // Compares incoming order to current best bid, then update state
            best_bid_price  = price;
            best_bid_shares = shares;

            // Call send_ouch for TX packet building
            send_ouch(next_order_id++, price, shares);                          // for debugging will remove in performance model
            serial_write("New best bid: price=");                               // for debugging will remove in performance model
            print_hex(price);
            serial_write(" shares=");                                           // for debugging will remove in performance model
            print_hex(shares);
            serial_write("\r\n");                                               // for debugging will remove in performance model
        }
    }
    uint64_t t_logic_end = read_tsc();    // for debugging, remove in perf model
    serial_write("LOGIC cycles: ");       // for debugging, remove in perf model
    print_hex((uint32_t)((t_logic_end - t_logic_start) >> 32));
    print_hex((uint32_t)(t_logic_end - t_logic_start));
    serial_write("\r\n");                 // for debugging, remove in perf model
}


// Kernel entry point: set up and hand off to the RX/TX loop
int main(void) {
    uint64_t t0, t1, t2, t3;

    serial_init();
    t0 = read_tsc();                // for debugging, remove in performance model

    pci_enumerate();
    t1 = read_tsc();                // for debugging, remove in performance model
    serial_write("BOOT cycles: ");
    print_hex((uint32_t)((t1 - t0) >> 32));
    print_hex((uint32_t)(t1 - t0));
    serial_write("\r\n");           // for debugging, remove in performance model

    init_rx_ring();
    t2 = read_tsc();                // for debugging, remove in performance model
    serial_write("RXINIT cycles: ");
    print_hex((uint32_t)((t2 - t1) >> 32));
    print_hex((uint32_t)(t2 - t1));
    serial_write("\r\n");           // for debugging, remove in performance model

    init_tx_ring();
    t3 = read_tsc();                // for debugging, remove in performance model
    serial_write("TXINIT cycles: ");
    print_hex((uint32_t)((t3 - t2) >> 32));
    print_hex((uint32_t)(t3 - t2));
    serial_write("\r\n");           // for debugging, remove in performance model

    poll_rx_loop();
    return 0;  // unreachable
}