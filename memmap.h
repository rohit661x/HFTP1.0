#ifndef MEMMAP_H
#define MEMMAP_H
// Make fixed-width integer types available
#include <stdint.h>  // for uint*_t types
// include guard, prevents the contents of memmap.h from being processed more than once

// Physical memory layout
#define RX_DESC_BASE        0x00100000UL  // RX descriptor ring base
#define RX_PKT_BASE         0x00200000UL  // RX packet buffer base
#define TX_DESC_BASE        0x00300000UL  // TX descriptor ring base

#define TX_PKT_BASE         0x00400000UL  // TX packet buffer base

// Sizes
#define DESC_SIZE          16U      // Size of each descriptor in bytes

#define PKT_BUF_SIZE       64U      // Size of each packet buffer in bytes
// MoldUDP64 packet total length: 8B session + 4B length + 11B payload
#define MOLDUDP64_PACKET_LEN 23U

// MMIO register offsets (relative to nic_mmio_base)
#define NIC_REG_CTRL           0x0000U  // Control
#define NIC_REG_RXDESC_LO      0x2800U  // RX descriptor base (low 32 bits)
#define NIC_REG_RXDESC_HI      0x2804U  // RX descriptor base (high 32 bits)
#define NIC_REG_RXDESC_LEN     0x2808U  // RX ring length in bytes
#define NIC_REG_RXDESC_HEAD    0x2810U  // RX consumer index, head is where the software has finished processing packets
#define NIC_REG_RXDESC_TAIL    0x2818U  // RX producer index, tail is where the NIC has finished DMA'ing the packets
#define NIC_REG_TXDESC_LO      0x3800U  // TX descriptor base (low 32 bits)
#define NIC_REG_TXDESC_HI      0x3804U  // TX descriptor base (high 32 bits)
#define NIC_REG_TXDESC_LEN     0x3808U  // TX ring length in bytes
#define NIC_REG_TXDESC_HEAD    0x3810U  // TX consumer index, NIC finishes sending the packet
#define NIC_REG_TXDESC_TAIL    0x3818U  // TX producer index, software placing the packet 

// Descriptor ring sizes = 16 
#define RX_RING_SIZE       16U //never negative 
#define TX_RING_SIZE       16U //never negative

typedef struct __attribute__((packed, aligned(16))) {
    uint64_t addr;
    uint32_t length;
    uint8_t  status;
    uint8_t  pad[7];
} rx_desc_t;

// Parsed message structure (aligned to 64 bytes for cache line)
typedef struct __attribute__((packed, aligned(64))) {
    char     msg_type;       // 1B
    uint16_t stock_locate;   // 2B
    uint32_t price;          // 4B
    uint32_t shares;         // 4B
    uint8_t  pad[64 - (1 + 2 + 4 + 4)];
} hw_msg_small;

#define NIC_MMIO_BASE 0x00500000UL

_Static_assert(PKT_BUF_SIZE >= MOLDUDP64_PACKET_LEN,
    "PKT_BUF_SIZE must hold a full MoldUDP64 packet");
_Static_assert((RX_DESC_BASE & (sizeof(rx_desc_t)-1)) == 0,
    "RX_DESC_BASE must be aligned to descriptor size");


#endif // MEMMAP_H
