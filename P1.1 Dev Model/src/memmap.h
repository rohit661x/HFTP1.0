#ifndef MEMMAP_H
#define MEMMAP_H
// include guard, prevents the contents of memmap.h from being processed more than once

// Physical memory layout
#define RX_DESC_BASE        0x00100000UL  // RX descriptor ring base
#define RX_PKT_BASE         0x00200000UL  // RX packet buffer base
#define TX_DESC_BASE        0x00300000UL  // TX descriptor ring base

#define TX_PKT_BASE         0x00400000UL  // TX packet buffer base

// Sizes
#define DESC_SIZE          16U      // Size of each descriptor in bytes
#define PKT_BUF_SIZE       64U      // Size of each packet buffer in bytes

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

#endif // MEMMAP_H