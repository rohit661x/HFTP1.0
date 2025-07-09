/*
 * hw_nic_sim.c - QEMU MoldUDP64 NIC simulator plugin
 *
 * Listens for UDP on port 9000, writes each packet into guest RX ring,
 * sets descriptor DD bit, and MMIO-bumps RXDESC_TAIL.
 */


#include "hw/pci/pci.h"
#include "qemu/osdep.h"
#include "sysemu/sysemu.h"
#include "qapi/error.h"
#include "exec/memory.h"
#include "qemu/log.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <stddef.h>    // for offsetof

/* Descriptor layout used by the guest parser */
typedef struct {
    uint64_t addr;
    uint32_t length;
    uint8_t  status;
    uint8_t  pad[7];
} RxDesc;
 
#include "../../src/memmap.h"
#define NIC_MMIO_BASE 0x00500000UL
 
 typedef struct {
     PCIDevice parent_obj;
     MemoryRegion mmio;
     int udp_fd;
     QEMUBH *rx_bh;
     uint32_t rx_tail;
     hwaddr addr_base;
 } NICSimState;


 
 static void nic_sim_rx_bh(void *opaque) {
     NICSimState *s = opaque;
     uint8_t packet_buffer[PKT_BUF_SIZE];
     ssize_t len;
 
     while ((len = recv(s->udp_fd, packet_buffer, sizeof(packet_buffer), MSG_DONTWAIT)) > 0) {
         /* 1) DMA into guest memory */
         hwaddr len_addr = RX_DESC_BASE + s->rx_tail * DESC_SIZE + offsetof(RxDesc, length);
         address_space_write(&address_space_memory, len_addr, (uint8_t*)&len, 4);
        

         // 2. Write to guest packet buffer
         hwaddr packet_addr = RX_PKT_BASE + s->rx_tail * PKT_BUF_SIZE;
         address_space_write(&address_space_memory, packet_addr, packet_buffer, len);

         /* 3) Set DD status */
         hwaddr stat_addr = RX_DESC_BASE + s->rx_tail * DESC_SIZE + offsetof(RxDesc, status);
         uint8_t dd = 1;  // Descriptor done bit
         address_space_write(&address_space_memory, stat_addr, &dd, 1);
 
        /* 4) MMIO write to bump tail */
        uint32_t new_tail = s->rx_tail;
        hwaddr tail_reg_addr = NIC_MMIO_BASE + NIC_REG_RXDESC_TAIL;
        address_space_write(&address_space_memory, tail_reg_addr, &new_tail, 4);
 
         /* Advance ring */
         s->rx_tail = (s->rx_tail + 1) % RX_RING_SIZE;
         qemu_log("Injected MoldUDP64 packet: len=%zd bytes, tail index=%u\n", len, s->rx_tail);
     }
 
     /* Reschedule bottom half */
     qemu_bh_schedule(s->rx_bh);
 }
 
 static void nic_sim_realize(DeviceState *dev, Error **errp) {
     NICSimState *s = NICSIM(dev);
     struct sockaddr_in addr;
 
     /* Initialize UDP socket */
     s->udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
     fcntl(s->udp_fd, F_SETFL, O_NONBLOCK);
     addr.sin_family = AF_INET;
     addr.sin_port = htons(9000);
     addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
     if (bind(s->udp_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
         error_setg(errp, "Failed to bind UDP port 9000");
         return;
     }
 
     /* Initialize RX tail */
     s->rx_tail = 0;
 
     /* Create bottom half */
     s->rx_bh = qemu_bh_new(nic_sim_rx_bh, s);
     qemu_bh_schedule(s->rx_bh);
 }
 
 static const MemoryRegionOps nic_sim_mmio_ops = {
     .write = NULL,  /* Unused: only guest writes RXDESC_HEAD */
     .read = NULL,
     .endianness = DEVICE_NATIVE_ENDIAN,
 };
 
 static void nic_sim_class_init(ObjectClass *klass, void *data) {
     DeviceClass *dc = DEVICE_CLASS(klass);
     dc->realize = nic_sim_realize;
 }
 
 static const TypeInfo nic_sim_info = {
     .name          = "pci-nic-sim",
     .parent        = TYPE_PCI_DEVICE,
     .instance_size = sizeof(NICSimState),
     .class_init    = nic_sim_class_init,
 };
 
 static void nic_sim_register_types(void) {
     type_register_static(&nic_sim_info);
 }
 
 type_init(nic_sim_register_types);