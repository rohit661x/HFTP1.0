#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// Constants (adjust to match your kernel definitions)
#define RX_PKT_BASE       0x100000
#define PKT_BUF_SIZE      2048
#define PARSED_BASE       0x300000
#define PARSED_RING_SIZE  16
#define NIC_REG_PARSED_TAIL 0x20

// Payload offsets and sizes
#define OFFSET_SESSION   0
#define OFFSET_LENGTH    8
#define OFFSET_PAYLOAD   12
#define PAYLOAD_LEN      11

// Parsed message structure (must match kernel's hw_msg_small)
typedef struct {
    char     msg_type;      // 1 byte
    uint16_t stock_locate;  // 2 bytes
    uint32_t price;         // 4 bytes
    uint32_t shares;        // 4 bytes
} hw_msg_small;

// Simulated MMIO register for parsed tail (for test harness)
static uint32_t parsed_tail = 0;

// Function prototype
int moldudp64_parser_c(uint8_t *pkt_base, uint32_t pkt_len, hw_msg_small *out_msg);

// C prototype implementation
int moldudp64_parser_c(uint8_t *pkt_base, uint32_t pkt_len, hw_msg_small *out_msg) {
    // 1) Check length field validity
    if (pkt_len < OFFSET_PAYLOAD + PAYLOAD_LEN) {
        return -1;  // too short
    }
    // 2) Read BE length
    uint32_t be_len = (pkt_base[OFFSET_LENGTH] << 24) |
                      (pkt_base[OFFSET_LENGTH+1] << 16) |
                      (pkt_base[OFFSET_LENGTH+2] << 8) |
                       pkt_base[OFFSET_LENGTH+3];
    if (be_len != PAYLOAD_LEN) {
        return -2;  // unexpected payload length
    }
    // 3) Extract fields
    uint8_t *p = pkt_base + OFFSET_PAYLOAD;
    hw_msg_small msg;
    msg.msg_type     = (char)p[0];
    msg.stock_locate = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
    msg.price        = (uint32_t)p[3] | ((uint32_t)p[4] << 8) |
                       ((uint32_t)p[5] << 16) | ((uint32_t)p[6] << 24);
    msg.shares       = (uint32_t)p[7] | ((uint32_t)p[8] << 8) |
                       ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 24);

    // 4) Write into parsed-ring in guest RAM (simulated here)
    hw_msg_small *ring = (hw_msg_small *)(PARSED_BASE);
    ring[parsed_tail] = msg;

    // 5) Bump parsed tail and write MMIO (simulated)
    parsed_tail = (parsed_tail + 1) % PARSED_RING_SIZE;
    // In real kernel we'd do: nic_write32(NIC_REG_PARSED_TAIL, parsed_tail);

    // Output parsed message to caller
    if (out_msg) {
        *out_msg = msg;
    }

    return 0;  // success
}

// Basic test harness
#ifdef TEST_PARSER_C
int main(void) {
    // Build a test packet: session+length+payload
    uint8_t test_pkt[OFFSET_PAYLOAD + PAYLOAD_LEN];
    memset(test_pkt, 0, sizeof(test_pkt));
    // session (ignored)
    // length = PAYLOAD_LEN
    test_pkt[OFFSET_LENGTH+0] = (PAYLOAD_LEN >> 24) & 0xFF;
    test_pkt[OFFSET_LENGTH+1] = (PAYLOAD_LEN >> 16) & 0xFF;
    test_pkt[OFFSET_LENGTH+2] = (PAYLOAD_LEN >> 8) & 0xFF;
    test_pkt[OFFSET_LENGTH+3] = (PAYLOAD_LEN) & 0xFF;
    // payload: msg_type='A', stock_locate=0x0042, price=100, shares=200
    test_pkt[OFFSET_PAYLOAD + 0] = 'A';
    test_pkt[OFFSET_PAYLOAD + 1] = 0x42;
    test_pkt[OFFSET_PAYLOAD + 2] = 0x00;
    // price = 100
    test_pkt[OFFSET_PAYLOAD + 3] = 100 & 0xFF;
    // others zero for simplicity

    hw_msg_small parsed;
    int ret = moldudp64_parser_c(test_pkt, sizeof(test_pkt), &parsed);
    if (ret != 0) {
        printf("Parser returned error %d\n", ret);
        return 1;
    }
    printf("Parsed msg_type=%c, stock_locate=0x%04X\n",
           parsed.msg_type, parsed.stock_locate);
    return 0;
}
#endif