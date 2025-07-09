//input:  parser’s msg_valid, msg_bus
//storage: FSM registers for rx_base, rx_len, rx_head, rx_tail
//on parser output: call DPI helper to write into guest RAM & MMIO.

module nic(
  input  logic        clk,
  input  logic        nreset,
  input  logic        msg_valid,
  input  logic        msg_start,
  input  logic [15:0] msg_len,
  input  logic [63:0] msg_data,
  input  logic [31:0] nic_mmio_base,
  // AXI-UDP interface inputs to parser:
  input  logic        udp_valid,
  input  logic [63:0] udp_data,
  // (add any other parser ports as needed)
  output logic        udp_ready
);

// DPI helper to write guest RAM or MMIO via QEMU GDB
import "DPI-C" function void gdb_write(
  longint addr,
  int      data,
  int      size
);


:
moldudp64 #(
  // parameter overrides…
) parser_i (
  .clk          (clk),
  .nreset       (nreset),

  // AXI-UDP in…
  .udp_axis_tvalid_i  (udp_valid),
  .udp_axis_tdata_i   (udp_data),
  // …other AXI signals…

  // **Parser outputs → feed into your NIC FSM:**
  .mold_msg_v_o       (msg_valid),
  .mold_msg_start_o   (msg_start),
  .mold_msg_len_o     (msg_len),
  .mold_msg_data_o    (msg_data),
  .mold_msg_ov_v_o    (ov_valid),
  .mold_msg_ov_data_o (ov_data),
  .mold_msg_ov_len_o  (ov_len)
);


// RX descriptor ring base physical address
localparam RX_DESC_BASE        = 32'h0010_0000;
// RX packet buffer base physical address
localparam RX_PKT_BASE         = 32'h0020_0000;
// Offset of RX tail register in NIC MMIO space
localparam NIC_REG_RXDESC_TAIL = 32'h0000_0018;
// Sizes
localparam DESC_SIZE           = 16;  // bytes per descriptor
localparam PKT_SIZE            = 64;  // bytes per packet buffer
localparam RX_RING_SIZE        = 16;  // number of descriptors

// NIC internal state
reg [31:0] rx_tail;

// On each parsed message, inject into guest RAM and update descriptor ring
always @(posedge clk) begin
  if (nreset) begin
    // When parser signals start of a new message
    if (msg_valid && msg_start) begin
      // 1) Write each byte of the parsed message into the packet buffer
      for (integer i = 0; i < msg_len; i = i + 1) begin
        gdb_write(
          RX_PKT_BASE + rx_tail*PKT_SIZE + i,
          (msg_data >> (8*i)) & 8'hFF,
          1
        );
      end
      // 2) Mark descriptor as done (status = 1) at offset 8 within descriptor
      gdb_write(
        RX_DESC_BASE + rx_tail*DESC_SIZE + 8,
        1,
        1
      );
      // 3) Update the NIC’s RX tail register via MMIO
      gdb_write(
        nic_mmio_base + NIC_REG_RXDESC_TAIL,
        rx_tail,
        4
      );
      // 4) Increment tail pointer
      rx_tail <= (rx_tail + 1) % RX_RING_SIZE;
    end
  end
end

initial begin
  rx_tail = 0;
end

endmodule