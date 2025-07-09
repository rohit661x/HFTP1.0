module moldudp64_tb;	
integer bin_f;
parameter AXI_DATA_W = 64;
parameter AXI_KEEP_W = AXI_DATA_W/8;
parameter KEEP_LW = $clog2(AXI_KEEP_W) + 1;
parameter LEN   = 8;
parameter ML_W  = 2*LEN;
parameter SID_W = 10*LEN;// session id
parameter SEQ_NUM_W = 8*LEN; // sequence number
parameter MH_W  = 20*LEN;// header 
// overlap fields
parameter OV_DATA_W  = 64-ML_W;//48
parameter OV_KEEP_W  = (OV_DATA_W/8);//6
parameter OV_KEEP_LW = 3; //$clog2(OV_KEEP_W+1),

`ifdef DEBUG_ID
parameter DEBUG_ID_W = SEQ_NUM_W + SID_W;
`endif

reg clk = 0;
reg nreset = 1'b0;	

logic [MH_W-1:0]       moldudp_header;
logic [ML_W-1:0]       moldudp_msg_len;
logic                  udp_axis_tvalid_i;
logic [AXI_KEEP_W-1:0] udp_axis_tkeep_i;
logic [AXI_DATA_W-1:0] udp_axis_tdata_i;
logic                  udp_axis_tlast_i;
logic                  udp_axis_tuser_i;
logic                  udp_axis_tready_o;

logic                  mold_msg_v_o;
logic                  mold_msg_start_o;
logic [KEEP_LW-1:0]    mold_msg_len_o;
logic [AXI_DATA_W-1:0] mold_msg_data_o;

logic                  mold_msg_ov_v_o;
logic [OV_KEEP_LW-1:0] mold_msg_ov_len_o;
logic [OV_DATA_W-1:0]  mold_msg_ov_data_o;


`ifdef DEBUG_ID
logic [DEBUG_ID_W-1:0] mold_msg_debug_id_o;
`endif

`ifdef MOLD_MSG_IDS
logic [SID_W-1:0]      mold_msg_sid_o;
logic [SEQ_NUM_W-1:0]  mold_msg_seq_num_o;
`endif

`ifdef MISS_DET
logic                 miss_seq_num_v_o;
logic [SID_W-1:0]     miss_seq_num_sid_o;
logic [SEQ_NUM_W-1:0] miss_seq_num_start_o;	
logic [SEQ_NUM_W-1:0] miss_seq_num_cnt_o;
logic                 miss_sid_v_o;
logic [SID_W-1:0]     miss_sid_start_o;
logic [SEQ_NUM_W-1:0] miss_sid_seq_num_start_o;
logic [SID_W-1:0]     miss_sid_cnt_o;
logic [SEQ_NUM_W-1:0] miss_sid_seq_num_end_o;
`endif

`ifdef HEARTBEAT
logic flatlined_v_o;
`endif
// AXI payload generation:
localparam TOTAL_MSGS = 1 + 3 + 5; // R + 3xA (non-AAPL) + 5xA (AAPL)
localparam MSG_LEN = 14;           // 1+1+2+2+8 = 14 bytes per message
localparam TOTAL_BYTES = 128;      // pad to 128 (multiple of 8)
reg [7:0] raw_bytes [0:TOTAL_BYTES-1];

// --- constants for synthetic messages ---
localparam [15:0] AAPL_LOC     = 16'h0042;
localparam [15:0] NON_AAPL_LOC = 16'h0043;
integer addr, beats, i, idx, k, j, m, base;

initial begin
  $dumpfile("wave/moldudp64_tb.vcd"); // create a VCD waveform dump called "wave.vcd"
  $dumpvars(0, moldudp64_tb);
 //bin_f = $fopen("in.bin", "wb");

  for (i = 0; i < TOTAL_BYTES; i = i + 1)
    raw_bytes[i] = 8'h00;

  // initialize index into raw_bytes
  idx = 0;

  // 1) R-message
  raw_bytes[idx+0]  = 8'h01;      // valid
  raw_bytes[idx+1]  = "R";        // msg_type
  raw_bytes[idx+2]  = AAPL_LOC[7:0];   // stock_locate LE
  raw_bytes[idx+3]  = AAPL_LOC[15:8];
  raw_bytes[idx+4]  = 8'h08;      // length = 8
  raw_bytes[idx+5]  = 8'h00;
  raw_bytes[idx+6]  = "A";
  raw_bytes[idx+7]  = "A";
  raw_bytes[idx+8]  = "P";
  raw_bytes[idx+9]  = "L";
  raw_bytes[idx+10] = " ";
  raw_bytes[idx+11] = " ";
  raw_bytes[idx+12] = " ";
  raw_bytes[idx+13] = " ";
  idx = idx + 14;

  // 2) 3x A-message for NON_AAPL_LOC
  for (k = 0; k < 3; k = k + 1) begin
    raw_bytes[idx+0]  = 8'h01; // valid
    raw_bytes[idx+1]  = "A";   // msg_type
    raw_bytes[idx+2]  = NON_AAPL_LOC[7:0];
    raw_bytes[idx+3]  = NON_AAPL_LOC[15:8];
    raw_bytes[idx+4]  = 8'h06; // length = 6
    raw_bytes[idx+5]  = 8'h00;
    // data: order_id=0x2000+k, price=0x6400, size=50
    raw_bytes[idx+6]  = ((16'h2000 + k) >> 8) & 8'hFF;
    raw_bytes[idx+7]  = (16'h2000 + k) & 8'hFF;
    raw_bytes[idx+8]  = (16'h6400 >> 8);
    raw_bytes[idx+9]  = (16'h6400 & 8'hFF);
    raw_bytes[idx+10] = 8'h00; // upper byte of size (always 0)
    raw_bytes[idx+11] = 8'h32; // size=50
    raw_bytes[idx+12] = 8'h00; // pad
    raw_bytes[idx+13] = 8'h00; // pad
    idx = idx + 14;
  end

  // 3) 5x A-message for AAPL_LOC
  for (j = 0; j < 5; j = j + 1) begin
    raw_bytes[idx+0]  = 8'h01; // valid
    raw_bytes[idx+1]  = "A";   // msg_type
    raw_bytes[idx+2]  = AAPL_LOC[7:0];
    raw_bytes[idx+3]  = AAPL_LOC[15:8];
    raw_bytes[idx+4]  = 8'h06; // length = 6
    raw_bytes[idx+5]  = 8'h00;
    // data: order_id=0x1000+j, price=0x7D00, size=100
    raw_bytes[idx+6]  = ((16'h1000 + j) >> 8) & 8'hFF;
    raw_bytes[idx+7]  = (16'h1000 + j) & 8'hFF;
    raw_bytes[idx+8]  = (16'h7D00 >> 8);
    raw_bytes[idx+9]  = (16'h7D00 & 8'hFF);
    raw_bytes[idx+10] = 8'h00; // upper byte of size (always 0)
    raw_bytes[idx+11] = 8'h64; // size=100
    raw_bytes[idx+12] = 8'h00; // pad
    raw_bytes[idx+13] = 8'h00; // pad
    idx = idx + 14;
  end

  // pad remainder with zeros (already done above)

  $display("Test start");
  udp_axis_tvalid_i = 1'b0;
  udp_axis_tkeep_i  = {AXI_KEEP_W{1'bx}};
  udp_axis_tdata_i  = {AXI_DATA_W{1'bx}};
  udp_axis_tlast_i  = 1'bx;
  udp_axis_tuser_i  = 1'bx;
  #10;
  nreset = 1'b1;
  #10;

  // Drive AXI stream from raw_bytes
  beats = TOTAL_BYTES / 8;
  for (addr = 0; addr < beats; addr = addr + 1) begin
    @(posedge clk);
    udp_axis_tvalid_i = 1'b1;
    udp_axis_tlast_i  = (addr == beats-1);
    udp_axis_tkeep_i = {AXI_KEEP_W{1'b1}};
    udp_axis_tdata_i = {
      raw_bytes[8*addr+0],
      raw_bytes[8*addr+1],
      raw_bytes[8*addr+2],
      raw_bytes[8*addr+3],
      raw_bytes[8*addr+4],
      raw_bytes[8*addr+5],
      raw_bytes[8*addr+6],
      raw_bytes[8*addr+7]
    };
  end
  // Deassert valid
  @(posedge clk);
  udp_axis_tvalid_i = 1'b0;

  // Wait a few cycles for parser to finish
  repeat (20) @(posedge clk);
  $display("Test end");
  $finish;
end

// Stock locate extraction (16 bits) from header: for this TB, just use lower 16 bits of header
wire [15:0] stock_locate = moldudp_header[15:0];

 /* Make a regular pulsing clock. */
always #5 clk = !clk;

moldudp64 #(
	`ifdef DEBUG_ID
	.DEBUG_ID_W(DEBUG_ID_W),
	`endif
	.AXI_DATA_W(AXI_DATA_W),
	.AXI_KEEP_W(AXI_KEEP_W),
	.SID_W(SID_W),
	.SEQ_NUM_W(SEQ_NUM_W),
	.ML_W(ML_W),
	.EOS_MSG_CNT(16'hffff)
) m_moldudp64(
	.clk(clk),
	.nreset(nreset),
	
	.udp_axis_tvalid_i(udp_axis_tvalid_i),
	.udp_axis_tkeep_i (udp_axis_tkeep_i ),
	.udp_axis_tdata_i (udp_axis_tdata_i ),
	.udp_axis_tlast_i (udp_axis_tlast_i ),
	.udp_axis_tuser_i (udp_axis_tuser_i ),
	.udp_axis_tready_o(udp_axis_tready_o),
	
	`ifdef MISS_DET
	.miss_seq_num_v_o    (miss_seq_num_v_o),
	.miss_seq_num_sid_o  (miss_seq_num_sid_o),
	.miss_seq_num_start_o(miss_seq_num_start_o),	
	.miss_seq_num_cnt_o  (miss_seq_num_cnt_o),
		
	.miss_sid_v_o            (miss_sid_v_o),
	.miss_sid_start_o        (miss_sid_start_o),
	.miss_sid_seq_num_start_o(miss_sid_seq_num_start_o),
	.miss_sid_cnt_o          (miss_sid_cnt_o),
	.miss_sid_seq_num_end_o  (miss_sid_seq_num_end_o),
	`endif
	
	`ifdef HEARTBEAT
	.flatlined_v_o   (flatlined_v_o     ),
	`endif
	
	`ifdef MOLD_MSG_IDS
	.mold_msg_sid_o    (mold_msg_sid_o    ),
	.mold_msg_seq_num_o(mold_msg_seq_num_o),
	`endif

	`ifdef DEBUG_ID
	.mold_msg_debug_id_o(mold_msg_debug_id_o),
	`endif
	
	.mold_msg_v_o    (mold_msg_v_o    ),
	.mold_msg_start_o(mold_msg_start_o),
	.mold_msg_len_o  (mold_msg_len_o  ),
	.mold_msg_data_o (mold_msg_data_o ),

	.mold_msg_ov_v_o    (mold_msg_ov_v_o    ),
	.mold_msg_ov_len_o  (mold_msg_ov_len_o ),
	.mold_msg_ov_data_o (mold_msg_ov_data_o )
);
// xchecks
always @(posedge clk) begin
	if ( nreset ) begin
		`ifdef HEARTBEAT
		assert( ~$isunknown(flatlined_v_o));
		`endif
		assert( ~$isunknown(mold_msg_v_o ));
		if ( mold_msg_v_o ) begin
			assert( ~$isunknown(mold_msg_start_o));
			assert( ~$isunknown(mold_msg_len_o));
			`ifdef MOLD_MSG_IDS
			assert( ~$isunknown(mold_msg_seq_num_o));
			assert( ~$isunknown(mold_msg_sid_o));
			`endif
			`ifdef DEBUG_ID
			assert( ~$isunknown(mold_msg_debug_id_o));
			`endif
			end
	end
end
/*
genvar s,i;
generate
for( s = 0; s < ITCH_N; s++ ) begin
	for( i = 0; i < AXI_KEEP_W; i++) begin
		logic [7:0] masked_data;
		assign masked_data ={8{ (mold_msg_len_o[s*KEEP_LW+KEEP_LW-1:s*KEEP_LW] >= i) & mold_msg_v_o[s] }} & mold_msg_data_o[s*AXI_DATA_W+8*i+7:s*AXI_DATA_W+8*i];
		always @(posedge clk) begin
			if ( nreset ) begin
				assert( ~$isunknown( masked_data ));
				`ifdef DEBUG
				if ( $isunknown( masked_data )) begin
					$display("%t i %d masked data %h, len %d, data %h",
						$time,i,masked_data, mold_msg_len_o[s*KEEP_LW+KEEP_LW-1:s*KEEP_LW],
						 mold_msg_data_o[s*AXI_DATA_W+8*i+7:s*AXI_DATA_W+8*i]);
				end
				`endif
			end
		end
	
	end
end
endgenerate
*/

// --- Serialize parsed messages to in.bin as they come out of the parser ---
always @(posedge clk) begin
  if (nreset && mold_msg_v_o) begin
    // valid
    $fwrite(bin_f, "%c", 8'h01);
    // msg_type: use the first byte of parsed data instead of header
    $fwrite(bin_f, "%c", mold_msg_data_o[7:0]);
    // stock_locate (16 bits LE) from parsed payload bytes [15:8] and [23:16]
    $fwrite(bin_f, "%c", mold_msg_data_o[15:8]);
    $fwrite(bin_f, "%c", mold_msg_data_o[23:16]);
    // length (16 bits LE)
    $fwrite(bin_f, "%c", mold_msg_len_o[7:0]);
    $fwrite(bin_f, "%c", mold_msg_len_o[15:8]);
    // data (up to 8 bytes)
    $fwrite(bin_f, "%c", mold_msg_data_o[7:0]);
    $fwrite(bin_f, "%c", mold_msg_data_o[15:8]);
    $fwrite(bin_f, "%c", mold_msg_data_o[23:16]);
    $fwrite(bin_f, "%c", mold_msg_data_o[31:24]);
    $fwrite(bin_f, "%c", mold_msg_data_o[39:32]);
    $fwrite(bin_f, "%c", mold_msg_data_o[47:40]);
    $fwrite(bin_f, "%c", mold_msg_data_o[55:48]);
    $fwrite(bin_f, "%c", mold_msg_data_o[63:56]);
  end
end

endmodule