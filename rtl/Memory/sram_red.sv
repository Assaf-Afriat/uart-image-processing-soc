// Simple framebuffer SRAM: 8-bit wide, synchronous write, optional synchronous read
module sram_red #(
  parameter int ADDR_W = 14,          // depth = 2^ADDR_W = 16384 (256x256 / 4 pixels per word)
  parameter int DATA_W = 32
)(
  input  logic                 clk,

  // Write port (from your FIFO->SRAM writer)
  input  logic                 wr_en,
  input  logic [ADDR_W-1:0]    wr_addr,
  input  logic [DATA_W-1:0]    wr_data,

  // Optional read port (for debug or display pipeline)
  input  logic                 rd_en,
  input  logic [ADDR_W-1:0]    rd_addr,
  output logic [DATA_W-1:0]    rd_data
);

  localparam int DEPTH = 1 << ADDR_W;

  // Memory array (infers block RAM on many FPGAs)
  logic [DATA_W-1:0] mem_sram_red [0:DEPTH-1];

  // Synchronous write (1 byte per cycle)
  always_ff @(posedge clk) begin
    if (wr_en) begin
      mem_sram_red[wr_addr] <= wr_data;
    end
  end

  // Synchronous read (1-cycle latency)
  always_ff @(posedge clk) begin
    if (rd_en) begin
      rd_data <= mem_sram_red[rd_addr];
    end
  end

endmodule