// async_fifo_gray_af.sv
// Async FIFO with Gray-code pointers + 2FF CDC sync + EMPTY/FULL + ALMOST_EMPTY/ALMOST_FULL (CDC-correct)
//
// Notes:
// - DEPTH is power-of-2: DEPTH = 2**ADDR_W
// - Pointer width PTR_W = ADDR_W+1 (extra MSB is wrap bit)
// - Flags are generated in their *local* clock domains using synced opposite pointers.
//
// Typical use:
// - Write side (SEQ B): push when !full (and usually pause when almost_full)
// - Read  side (SEQ A): pop  when !empty (and usually pause when almost_empty)

module FIFO_Async #(
  parameter int DATA_W = 32,
  parameter int FIFO_DEPTH = 16,
  parameter int ADDR_W = $clog2(FIFO_DEPTH),
  parameter int PTR_W  = ADDR_W + 1,           // pointer width (wrap bit included)
  parameter int ALMOST_FULL_LEVEL  = 12,        // assert when free slots <= this
  parameter int ALMOST_EMPTY_LEVEL = 4         // assert when words  <= this
)(
  // write domain
  input  logic              wr_clk,
  input  logic              wr_rst_n,
  input  logic              push,
  input  logic [DATA_W-1:0] data_in,
  output logic              full,
  output logic              almost_full,

  // read domain
  input  logic              rd_clk,
  input  logic              rd_rst_n,
  input  logic              pop,
  output logic [DATA_W-1:0] data_out,
  output logic              empty,
  output logic              almost_empty
);

  // -------------------------
  // Simple memory model
  // -------------------------
  logic [DATA_W-1:0] mem [0:FIFO_DEPTH-1];

  // -------------------------
  // Gray/Bin helpers
  // -------------------------
  function automatic logic [PTR_W-1:0] bin2gray(input logic [PTR_W-1:0] b);
    return (b >> 1) ^ b;                                         
  endfunction

  function automatic logic [PTR_W-1:0] gray2bin(input logic [PTR_W-1:0] g);
    logic [PTR_W-1:0] b;
    int i;
    begin
      b[PTR_W-1] = g[PTR_W-1];
      for (i = PTR_W-2; i >= 0; i--) begin
        b[i] = b[i+1] ^ g[i];
      end
      return b;
    end
  endfunction

  // -------------------------
  // WRITE DOMAIN POINTERS
  // -------------------------
  logic [PTR_W-1:0] wbin,  wbin_next;
  logic [PTR_W-1:0] wgray, wgray_next;

  // Sync read gray pointer into write domain (2FF)
  logic [PTR_W-1:0] rgray;
  logic [PTR_W-1:0] rgray_wff1, rgray_wsync;

  // compute next (unconditionally to avoid combinatorial loop)
  always_comb begin
    wbin_next  = wbin + 1;
    wgray_next = bin2gray(wbin_next);
  end

  // 2FF sync rgray into wr_clk domain
  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      rgray_wff1  <= '0;
      rgray_wsync <= '0;
    end else begin
      rgray_wff1  <= rgray;
      rgray_wsync <= rgray_wff1;
    end
  end

  // FULL (Cummings): compare next wgray to rgray synced with top 2 bits inverted
  logic [PTR_W-1:0] rgray_winv;
  assign rgray_winv = {~rgray_wsync[PTR_W-1:PTR_W-2], rgray_wsync[PTR_W-3:0]};

  always_comb begin
    full = (wgray_next == rgray_winv);
  end

  // write enable gated (after full computation to break loop)
  logic wpush;
  assign wpush = push && !full;

  // update write pointers + memory write
  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wbin  <= '0;
      wgray <= '0;
    end else if (wpush) begin
      wbin  <= wbin_next;
      wgray <= wgray_next;
      mem[wbin[ADDR_W-1:0]] <= data_in;
    end
  end

  // -------------------------
  // READ DOMAIN POINTERS
  // -------------------------
  logic [PTR_W-1:0] rbin,  rbin_next;
  logic [PTR_W-1:0] rgray, rgray_next;

  // Sync write gray pointer into read domain (2FF)
  logic [PTR_W-1:0] wgray_rff1, wgray_rsync;

  // compute next (unconditionally to avoid combinatorial loop)
  always_comb begin
    rbin_next  = rbin + 1;
    rgray_next = bin2gray(rbin_next);
  end

  // 2FF sync wgray into rd_clk domain
  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      wgray_rff1  <= '0;
      wgray_rsync <= '0;
    end else begin
      wgray_rff1  <= wgray;
      wgray_rsync <= wgray_rff1;
    end
  end

  // EMPTY: current rgray equals synced wgray (standard Cummings formulation)
  // NOTE: Using rgray (current), NOT rgray_next
  // When read and write pointers are equal, FIFO is empty
  always_comb begin
    empty = (rgray == wgray_rsync);
  end

  // read enable gated (after empty computation to break loop)
  logic rpop;
  assign rpop = pop && !empty;

  // update read pointers
  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rbin  <= '0;
      rgray <= '0;
    end else if (rpop) begin
      rbin  <= rbin_next;
      rgray <= rgray_next;
    end
  end

  // FWFT-style data out (combinational read from mem)
  // If you need BRAM inference, switch to registered/synchronous read.
  assign data_out = mem[rbin[ADDR_W-1:0]];

  // -------------------------
  // ALMOST flags (CDC-correct)
  // Compute occupancy locally using:
  //  - local binary ptr
  //  - synced opposite ptr (gray->bin conversion)
  //
  // In write domain:
  //   used = wbin - rbin_sync
  //   free = DEPTH - used
  //   almost_full when free <= ALMOST_FULL_LEVEL
  //
  // In read domain:
  //   used = wbin_sync - rbin
  //   almost_empty when used <= ALMOST_EMPTY_LEVEL
  // -------------------------

  // Write domain occupancy
  logic [PTR_W-1:0] rbin_wsync;
  logic [PTR_W-1:0] w_used;
  logic [PTR_W-1:0] w_free;

  assign rbin_wsync = gray2bin(rgray_wsync);
  assign w_used     = wbin - rbin_wsync;              // modulo with wrap bit
  assign w_free     = FIFO_DEPTH[PTR_W-1:0] - w_used;

  always_comb begin
    almost_full = (w_free <= ALMOST_FULL_LEVEL[PTR_W-1:0]);
  end

  // Read domain occupancy
  logic [PTR_W-1:0] wbin_rsync;
  logic [PTR_W-1:0] r_used;

  assign wbin_rsync = gray2bin(wgray_rsync);
  assign r_used     = wbin_rsync - rbin;              // words available to read

  always_comb begin
    almost_empty = (r_used <= ALMOST_EMPTY_LEVEL[PTR_W-1:0]);
  end

endmodule
//assaf


