`timescale 1ns / 1ps

/**
 * Module: Sequencer_Burst_Tx
 * Description: Burst Sequencer managing data flow between external Triple-RAM
 *              (Red, Green, Blue) and a TX MAC via three internal Async FIFOs.
 *
 * Architecture:
 *   - Write side (wr_clk): Fetches 32-bit words from RAMs, pushes into 3 Async FIFOs
 *   - Read side  (rd_clk): Pops from FIFOs, reconstructs 96-bit pixel_packet (4 pixels)
 *   - Each FIFO handles one color channel (32-bit wide, 16 rows deep)
 *   - CDC between wr_clk (RAM side) and rd_clk (MAC side) handled by Gray-code FIFOs
 *
 * Pixel Packet Format [95:0]:
 *   {R3,G3,B3, R2,G2,B2, R1,G1,B1, R0,G0,B0}
 *    P3[95:72]  P2[71:48]  P1[47:24]  P0[23:0]
 *
 * Write-side push:  activate_burst & img_ctrl_start & !full & (counter < PIXEL_COUNT)
 * Read-side  pop:   !tx_busy & !empty & (counter < PIXEL_COUNT)
 * pixel_packet_ready is asserted exactly 1 rd_clk cycle after pop (registered output).
 */

module Sequencer_Burst_Tx #(
    parameter RAM_ADDR_WIDTH = 14,          // $clog2(PIXEL_COUNT / 4)
    parameter RAM_DATA_WIDTH = 32,          // 4 pixels per word {P3,P2,P1,P0}
    parameter FIFO_WIDTH     = 32,          // Matches RAM data width
    parameter FIFO_DEPTH     = 16,          // 16 rows per FIFO (power-of-2)
    parameter PIXEL_COUNT    = 65536        // Total pixels in frame
)(
    // -------------------------------------------------------------------------
    // Clocks and Reset
    // -------------------------------------------------------------------------
    input  logic                         wr_clk,       // Write clock (RAM side)
    input  logic                         rd_clk,       // Read clock  (MAC side)
    input  logic                         rst_n,        // Async active-low reset

    // -------------------------------------------------------------------------
    // RAM Interface (wr_clk domain)
    // -------------------------------------------------------------------------
    input  logic [RAM_DATA_WIDTH-1:0]    red_RAM_data,    // [31:0] {R3,R2,R1,R0}
    input  logic [RAM_DATA_WIDTH-1:0]    green_RAM_data,  // [31:0] {G3,G2,G1,G0}
    input  logic [RAM_DATA_WIDTH-1:0]    blue_RAM_data,   // [31:0] {B3,B2,B1,B0}
    output logic [RAM_ADDR_WIDTH-1:0]    red_RAM_addr,
    output logic [RAM_ADDR_WIDTH-1:0]    green_RAM_addr,
    output logic [RAM_ADDR_WIDTH-1:0]    blue_RAM_addr,
    output logic                         red_read_en,     // 1 bit to RAM
    output logic                         green_read_en,   // 1 bit to RAM
    output logic                         blue_read_en,    // 1 bit to RAM

    // -------------------------------------------------------------------------
    // MAC Interface (rd_clk domain)
    // -------------------------------------------------------------------------
    input  logic                         tx_busy,           // MAC busy (active high)
    output logic [95:0]                  pixel_packet,      // 4-pixel burst packet
    output logic                         pixel_packet_ready,// 1 cycle after pop

    // -------------------------------------------------------------------------
    // RGF I/O (Register File Interface)
    // -------------------------------------------------------------------------
    input  logic                         img_ctrl_start,    // RGF: start image TX
    input  logic                         activate_burst,    // RGF: enable burst (active high)
    input  logic                         img_complete,      // RGF: image fully written to RAM
    output logic                         frame_tx_active,   // RGF: frame TX in progress (read side)
    output logic [$clog2(PIXEL_COUNT):0] pixel_counter      // RGF: read-side pixel progress
);

    // =========================================================================
    // Local Parameters
    // =========================================================================
    localparam int TOTAL_ADDR = PIXEL_COUNT / 4;              // 4-pixel groups
    localparam int ACNT_W     = $clog2(TOTAL_ADDR + 1);       // Counter width

    // =========================================================================
    // FIFO Interface Signals
    // =========================================================================
    logic fifo_push;                          // Shared push (all 3 FIFOs)
    logic fifo_pop;                           // Shared pop  (all 3 FIFOs)

    // Full flags (wr_clk domain)
    logic red_full,   green_full,   blue_full;
    logic red_af,     green_af,     blue_af;       // almost_full (unused, required by port)
    logic any_full;

    // Empty flags (rd_clk domain)
    logic red_empty,  green_empty,  blue_empty;
    logic red_ae,     green_ae,     blue_ae;       // almost_empty (unused, required by port)
    logic any_empty;

    // FIFO data outputs (rd_clk domain, FWFT)
    logic [FIFO_WIDTH-1:0] red_fifo_out;
    logic [FIFO_WIDTH-1:0] green_fifo_out;
    logic [FIFO_WIDTH-1:0] blue_fifo_out;

    assign any_full  = red_full  | green_full  | blue_full;
    assign any_empty = red_empty | green_empty | blue_empty;

    // =========================================================================
    //  WRITE SIDE (wr_clk domain)
    // =========================================================================
    // SRAM has 1-cycle read latency. We need to:
    //   Cycle N:   Assert read_en, provide address
    //   Cycle N+1: SRAM data valid, push to FIFO
    // =========================================================================

    // Write address counter - increments by 1 per SRAM read request
    logic [ACNT_W-1:0] wr_addr_cnt;

    // SRAM Read Request - asserted when we want to read from SRAM
    // CRITICAL: Must NOT issue a new read while a previous read's push is pending.
    // The pipeline is: Cycle N: sram_read_req (SRAM read issued, addr increments)
    //                  Cycle N+1: sram_read_req_d1 (SRAM data valid, push to FIFO)
    // If we allow back-to-back reads, the push at N+1 might fill the FIFO,
    // causing the push at N+2 to fail (fifo_push gated by ~any_full).
    // The data from cycle N+1's read is LOST because wr_addr_cnt already advanced.
    // Fix: Don't issue a new read if a push is still pending (sram_read_req_d1 = 1).
    logic sram_read_req;
    assign sram_read_req = activate_burst & img_ctrl_start
                         & ~any_full
                         & ~sram_read_req_d1  // Prevent pipeline hazard: wait for pending push
                         & (wr_addr_cnt < TOTAL_ADDR[ACNT_W-1:0]);

    // Delay the read request by 1 cycle - this becomes the FIFO push
    // (because SRAM data is valid 1 cycle after read request)
    logic sram_read_req_d1;
    always_ff @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n)
            sram_read_req_d1 <= 1'b0;
        else
            sram_read_req_d1 <= sram_read_req;
    end

    // FIFO push happens when SRAM data is valid (1 cycle after read request)
    // Also check FIFO not full as a safety measure
    assign fifo_push = sram_read_req_d1 & ~any_full;

    // Write address counter - increment on read request (not push)
    always_ff @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n)
            wr_addr_cnt <= '0;
        else if (sram_read_req)
            wr_addr_cnt <= wr_addr_cnt + 1'b1;
    end

    // RAM Addressing - all RAMs share the same address
    assign red_RAM_addr   = wr_addr_cnt[RAM_ADDR_WIDTH-1:0];
    assign green_RAM_addr = wr_addr_cnt[RAM_ADDR_WIDTH-1:0];
    assign blue_RAM_addr  = wr_addr_cnt[RAM_ADDR_WIDTH-1:0];

    // RAM Read Enable - tied to read request (not FIFO push)
    assign red_read_en   = sram_read_req;
    assign green_read_en = sram_read_req;
    assign blue_read_en  = sram_read_req;

    // =========================================================================
    //  ASYNC FIFO: Red Channel
    // =========================================================================
    FIFO_Async #(
        .DATA_W     (FIFO_WIDTH),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) fifo_red (
        .wr_clk       (wr_clk),
        .wr_rst_n     (rst_n),
        .push         (fifo_push),
        .data_in      (red_RAM_data),
        .full         (red_full),
        .almost_full  (red_af),
        .rd_clk       (rd_clk),
        .rd_rst_n     (rst_n),
        .pop          (fifo_pop),
        .data_out     (red_fifo_out),
        .empty        (red_empty),
        .almost_empty (red_ae)
    );

    // =========================================================================
    //  ASYNC FIFO: Green Channel
    // =========================================================================
    FIFO_Async #(
        .DATA_W     (FIFO_WIDTH),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) fifo_green (
        .wr_clk       (wr_clk),
        .wr_rst_n     (rst_n),
        .push         (fifo_push),
        .data_in      (green_RAM_data),
        .full         (green_full),
        .almost_full  (green_af),
        .rd_clk       (rd_clk),
        .rd_rst_n     (rst_n),
        .pop          (fifo_pop),
        .data_out     (green_fifo_out),
        .empty        (green_empty),
        .almost_empty (green_ae)
    );

    // =========================================================================
    //  ASYNC FIFO: Blue Channel
    // =========================================================================
    FIFO_Async #(
        .DATA_W     (FIFO_WIDTH),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) fifo_blue (
        .wr_clk       (wr_clk),
        .wr_rst_n     (rst_n),
        .push         (fifo_push),
        .data_in      (blue_RAM_data),
        .full         (blue_full),
        .almost_full  (blue_af),
        .rd_clk       (rd_clk),
        .rd_rst_n     (rst_n),
        .pop          (fifo_pop),
        .data_out     (blue_fifo_out),
        .empty        (blue_empty),
        .almost_empty (blue_ae)
    );

    // =========================================================================
    //  READ SIDE (rd_clk domain)
    // =========================================================================

    // Read address counter - increments by 1 per pop (each pop = 4 pixels)
    logic [ACNT_W-1:0] rd_addr_cnt;

    // Pop Condition
    assign fifo_pop = ~tx_busy & ~any_empty
                    & (rd_addr_cnt < TOTAL_ADDR[ACNT_W-1:0]);

    // Read address counter register
    always_ff @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n)
            rd_addr_cnt <= '0;
        else if (fifo_pop)
            rd_addr_cnt <= rd_addr_cnt + 1'b1;
    end

    // Pixel counter output (read-side progress, in pixel units = addr * 4)
    assign pixel_counter = {rd_addr_cnt, 2'b00};

    // -------------------------------------------------------------------------
    // frame_tx_active: Asserted on the first pop, de-asserted when full frame
    // has been transmitted (rd_addr_cnt reaches TOTAL_ADDR).
    // -------------------------------------------------------------------------
    always_ff @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n)
            frame_tx_active <= 1'b0;
        else if (fifo_pop && !frame_tx_active)
            frame_tx_active <= 1'b1;           // First pop: frame TX started
        else if (rd_addr_cnt == TOTAL_ADDR[ACNT_W-1:0])
            frame_tx_active <= 1'b0;           // Full frame done
    end

    // =========================================================================
    //  MESSAGE BUFFER (rd_clk domain)
    //  Composes the 96-bit pixel_packet directly from FIFO FWFT outputs.
    //  pixel_packet_ready is asserted on the same cycle as pop.
    // =========================================================================

    // pixel_packet_ready: same cycle as pop (FWFT data is valid immediately)
    assign pixel_packet_ready = fifo_pop;

    // Compose pixel_packet [95:0] = {P3, P2, P1, P0}
    // Each pixel Pn = {Rn[7:0], Gn[7:0], Bn[7:0]} = 24 bits
    // RAM data format: [7:0]=pixel0, [15:8]=pixel1, [23:16]=pixel2, [31:24]=pixel3
    assign pixel_packet = {
        red_fifo_out[31:24], green_fifo_out[31:24], blue_fifo_out[31:24],  // P3 [95:72]
        red_fifo_out[23:16], green_fifo_out[23:16], blue_fifo_out[23:16],  // P2 [71:48]
        red_fifo_out[15:8],  green_fifo_out[15:8],  blue_fifo_out[15:8],   // P1 [47:24]
        red_fifo_out[7:0],   green_fifo_out[7:0],   blue_fifo_out[7:0]   
    };

endmodule
