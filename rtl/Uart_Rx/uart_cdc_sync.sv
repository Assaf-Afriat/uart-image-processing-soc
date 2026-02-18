// ============================================================================
// UART CDC Synchronizer
// ============================================================================
// Handles clock domain crossing between:
//   - RX Domain (176 MHz): uart_rx_PHY, uart_rx_MAC, uart_classifier
//   - System Domain: sequencers, RGF modules
//
// CDC Strategy:
//   - 2-FF synchronizers for control signals (both directions)
//   - Data buses pass through (stable during handshake - UART is slow)
//   - Handshake ensures data validity across domains
//
// Timing Analysis:
//   - UART message: ~29µs (16 bytes @ 5.5Mbaud)
//   - 2-FF sync delay: ~20ns (worst case)
//   - Margin: ~1000x - very safe
//
// ============================================================================

import parser_pkg::*;

module uart_cdc_sync (
    // ========================================================================
    // Clock and Reset
    // ========================================================================
    input  logic         clk_rx,          // RX domain clock (176 MHz)
    input  logic         clk_sys,         // System domain clock
    input  logic         rst_rx_n,        // RX domain reset (active low)
    input  logic         rst_sys_n,       // System domain reset (active low)
    
    // ========================================================================
    // RX Domain Interface (from uart_classifier)
    // ========================================================================
    // Control signals (need sync)
    input  logic         rx_data_available,
    input  logic         rx_burst_on,
    input  logic         rx_valid_msg,
    
    // Data signals (stable during handshake - pass through)
    input  msg_type_e    rx_classified_type,
    input  logic [7:0]   rx_parsed_addr,
    input  logic [15:0]  rx_parsed_offset_addr,
    input  logic [15:0]  rx_parsed_data_high,
    input  logic [15:0]  rx_parsed_data_low,
    input  logic [31:0]  rx_parsed_height,
    input  logic [31:0]  rx_parsed_width,
    input  logic [7:0]   rx_pixel_r,
    input  logic [7:0]   rx_pixel_g,
    input  logic [7:0]   rx_pixel_b,
    input  logic [31:0]  rx_burst_red,
    input  logic [31:0]  rx_burst_green,
    input  logic [31:0]  rx_burst_blue,
    
    // ========================================================================
    // System Domain Interface (to sequencers/RGF)
    // ========================================================================
    // Control signals (synchronized)
    output logic         sys_data_available,
    output logic         sys_burst_on,
    output logic         sys_valid_msg,
    
    // Data signals (directly passed - stable during handshake)
    output msg_type_e    sys_classified_type,
    output logic [7:0]   sys_parsed_addr,
    output logic [15:0]  sys_parsed_offset_addr,
    output logic [15:0]  sys_parsed_data_high,
    output logic [15:0]  sys_parsed_data_low,
    output logic [31:0]  sys_parsed_height,
    output logic [31:0]  sys_parsed_width,
    output logic [7:0]   sys_pixel_r,
    output logic [7:0]   sys_pixel_g,
    output logic [7:0]   sys_pixel_b,
    output logic [31:0]  sys_burst_red,
    output logic [31:0]  sys_burst_green,
    output logic [31:0]  sys_burst_blue,
    
    // ========================================================================
    // System Domain Interface (from sequencers/RGF)
    // ========================================================================
    input  logic         sys_seq_ready,     // Sequencer consumed data
    input  logic         sys_burst_done,    // End of burst image
    
    // ========================================================================
    // RX Domain Interface (to uart_classifier)
    // ========================================================================
    output logic         rx_seq_ready,      // Synchronized to RX domain
    output logic         rx_burst_done      // Synchronized to RX domain
);

    // ========================================================================
    // RX → System Domain Synchronizers (176MHz → sys_clk)
    // ========================================================================
    // 2-FF synchronizers for control signals
    
    // data_available sync
    logic data_avail_sync1, data_avail_sync2;
    
    always_ff @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            data_avail_sync1 <= 1'b0;
            data_avail_sync2 <= 1'b0;
        end else begin
            data_avail_sync1 <= rx_data_available;
            data_avail_sync2 <= data_avail_sync1;
        end
    end
    
    // CRITICAL: Delay sys_data_available by one cycle to align with data sampling.
    // Data is sampled on data_avail_rising (registered, so updates next cycle).
    // sys_data_available must also be delayed so the sequencer sees both
    // the new data AND new data_available on the same cycle.
    logic sys_data_available_d;
    always_ff @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)
            sys_data_available_d <= 1'b0;
        else
            sys_data_available_d <= data_avail_sync2;
    end
    assign sys_data_available = sys_data_available_d;
    
    // burst_on sync
    logic burst_on_sync1, burst_on_sync2;
    
    always_ff @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            burst_on_sync1 <= 1'b0;
            burst_on_sync2 <= 1'b0;
        end else begin
            burst_on_sync1 <= rx_burst_on;
            burst_on_sync2 <= burst_on_sync1;
        end
    end
    
    assign sys_burst_on = burst_on_sync2;
    
    // valid_msg sync
    logic valid_msg_sync1, valid_msg_sync2;
    
    always_ff @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            valid_msg_sync1 <= 1'b0;
            valid_msg_sync2 <= 1'b0;
        end else begin
            valid_msg_sync1 <= rx_valid_msg;
            valid_msg_sync2 <= valid_msg_sync1;
        end
    end
    
    assign sys_valid_msg = valid_msg_sync2;
    
    // ========================================================================
    // Data Signal Pass-Through (RX → System)
    // ========================================================================
    // CRITICAL: Only sample data when we detect the rising edge of the
    // synchronized data_available signal. This ensures data is stable
    // (has been set for at least one 176MHz cycle) before we capture it.
    //
    // Use a 3rd register for proper edge detection on the STABLE sync2 signal.
    // Using sync1 directly is risky because it could still be metastable.
    // sync2 is guaranteed stable after the 2-FF synchronizer settles.
    
    // Third register for edge detection (uses stable sync2 output)
    logic data_avail_sync3;
    always_ff @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)
            data_avail_sync3 <= 1'b0;
        else
            data_avail_sync3 <= data_avail_sync2;
    end
    
    // Detect rising edge of synchronized data_available (using STABLE signals)
    logic data_avail_rising;
    assign data_avail_rising = data_avail_sync2 && !data_avail_sync3;
    
    // Registered data capture - ONLY sample on rising edge of sync signal
    always_ff @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            sys_classified_type    <= MSG_NONE;
            sys_parsed_addr        <= 8'd0;
            sys_parsed_offset_addr <= 16'd0;
            sys_parsed_data_high   <= 16'd0;
            sys_parsed_data_low    <= 16'd0;
            sys_parsed_height      <= 32'd0;
            sys_parsed_width       <= 32'd0;
            sys_pixel_r            <= 8'd0;
            sys_pixel_g            <= 8'd0;
            sys_pixel_b            <= 8'd0;
            sys_burst_red          <= 32'd0;
            sys_burst_green        <= 32'd0;
            sys_burst_blue         <= 32'd0;
        end else if (data_avail_rising) begin
            // Sample data ONLY when new data arrives (rising edge of sync)
            // Data has been stable in RX domain, safe to capture now
            sys_classified_type    <= rx_classified_type;
            sys_parsed_addr        <= rx_parsed_addr;
            sys_parsed_offset_addr <= rx_parsed_offset_addr;
            sys_parsed_data_high   <= rx_parsed_data_high;
            sys_parsed_data_low    <= rx_parsed_data_low;
            sys_parsed_height      <= rx_parsed_height;
            sys_parsed_width       <= rx_parsed_width;
            sys_pixel_r            <= rx_pixel_r;
            sys_pixel_g            <= rx_pixel_g;
            sys_pixel_b            <= rx_pixel_b;
            sys_burst_red          <= rx_burst_red;
            sys_burst_green        <= rx_burst_green;
            sys_burst_blue         <= rx_burst_blue;
        end
        // else: hold previous values
    end
    
    // ========================================================================
    // System → RX Domain Synchronizers (sys_clk → 176MHz)
    // ========================================================================
    // seq_ready: Can be level or pulse from sequencer
    // Using 2-FF sync - works for both level and slow pulses
    
    logic seq_ready_sync1, seq_ready_sync2;
    
    always_ff @(posedge clk_rx or negedge rst_rx_n) begin
        if (!rst_rx_n) begin
            seq_ready_sync1 <= 1'b0;
            seq_ready_sync2 <= 1'b0;
        end else begin
            seq_ready_sync1 <= sys_seq_ready;
            seq_ready_sync2 <= seq_ready_sync1;
        end
    end
    
    assign rx_seq_ready = seq_ready_sync2;
    
    // burst_done: Pulse from sequencer when image transfer complete
    // Using 2-FF sync
    
    logic burst_done_sync1, burst_done_sync2;
    
    always_ff @(posedge clk_rx or negedge rst_rx_n) begin
        if (!rst_rx_n) begin
            burst_done_sync1 <= 1'b0;
            burst_done_sync2 <= 1'b0;
        end else begin
            burst_done_sync1 <= sys_burst_done;
            burst_done_sync2 <= burst_done_sync1;
        end
    end
    
    assign rx_burst_done = burst_done_sync2;

endmodule
