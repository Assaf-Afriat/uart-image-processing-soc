// ============================================================================
// RX Message Handler
// ============================================================================
// Encapsulates:
//   - RGF Manager (handles RGF read/write messages)
//   - Sequencer RX Image Single (handles single pixel writes)
//   - Sequencer RX Image Burst (handles burst pixel writes)
//
// Provides unified handshake interface to classifier:
//   - Receives all messages from classifier via CDC
//   - Routes to appropriate sub-module based on message type
//   - Guarantees handshake response for ANY valid message
//
// ============================================================================

import parser_pkg::*;

module rx_msg_handler (
    input  logic         clk,
    input  logic         rst_n,
    
    // ========================================================================
    // Interface from CDC Sync (classifier outputs, synchronized)
    // ========================================================================
    input  logic         data_available,      // Message ready from classifier
    input  logic         burst_on,            // Burst mode active
    input  logic         valid_msg,           // Message passed validation
    input  msg_type_e    classified_type,     // Message type
    input  logic [7:0]   parsed_addr,         // Address
    input  logic [15:0]  parsed_offset_addr,  // Offset address
    input  logic [15:0]  parsed_data_high,    // Data high (RGF)
    input  logic [15:0]  parsed_data_low,     // Data low (RGF)
    input  logic [31:0]  parsed_height,       // Image height
    input  logic [31:0]  parsed_width,        // Image width
    input  logic [7:0]   pixel_r,             // Single pixel R
    input  logic [7:0]   pixel_g,             // Single pixel G
    input  logic [7:0]   pixel_b,             // Single pixel B
    input  logic [31:0]  burst_red,           // Burst pixels R channel
    input  logic [31:0]  burst_green,         // Burst pixels G channel
    input  logic [31:0]  burst_blue,          // Burst pixels B channel
    
    // ========================================================================
    // Handshake back to CDC/Classifier
    // ========================================================================
    output logic         seq_ready,           // Acknowledge message consumed
    output logic         burst_done,          // End of burst image
    
    // ========================================================================
    // Configuration inputs (from RGF_IMG)
    // ========================================================================
    input  logic [7:0]   hw_img_width,        // Image width for single pixel seq
    input  logic [7:0]   hw_img_height,       // Image height for single pixel seq
    
    // ========================================================================
    // RGF Interface outputs (directly to RGF modules)
    // ========================================================================
    output logic         rgf_leg_img,
    output logic         rgf_leg_tx_fifo,
    output logic         rgf_leg_led,
    output logic         rgf_leg_sys,
    output logic         rgf_leg_ctrl,
    output logic         rgf_leg_pwm,
    output logic [15:0]  rgf_offset_addr,
    output logic [31:0]  rgf_wr_data,
    output logic         rgf_wr_en,
    output logic         rgf_rd_en,
    
    // ========================================================================
    // RGF read path (for TX response)
    // ========================================================================
    input  logic [31:0]  rgf_read_data,       // Combined read data from all RGFs
    input  logic         cmpsr_busy,          // Composer busy
    output logic [31:0]  rgf_addr_to_cmpsr,   // Address to composer
    output logic [31:0]  rgf_data_to_cmpsr,   // Data to composer
    output logic         rgf_start_req_to_cmpsr, // Start request to composer
    output logic         rgf_read_active,     // RGF read in progress
    
    // ========================================================================
    // SRAM Write Interface - Single Pixel Sequencer
    // ========================================================================
    output logic [31:0]  sing_red_write_data,
    output logic [31:0]  sing_green_write_data,
    output logic [31:0]  sing_blue_write_data,
    output logic         sing_sram_r_wr_en,
    output logic [13:0]  sing_sram_r_addr,
    output logic         sing_sram_g_wr_en,
    output logic [13:0]  sing_sram_g_addr,
    output logic         sing_sram_b_wr_en,
    output logic [13:0]  sing_sram_b_addr,
    output logic         sing_seq_busy,
    output logic         sing_seq_done,
    
    // ========================================================================
    // SRAM Write Interface - Burst Sequencer
    // ========================================================================
    output logic [31:0]  burst_red_write_data,
    output logic [31:0]  burst_green_write_data,
    output logic [31:0]  burst_blue_write_data,
    output logic         burst_sram_r_wr_en,
    output logic [13:0]  burst_sram_r_addr,
    output logic         burst_sram_g_wr_en,
    output logic [13:0]  burst_sram_g_addr,
    output logic         burst_sram_b_wr_en,
    output logic [13:0]  burst_sram_b_addr,
    output logic         burst_seq_busy,
    output logic         burst_seq_done_out   // Also drives burst_done
);

    // ========================================================================
    // Internal signals
    // ========================================================================
    
    // Individual module handshake signals
    logic rgf_got_msg;
    logic sing_got_msg;
    logic burst_got_msg;
    
    // Internal burst done from sequencer
    logic burst_seq_done_internal;
    
    // ========================================================================
    // Unified Handshake Logic
    // ========================================================================
    // ANY module that handles a message will assert its got_msg signal.
    // We OR them together to create a unified seq_ready back to classifier.
    // This guarantees that every valid message gets acknowledged.
    
    assign seq_ready = rgf_got_msg | sing_got_msg | burst_got_msg;
    
    // Burst done comes from the burst sequencer
    assign burst_done = burst_seq_done_internal;
    assign burst_seq_done_out = burst_seq_done_internal;

    // ========================================================================
    // RGF Manager Instance
    // ========================================================================
    rgf_manager u_rgf_manager (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .raw_base_address       (parsed_addr),
        .raw_offset_address     (parsed_offset_addr),
        .raw_data_high          (parsed_data_high),
        .raw_data_low           (parsed_data_low),
        .Msg_Type               (classified_type),
        .read_data_from_rgf     (rgf_read_data),
        .cmpsr_busy             (cmpsr_busy),
        .now_image_read_single  (1'b0),  // Not used currently
        .now_image_read_burst   (1'b0),  // Not used currently
        .new_msg_valid          (data_available),
        .rgf_leg_img            (rgf_leg_img),
        .rgf_leg_tx_fifo        (rgf_leg_tx_fifo),
        .rgf_leg_led            (rgf_leg_led),
        .rgf_leg_sys            (rgf_leg_sys),
        .rgf_leg_ctrl           (rgf_leg_ctrl),
        .rgf_leg_pwm            (rgf_leg_pwm),
        .offset_address         (rgf_offset_addr),
        .wr_data                (rgf_wr_data),
        .wr_en                  (rgf_wr_en),
        .rd_en                  (rgf_rd_en),
        .raw_address_to_cmpsr   (rgf_addr_to_cmpsr),
        .data_out_to_cmpsr      (rgf_data_to_cmpsr),
        .start_request_to_cmpsr (rgf_start_req_to_cmpsr),
        .rgf_read               (rgf_read_active),
        .got_msg_from_class     (rgf_got_msg)
    );

    // ========================================================================
    // Sequencer RX Image Single Instance
    // ========================================================================
    seq_rx_image_single u_seq_rx_image_single (
        .clk              (clk),
        .rst_n            (rst_n),
        .img_width        (hw_img_width),
        .img_height       (hw_img_height),
        .Msg_Type         (classified_type),
        .red_pixel        (pixel_r),
        .green_pixel      (pixel_g),
        .blue_pixel       (pixel_b),
        .new_msg_valid    (data_available),
        .red_write_data   (sing_red_write_data),
        .green_write_data (sing_green_write_data),
        .blue_write_data  (sing_blue_write_data),
        .sram_r_wr_en     (sing_sram_r_wr_en),
        .sram_r_addr_wr   (sing_sram_r_addr),
        .sram_g_wr_en     (sing_sram_g_wr_en),
        .sram_g_addr_wr   (sing_sram_g_addr),
        .sram_b_wr_en     (sing_sram_b_wr_en),
        .sram_b_addr_wr   (sing_sram_b_addr),
        .rx_seq_single_busy (sing_seq_busy),
        .rx_seq_single_dn   (sing_seq_done),
        .got_msg_from_class (sing_got_msg)
    );

    // ========================================================================
    // Sequencer RX Image Burst Instance
    // ========================================================================
    seq_rx_image_burst u_seq_rx_image_burst (
        .clk                (clk),
        .rst_n              (rst_n),
        .Msg_Type           (classified_type),
        .img_height         (parsed_height[15:0]),  // 16-bit to support 256x256
        .img_width          (parsed_width[15:0]),   // 16-bit to support 256x256
        .red_burst          (burst_red),
        .green_burst        (burst_green),
        .blue_burst         (burst_blue),
        .new_msg_valid      (data_available),
        .red_wr_data        (burst_red_write_data),
        .green_wr_data      (burst_green_write_data),
        .blue_wr_data       (burst_blue_write_data),
        .sram_r_wr_en       (burst_sram_r_wr_en),
        .sram_r_addr_wr     (burst_sram_r_addr),
        .sram_g_wr_en       (burst_sram_g_wr_en),
        .sram_g_addr_wr     (burst_sram_g_addr),
        .sram_b_wr_en       (burst_sram_b_wr_en),
        .sram_b_addr_wr     (burst_sram_b_addr),
        .rx_seq_burst_busy  (burst_seq_busy),
        .rx_seq_burst_dn    (burst_seq_done_internal),
        .got_msg_from_class (burst_got_msg)
    );

endmodule
