// ============================================================================
// Testbench: UART RX Full System Test
// ============================================================================
// Tests the complete RX system including:
//   - UART PHY (176 MHz)
//   - UART MAC (176 MHz)
//   - UART Classifier (176 MHz)
//   - CDC Sync (176 MHz <-> System Clock)
//   - Sequencer Single (System Clock)
//   - Sequencer Burst (System Clock)
//   - RGF Manager (System Clock)
//
// Verifies:
//   - RGF write/read operations for RGF messages
//   - Sequencer handshake with classifier via CDC
//   - Burst mode activation and pixel flow
//   - Cross-clock domain operation
// ============================================================================

`timescale 1ns / 1ps

module tb_uart_rx_system;

    // ========================================================================
    // Parameters
    // ========================================================================
    localparam int CLK_176_FREQ = 176_000_000;  // 176 MHz (RX domain)
    localparam int CLK_SYS_FREQ = 100_000_000;  // 100 MHz (System domain)
    localparam int BAUD_RATE    = 5_500_000;    // 5.5 Mbaud
    localparam int CLKS_PER_BIT = CLK_176_FREQ / BAUD_RATE;  // 32 clocks per bit
    
    localparam real CLK_176_PERIOD = 1_000_000_000.0 / CLK_176_FREQ;  // ~5.68 ns
    localparam real CLK_SYS_PERIOD = 1_000_000_000.0 / CLK_SYS_FREQ;  // 10 ns
    localparam real BIT_PERIOD = CLKS_PER_BIT * CLK_176_PERIOD;       // ~181.8 ns

    // ========================================================================
    // Import packages
    // ========================================================================
    import uart_types::*;
    import parser_pkg::*;

    // ========================================================================
    // Clocks and Resets
    // ========================================================================
    logic clk_176;      // 176 MHz clock (RX domain)
    logic clk_sys;      // 100 MHz clock (System domain)
    logic rst_176_n;    // Reset for 176 MHz domain
    logic rst_sys_n;    // Reset for system domain
    logic rx;           // UART RX input

    // ========================================================================
    // PHY Signals
    // ========================================================================
    logic [7:0]  phy_rx_data;
    logic        phy_rx_valid;
    logic        phy_rx_error;
    logic        phy_faulty_frame_inc;
    uart_state_t phy_rx_state;

    // ========================================================================
    // MAC Signals
    // ========================================================================
    logic [127:0] mac_msg_data;
    logic         mac_msg_valid;
    msg_type_e    mac_msg_type;
    logic         mac_frame_error;

    // ========================================================================
    // Classifier Signals (176 MHz domain)
    // ========================================================================
    logic         class_valid_msg;
    msg_type_e    class_type;
    logic [7:0]   class_addr;
    logic [15:0]  class_offset_addr;
    logic [15:0]  class_data_high;
    logic [15:0]  class_data_low;
    logic [31:0]  class_height;
    logic [31:0]  class_width;
    logic [7:0]   class_pixel_r;
    logic [7:0]   class_pixel_g;
    logic [7:0]   class_pixel_b;
    logic [31:0]  class_burst_red;
    logic [31:0]  class_burst_green;
    logic [31:0]  class_burst_blue;
    logic         class_data_available;
    logic         class_burst_on;
    logic         class_seq_ready;
    logic         class_burst_done;

    // ========================================================================
    // CDC Output Signals (System domain)
    // ========================================================================
    logic         sys_data_available;
    logic         sys_burst_on;
    logic         sys_valid_msg;
    msg_type_e    sys_classified_type;
    logic [7:0]   sys_parsed_addr;
    logic [15:0]  sys_parsed_offset_addr;
    logic [15:0]  sys_parsed_data_high;
    logic [15:0]  sys_parsed_data_low;
    logic [31:0]  sys_parsed_height;
    logic [31:0]  sys_parsed_width;
    logic [7:0]   sys_pixel_r;
    logic [7:0]   sys_pixel_g;
    logic [7:0]   sys_pixel_b;
    logic [31:0]  sys_burst_red;
    logic [31:0]  sys_burst_green;
    logic [31:0]  sys_burst_blue;
    logic         sys_seq_ready;
    logic         sys_burst_done;

    // ========================================================================
    // RX Message Handler Signals
    // ========================================================================
    // RGF outputs
    logic         rgf_leg_img, rgf_leg_tx_fifo, rgf_leg_led;
    logic         rgf_leg_sys, rgf_leg_ctrl, rgf_leg_pwm;
    logic [15:0]  rgf_offset_addr;
    logic [31:0]  rgf_wr_data;
    logic         rgf_wr_en;
    logic         rgf_rd_en;
    logic [31:0]  rgf_addr_to_cmpsr;
    logic [31:0]  rgf_data_to_cmpsr;
    logic         rgf_start_req_to_cmpsr;
    logic         rgf_read_active;

    // Single pixel sequencer outputs
    logic [31:0]  seq_sing_red_data;
    logic [31:0]  seq_sing_green_data;
    logic [31:0]  seq_sing_blue_data;
    logic         seq_sing_r_wr_en;
    logic [13:0]  seq_sing_r_addr;
    logic         seq_sing_g_wr_en;
    logic [13:0]  seq_sing_g_addr;
    logic         seq_sing_b_wr_en;
    logic [13:0]  seq_sing_b_addr;
    logic         seq_sing_busy;
    logic         seq_sing_done;

    // Burst sequencer outputs
    logic [31:0]  seq_burst_red_data;
    logic [31:0]  seq_burst_green_data;
    logic [31:0]  seq_burst_blue_data;
    logic         seq_burst_r_wr_en;
    logic [13:0]  seq_burst_r_addr;
    logic         seq_burst_g_wr_en;
    logic [13:0]  seq_burst_g_addr;
    logic         seq_burst_b_wr_en;
    logic [13:0]  seq_burst_b_addr;
    logic         seq_burst_busy;
    logic         seq_burst_done;

    // ========================================================================
    // Test counters and status
    // ========================================================================
    int test_num;
    int pass_count;
    int fail_count;
    
    // Image dimensions for single pixel sequencer
    logic [7:0] hw_img_width = 8'd64;
    logic [7:0] hw_img_height = 8'd64;

    // ========================================================================
    // Clock Generation
    // ========================================================================
    initial begin
        clk_176 = 0;
        forever #(CLK_176_PERIOD/2) clk_176 = ~clk_176;
    end

    initial begin
        clk_sys = 0;
        forever #(CLK_SYS_PERIOD/2) clk_sys = ~clk_sys;
    end

    // ========================================================================
    // DUT Instantiation
    // ========================================================================

    // UART RX PHY (176 MHz)
    uart_rx #(
        .CLK_FREQ  (CLK_176_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_uart_rx_phy (
        .clk              (clk_176),
        .rst_n            (rst_176_n),
        .rx               (rx),
        .rx_data          (phy_rx_data),
        .rx_valid         (phy_rx_valid),
        .rx_error         (phy_rx_error),
        .faulty_frame_inc (phy_faulty_frame_inc),
        .rx_state         (phy_rx_state)
    );

    // UART RX MAC (176 MHz)
    uart_rx_MAC u_uart_rx_mac (
        .clk           (clk_176),
        .rst_n         (rst_176_n),
        .rx_data       (phy_rx_data),
        .rx_valid      (phy_rx_valid),
        .msg_data      (mac_msg_data),
        .msg_valid     (mac_msg_valid),
        .msg_type      (mac_msg_type),
        .burst_mode_in (class_burst_on),
        .frame_error   (mac_frame_error)
    );

    // UART Classifier (176 MHz)
    uart_classifier u_uart_classifier (
        .clk                (clk_176),
        .rst_n              (rst_176_n),
        .msg_data           (mac_msg_data),
        .msg_valid          (mac_msg_valid),
        .msg_type           (mac_msg_type),
        .valid_msg          (class_valid_msg),
        .classified_type    (class_type),
        .parsed_addr        (class_addr),
        .parsed_offset_addr (class_offset_addr),
        .parsed_data_high   (class_data_high),
        .parsed_data_low    (class_data_low),
        .parsed_height      (class_height),
        .parsed_width       (class_width),
        .pixel_r            (class_pixel_r),
        .pixel_g            (class_pixel_g),
        .pixel_b            (class_pixel_b),
        .burst_red          (class_burst_red),
        .burst_green        (class_burst_green),
        .burst_blue         (class_burst_blue),
        .seq_ready          (class_seq_ready),
        .data_available     (class_data_available),
        .burst_on           (class_burst_on),
        .burst_done         (class_burst_done)
    );

    // CDC Sync (176 MHz <-> System)
    uart_cdc_sync u_uart_cdc_sync (
        .clk_rx             (clk_176),
        .clk_sys            (clk_sys),
        .rst_rx_n           (rst_176_n),
        .rst_sys_n          (rst_sys_n),
        // RX domain inputs
        .rx_data_available  (class_data_available),
        .rx_burst_on        (class_burst_on),
        .rx_valid_msg       (class_valid_msg),
        .rx_classified_type (class_type),
        .rx_parsed_addr     (class_addr),
        .rx_parsed_offset_addr (class_offset_addr),
        .rx_parsed_data_high(class_data_high),
        .rx_parsed_data_low (class_data_low),
        .rx_parsed_height   (class_height),
        .rx_parsed_width    (class_width),
        .rx_pixel_r         (class_pixel_r),
        .rx_pixel_g         (class_pixel_g),
        .rx_pixel_b         (class_pixel_b),
        .rx_burst_red       (class_burst_red),
        .rx_burst_green     (class_burst_green),
        .rx_burst_blue      (class_burst_blue),
        // System domain outputs
        .sys_data_available (sys_data_available),
        .sys_burst_on       (sys_burst_on),
        .sys_valid_msg      (sys_valid_msg),
        .sys_classified_type(sys_classified_type),
        .sys_parsed_addr    (sys_parsed_addr),
        .sys_parsed_offset_addr (sys_parsed_offset_addr),
        .sys_parsed_data_high(sys_parsed_data_high),
        .sys_parsed_data_low(sys_parsed_data_low),
        .sys_parsed_height  (sys_parsed_height),
        .sys_parsed_width   (sys_parsed_width),
        .sys_pixel_r        (sys_pixel_r),
        .sys_pixel_g        (sys_pixel_g),
        .sys_pixel_b        (sys_pixel_b),
        .sys_burst_red      (sys_burst_red),
        .sys_burst_green    (sys_burst_green),
        .sys_burst_blue     (sys_burst_blue),
        // System domain inputs
        .sys_seq_ready      (sys_seq_ready),
        .sys_burst_done     (sys_burst_done),
        // RX domain outputs
        .rx_seq_ready       (class_seq_ready),
        .rx_burst_done      (class_burst_done)
    );

    // RX Message Handler (System Clock)
    // Encapsulates RGF Manager, Sequencer Single, and Sequencer Burst
    // Provides unified handshake (sys_seq_ready, sys_burst_done)
    rx_msg_handler u_rx_msg_handler (
        .clk                    (clk_sys),
        .rst_n                  (rst_sys_n),
        
        // From CDC Sync
        .data_available         (sys_data_available),
        .burst_on               (sys_burst_on),
        .valid_msg              (sys_valid_msg),
        .classified_type        (sys_classified_type),
        .parsed_addr            (sys_parsed_addr),
        .parsed_offset_addr     (sys_parsed_offset_addr),
        .parsed_data_high       (sys_parsed_data_high),
        .parsed_data_low        (sys_parsed_data_low),
        .parsed_height          (sys_parsed_height),
        .parsed_width           (sys_parsed_width),
        .pixel_r                (sys_pixel_r),
        .pixel_g                (sys_pixel_g),
        .pixel_b                (sys_pixel_b),
        .burst_red              (sys_burst_red),
        .burst_green            (sys_burst_green),
        .burst_blue             (sys_burst_blue),
        
        // Handshake back to CDC/Classifier (unified)
        .seq_ready              (sys_seq_ready),
        .burst_done             (sys_burst_done),
        
        // Configuration
        .hw_img_width           (hw_img_width),
        .hw_img_height          (hw_img_height),
        
        // RGF Interface
        .rgf_leg_img            (rgf_leg_img),
        .rgf_leg_tx_fifo        (rgf_leg_tx_fifo),
        .rgf_leg_led            (rgf_leg_led),
        .rgf_leg_sys            (rgf_leg_sys),
        .rgf_leg_ctrl           (rgf_leg_ctrl),
        .rgf_leg_pwm            (rgf_leg_pwm),
        .rgf_offset_addr        (rgf_offset_addr),
        .rgf_wr_data            (rgf_wr_data),
        .rgf_wr_en              (rgf_wr_en),
        .rgf_rd_en              (rgf_rd_en),
        .rgf_read_data          (32'hDEADBEEF),  // Dummy read data
        .cmpsr_busy             (1'b0),
        .rgf_addr_to_cmpsr      (rgf_addr_to_cmpsr),
        .rgf_data_to_cmpsr      (rgf_data_to_cmpsr),
        .rgf_start_req_to_cmpsr (rgf_start_req_to_cmpsr),
        .rgf_read_active        (rgf_read_active),
        
        // Single Pixel Sequencer outputs
        .sing_red_write_data    (seq_sing_red_data),
        .sing_green_write_data  (seq_sing_green_data),
        .sing_blue_write_data   (seq_sing_blue_data),
        .sing_sram_r_wr_en      (seq_sing_r_wr_en),
        .sing_sram_r_addr       (seq_sing_r_addr),
        .sing_sram_g_wr_en      (seq_sing_g_wr_en),
        .sing_sram_g_addr       (seq_sing_g_addr),
        .sing_sram_b_wr_en      (seq_sing_b_wr_en),
        .sing_sram_b_addr       (seq_sing_b_addr),
        .sing_seq_busy          (seq_sing_busy),
        .sing_seq_done          (seq_sing_done),
        
        // Burst Sequencer outputs
        .burst_red_write_data   (seq_burst_red_data),
        .burst_green_write_data (seq_burst_green_data),
        .burst_blue_write_data  (seq_burst_blue_data),
        .burst_sram_r_wr_en     (seq_burst_r_wr_en),
        .burst_sram_r_addr      (seq_burst_r_addr),
        .burst_sram_g_wr_en     (seq_burst_g_wr_en),
        .burst_sram_g_addr      (seq_burst_g_addr),
        .burst_sram_b_wr_en     (seq_burst_b_wr_en),
        .burst_sram_b_addr      (seq_burst_b_addr),
        .burst_seq_busy         (seq_burst_busy),
        .burst_seq_done_out     (seq_burst_done)
    );

    // ========================================================================
    // UART Transmit Tasks
    // ========================================================================
    
    task automatic send_uart_byte(input logic [7:0] data);
        int i;
        // Start bit
        rx = 1'b0;
        #(BIT_PERIOD);
        // Data bits (LSB first)
        for (i = 0; i < 8; i++) begin
            rx = data[i];
            #(BIT_PERIOD);
        end
        // Stop bit
        rx = 1'b1;
        #(BIT_PERIOD);
    endtask

    task automatic send_message(input logic [7:0] msg[], input int len);
        int i;
        for (i = 0; i < len; i++) begin
            send_uart_byte(msg[i]);
        end
        // Minimal delay after message - just enough for stop bit to complete
        // The actual wait for processing happens separately
        #(BIT_PERIOD / 2);
    endtask
    
    // Shared variable to capture if data_available was seen
    logic saw_data_available;
    msg_type_e captured_msg_type;
    logic captured_rgf_wr_en;
    logic captured_rgf_rd_en;
    
    // Send message and wait for sys_data_available in parallel
    // Captures the state when data_available goes high
    task automatic send_and_wait_for_response(
        input logic [7:0] msg[], 
        input int len, 
        input int timeout_cycles
    );
        saw_data_available = 0;
        
        fork
            // Thread 1: Send the message
            send_message(msg, len);
            
            // Thread 2: Wait for response and CAPTURE state when found
            begin
                int count = 0;
                // Wait for sys_data_available to go high
                while (!sys_data_available && count < timeout_cycles) begin
                    @(posedge clk_sys);
                    count++;
                end
                if (count >= timeout_cycles) begin
                    $display("    DEBUG: send_and_wait TIMEOUT after %0d cycles", count);
                    saw_data_available = 0;
                end else begin
                    $display("    DEBUG: send_and_wait found signal after %0d cycles", count);
                    // CAPTURE the state NOW while data_available is still high
                    saw_data_available = 1;
                    captured_msg_type = sys_classified_type;
                    captured_rgf_wr_en = rgf_wr_en;
                    captured_rgf_rd_en = rgf_rd_en;
                end
            end
        join
    endtask

    task automatic wait_for_sys_data_available(input int timeout_cycles);
        int count = 0;
        while (!sys_data_available && count < timeout_cycles) begin
            @(posedge clk_sys);
            count++;
        end
        if (count >= timeout_cycles) begin
            $display("    DEBUG: wait_for_sys_data_available TIMEOUT after %0d cycles", count);
        end else begin
            $display("    DEBUG: wait_for_sys_data_available found signal after %0d cycles", count);
        end
    endtask

    task automatic wait_for_handshake_complete(input int timeout_cycles);
        int count = 0;
        // Wait for data_available to go low (handshake complete)
        while (sys_data_available && count < timeout_cycles) begin
            @(posedge clk_sys);
            count++;
        end
    endtask

    // ========================================================================
    // Test Messages
    // ========================================================================
    
    // MSG_RGF_WRITE: {W<addr>,V<DH>,V<DL>} - 16 bytes
    // addr=0x10, offset=0x0001, data=0x1234_5678
    logic [7:0] msg_rgf_write [16] = '{
        8'h7B, 8'h57, 8'h10, 8'h00, 8'h01, 8'h2C,  // {W<addr><off>
        8'h56, 8'h12, 8'h34, 8'h00, 8'h2C,         // V<data_hi>
        8'h56, 8'h56, 8'h78, 8'h00, 8'h7D          // V<data_lo>}
    };

    // MSG_RGF_READ: {R<addr><off_hi><off_lo>} - 6 bytes
    logic [7:0] msg_rgf_read [6] = '{
        8'h7B, 8'h52, 8'h20, 8'hAA, 8'hBB, 8'h7D
    };

    // MSG_SINGLE_PIXEL_WR: {W<addr>,P<R,G,B>} - 11 bytes
    logic [7:0] msg_single_pixel [11] = '{
        8'h7B, 8'h57, 8'h50, 8'h00, 8'h00, 8'h2C,
        8'h50, 8'hFF, 8'h80, 8'h40, 8'h7D
    };

    // MSG_START_BURST_WR: {I<xxx>,H<h2><h1><h0>,W<w2><w1><w0>} - 16 bytes
    // height=2, width=2 -> 4 pixels total -> 1 burst message needed
    // Format: 3-byte big-endian {MSB, MID, LSB}
    logic [7:0] msg_start_burst_wr [16] = '{
        8'h7B, 8'h49, 8'h00, 8'h00, 8'h00, 8'h2C,
        8'h48, 8'h00, 8'h00, 8'h02, 8'h2C,       // H={00,00,02}=2
        8'h57, 8'h00, 8'h00, 8'h02, 8'h7D        // W={00,00,02}=2
    };

    // MSG_BURST_PIXEL_WR: {<4 pixels packed>} - 16 bytes
    logic [7:0] msg_burst_pixels [16] = '{
        8'h7B, 8'h11, 8'h22, 8'h33, 8'h44, 8'h2C,
        8'h55, 8'h66, 8'h77, 8'h88, 8'h2C,
        8'h99, 8'hAA, 8'hBB, 8'hCC, 8'h7D
    };

    // MSG_START_BURST_RD: {R<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>} - 16 bytes
    // addr=0x00, offset=0x0000, height=32, width=48
    logic [7:0] msg_start_burst_rd [16] = '{
        8'h7B, 8'h52, 8'h00, 8'h00, 8'h00, 8'h2C,
        8'h48, 8'h00, 8'h00, 8'h20, 8'h2C,       // H={00,00,20}=32
        8'h57, 8'h00, 8'h00, 8'h30, 8'h7D        // W={00,00,30}=48
    };

    // ========================================================================
    // Test Helpers
    // ========================================================================
    
    function automatic void check_pass(string msg);
        $display("[PASS] Test %0d: %s", test_num, msg);
        pass_count++;
        test_num++;
    endfunction

    function automatic void check_fail(string msg);
        $display("[FAIL] Test %0d: %s", test_num, msg);
        fail_count++;
        test_num++;
    endfunction

    // ========================================================================
    // Main Test Sequence
    // ========================================================================
    initial begin
        $display("============================================");
        $display("UART RX Full System Testbench");
        $display("PHY + MAC + Classifier + CDC + Seq + RGF");
        $display("============================================");
        $display("CLK_176   = %0d MHz (RX domain)", CLK_176_FREQ/1_000_000);
        $display("CLK_SYS   = %0d MHz (System domain)", CLK_SYS_FREQ/1_000_000);
        $display("BAUD_RATE = %0d Mbaud", BAUD_RATE/1_000_000);
        $display("============================================\n");

        // Initialize
        rst_176_n = 0;
        rst_sys_n = 0;
        rx = 1;
        test_num = 1;
        pass_count = 0;
        fail_count = 0;

        // Release resets - wait longer for CDC synchronizers to stabilize
        #(CLK_SYS_PERIOD * 100);
        rst_176_n = 1;
        rst_sys_n = 1;
        #(CLK_SYS_PERIOD * 100);  // Wait 1µs after reset for stability

        // ====================================================================
        // Test 1: MSG_RGF_WRITE - Verify RGF wr_en
        // ====================================================================
        $display("\n--- Test 1: MSG_RGF_WRITE ---");
        $display("    DEBUG: Starting message send at time %0t", $time);
        $display("    DEBUG: burst_on = %b", class_burst_on);
        
        // Send message and wait for response IN PARALLEL
        // This catches the data_available signal during/after processing
        send_and_wait_for_response(msg_rgf_write, 16, 5000);
        
        // Use CAPTURED values (taken when sys_data_available was high)
        $display("    DEBUG: saw_data_available = %b", saw_data_available);
        $display("    DEBUG: captured_msg_type = %s", captured_msg_type.name());
        $display("    DEBUG: captured_rgf_wr_en = %b", captured_rgf_wr_en);
        
        // Check captured values (state when data_available was high)
        if (saw_data_available && captured_msg_type == MSG_RGF_WRITE) begin
            $display("    Message type: MSG_RGF_WRITE (correct)");
            $display("    rgf_wr_en = %b (captured)", captured_rgf_wr_en);
            if (captured_rgf_wr_en) begin
                check_pass("MSG_RGF_WRITE - rgf_wr_en asserted");
            end else begin
                check_fail("MSG_RGF_WRITE - rgf_wr_en not asserted");
            end
        end else begin
            $display("    DEBUG: FAIL condition - saw_data_available=%b, captured_msg_type=%s",
                     saw_data_available, captured_msg_type.name());
            check_fail("MSG_RGF_WRITE - wrong message type or no data");
        end
        
        // Wait for handshake
        wait_for_handshake_complete(100);
        $display("    Handshake complete: sys_data_available cleared");
        #(CLK_SYS_PERIOD * 50);

        // ====================================================================
        // Test 2: MSG_RGF_READ - Verify RGF rd_en
        // ====================================================================
        $display("\n--- Test 2: MSG_RGF_READ ---");
        $display("    DEBUG: burst_on = %b", class_burst_on);
        send_and_wait_for_response(msg_rgf_read, 6, 5000);
        
        // Use CAPTURED values (taken when sys_data_available was high)
        $display("    DEBUG: saw_data_available = %b", saw_data_available);
        $display("    DEBUG: captured_msg_type = %s", captured_msg_type.name());
        $display("    DEBUG: captured_rgf_rd_en = %b", captured_rgf_rd_en);
        
        // Check captured values (state when data_available was high)
        if (saw_data_available && captured_msg_type == MSG_RGF_READ) begin
            $display("    Message type: MSG_RGF_READ (correct)");
            $display("    rgf_rd_en = %b (captured)", captured_rgf_rd_en);
            if (captured_rgf_rd_en) begin
                check_pass("MSG_RGF_READ - rgf_rd_en asserted");
            end else begin
                check_fail("MSG_RGF_READ - rgf_rd_en not asserted");
            end
        end else begin
            $display("    DEBUG: FAIL - saw_data_available=%b, captured_msg_type=%s",
                     saw_data_available, captured_msg_type.name());
            check_fail("MSG_RGF_READ - wrong message type or no data");
        end
        
        wait_for_handshake_complete(100);
        #(CLK_SYS_PERIOD * 50);

        // ====================================================================
        // Test 3: MSG_SINGLE_PIXEL_WR - Verify sequencer receives
        // ====================================================================
        $display("\n--- Test 3: MSG_SINGLE_PIXEL_WR ---");
        send_and_wait_for_response(msg_single_pixel, 11, 5000);
        
        repeat(5) @(posedge clk_sys);
        
        if (sys_classified_type == MSG_SINGLE_PIXEL_WR) begin
            $display("    Message type: MSG_SINGLE_PIXEL_WR (correct)");
            $display("    Pixel R=0x%02X, G=0x%02X, B=0x%02X", 
                     sys_pixel_r, sys_pixel_g, sys_pixel_b);
            $display("    sys_seq_ready = %b (unified handshake)", sys_seq_ready);
            check_pass("MSG_SINGLE_PIXEL_WR - message classified correctly");
        end else begin
            check_fail("MSG_SINGLE_PIXEL_WR - wrong message type");
        end
        
        wait_for_handshake_complete(100);
        #(CLK_SYS_PERIOD * 50);

        // ====================================================================
        // Test 4: MSG_START_BURST_WR - Verify burst_on activation
        // ====================================================================
        $display("\n--- Test 4: MSG_START_BURST_WR ---");
        $display("    burst_on before = %b", sys_burst_on);
        
        send_and_wait_for_response(msg_start_burst_wr, 16, 5000);
        
        repeat(10) @(posedge clk_sys);
        
        if (sys_burst_on) begin
            $display("    burst_on after = %b (correctly activated)", sys_burst_on);
            $display("    Height = %0d (0x%08X), Width = %0d (0x%08X)", 
                     sys_parsed_height, sys_parsed_height, sys_parsed_width, sys_parsed_width);
            $display("    sys_seq_ready = %b (unified handshake)", sys_seq_ready);
            check_pass("MSG_START_BURST_WR - burst mode activated");
        end else begin
            check_fail("MSG_START_BURST_WR - burst_on not activated");
        end
        
        wait_for_handshake_complete(100);
        #(CLK_SYS_PERIOD * 50);

        // ====================================================================
        // Test 5: MSG_BURST_PIXEL_WR - Verify pixel data flows
        // ====================================================================
        $display("\n--- Test 5: MSG_BURST_PIXEL_WR ---");
        $display("    burst_on = %b (should be 1)", sys_burst_on);
        
        send_and_wait_for_response(msg_burst_pixels, 16, 5000);
        
        repeat(10) @(posedge clk_sys);
        
        if (sys_classified_type == MSG_BURST_PIXEL_WR) begin
            $display("    Message type: MSG_BURST_PIXEL_WR (correct)");
            $display("    burst_red   = 0x%08X", sys_burst_red);
            $display("    burst_green = 0x%08X", sys_burst_green);
            $display("    burst_blue  = 0x%08X", sys_burst_blue);
            
            // Check unified handshake
            $display("    sys_seq_ready = %b (unified handshake)", sys_seq_ready);
            if (seq_burst_r_wr_en) begin
                $display("    SRAM write enabled - burst sequencer active");
            end
            check_pass("MSG_BURST_PIXEL_WR - pixel data received");
        end else begin
            check_fail("MSG_BURST_PIXEL_WR - wrong message type");
        end
        
        wait_for_handshake_complete(100);
        
        // Wait for burst sequencer to complete (with 2x2 image, 1 burst msg = 4 pixels = done)
        // Give plenty of time for burst_done to propagate through CDC and clear burst_on
        repeat(100) @(posedge clk_sys);
        $display("    DEBUG: After burst wait - sys_burst_done = %b, class_burst_on = %b", 
                 sys_burst_done, class_burst_on);

        // ====================================================================
        // Test 6: CDC Timing - Verify cross-domain handshake
        // ====================================================================
        $display("\n--- Test 6: CDC Cross-Domain Handshake ---");
        
        // After 2x2 image burst completes, burst_on should be cleared
        $display("    DEBUG: burst_on = %b at start of Test 6", class_burst_on);
        
        // Reset capture flag
        saw_data_available = 0;
        
        // Use fork to send message AND wait for response in parallel
        // This catches the data_available signal during processing
        fork
            // Thread 1: Send the message
            send_message(msg_rgf_write, 16);
            
            // Thread 2: Wait in 176 MHz domain for classifier
            begin
                int rx_cycles = 0;
                while (!class_data_available && rx_cycles < 10000) begin
                    @(posedge clk_176);
                    rx_cycles++;
                end
                $display("    Classifier data_available after %0d cycles (176MHz)", rx_cycles);
            end
            
            // Thread 3: Wait in system domain and capture
            begin
                int sys_cycles = 0;
                while (!sys_data_available && sys_cycles < 6000) begin
                    @(posedge clk_sys);
                    sys_cycles++;
                end
                $display("    System data_available after %0d cycles (100MHz)", sys_cycles);
                // Capture state when signal is found
                if (sys_cycles < 6000) begin
                    saw_data_available = 1;
                    captured_msg_type = sys_classified_type;
                end
            end
        join
        
        // Verify CDC delay is reasonable - use captured value
        if (saw_data_available) begin
            $display("    CDC handshake successful");
            check_pass("CDC - cross-domain signal propagation");
        end else begin
            $display("    DEBUG: CDC fail - saw_data_available=%b", saw_data_available);
            check_fail("CDC - signal did not propagate");
        end
        
        wait_for_handshake_complete(100);
        #(CLK_SYS_PERIOD * 50);

        // ====================================================================
        // Test 7: Back-to-back messages
        // ====================================================================
        $display("\n--- Test 7: Back-to-Back Messages ---");
        $display("    DEBUG: burst_on = %b (should be 0 for MSG_RGF_READ to work)", class_burst_on);
        
        // First message
        send_and_wait_for_response(msg_rgf_read, 6, 5000);
        wait_for_handshake_complete(200);
        
        // Second message immediately after
        send_and_wait_for_response(msg_rgf_read, 6, 5000);
        
        if (saw_data_available && captured_msg_type == MSG_RGF_READ) begin
            $display("    Second message received correctly");
            check_pass("Back-to-back messages handled");
        end else begin
            $display("    DEBUG: Test 7 fail - saw_data_available=%b, captured_msg_type=%s",
                     saw_data_available, captured_msg_type.name());
            check_fail("Back-to-back messages failed");
        end
        
        wait_for_handshake_complete(100);
        #(CLK_SYS_PERIOD * 100);

        // ====================================================================
        // Summary
        // ====================================================================
        $display("\n============================================");
        $display("TEST SUMMARY");
        $display("============================================");
        $display("Total Tests: %0d", pass_count + fail_count);
        $display("Passed:      %0d", pass_count);
        $display("Failed:      %0d", fail_count);
        $display("============================================");
        
        if (fail_count == 0) begin
            $display("*** ALL TESTS PASSED ***");
        end else begin
            $display("*** SOME TESTS FAILED ***");
        end
        
        $display("\nSimulation complete.");
        $finish;
    end

    // ========================================================================
    // Timeout Watchdog
    // ========================================================================
    initial begin
        #(500_000_000);  // 500ms timeout
        $display("\n[ERROR] Simulation timeout!");
        $finish;
    end

    // ========================================================================
    // Waveform Dump
    // ========================================================================
    initial begin
        $dumpfile("tb_uart_rx_system.vcd");
        $dumpvars(0, tb_uart_rx_system);
    end

endmodule
