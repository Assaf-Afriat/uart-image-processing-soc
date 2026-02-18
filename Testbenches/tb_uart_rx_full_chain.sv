// ============================================================================
// Testbench: UART RX Full Chain (PHY + MAC + Classifier)
// ============================================================================
// Tests the complete RX chain including:
//   - UART PHY (serial to parallel)
//   - UART MAC (message assembly and validation)
//   - UART Classifier (parsing, handshake, burst mode control)
//
// Verifies:
//   - Correct message type classification
//   - Proper data extraction (addr, offset, data, pixels)
//   - Handshake protocol (data_available / seq_ready)
//   - Burst mode activation and deactivation
// ============================================================================

`timescale 1ns / 1ps

module tb_uart_rx_full_chain;

    // ========================================================================
    // Parameters
    // ========================================================================
    localparam int CLK_FREQ   = 176_000_000;  // 176 MHz
    localparam int BAUD_RATE  = 5_500_000;    // 5.5 Mbaud
    localparam int CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;  // 32 clocks per bit
    localparam real CLK_PERIOD = 1_000_000_000.0 / CLK_FREQ;  // ~5.68 ns
    localparam real BIT_PERIOD = CLKS_PER_BIT * CLK_PERIOD;   // ~181.8 ns

    // ========================================================================
    // Import packages
    // ========================================================================
    import uart_types::*;
    import parser_pkg::*;

    // ========================================================================
    // Testbench Signals
    // ========================================================================
    logic        clk;
    logic        rst_n;
    logic        rx;

    // PHY outputs
    logic [7:0]  phy_rx_data;
    logic        phy_rx_valid;
    logic        phy_rx_error;
    logic        phy_faulty_frame_inc;
    uart_state_t phy_rx_state;

    // MAC outputs
    logic [127:0] mac_msg_data;
    logic         mac_msg_valid;
    msg_type_e    mac_msg_type;
    logic         mac_frame_error;

    // Classifier outputs
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

    // Handshake signals (from sequencer - simulated)
    logic         seq_ready;
    logic         burst_done;

    // Test counters
    int test_num;
    int pass_count;
    int fail_count;

    // ========================================================================
    // DUT Instantiation
    // ========================================================================
    
    // UART RX PHY
    uart_rx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_uart_rx_phy (
        .clk              (clk),
        .rst_n            (rst_n),
        .rx               (rx),
        .rx_data          (phy_rx_data),
        .rx_valid         (phy_rx_valid),
        .rx_error         (phy_rx_error),
        .faulty_frame_inc (phy_faulty_frame_inc),
        .rx_state         (phy_rx_state)
    );

    // UART RX MAC
    uart_rx_MAC u_uart_rx_mac (
        .clk           (clk),
        .rst_n         (rst_n),
        .rx_data       (phy_rx_data),
        .rx_valid      (phy_rx_valid),
        .msg_data      (mac_msg_data),
        .msg_valid     (mac_msg_valid),
        .msg_type      (mac_msg_type),
        .burst_mode_in (class_burst_on),  // Connected from classifier
        .frame_error   (mac_frame_error)
    );

    // UART Classifier
    uart_classifier u_uart_classifier (
        .clk                (clk),
        .rst_n              (rst_n),
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
        .seq_ready          (seq_ready),
        .data_available     (class_data_available),
        .burst_on           (class_burst_on),
        .burst_done         (burst_done)
    );

    // ========================================================================
    // Clock Generation
    // ========================================================================
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // ========================================================================
    // UART Transmit Tasks
    // ========================================================================
    
    // Task: Send a single UART byte (start bit + 8 data bits + stop bit)
    task automatic send_uart_byte(input logic [7:0] data);
        int i;
        
        // Start bit (low)
        rx = 1'b0;
        #(BIT_PERIOD);
        
        // Data bits (LSB first)
        for (i = 0; i < 8; i++) begin
            rx = data[i];
            #(BIT_PERIOD);
        end
        
        // Stop bit (high)
        rx = 1'b1;
        #(BIT_PERIOD);
    endtask

    // Task: Send an array of bytes
    task automatic send_message(input logic [7:0] msg[], input int len);
        int i;
        for (i = 0; i < len; i++) begin
            send_uart_byte(msg[i]);
        end
        // Small gap between messages
        #(BIT_PERIOD * 2);
    endtask

    // Task: Wait for classifier data_available
    task automatic wait_for_data_available(input int timeout_cycles);
        int count;
        count = 0;
        while (!class_data_available && count < timeout_cycles) begin
            @(posedge clk);
            count++;
        end
    endtask

    // Task: Simulate sequencer consuming data (handshake)
    task automatic sequencer_consume();
        // Assert seq_ready for one cycle
        seq_ready = 1'b1;
        @(posedge clk);
        @(posedge clk);
        seq_ready = 1'b0;
        @(posedge clk);
    endtask

    // ========================================================================
    // Test Messages
    // ========================================================================
    
    // MSG_RGF_WRITE: {W<addr>,V<DH>,V<DL>} - 16 bytes
    // addr=0x10, offset=0x0001, data_high=0x1234, data_low=0x5678
    logic [7:0] msg_rgf_write [16] = '{
        8'h7B,  // '{'
        8'h57,  // 'W'
        8'h10,  // addr = 0x10
        8'h00,  // offset_hi = 0x00
        8'h01,  // offset_lo = 0x01
        8'h2C,  // ','
        8'h56,  // 'V'
        8'h12,  // data_high byte 0
        8'h34,  // data_high byte 1
        8'h00,  // pad
        8'h2C,  // ','
        8'h56,  // 'V'
        8'h56,  // data_low byte 0
        8'h78,  // data_low byte 1
        8'h00,  // pad
        8'h7D   // '}'
    };

    // MSG_RGF_READ: {R<addr><off_hi><off_lo>} - 6 bytes
    // addr=0x20, offset=0xAABB
    logic [7:0] msg_rgf_read [6] = '{
        8'h7B,  // '{'
        8'h52,  // 'R'
        8'h20,  // addr = 0x20
        8'hAA,  // offset_hi = 0xAA
        8'hBB,  // offset_lo = 0xBB
        8'h7D   // '}'
    };

    // MSG_SINGLE_PIXEL_WR: {W<addr>,P<R,G,B>} - 11 bytes
    // addr=0x50, offset=0x0000, R=0xFF, G=0x80, B=0x40
    logic [7:0] msg_single_pixel [11] = '{
        8'h7B,  // '{'
        8'h57,  // 'W'
        8'h50,  // addr = 0x50
        8'h00,  // offset_hi
        8'h00,  // offset_lo
        8'h2C,  // ','
        8'h50,  // 'P'
        8'hFF,  // Red = 0xFF
        8'h80,  // Green = 0x80
        8'h40,  // Blue = 0x40
        8'h7D   // '}'
    };

    // MSG_START_BURST_WR: {I<xxx>,H<h2><h1><h0>,W<w2><w1><w0>} - 16 bytes
    // height=64 (0x000040), width=128 (0x000080)
    // Format: byte7=MSB, byte8=MID, byte9=LSB (big-endian 3-byte)
    logic [7:0] msg_start_burst_wr [16] = '{
        8'h7B,  // '{'       byte 0
        8'h49,  // 'I'       byte 1
        8'h00,  // reserved  byte 2
        8'h00,  // reserved  byte 3
        8'h00,  // reserved  byte 4
        8'h2C,  // ','       byte 5
        8'h48,  // 'H'       byte 6
        8'h00,  // h2 (MSB)  byte 7   height = {00,00,40} = 64
        8'h00,  // h1        byte 8
        8'h40,  // h0 (LSB)  byte 9
        8'h2C,  // ','       byte 10
        8'h57,  // 'W'       byte 11
        8'h00,  // w2 (MSB)  byte 12  width = {00,00,80} = 128
        8'h00,  // w1        byte 13
        8'h80,  // w0 (LSB)  byte 14
        8'h7D   // '}'       byte 15
    };

    // MSG_BURST_PIXEL_WR: {<4 pixels packed>} - 16 bytes
    // P0: R=0x11, G=0x22, B=0x33
    // P1: R=0x44, G=0x55, B=0x66
    // P2: R=0x77, G=0x88, B=0x99
    // P3: R=0xAA, G=0xBB, B=0xCC
    logic [7:0] msg_burst_pixels [16] = '{
        8'h7B,  // '{'
        8'h11,  // R0 = 0x11
        8'h22,  // G0 = 0x22
        8'h33,  // B0 = 0x33
        8'h44,  // R1 = 0x44
        8'h2C,  // ','  (byte 5)
        8'h55,  // G1 = 0x55
        8'h66,  // B1 = 0x66
        8'h77,  // R2 = 0x77
        8'h88,  // G2 = 0x88
        8'h2C,  // ','  (byte 10)
        8'h99,  // B2 = 0x99
        8'hAA,  // R3 = 0xAA
        8'hBB,  // G3 = 0xBB
        8'hCC,  // B3 = 0xCC
        8'h7D   // '}'
    };

    // MSG_START_BURST_RD: {R<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>} - 16 bytes
    // addr=0x00, offset=0x0000, height=32 (0x000020), width=48 (0x000030)
    logic [7:0] msg_start_burst_rd [16] = '{
        8'h7B,  // '{'       byte 0
        8'h52,  // 'R'       byte 1
        8'h00,  // addr      byte 2
        8'h00,  // off_hi    byte 3
        8'h00,  // off_lo    byte 4
        8'h2C,  // ','       byte 5
        8'h48,  // 'H'       byte 6
        8'h00,  // h2 (MSB)  byte 7   height = {00,00,20} = 32
        8'h00,  // h1        byte 8
        8'h20,  // h0 (LSB)  byte 9
        8'h2C,  // ','       byte 10
        8'h57,  // 'W'       byte 11
        8'h00,  // w2 (MSB)  byte 12  width = {00,00,30} = 48
        8'h00,  // w1        byte 13
        8'h30,  // w0 (LSB)  byte 14
        8'h7D   // '}'       byte 15
    };

    // ========================================================================
    // Test Checking Functions
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
        // Initialize
        $display("============================================");
        $display("UART RX Full Chain Testbench");
        $display("(PHY + MAC + Classifier)");
        $display("============================================");
        $display("CLK_FREQ  = %0d Hz", CLK_FREQ);
        $display("BAUD_RATE = %0d baud", BAUD_RATE);
        $display("============================================\n");

        // Reset
        rst_n = 0;
        rx = 1;  // Idle high
        seq_ready = 0;
        burst_done = 0;
        test_num = 1;
        pass_count = 0;
        fail_count = 0;

        #(CLK_PERIOD * 100);
        rst_n = 1;
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 1: MSG_RGF_WRITE with handshake
        // ====================================================================
        $display("\n--- Test 1: MSG_RGF_WRITE + Handshake ---");
        send_message(msg_rgf_write, 16);
        wait_for_data_available(2000);
        
        if (class_data_available && class_type == MSG_RGF_WRITE) begin
            // Verify parsed values
            if (class_addr == 8'h10 && class_offset_addr == 16'h0001) begin
                $display("    Addr=0x%02X, Offset=0x%04X (correct)", class_addr, class_offset_addr);
                check_pass("MSG_RGF_WRITE - correct parsing");
            end else begin
                $display("    Addr=0x%02X (exp 0x10), Offset=0x%04X (exp 0x0001)", class_addr, class_offset_addr);
                check_fail("MSG_RGF_WRITE - wrong addr/offset");
            end
        end else begin
            check_fail("MSG_RGF_WRITE - no data available");
        end
        
        // Test handshake: consume data
        $display("    Testing handshake...");
        sequencer_consume();
        #(CLK_PERIOD * 10);
        
        if (!class_data_available) begin
            $display("    Handshake OK: data_available cleared after seq_ready");
            pass_count++;
        end else begin
            $display("    Handshake FAIL: data_available not cleared");
            fail_count++;
        end
        test_num++;
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 2: MSG_RGF_READ
        // ====================================================================
        $display("\n--- Test 2: MSG_RGF_READ ---");
        send_message(msg_rgf_read, 6);
        wait_for_data_available(2000);
        
        if (class_data_available && class_type == MSG_RGF_READ) begin
            if (class_addr == 8'h20 && class_offset_addr == 16'hAABB) begin
                $display("    Addr=0x%02X, Offset=0x%04X (correct)", class_addr, class_offset_addr);
                check_pass("MSG_RGF_READ - correct parsing");
            end else begin
                check_fail("MSG_RGF_READ - wrong values");
            end
        end else begin
            check_fail("MSG_RGF_READ - no data available");
        end
        sequencer_consume();
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 3: MSG_SINGLE_PIXEL_WR
        // ====================================================================
        $display("\n--- Test 3: MSG_SINGLE_PIXEL_WR ---");
        send_message(msg_single_pixel, 11);
        wait_for_data_available(2000);
        
        if (class_data_available && class_type == MSG_SINGLE_PIXEL_WR) begin
            if (class_pixel_r == 8'hFF && class_pixel_g == 8'h80 && class_pixel_b == 8'h40) begin
                $display("    R=0x%02X, G=0x%02X, B=0x%02X (correct)", 
                         class_pixel_r, class_pixel_g, class_pixel_b);
                check_pass("MSG_SINGLE_PIXEL_WR - correct RGB");
            end else begin
                $display("    R=0x%02X, G=0x%02X, B=0x%02X (expected FF,80,40)", 
                         class_pixel_r, class_pixel_g, class_pixel_b);
                check_fail("MSG_SINGLE_PIXEL_WR - wrong RGB values");
            end
        end else begin
            check_fail("MSG_SINGLE_PIXEL_WR - no data available");
        end
        sequencer_consume();
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 4: MSG_START_BURST_WR (enables burst_on)
        // ====================================================================
        $display("\n--- Test 4: MSG_START_BURST_WR + burst_on ---");
        $display("    burst_on before = %b", class_burst_on);
        send_message(msg_start_burst_wr, 16);
        wait_for_data_available(2000);
        
        if (class_data_available && class_type == MSG_START_BURST_WR) begin
            // Check burst_on is now active
            if (class_burst_on) begin
                $display("    burst_on after = %b (correctly activated)", class_burst_on);
                check_pass("MSG_START_BURST_WR - burst_on activated");
            end else begin
                check_fail("MSG_START_BURST_WR - burst_on not activated");
            end
            
            // Verify height/width (3-byte big-endian: {byte7,byte8,byte9} padded to 32 bits)
            // height=64 -> parsed_height = 32'h00000040, width=128 -> parsed_width = 32'h00000080
            if (class_height == 32'h00000040 && class_width == 32'h00000080) begin
                $display("    Height=%0d, Width=%0d (correct)", class_height, class_width);
                pass_count++;
            end else begin
                $display("    Height=0x%08X (exp 0x00000040), Width=0x%08X (exp 0x00000080)", 
                         class_height, class_width);
                fail_count++;
            end
            test_num++;
        end else begin
            check_fail("MSG_START_BURST_WR - no data available");
        end
        sequencer_consume();
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 5: MSG_BURST_PIXEL_WR (while burst_on is active)
        // ====================================================================
        $display("\n--- Test 5: MSG_BURST_PIXEL_WR (burst mode active) ---");
        $display("    burst_on = %b (should be 1)", class_burst_on);
        
        send_message(msg_burst_pixels, 16);
        wait_for_data_available(2000);
        
        if (class_data_available && class_type == MSG_BURST_PIXEL_WR) begin
            // Verify burst pixel grouping
            // Classifier packs: burst_red = {P0_R, P1_R, P2_R, P3_R} = {R0, R1, R2, R3}
            //   R0=0x11, R1=0x44, R2=0x77, R3=0xAA -> burst_red = 0x114477AA
            //   G0=0x22, G1=0x55, G2=0x88, G3=0xBB -> burst_green = 0x225588BB
            //   B0=0x33, B1=0x66, B2=0x99, B3=0xCC -> burst_blue = 0x336699CC
            
            $display("    burst_red   = 0x%08X (exp 0x114477AA)", class_burst_red);
            $display("    burst_green = 0x%08X (exp 0x225588BB)", class_burst_green);
            $display("    burst_blue  = 0x%08X (exp 0x336699CC)", class_burst_blue);
            
            if (class_burst_red   == 32'h114477AA && 
                class_burst_green == 32'h225588BB && 
                class_burst_blue  == 32'h336699CC) begin
                check_pass("MSG_BURST_PIXEL_WR - correct pixel grouping");
            end else begin
                check_fail("MSG_BURST_PIXEL_WR - wrong pixel values");
            end
        end else begin
            check_fail("MSG_BURST_PIXEL_WR - no data available");
        end
        sequencer_consume();
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 6: burst_done clears burst_on
        // ====================================================================
        $display("\n--- Test 6: burst_done clears burst_on ---");
        $display("    burst_on before = %b (should be 1)", class_burst_on);
        
        // Assert burst_done
        burst_done = 1'b1;
        @(posedge clk);
        @(posedge clk);
        burst_done = 1'b0;
        @(posedge clk);
        @(posedge clk);
        
        if (!class_burst_on) begin
            $display("    burst_on after = %b (correctly deactivated)", class_burst_on);
            check_pass("burst_done - burst_on cleared");
        end else begin
            $display("    burst_on after = %b (should be 0)", class_burst_on);
            check_fail("burst_done - burst_on not cleared");
        end
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 7: MSG_START_BURST_RD
        // ====================================================================
        $display("\n--- Test 7: MSG_START_BURST_RD ---");
        send_message(msg_start_burst_rd, 16);
        wait_for_data_available(2000);
        
        if (class_data_available && class_type == MSG_START_BURST_RD) begin
            if (class_addr == 8'h00 && class_offset_addr == 16'h0000) begin
                $display("    Addr=0x%02X, Offset=0x%04X (correct)", class_addr, class_offset_addr);
                $display("    Height=0x%08X, Width=0x%08X", class_height, class_width);
                check_pass("MSG_START_BURST_RD - correct parsing");
            end else begin
                $display("    Addr=0x%02X (exp 0x00), Offset=0x%04X (exp 0x0000)", 
                         class_addr, class_offset_addr);
                check_fail("MSG_START_BURST_RD - wrong values");
            end
        end else begin
            check_fail("MSG_START_BURST_RD - no data available");
        end
        sequencer_consume();
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 8: Back-to-back messages without consuming
        // ====================================================================
        $display("\n--- Test 8: Data hold until consumed ---");
        send_message(msg_rgf_read, 6);
        wait_for_data_available(2000);
        
        // Don't consume - data should stay valid
        #(CLK_PERIOD * 50);
        
        if (class_data_available) begin
            $display("    data_available stays high when not consumed");
            check_pass("Data hold - remains available");
        end else begin
            check_fail("Data hold - cleared unexpectedly");
        end
        
        // Now consume
        sequencer_consume();
        #(CLK_PERIOD * 10);
        
        if (!class_data_available) begin
            $display("    data_available cleared after consume");
            pass_count++;
        end else begin
            $display("    data_available not cleared");
            fail_count++;
        end
        test_num++;
        #(CLK_PERIOD * 100);

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
        #(200_000_000);  // 200ms timeout
        $display("\n[ERROR] Simulation timeout!");
        $finish;
    end

    // ========================================================================
    // Optional: Waveform Dump
    // ========================================================================
    initial begin
        $dumpfile("tb_uart_rx_full_chain.vcd");
        $dumpvars(0, tb_uart_rx_full_chain);
    end

endmodule
