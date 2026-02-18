// ============================================================================
// Testbench: UART RX PHY + MAC
// ============================================================================
// Tests all message types through the PHY and MAC layers:
//   - MSG_RGF_WRITE:      {W<addr>,V<DH>,V<DL>}        16 bytes
//   - MSG_RGF_READ:       {R<addr><off_hi><off_lo>}    6 bytes
//   - MSG_SINGLE_PIXEL_WR:{W<addr>,P<R,G,B>}           11 bytes
//   - MSG_START_BURST_WR: {I<xxx>,H<xxx>,W<xxx>}       16 bytes
//   - MSG_BURST_PIXEL_WR: {<4 pixels packed>}          16 bytes (burst mode)
//   - MSG_START_BURST_RD: {R<addr>,H<xxx>,W<xxx>}      16 bytes
// ============================================================================

`timescale 1ns / 1ps

module tb_uart_rx_phy_mac;

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

    // Burst mode control (from classifier - simulated)
    logic         burst_mode;

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
        .burst_mode_in (burst_mode),
        .frame_error   (mac_frame_error)
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

    // Task: Wait for MAC message valid
    task automatic wait_for_mac_valid(input int timeout_cycles);
        int count;
        count = 0;
        while (!mac_msg_valid && count < timeout_cycles) begin
            @(posedge clk);
            count++;
        end
    endtask

    // ========================================================================
    // Test Result Checking
    // ========================================================================
    task automatic check_result(
        input string test_name,
        input msg_type_e expected_type,
        input logic expect_valid
    );
        if (expect_valid) begin
            if (mac_msg_valid && mac_msg_type == expected_type) begin
                $display("[PASS] Test %0d: %s - Got msg_type=%s", 
                         test_num, test_name, mac_msg_type.name());
                pass_count++;
            end else if (mac_msg_valid) begin
                $display("[FAIL] Test %0d: %s - Expected %s, Got %s", 
                         test_num, test_name, expected_type.name(), mac_msg_type.name());
                fail_count++;
            end else begin
                $display("[FAIL] Test %0d: %s - No valid message received", 
                         test_num, test_name);
                fail_count++;
            end
        end else begin
            // Expect invalid/dropped message
            if (!mac_msg_valid || mac_msg_type == MSG_INVALID) begin
                $display("[PASS] Test %0d: %s - Message correctly rejected", 
                         test_num, test_name);
                pass_count++;
            end else begin
                $display("[FAIL] Test %0d: %s - Expected rejection, got valid", 
                         test_num, test_name);
                fail_count++;
            end
        end
        test_num++;
    endtask

    // ========================================================================
    // Test Messages
    // ========================================================================
    
    // MSG_RGF_WRITE: {W<addr>,V<DH>,V<DL>} - 16 bytes
    // Example: {W, 0x10, 0x00, 0x01, ',', V, 0x00, 0x12, 0x34, ',', V, 0x00, 0x56, 0x78, '}' 
    logic [7:0] msg_rgf_write [16] = '{
        8'h7B,  // '{'
        8'h57,  // 'W'
        8'h10,  // addr = 0x10
        8'h00,  // offset_hi
        8'h01,  // offset_lo
        8'h2C,  // ','
        8'h56,  // 'V'
        8'h00,  // pad
        8'h12,  // data_high = 0x12
        8'h34,  // (more data)
        8'h2C,  // ','
        8'h56,  // 'V'
        8'h00,  // pad
        8'h56,  // data_low = 0x56
        8'h78,  // (more data)
        8'h7D   // '}'
    };

    // MSG_RGF_READ: {R<addr><off_hi><off_lo>} - 6 bytes
    logic [7:0] msg_rgf_read [6] = '{
        8'h7B,  // '{'
        8'h52,  // 'R'
        8'h20,  // addr = 0x20
        8'hAA,  // offset_hi
        8'hBB,  // offset_lo
        8'h7D   // '}'
    };

    // MSG_SINGLE_PIXEL_WR: {W<addr>,P<R,G,B>} - 11 bytes
    logic [7:0] msg_single_pixel [11] = '{
        8'h7B,  // '{'
        8'h57,  // 'W'
        8'h50,  // addr = 0x50 (pixel range)
        8'h00,  // offset_hi
        8'h00,  // offset_lo
        8'h2C,  // ','
        8'h50,  // 'P'
        8'hFF,  // Red = 0xFF
        8'h80,  // Green = 0x80
        8'h40,  // Blue = 0x40
        8'h7D   // '}'
    };

    // MSG_START_BURST_WR: {I<xxx>,H<xxx>,W<xxx>} - 16 bytes
    logic [7:0] msg_start_burst_wr [16] = '{
        8'h7B,  // '{'
        8'h49,  // 'I'
        8'h00,  // reserved
        8'h00,  // reserved
        8'h00,  // reserved
        8'h2C,  // ','
        8'h48,  // 'H'
        8'h00,  // height_hi
        8'h01,  // height_mid
        8'h00,  // height_lo = 256
        8'h2C,  // ','
        8'h57,  // 'W'
        8'h00,  // width_hi
        8'h01,  // width_mid
        8'h00,  // width_lo = 256
        8'h7D   // '}'
    };

    // MSG_BURST_PIXEL_WR: {<4 pixels packed>} - 16 bytes (burst mode)
    // Format: {<R0,G0,B0,R1>,<G1,B1,R2,G2>,<B2,R3,G3,B3>}
    logic [7:0] msg_burst_pixels [16] = '{
        8'h7B,  // '{'
        8'hFF,  // R0
        8'h00,  // G0
        8'h00,  // B0
        8'h00,  // R1
        8'h2C,  // ','  (byte 5)
        8'hFF,  // G1
        8'h00,  // B1
        8'h00,  // R2
        8'h00,  // G2
        8'h2C,  // ','  (byte 10)
        8'hFF,  // B2
        8'h00,  // R3
        8'h00,  // G3
        8'h00,  // B3
        8'h7D   // '}'
    };

    // MSG_START_BURST_RD: {R<addr>,H<xxx>,W<xxx>} - 16 bytes
    logic [7:0] msg_start_burst_rd [16] = '{
        8'h7B,  // '{'
        8'h52,  // 'R'
        8'h60,  // addr = 0x60
        8'h00,  // offset_hi
        8'h00,  // offset_lo
        8'h2C,  // ','
        8'h48,  // 'H'
        8'h00,  // height bytes
        8'h00,
        8'h80,  // height = 128
        8'h2C,  // ','
        8'h57,  // 'W'
        8'h00,  // width bytes
        8'h00,
        8'h80,  // width = 128
        8'h7D   // '}'
    };

    // Invalid message (bad opcode)
    logic [7:0] msg_invalid [6] = '{
        8'h7B,  // '{'
        8'h58,  // 'X' - invalid opcode
        8'h00,
        8'h00,
        8'h00,
        8'h7D   // '}'
    };

    // ========================================================================
    // Main Test Sequence
    // ========================================================================
    initial begin
        // Initialize
        $display("============================================");
        $display("UART RX PHY + MAC Testbench");
        $display("============================================");
        $display("CLK_FREQ  = %0d Hz", CLK_FREQ);
        $display("BAUD_RATE = %0d baud", BAUD_RATE);
        $display("CLKS_PER_BIT = %0d", CLKS_PER_BIT);
        $display("============================================\n");

        // Reset
        rst_n = 0;
        rx = 1;  // Idle high
        burst_mode = 0;
        test_num = 1;
        pass_count = 0;
        fail_count = 0;

        #(CLK_PERIOD * 100);
        rst_n = 1;
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 1: MSG_RGF_WRITE
        // ====================================================================
        $display("\n--- Test 1: MSG_RGF_WRITE ---");
        send_message(msg_rgf_write, 16);
        wait_for_mac_valid(1000);
        check_result("MSG_RGF_WRITE", MSG_RGF_WRITE, 1);
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 2: MSG_RGF_READ
        // ====================================================================
        $display("\n--- Test 2: MSG_RGF_READ ---");
        send_message(msg_rgf_read, 6);
        wait_for_mac_valid(1000);
        check_result("MSG_RGF_READ", MSG_RGF_READ, 1);
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 3: MSG_SINGLE_PIXEL_WR
        // ====================================================================
        $display("\n--- Test 3: MSG_SINGLE_PIXEL_WR ---");
        send_message(msg_single_pixel, 11);
        wait_for_mac_valid(1000);
        check_result("MSG_SINGLE_PIXEL_WR", MSG_SINGLE_PIXEL_WR, 1);
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 4: MSG_START_BURST_WR
        // ====================================================================
        $display("\n--- Test 4: MSG_START_BURST_WR ---");
        send_message(msg_start_burst_wr, 16);
        wait_for_mac_valid(1000);
        check_result("MSG_START_BURST_WR", MSG_START_BURST_WR, 1);
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 5: MSG_BURST_PIXEL_WR (with burst_mode enabled)
        // ====================================================================
        $display("\n--- Test 5: MSG_BURST_PIXEL_WR (burst mode) ---");
        burst_mode = 1;  // Enable burst mode
        send_message(msg_burst_pixels, 16);
        wait_for_mac_valid(1000);
        check_result("MSG_BURST_PIXEL_WR", MSG_BURST_PIXEL_WR, 1);
        burst_mode = 0;  // Disable burst mode
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 6: MSG_START_BURST_RD
        // ====================================================================
        $display("\n--- Test 6: MSG_START_BURST_RD ---");
        send_message(msg_start_burst_rd, 16);
        wait_for_mac_valid(1000);
        check_result("MSG_START_BURST_RD", MSG_START_BURST_RD, 1);
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 7: Invalid Message (should be rejected)
        // ====================================================================
        $display("\n--- Test 7: Invalid Message ---");
        send_message(msg_invalid, 6);
        wait_for_mac_valid(500);
        check_result("Invalid Opcode", MSG_INVALID, 1);
        #(CLK_PERIOD * 100);

        // ====================================================================
        // Test 8: Back-to-back messages
        // ====================================================================
        $display("\n--- Test 8: Back-to-back Messages ---");
        send_message(msg_rgf_read, 6);
        wait_for_mac_valid(1000);
        if (mac_msg_valid && mac_msg_type == MSG_RGF_READ) begin
            $display("[PASS] Test %0d: Back-to-back msg 1", test_num);
            pass_count++;
        end else begin
            $display("[FAIL] Test %0d: Back-to-back msg 1", test_num);
            fail_count++;
        end
        test_num++;
        
        #(CLK_PERIOD * 50);
        send_message(msg_rgf_write, 16);
        wait_for_mac_valid(1000);
        if (mac_msg_valid && mac_msg_type == MSG_RGF_WRITE) begin
            $display("[PASS] Test %0d: Back-to-back msg 2", test_num);
            pass_count++;
        end else begin
            $display("[FAIL] Test %0d: Back-to-back msg 2", test_num);
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
        #(100_000_000);  // 100ms timeout
        $display("\n[ERROR] Simulation timeout!");
        $finish;
    end

    // ========================================================================
    // Optional: Waveform Dump
    // ========================================================================
    initial begin
        $dumpfile("tb_uart_rx_phy_mac.vcd");
        $dumpvars(0, tb_uart_rx_phy_mac);
    end

endmodule
