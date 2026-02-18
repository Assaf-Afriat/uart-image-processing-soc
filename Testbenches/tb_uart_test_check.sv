`timescale 1ns / 1ps

module tb_uart_burst_check;

    // ========================================================================
    // 1. Parameter Definitions
    // ========================================================================
    // Clock setup: Using 176MHz as the main system clock for simulation
    // This matches the frequency your PHY expects.
    localparam real CLK_FREQ_HZ = 176_000_000.0;
    localparam real CLK_PERIOD_NS = 1000000000.0 / CLK_FREQ_HZ;

    // UART Setup (Must match PHY settings: 5.5 MBaud)
    localparam int  BAUD_RATE   = 5_500_000;
    localparam real BIT_PERIOD_NS = 1000000000.0 / real'(BAUD_RATE);

    // ========================================================================
    // 2. DUT Signals
    // ========================================================================
    logic        clk;
    logic        rst_n;
    logic [15:0] sw;
    logic        btn_c;
    logic        rx;
    logic        tx;
    logic [7:0]  seg7_an;
    logic [6:0]  seg7_seg;
    logic [15:0] led;
    
    // RGB LED Outputs (Dummy connections for Top)
    logic R_16_out, G_16_out, B_16_out;
    logic R_17_out, G_17_out, B_17_out;

    // ========================================================================
    // 3. DUT Instantiation
    // ========================================================================
    uart_top uut (
        .clk      (clk),
        .i_rst_n  (rst_n),
        .sw       (sw),
        .btn_c    (btn_c),
        .rx       (rx),
        .tx       (tx),
        .seg7_an  (seg7_an),
        .seg7_seg (seg7_seg),
        .led      (led),
        .R_16_out (R_16_out),
        .G_16_out (G_16_out),
        .B_16_out (B_16_out),
        .R_17_out (R_17_out),
        .G_17_out (G_17_out),
        .B_17_out (B_17_out)
    );

    // ========================================================================
    // 4. Clock Generation
    // ========================================================================
    initial begin
        clk = 0;
        forever #(CLK_PERIOD_NS / 2.0) clk = ~clk;
    end

    // ========================================================================
    // 5. UART Driver Task
    // ========================================================================
    // Simulates sending one byte from the PC to the FPGA
    task send_byte(input logic [7:0] data);
        integer i;
        begin
            // Start Bit (Low)
            rx = 1'b0;
            #(BIT_PERIOD_NS);

            // Data Bits (LSB First)
            for (i = 0; i < 8; i++) begin
                rx = data[i];
                #(BIT_PERIOD_NS);
            end

            // Stop Bit (High)
            rx = 1'b1;
            #(BIT_PERIOD_NS);
        end
    endtask

    // ========================================================================
    // 6. Main Test Sequence
    // ========================================================================
    initial begin
        // --- Initialization ---
        rx = 1'b1;      // UART Idle state is High
        sw = 16'd0;     // Default switches
        btn_c = 1'b0;   // Default button
        
        // --- Reset Sequence ---
        $display("[TB] Starting Reset...");
        rst_n = 1'b0;
        #(100);
        rst_n = 1'b1;
        #(1000);
        $display("[TB] Reset Complete.");

        // --- Step 1: Send Start Burst Command ---
        // Format: {I<dummy>,H<Height>,W<Width>}
        // Sending Height=1, Width=4 (one burst packet)
        $display("[TB] Sending Start Burst Command...");
        
        send_byte(8'h7B); // '{'
        send_byte(8'h49); // 'I' (Opcode for Image Burst)
        send_byte(8'h00); // Dummy 1
        send_byte(8'h00); // Dummy 2
        send_byte(8'h00); // Dummy 3
        send_byte(8'h2C); // ','
        send_byte(8'h48); // 'H'
        send_byte(8'h00); // H_High
        send_byte(8'h00); // H_Mid
        send_byte(8'h01); // H_Low = 1
        send_byte(8'h2C); // ','
        send_byte(8'h57); // 'W'
        send_byte(8'h00); // W_High
        send_byte(8'h00); // W_Mid
        send_byte(8'h04); // W_Low = 4
        send_byte(8'h7D); // '}'

        // Wait for Classifier and MAC to process state change
        // Burst mode activation takes a few cycles
        #(2000); 

        // --- Step 2: Send Burst Data Packet ---
        // Format: { R0,B0,G0,R1 , B1,G1,R2,B2 , G2,R3,B3,G3 }
        // We will send values that are easy to spot in the waveform/memory:
        // Pixel 0: R=10, G=20, B=30
        // Pixel 1: R=11, G=21, B=31
        // Pixel 2: R=12, G=22, B=32
        // Pixel 3: R=13, G=23, B=33
        
        $display("[TB] Sending Pixel Burst Data...");
        
        send_byte(8'h7B); // '{'
        
        // Chunk 1
        send_byte(8'h10); // R0
        send_byte(8'h30); // B0
        send_byte(8'h20); // G0
        send_byte(8'h11); // R1
        
        send_byte(8'h2C); // ','
        
        // Chunk 2
        send_byte(8'h31); // B1
        send_byte(8'h21); // G1
        send_byte(8'h12); // R2
        send_byte(8'h32); // B2

        send_byte(8'h2C); // ','
        
        // Chunk 3
        send_byte(8'h22); // G2
        send_byte(8'h13); // R3
        send_byte(8'h33); // B3
        send_byte(8'h23); // G3
        
        send_byte(8'h7D); // '}'

        // --- Step 3: Wait for Processing ---
        $display("[TB] Waiting for Sequencer to write to SRAM...");
        // Give enough time for the UART bytes to arrive and be processed
        #(10000); 

        // --- Step 4: Verification (Backdoor Access) ---
        verify_sram_content();

        $display("[TB] Test Complete.");
        $stop;
    end

    // ========================================================================
    // 7. Verification Task
    // ========================================================================
    task verify_sram_content();
        logic [31:0] read_red, read_green, read_blue;
        logic [31:0] expected_red, expected_green, expected_blue;

        // Construct Expected 32-bit Words (Little Endian: {P3, P2, P1, P0})
        // Based on the data sent in Step 2:
        expected_red   = {8'h13, 8'h12, 8'h11, 8'h10};
        expected_green = {8'h23, 8'h22, 8'h21, 8'h20};
        expected_blue  = {8'h33, 8'h32, 8'h31, 8'h30};

        // Backdoor read from the SRAM memory arrays
        // Ensure hierarchy path matches your instance names in uart_top
        read_red   = uut.u_sram_red.mem_sram_red[0];
        read_green = uut.u_sram_green.mem_sram_green[0];
        read_blue  = uut.u_sram_blue.mem_sram_blue[0];

        // Check Red
        if (read_red === expected_red) 
            $display("[PASS] SRAM Red Address 0: Expected %h, Got %h", expected_red, read_red);
        else 
            $error("[FAIL] SRAM Red Address 0: Expected %h, Got %h", expected_red, read_red);

        // Check Green
        if (read_green === expected_green) 
            $display("[PASS] SRAM Green Address 0: Expected %h, Got %h", expected_green, read_green);
        else 
            $error("[FAIL] SRAM Green Address 0: Expected %h, Got %h", expected_green, read_green);

        // Check Blue
        if (read_blue === expected_blue) 
            $display("[PASS] SRAM Blue Address 0: Expected %h, Got %h", expected_blue, read_blue);
        else 
            $error("[FAIL] SRAM Blue Address 0: Expected %h, Got %h", expected_blue, read_blue);

    endtask

endmodule