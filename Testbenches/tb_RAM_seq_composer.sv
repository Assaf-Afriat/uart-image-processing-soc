`timescale 1ns / 1ps

/**
 * Testbench: tb_RAM_seq_composer
 * Tests the full data pipeline:
 *   SRAM (Red/Green/Blue) → Sequencer_Burst_Tx → msg_compposer → [MAC emulation]
 *
 * - Two asynchronous clocks: wr_clk (RAM side), rd_clk (MAC/Composer side)
 * - Three actual sram_red/green/blue instances (DATA_W=32, sync read, 1-cycle latency)
 * - Sequencer pipelined push accounts for sync RAM latency
 * - MAC backpressure emulation via tx_mac_busy toggle
 * - Console monitoring of composed messages
 */

module tb_RAM_seq_composer;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int RAM_ADDR_WIDTH = 14;           // $clog2(16384)
    localparam int RAM_DATA_WIDTH = 32;
    localparam int FIFO_WIDTH     = 32;
    localparam int FIFO_DEPTH     = 16;
    localparam int PIXEL_COUNT    = 65536;        // 256 x 256
    localparam int RAM_DEPTH      = PIXEL_COUNT / 4;  // 16384

    localparam int IMG_WIDTH      = 256;
    localparam int IMG_HEIGHT     = 256;

    // Clock periods (asynchronous clocks)
    localparam realtime WR_CLK_PERIOD = 10ns;     // 100 MHz
    localparam realtime RD_CLK_PERIOD = 13.33ns;  //  75 MHz

    // MAC busy duration (in rd_clk cycles) after each burst
    localparam int MAC_BUSY_CYCLES = 4;

    // =========================================================================
    // Signal Declarations
    // =========================================================================

    // Clocks and Reset
    logic wr_clk, rd_clk, rst_n;

    // --- SRAM Write Port (used by TB for initialization) ---
    logic                      sram_wr_en;
    logic [RAM_ADDR_WIDTH-1:0] sram_wr_addr;
    logic [RAM_DATA_WIDTH-1:0] sram_wr_data_red;
    logic [RAM_DATA_WIDTH-1:0] sram_wr_data_green;
    logic [RAM_DATA_WIDTH-1:0] sram_wr_data_blue;

    // --- SRAM Read Port ↔ Sequencer ---
    logic [RAM_DATA_WIDTH-1:0] red_RAM_data,  green_RAM_data,  blue_RAM_data;
    logic [RAM_ADDR_WIDTH-1:0] red_RAM_addr,  green_RAM_addr,  blue_RAM_addr;
    logic                      red_read_en,   green_read_en,   blue_read_en;

    // --- Sequencer ↔ MAC/Composer ---
    logic                      tx_busy;
    logic [95:0]               pixel_packet;
    logic                      pixel_packet_ready;

    // --- RGF I/O ---
    logic                      img_ctrl_start;
    logic                      activate_burst;
    logic                      img_complete;
    logic                      frame_tx_active;
    logic [$clog2(PIXEL_COUNT):0] pixel_counter;

    // --- Composer I/O ---
    logic [31:0]               red_burst_data, green_burst_data, blue_burst_data;
    logic [127:0]              data_out_msg_cmps;
    logic                      to_mac_tx_start_req;
    logic                      cmpsr_busy;

    // --- MAC Emulation ---
    logic                      tx_mac_busy;
    int                        mac_busy_cnt;

    // --- Monitoring ---
    int                        msg_count;

    // =========================================================================
    // Clock Generation (Asynchronous)
    // =========================================================================
    initial wr_clk = 1'b0;
    always #(WR_CLK_PERIOD / 2) wr_clk = ~wr_clk;

    initial rd_clk = 1'b0;
    always #(RD_CLK_PERIOD / 2) rd_clk = ~rd_clk;

    // =========================================================================
    // SRAM Instantiation (DATA_W=32, ADDR_W=14, sync read 1-cycle latency)
    // =========================================================================
    sram_red #(
        .ADDR_W (RAM_ADDR_WIDTH),
        .DATA_W (RAM_DATA_WIDTH)
    ) u_sram_red (
        .clk     (wr_clk),
        // Write port (TB initialization)
        .wr_en   (sram_wr_en),
        .wr_addr (sram_wr_addr),
        .wr_data (sram_wr_data_red),
        // Read port (Sequencer)
        .rd_en   (red_read_en),
        .rd_addr (red_RAM_addr),
        .rd_data (red_RAM_data)
    );

    sram_green #(
        .ADDR_W (RAM_ADDR_WIDTH),
        .DATA_W (RAM_DATA_WIDTH)
    ) u_sram_green (
        .clk     (wr_clk),
        // Write port (TB initialization)
        .wr_en   (sram_wr_en),
        .wr_addr (sram_wr_addr),
        .wr_data (sram_wr_data_green),
        // Read port (Sequencer)
        .rd_en   (green_read_en),
        .rd_addr (green_RAM_addr),
        .rd_data (green_RAM_data)
    );

    sram_blue #(
        .ADDR_W (RAM_ADDR_WIDTH),
        .DATA_W (RAM_DATA_WIDTH)
    ) u_sram_blue (
        .clk     (wr_clk),
        // Write port (TB initialization)
        .wr_en   (sram_wr_en),
        .wr_addr (sram_wr_addr),
        .wr_data (sram_wr_data_blue),
        // Read port (Sequencer)
        .rd_en   (blue_read_en),
        .rd_addr (blue_RAM_addr),
        .rd_data (blue_RAM_data)
    );

    // =========================================================================
    // Sequencer_Burst_Tx Instantiation
    // =========================================================================
    Sequencer_Burst_Tx #(
        .RAM_ADDR_WIDTH (RAM_ADDR_WIDTH),
        .RAM_DATA_WIDTH (RAM_DATA_WIDTH),
        .FIFO_WIDTH     (FIFO_WIDTH),
        .FIFO_DEPTH     (FIFO_DEPTH),
        .PIXEL_COUNT    (PIXEL_COUNT)
    ) u_sequencer (
        .wr_clk             (wr_clk),
        .rd_clk             (rd_clk),
        .rst_n              (rst_n),
        // RAM interface
        .red_RAM_data        (red_RAM_data),
        .green_RAM_data      (green_RAM_data),
        .blue_RAM_data       (blue_RAM_data),
        .red_RAM_addr        (red_RAM_addr),
        .green_RAM_addr      (green_RAM_addr),
        .blue_RAM_addr       (blue_RAM_addr),
        .red_read_en         (red_read_en),
        .green_read_en       (green_read_en),
        .blue_read_en        (blue_read_en),
        // MAC interface
        .tx_busy             (tx_busy),
        .pixel_packet        (pixel_packet),
        .pixel_packet_ready  (pixel_packet_ready),
        // RGF I/O
        .img_ctrl_start      (img_ctrl_start),
        .activate_burst      (activate_burst),
        .img_complete        (img_complete),
        .frame_tx_active     (frame_tx_active),
        .pixel_counter       (pixel_counter)
    );

    // =========================================================================
    // Pixel Packet → Per-Color Burst Data Decomposition
    // =========================================================================
    // pixel_packet = {R3,G3,B3, R2,G2,B2, R1,G1,B1, R0,G0,B0}
    // Reconstruct 32-bit per-color data as the composer expects.
    assign red_burst_data   = {pixel_packet[95:88], pixel_packet[71:64],
                               pixel_packet[47:40], pixel_packet[23:16]};
    assign green_burst_data = {pixel_packet[87:80], pixel_packet[63:56],
                               pixel_packet[39:32], pixel_packet[15:8]};
    assign blue_burst_data  = {pixel_packet[79:72], pixel_packet[55:48],
                               pixel_packet[31:24], pixel_packet[7:0]};

    // =========================================================================
    // msg_compposer Instantiation
    // =========================================================================
    msg_compposer u_composer (
        .clk                    (rd_clk),
        .rst_n                  (rst_n),
        // RGF interface (unused in burst mode)
        .now_rgf_read           (1'b0),
        .rgf_raw_address        (32'h0),
        .rgf_data               (32'h0),
        // Single image interface (unused)
        .now_image_read_single  (1'b0),
        .img_row_counter        (14'h0),
        .img_col_counter        (14'h0),
        .pixel_cell             (24'h0),
        // Burst image interface
        .now_image_read_burst   (1'b1),           // Always burst mode
        .red_burst_data         (red_burst_data),
        .green_burst_data       (green_burst_data),
        .blue_burst_data        (blue_burst_data),
        .to_cmpsr_start_req     (pixel_packet_ready),
        .tx_mac_busy            (tx_mac_busy),
        // Outputs
        .data_out_msg_cmps      (data_out_msg_cmps),
        .to_mac_tx_start_req    (to_mac_tx_start_req),
        .cmpsr_busy             (cmpsr_busy)
    );

    // =========================================================================
    // Sequencer tx_busy ← Composer cmpsr_busy
    // =========================================================================
    assign tx_busy = cmpsr_busy;

    // =========================================================================
    // MAC Busy Emulation (rd_clk domain)
    // Asserts tx_mac_busy for MAC_BUSY_CYCLES after each to_mac_tx_start_req.
    // =========================================================================
    always_ff @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_mac_busy  <= 1'b0;
            mac_busy_cnt <= 0;
        end else begin
            if (to_mac_tx_start_req && !tx_mac_busy) begin
                tx_mac_busy  <= 1'b1;
                mac_busy_cnt <= MAC_BUSY_CYCLES;
            end else if (mac_busy_cnt > 0) begin
                mac_busy_cnt <= mac_busy_cnt - 1;
                if (mac_busy_cnt == 1)
                    tx_mac_busy <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Task: Initialize SRAMs via Write Port (synchronous write)
    // =========================================================================
    // Gradient pattern:
    //   Red   = column index (pixel_index % 256)
    //   Green = row index    (pixel_index / 256)
    //   Blue  = constant 0x80
    //
    // Each 32-bit word: [7:0]=pixel0, [15:8]=pixel1, [23:16]=pixel2, [31:24]=pixel3
    task automatic initialize_srams();
        int pix_base, col, row;
        logic [7:0] r0, r1, r2, r3;
        logic [7:0] g0, g1, g2, g3;

        $display("[TB] Initializing SRAMs with %0dx%0d gradient pattern via write port...",
                 IMG_WIDTH, IMG_HEIGHT);

        for (int addr = 0; addr < RAM_DEPTH; addr++) begin
            pix_base = addr * 4;

            // Pixel 0
            col = pix_base % IMG_WIDTH;
            row = pix_base / IMG_WIDTH;
            r0  = col[7:0];  g0 = row[7:0];

            // Pixel 1
            col = (pix_base + 1) % IMG_WIDTH;
            row = (pix_base + 1) / IMG_WIDTH;
            r1  = col[7:0];  g1 = row[7:0];

            // Pixel 2
            col = (pix_base + 2) % IMG_WIDTH;
            row = (pix_base + 2) / IMG_WIDTH;
            r2  = col[7:0];  g2 = row[7:0];

            // Pixel 3
            col = (pix_base + 3) % IMG_WIDTH;
            row = (pix_base + 3) / IMG_WIDTH;
            r3  = col[7:0];  g3 = row[7:0];

            // Drive SRAM write port
            @(posedge wr_clk);
            sram_wr_en        <= 1'b1;
            sram_wr_addr      <= addr[RAM_ADDR_WIDTH-1:0];
            sram_wr_data_red  <= {r3, r2, r1, r0};
            sram_wr_data_green<= {g3, g2, g1, g0};
            sram_wr_data_blue <= {8'h80, 8'h80, 8'h80, 8'h80};
        end

        // De-assert write enable
        @(posedge wr_clk);
        sram_wr_en <= 1'b0;

        $display("[TB] SRAM initialization complete. %0d addresses loaded.", RAM_DEPTH);
    endtask

    // =========================================================================
    // Task: Apply Reset
    // =========================================================================
    task automatic apply_reset();
        $display("[TB] Applying reset...");
        rst_n          <= 1'b0;
        img_ctrl_start <= 1'b0;
        activate_burst <= 1'b0;
        img_complete   <= 1'b0;
        sram_wr_en     <= 1'b0;
        repeat (10) @(posedge wr_clk);
        rst_n <= 1'b1;
        repeat (5) @(posedge wr_clk);
        $display("[TB] Reset released at time %0t", $time);
    endtask

    // =========================================================================
    // Monitor: Display Composed Messages
    // =========================================================================
    always_ff @(posedge rd_clk) begin
        if (rst_n && to_mac_tx_start_req && !tx_mac_busy) begin
            msg_count <= msg_count + 1;

            // Print first 8, every 4096th, and last 4 messages
            if (msg_count < 8 || (msg_count % 4096 == 0) || msg_count >= (RAM_DEPTH - 4)) begin
                $display("[TB] MSG #%0d | time=%0t | pixel_cnt=%0d | msg=0x%032h",
                         msg_count, $time, pixel_counter, data_out_msg_cmps);

                // Decode burst message fields
                $display("       '{' R0=%02h B0=%02h G0=%02h R1=%02h ',' B1=%02h G1=%02h R2=%02h B2=%02h ',' G2=%02h R3=%02h B3=%02h G3=%02h '}'",
                         data_out_msg_cmps[119:112],  // R0
                         data_out_msg_cmps[111:104],  // B0
                         data_out_msg_cmps[103:96],   // G0
                         data_out_msg_cmps[95:88],    // R1
                         data_out_msg_cmps[79:72],    // B1
                         data_out_msg_cmps[71:64],    // G1
                         data_out_msg_cmps[63:56],    // R2
                         data_out_msg_cmps[55:48],    // B2
                         data_out_msg_cmps[39:32],    // G2
                         data_out_msg_cmps[31:24],    // R3
                         data_out_msg_cmps[23:16],    // B3
                         data_out_msg_cmps[15:8]);    // G3
            end
        end
    end

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        $display("==========================================================");
        $display("  tb_RAM_seq_composer - Burst Sequencer Pipeline Test");
        $display("  PIXEL_COUNT=%0d  RAM_DEPTH=%0d  FIFO_DEPTH=%0d",
                 PIXEL_COUNT, RAM_DEPTH, FIFO_DEPTH);
        $display("  wr_clk=%0.1fMHz  rd_clk=%0.1fMHz",
                 1000.0 / WR_CLK_PERIOD, 1000.0 / RD_CLK_PERIOD);
        $display("  MAC_BUSY_CYCLES=%0d", MAC_BUSY_CYCLES);
        $display("==========================================================");

        msg_count = 0;

        // ---- Phase 1: Reset ----
        apply_reset();

        // ---- Phase 2: Initialize SRAMs via write port ----
        initialize_srams();

        // Wait a few cycles for writes to settle
        repeat (5) @(posedge wr_clk);

        // ---- Phase 3: Signal image is in RAM, start burst ----
        $display("[TB] Starting burst transfer at time %0t...", $time);
        @(posedge wr_clk);
        img_complete   <= 1'b1;   // Image fully written to RAM
        activate_burst <= 1'b1;   // Enable burst mode
        img_ctrl_start <= 1'b1;   // Start image TX

        // ---- Phase 4: Wait for frame_tx_active to assert (first pop) ----
        $display("[TB] Waiting for frame_tx_active to assert...");
        wait (frame_tx_active === 1'b1);
        $display("[TB] frame_tx_active asserted at time %0t", $time);

        // ---- Phase 5: Wait for frame_tx_active to de-assert (full frame done) ----
        $display("[TB] Waiting for full frame transfer to complete...");
        wait (frame_tx_active === 1'b0);
        $display("[TB] frame_tx_active de-asserted at time %0t", $time);
        $display("[TB] Total messages sent: %0d (expected: %0d)", msg_count, RAM_DEPTH);

        // ---- Phase 6: De-assert controls ----
        @(posedge wr_clk);
        activate_burst <= 1'b0;
        img_ctrl_start <= 1'b0;

        // Let pipeline drain
        repeat (50) @(posedge rd_clk);

        // ---- Validation Summary ----
        $display("==========================================================");
        if (msg_count == RAM_DEPTH)
            $display("  PASS: All %0d burst messages transmitted.", msg_count);
        else
            $display("  FAIL: Expected %0d messages, got %0d.", RAM_DEPTH, msg_count);
        $display("  Final pixel_counter = %0d (expected: %0d)", pixel_counter, PIXEL_COUNT);
        $display("==========================================================");

        $finish;
    end

    // =========================================================================
    // Watchdog Timer
    // =========================================================================
    initial begin
        #100ms;
        $display("[TB] ERROR: Watchdog timeout at %0t. Simulation hung!", $time);
        $finish;
    end

endmodule
