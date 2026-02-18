`timescale 1ns / 1ps

/**
 * Testbench: tb_RAM_seq_composer_MAC
 * Full pipeline test:
 *   SRAM (32-bit, sync) → Sequencer_Burst_Tx → msg_compposer → uart_tx_mac_fsm → [UART Consumer]
 *
 * Data Flow:
 *   - SRAMs provide 32-bit words (4 pixels) to the Sequencer
 *   - Sequencer pushes through 3 Async FIFOs (CDC: wr_clk → rd_clk)
 *   - Composer formats 128-bit message from the burst data
 *   - MAC FSM serializes message into 16 bytes via tx_start/tx_data
 *   - TB UART Consumer captures bytes and prints full 16-byte messages
 *
 * Backpressure chain:
 *   uart_tx_mac_fsm.resp_active → msg_compposer.tx_mac_busy     (stalls composer)
 *   uart_tx_mac_fsm.resp_active → Sequencer_Burst_Tx.tx_busy    (stalls FIFO pops)
 */

module tb_RAM_seq_composer_MAC;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int RAM_ADDR_WIDTH = 14;
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

    // Simulated UART byte TX time (rd_clk cycles)
    localparam int UART_BYTE_CYCLES = 10;

    // =========================================================================
    // Signal Declarations
    // =========================================================================

    // Clocks and Reset
    logic wr_clk, rd_clk, rst_n;

    // --- SRAM Write Port (TB initialization) ---
    logic                      sram_wr_en;
    logic [RAM_ADDR_WIDTH-1:0] sram_wr_addr;
    logic [RAM_DATA_WIDTH-1:0] sram_wr_data_red;
    logic [RAM_DATA_WIDTH-1:0] sram_wr_data_green;
    logic [RAM_DATA_WIDTH-1:0] sram_wr_data_blue;

    // --- SRAM Read Port ↔ Sequencer ---
    logic [RAM_DATA_WIDTH-1:0] red_RAM_data,  green_RAM_data,  blue_RAM_data;
    logic [RAM_ADDR_WIDTH-1:0] red_RAM_addr,  green_RAM_addr,  blue_RAM_addr;
    logic                      red_read_en,   green_read_en,   blue_read_en;

    // --- Sequencer outputs ---
    logic [95:0]               pixel_packet;
    logic                      pixel_packet_ready;
    logic                      frame_tx_active;
    logic [$clog2(PIXEL_COUNT):0] pixel_counter;

    // --- Composer ↔ MAC FSM ---
    logic [127:0]              data_out_msg_cmps;
    logic                      to_mac_tx_start_req;
    logic                      cmpsr_busy;

    // --- MAC FSM outputs ---
    logic                      mac_tx_start;     // Byte TX pulse
    logic [7:0]                mac_tx_data;       // Byte to transmit
    logic                      mac_resp_active;   // Backpressure to composer + sequencer

    // --- UART PHY busy (emulated by TB) ---
    logic                      uart_phy_busy;

    // --- RGF controls ---
    logic                      img_ctrl_start;
    logic                      activate_burst;
    logic                      img_complete;

    // --- Per-color burst data (decomposed from pixel_packet) ---
    logic [31:0]               red_burst_data, green_burst_data, blue_burst_data;

    // --- Monitoring ---
    int                        msg_count;
    int                        byte_count;
    logic [7:0]                captured_msg [0:15];  // 16-byte capture buffer

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
        .wr_en   (sram_wr_en),
        .wr_addr (sram_wr_addr),
        .wr_data (sram_wr_data_red),
        .rd_en   (red_read_en),
        .rd_addr (red_RAM_addr),
        .rd_data (red_RAM_data)
    );

    sram_green #(
        .ADDR_W (RAM_ADDR_WIDTH),
        .DATA_W (RAM_DATA_WIDTH)
    ) u_sram_green (
        .clk     (wr_clk),
        .wr_en   (sram_wr_en),
        .wr_addr (sram_wr_addr),
        .wr_data (sram_wr_data_green),
        .rd_en   (green_read_en),
        .rd_addr (green_RAM_addr),
        .rd_data (green_RAM_data)
    );

    sram_blue #(
        .ADDR_W (RAM_ADDR_WIDTH),
        .DATA_W (RAM_DATA_WIDTH)
    ) u_sram_blue (
        .clk     (wr_clk),
        .wr_en   (sram_wr_en),
        .wr_addr (sram_wr_addr),
        .wr_data (sram_wr_data_blue),
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
        // MAC interface — backpressure directly from MAC FSM
        .tx_busy             (mac_resp_active),
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
        // RGF interface (unused)
        .now_rgf_read           (1'b0),
        .rgf_raw_address        (32'h0),
        .rgf_data               (32'h0),
        // Single image interface (unused)
        .now_image_read_single  (1'b0),
        .img_row_counter        (14'h0),
        .img_col_counter        (14'h0),
        .pixel_cell             (24'h0),
        // Burst image interface
        .now_image_read_burst   (1'b1),
        .red_burst_data         (red_burst_data),
        .green_burst_data       (green_burst_data),
        .blue_burst_data        (blue_burst_data),
        .to_cmpsr_start_req     (pixel_packet_ready),
        .tx_mac_busy            (mac_resp_active),     // Backpressure FROM MAC FSM
        // Outputs
        .data_out_msg_cmps      (data_out_msg_cmps),
        .to_mac_tx_start_req    (to_mac_tx_start_req),
        .cmpsr_busy             (cmpsr_busy)           // Backpressure TO Sequencer
    );

    // =========================================================================
    // uart_tx_mac_fsm Instantiation
    // =========================================================================
    uart_tx_mac_fsm u_mac_fsm (
        .clk            (rd_clk),
        .rst_n          (rst_n),
        // From Composer
        .start_tx_req   (to_mac_tx_start_req),   // Trigger from composer
        .read_data      (data_out_msg_cmps),      // 128-bit message
        // From UART PHY (emulated by TB)
        .tx_busy        (uart_phy_busy),
        // Outputs
        .tx_start       (mac_tx_start),           // Byte TX pulse
        .tx_data        (mac_tx_data),             // Byte to transmit
        .resp_active    (mac_resp_active)          // Backpressure → composer → sequencer
    );

    // =========================================================================
    // UART PHY Busy Emulation (rd_clk domain)
    // Simulates byte transmission time: busy for UART_BYTE_CYCLES after tx_start
    // =========================================================================
    int uart_busy_cnt;

    always_ff @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_phy_busy <= 1'b0;
            uart_busy_cnt <= 0;
        end else begin
            if (mac_tx_start && !uart_phy_busy) begin
                uart_phy_busy <= 1'b1;
                uart_busy_cnt <= UART_BYTE_CYCLES;
            end else if (uart_busy_cnt > 0) begin
                uart_busy_cnt <= uart_busy_cnt - 1;
                if (uart_busy_cnt == 1)
                    uart_phy_busy <= 1'b0;
            end
        end
    end

    // =========================================================================
    // UART Consumer: Capture bytes and print full 16-byte messages
    // =========================================================================
    always_ff @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_count <= 0;
            msg_count  <= 0;
        end else if (mac_tx_start && !uart_phy_busy) begin
            // Capture byte into buffer (MSB first: byte 15 down to 0)
            captured_msg[15 - byte_count] <= mac_tx_data;

            if (byte_count == 15) begin
                // Full 16-byte message captured
                byte_count <= 0;
                msg_count  <= msg_count + 1;
            end else begin
                byte_count <= byte_count + 1;
            end
        end
    end

    // Print captured messages (triggered 1 cycle after byte_count wraps)
    int prev_msg_count;

    always_ff @(posedge rd_clk) begin
        if (rst_n) begin
            prev_msg_count <= msg_count;

            if (msg_count != prev_msg_count) begin
                // Print first 8, every 4096th, and last 4 messages
                if (msg_count <= 8 || (msg_count % 4096 == 0) || msg_count >= (RAM_DEPTH - 3)) begin
                    $display("[UART] MSG #%0d | time=%0t | Bytes: %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h",
                             msg_count, $time,
                             captured_msg[15], captured_msg[14], captured_msg[13], captured_msg[12],
                             captured_msg[11], captured_msg[10], captured_msg[9],  captured_msg[8],
                             captured_msg[7],  captured_msg[6],  captured_msg[5],  captured_msg[4],
                             captured_msg[3],  captured_msg[2],  captured_msg[1],  captured_msg[0]);

                    // Also print as ASCII-like string where printable
                    $write("                  ASCII: ");
                    for (int i = 15; i >= 0; i--) begin
                        if (captured_msg[i] >= 8'h20 && captured_msg[i] <= 8'h7E)
                            $write("%c", captured_msg[i]);
                        else
                            $write(".");
                    end
                    $write("\n");
                end
            end
        end
    end

    // =========================================================================
    // Task: Initialize SRAMs via Write Port (synchronous write)
    // =========================================================================
    task automatic initialize_srams();
        int pix_base, col, row;
        logic [7:0] r0, r1, r2, r3;
        logic [7:0] g0, g1, g2, g3;

        $display("[TB] Initializing SRAMs with %0dx%0d gradient pattern...", IMG_WIDTH, IMG_HEIGHT);

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

            @(posedge wr_clk);
            sram_wr_en        <= 1'b1;
            sram_wr_addr      <= addr[RAM_ADDR_WIDTH-1:0];
            sram_wr_data_red  <= {r3, r2, r1, r0};
            sram_wr_data_green<= {g3, g2, g1, g0};
            sram_wr_data_blue <= {8'h80, 8'h80, 8'h80, 8'h80};
        end

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
    // Main Test Sequence
    // =========================================================================
    initial begin
        $display("==========================================================");
        $display("  tb_RAM_seq_composer_MAC - Full Pipeline Test");
        $display("  SRAM → Sequencer → Composer → MAC FSM → UART Consumer");
        $display("  PIXEL_COUNT=%0d  RAM_DEPTH=%0d  FIFO_DEPTH=%0d",
                 PIXEL_COUNT, RAM_DEPTH, FIFO_DEPTH);
        $display("  wr_clk=%0.1fMHz  rd_clk=%0.1fMHz  UART_BYTE_CYCLES=%0d",
                 1000.0 / WR_CLK_PERIOD, 1000.0 / RD_CLK_PERIOD, UART_BYTE_CYCLES);
        $display("==========================================================");

        // ---- Phase 1: Reset ----
        apply_reset();

        // ---- Phase 2: Initialize SRAMs ----
        initialize_srams();
        repeat (5) @(posedge wr_clk);

        // ---- Phase 3: Start Burst ----
        $display("[TB] Starting burst transfer at time %0t...", $time);
        @(posedge wr_clk);
        img_complete   <= 1'b1;
        activate_burst <= 1'b1;
        img_ctrl_start <= 1'b1;

        // ---- Phase 4: Wait for frame_tx_active ----
        $display("[TB] Waiting for frame_tx_active...");
        wait (frame_tx_active === 1'b1);
        $display("[TB] frame_tx_active asserted at time %0t", $time);

        // ---- Phase 5: Wait for full frame completion ----
        $display("[TB] Waiting for full frame transfer...");
        wait (frame_tx_active === 1'b0);
        $display("[TB] frame_tx_active de-asserted at time %0t", $time);

        // Wait for MAC FSM to finish last message
        wait (mac_resp_active === 1'b0);
        repeat (50) @(posedge rd_clk);

        // ---- Phase 6: De-assert controls ----
        @(posedge wr_clk);
        activate_burst <= 1'b0;
        img_ctrl_start <= 1'b0;

        repeat (50) @(posedge rd_clk);

        // ---- Validation Summary ----
        $display("==========================================================");
        $display("  Total UART messages captured: %0d (expected: %0d)", msg_count, RAM_DEPTH);
        $display("  Final pixel_counter = %0d (expected: %0d)", pixel_counter, PIXEL_COUNT);
        if (msg_count == RAM_DEPTH)
            $display("  RESULT: PASS");
        else
            $display("  RESULT: FAIL");
        $display("==========================================================");

        $finish;
    end

    // =========================================================================
    // Watchdog Timer
    // =========================================================================
    initial begin
        #500ms;
        $display("[TB] ERROR: Watchdog timeout at %0t!", $time);
        $finish;
    end

endmodule
