// Top-level UART module: UART RX/TX with mode selection
// Switch[15]: 0 = TX mode, 1 = RX mode
// RX: Receives 16-byte frame: {R###,C###,V###} and displays on 7-segment
// TX: Transmits data (to be implemented)
//Assaf 

import parser_pkg::*;

module uart_top (
    input  logic        clk,           // 100 MHz clock
    input  logic        i_rst_n,         // Active low reset
    
    // Switches
    input  logic [15:0] sw,             // Switches[0:15]
    
    // Buttons
    input  logic        btn_c,          // Button C: Latch switch values (hold for 1 sec)
    
    // UART
    input  logic        rx,             // UART RX input
    output logic        tx,             // UART TX output
    
    // 7-Segment Display
    output logic [7:0]  seg7_an,       // Anode select
    output logic [6:0]  seg7_seg,      // Segment outputs
    
    // Debug LEDs - State indicators
    output logic [15:0] led,            // LEDs: [0-3]=UART RX states, [4-5]=Buffer states, [6-8]=Status, [9-15]=TX states

    output logic        R_16_out,
    output logic        G_16_out,
    output logic        B_16_out,
    output logic        R_17_out,
    output logic        G_17_out,
    output logic        B_17_out
);


    // ============================================================
    // Clocking Wizard (PLL) - Two clocks for glitchless mux
    // ============================================================
    // User created two PLLs in Vivado:
    // - clk_wiz_0: Input clk (100 MHz) -> Output CLK_176MHZ (176 MHz PLL clock)
    // - clk_wiz_1: Input clk (100 MHz) -> Output CLK_100MHZ (100 MHz PLL clock)
    // Both PLLs have lock signals to indicate when clocks are stable
    logic CLK_100MHZ;          // PLL output clock (100 MHz from clk_wiz_1)
    logic CLK_100MHZ_locked;   // PLL lock status for 100 MHz clock
    logic CLK_176MHZ;          // PLL output clock (176 MHz from clk_wiz_0)
    logic CLK_176MHZ_locked;   // PLL lock status for 176 MHz clock
    
    // Instantiate Clocking Wizard IP for 176 MHz clock
    // Note: Module name and port names match Vivado IP configuration
    // PLLs need system reset directly (not synchronized to their output clocks)
    clk_wiz_0 u_clk_wiz (
        .clk_in1(clk),                  // Input: 100 MHz system clock
        .CLK_176MHZ(CLK_176MHZ),        // Output: 176 MHz PLL clock
        .resetn(rst_n),                 // Active low reset (use system reset, not synchronized)
        .CLK_176MHZ_locked(CLK_176MHZ_locked)  // PLL lock status
    );
    
    // Instantiate Clocking Wizard IP for 100 MHz clock
    // Note: Module name and port names match Vivado IP configuration
    // PLLs need system reset directly (not synchronized to their output clocks)
    clk_wiz_1 u_clk_wiz_1 (
        .clk_in1(clk),                  // Input: 100 MHz system clock
        .CLK_100MHZ(CLK_100MHZ),        // Output: 100 MHz PLL clock
        .resetn(rst_n),                 // Active low reset (use system reset, not synchronized)
        .CLK_100MHZ_locked(CLK_100MHZ_locked)  // PLL lock status
    );
    
    // ============================================================
    // Reset Synchronization for PLL clock domains
    // ============================================================
    // Synchronize reset to each PLL clock domain for logic running in those domains
    // Note: PLLs themselves use system reset directly (above)
    logic rst_n_100MHZ;   // Synchronized reset for CLK_100MHZ domain
    logic rst_n_176MHZ;   // Synchronized reset for CLK_176MHZ domain
    
    // Reset synchronizer for CLK_100MHZ domain (for logic running in this domain)
    reset_synchronizer u_rst_sync_100MHZ (
        .clk_dst     (CLK_100MHZ),   // Destination clock domain (100 MHz PLL clock)
        .rst_n_async (i_rst_n),      // Async reset input (system reset)
        .rst_n_sync  (rst_n_100MHZ)  // Synchronized reset output (for CLK_100MHZ domain)
    ); 
    
    // Reset synchronizer for CLK_176MHZ domain (for logic running in this domain)
    reset_synchronizer u_rst_sync_176MHZ (
        .clk_dst     (CLK_176MHZ),   // Destination clock domain (176 MHz PLL clock)
        .rst_n_async (i_rst_n),      // Async reset input (system reset)
        .rst_n_sync  (rst_n_176MHZ)  // Synchronized reset output (for CLK_176MHZ domain)
    );
    
    // ============================================================
    // Glitchless Clock Mux - Switch between 100 MHz and 176 MHz PLL clocks
    // ============================================================
    logic clk_tx;           // TX clock output (selected from mux)
    logic rst_n_tx;         // TX reset output (corresponds to selected clock)
    logic clk_sel;          // Clock selection: 0 = CLK_100MHZ, 1 = CLK_176MHZ
    logic reset_test;
    // Always select 176 MHz PLL clock for TX (can be made configurable later)
    assign clk_sel = 1'b1;  // 1 = select CLK_176MHZ (176 MHz)
    //assign clk_tx = CLK_176MHZ;
    //assign rst_n_tx = rst_n_176MHZ;
    // // Clock mux ensures safe switching: only switches when target clock is locked
    // // Both clocks are from PLLs, so both require lock signals before switching
    // // The mux outputs both the selected clock and its corresponding reset


     glitchless_clock_mux u_glitchless_clock_mux (
         .clk1         (CLK_100MHZ),          // PLL clock 1 (100 MHz)
         .clk2         (CLK_176MHZ),          // PLL clock 2 (176 MHz)
         .clk1_locked  (CLK_100MHZ_locked),   // 100 MHz PLL lock status
         .clk2_locked  (CLK_176MHZ_locked),   // 176 MHz PLL lock status
         .sel          (clk_sel),             // Clock selection: 0 = clk1, 1 = clk2
         .rst_n1       (rst_n_100MHZ),        // Reset for clk1 domain
         .rst_n2       (rst_n_176MHZ),        // Reset for clk2 domain
         .clk_out      (clk_tx),              // Selected clock output for TX
         .rst_n_out    (rst_n_tx)             // Selected reset output (corresponds to selected clock)
     );
    
    //assign rst_n_tx = rst_n_176MHZ;
    
    // Note: rst_n_tx is now provided by the clock mux output (rst_n_out)
    // It automatically corresponds to the selected clock domain
    
    // Reset synchronizer for clk domain
    reset_synchronizer u_rst_sync_clk (
        .clk_dst     (clk),         // Destination clock domain (system clock)
        .rst_n_async (i_rst_n),     // Async reset input (system reset)
        .rst_n_sync  (rst_n)        // Synchronized reset output (for clk domain)
    );



//==============================================================================
// 1. UART Rx PHY Instance
//==============================================================================
// Physical Layer: Receives serial bits, outputs bytes
//------------------------------------------------------------------------------

logic [7:0]  rx_phy_data_out;
logic        rx_phy_valid_out;
logic        rx_phy_error_out;
logic        rx_phy_faulty_inc_out;
uart_state_t rx_phy_state_out;

uart_rx #(
    .CLK_FREQ (176_000_000), 
    .BAUD_RATE(5_500_000)
) u_uart_rx_phy (
    .clk              (CLK_176MHZ),
    .rst_n            (rst_n_176MHZ),
    .rx               (rx),
    .rx_data          (rx_phy_data_out),
    .rx_valid         (rx_phy_valid_out),
    .rx_error         (rx_phy_error_out),
    .faulty_frame_inc (rx_phy_faulty_inc_out),
    .rx_state         (rx_phy_state_out)
);


//==============================================================================
// 2. UART Rx MAC Instance
//==============================================================================
// MAC Layer: Validates frames and handles burst mode switching
//------------------------------------------------------------------------------

logic [127:0] mac_msg_data_out;
logic         mac_msg_valid_out;
msg_type_e    MAC_Msg_Type;
logic         mac_frame_error_out;

uart_rx_MAC u_uart_rx_mac (
    .clk           (CLK_176MHZ),
    .rst_n         (rst_n_176MHZ),
    .rx_data       (rx_phy_data_out),   // Connects to PHY
    .rx_valid      (rx_phy_valid_out),  // Connects to PHY
    .burst_mode_in (class_burst_on_out),
    .msg_data      (mac_msg_data_out),
    .msg_valid     (mac_msg_valid_out),
    .msg_type      (MAC_Msg_Type),
    .frame_error   (mac_frame_error_out)
);


//==============================================================================
// 3. UART Classifier Instance (176 MHz Domain)
//==============================================================================
// Classifier: Parses MAC messages and routes data to Sequencers/RGF
// Runs in 176 MHz domain with PHY and MAC
//------------------------------------------------------------------------------

// Classifier outputs (176 MHz domain - directly to CDC)
logic         class_valid_msg_out;
msg_type_e    Class_Msg_Type;
logic [7:0]   class_parsed_addr_out;
logic [15:0]  class_parsed_offset_out;
logic [15:0]  class_data_high_out;
logic [15:0]  class_data_low_out;
logic [31:0]  class_height_out;
logic [31:0]  class_width_out;
logic [7:0]   class_pixel_r_out;
logic [7:0]   class_pixel_g_out;
logic [7:0]   class_pixel_b_out;
logic [31:0]  class_burst_red_out;
logic [31:0]  class_burst_green_out;
logic [31:0]  class_burst_blue_out;
logic         class_data_avail_out;
logic         class_burst_on_out;   // Connects back to MAC burst_mode_in

// Classifier inputs from CDC (synchronized from system domain)
logic         class_seq_ready_in;   // From CDC
logic         class_burst_done_in;  // From CDC

uart_classifier u_uart_classifier (
    .clk                (CLK_176MHZ),          // 176 MHz domain
    .rst_n              (rst_n_176MHZ),        // 176 MHz reset
    .msg_data           (mac_msg_data_out),    // Connects to MAC
    .msg_valid          (mac_msg_valid_out),   // Connects to MAC
    .msg_type           (MAC_Msg_Type),        // From MAC
    .seq_ready          (class_seq_ready_in),  // From CDC (synced)
    .burst_done         (class_burst_done_in), // From CDC (synced)
    .valid_msg          (class_valid_msg_out),
    .classified_type    (Class_Msg_Type),
    .parsed_addr        (class_parsed_addr_out),
    .parsed_offset_addr (class_parsed_offset_out),
    .parsed_data_high   (class_data_high_out),
    .parsed_data_low    (class_data_low_out),
    .parsed_height      (class_height_out),
    .parsed_width       (class_width_out),
    .pixel_r            (class_pixel_r_out),
    .pixel_g            (class_pixel_g_out),
    .pixel_b            (class_pixel_b_out),
    .burst_red          (class_burst_red_out),
    .burst_green        (class_burst_green_out),
    .burst_blue         (class_burst_blue_out),
    .data_available     (class_data_avail_out),
    .burst_on           (class_burst_on_out)
);


//==============================================================================
// 3b. CDC Synchronizer Instance (176 MHz <-> System Clock)
//==============================================================================
// Handles clock domain crossing between RX chain (176 MHz) and System (100 MHz)
//------------------------------------------------------------------------------

// System domain signals (output from CDC)
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

// System domain inputs to CDC (from rx_msg_handler)
logic         sys_seq_ready;
logic         sys_burst_done;

// These are now driven by rx_msg_handler (unified handshake)

// =========================================================================
// CDC GLITCH GUARD: Register combinational signals before CDC crossing
// =========================================================================
// sys_seq_ready and sys_burst_done are combinational outputs from the
// rx_msg_handler (OR of multiple got_msg signals / burst sequencer state).
// When the burst sequencer state machine transitions (e.g., WRITE_TO_SRAM
// 0011 -> CHECK_ROW_COUNT 0100), multi-bit state changes can produce brief
// combinational glitches. The 176MHz CDC sampler can capture these glitches,
// causing spurious burst_done/seq_ready pulses.
// Fix: Register in the 100MHz domain first -> guaranteed glitch-free.
// =========================================================================
logic sys_seq_ready_reg;
logic sys_burst_done_reg;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sys_seq_ready_reg  <= 1'b0;
        sys_burst_done_reg <= 1'b0;
    end else begin
        sys_seq_ready_reg  <= sys_seq_ready;
        sys_burst_done_reg <= sys_burst_done;
    end
end

uart_cdc_sync u_uart_cdc_sync (
    // Clocks and Resets
    .clk_rx             (CLK_176MHZ),
    .clk_sys            (clk),
    .rst_rx_n           (rst_n_176MHZ),
    .rst_sys_n          (rst_n),
    
    // RX Domain inputs (from classifier @ 176 MHz)
    .rx_data_available  (class_data_avail_out),
    .rx_burst_on        (class_burst_on_out),
    .rx_valid_msg       (class_valid_msg_out),
    .rx_classified_type (Class_Msg_Type),
    .rx_parsed_addr     (class_parsed_addr_out),
    .rx_parsed_offset_addr (class_parsed_offset_out),
    .rx_parsed_data_high(class_data_high_out),
    .rx_parsed_data_low (class_data_low_out),
    .rx_parsed_height   (class_height_out),
    .rx_parsed_width    (class_width_out),
    .rx_pixel_r         (class_pixel_r_out),
    .rx_pixel_g         (class_pixel_g_out),
    .rx_pixel_b         (class_pixel_b_out),
    .rx_burst_red       (class_burst_red_out),
    .rx_burst_green     (class_burst_green_out),
    .rx_burst_blue      (class_burst_blue_out),
    
    // System Domain outputs (to sequencers/RGF @ sys clk)
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
    
    // System Domain inputs (from sequencers/RGF @ sys clk)
    // Use REGISTERED versions to prevent combinational glitches across CDC
    .sys_seq_ready      (sys_seq_ready_reg),
    .sys_burst_done     (sys_burst_done_reg),
    
    // RX Domain outputs (to classifier @ 176 MHz)
    .rx_seq_ready       (class_seq_ready_in),
    .rx_burst_done      (class_burst_done_in)
);


//==============================================================================
// 4. RX Message Handler Instance
//==============================================================================
// Encapsulates RGF Manager, Sequencer Single, and Sequencer Burst
// Provides unified handshake interface to classifier via CDC
//------------------------------------------------------------------------------

// Single Pixel Sequencer outputs
logic [31:0] seq_sing_red_data_out;
logic [31:0] seq_sing_green_data_out;
logic [31:0] seq_sing_blue_data_out;
logic        seq_sing_r_wr_en_out;
logic [13:0] seq_sing_r_addr_out;
logic        seq_sing_g_wr_en_out;
logic [13:0] seq_sing_g_addr_out;
logic        seq_sing_b_wr_en_out;
logic [13:0] seq_sing_b_addr_out;
logic        seq_sing_busy_out;
logic        seq_sing_done_out;

// Burst Sequencer outputs
logic [31:0] seq_burst_red_wr_data_out;
logic [31:0] seq_burst_green_wr_data_out;
logic [31:0] seq_burst_blue_wr_data_out;
logic        seq_burst_r_wr_en_out;
logic [13:0] seq_burst_r_addr_out;
logic        seq_burst_g_wr_en_out;
logic [13:0] seq_burst_g_addr_out;
logic        seq_burst_b_wr_en_out;
logic [13:0] seq_burst_b_addr_out;
logic        seq_burst_busy_out;
logic        seq_burst_done_out;

// RGF Manager outputs
logic [31:0] rgf_read_data_combined;
logic        rgf_leg_img;
logic        rgf_leg_tx_fifo;
logic        rgf_leg_led;
logic        rgf_leg_sys;
logic        rgf_leg_ctrl;
logic        rgf_leg_pwm;
logic [15:0] rgf_offset_addr;
logic [31:0] rgf_wr_data;
logic        rgf_wr_en;
logic        rgf_rd_en;
logic [31:0] rgf_addr_to_cmpsr_out;
logic [31:0] rgf_data_to_cmpsr_out;
logic        rgf_start_req_to_cmpsr_out;
logic        rgf_read_active_out;

// --- Collect Read Data from all RGFs (32-bit) ---
assign rgf_read_data_combined = led_rgf_data_out | cnt_rgf_data_out | pwm_rgf_data_out | sys_rgf_data_out | img_rgf_data_out | seq_tx_img_fifo_rgf_data_out;

//------------------------------------------------------------------------------
// CDC: Message Composer (176MHz) → RGF Manager (100MHz)
//------------------------------------------------------------------------------
// Sync cmpsr_busy from 176MHz to 100MHz
logic cmpsr_busy_sync1, cmpsr_busy_sync;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cmpsr_busy_sync1 <= 1'b0;
        cmpsr_busy_sync  <= 1'b0;
    end else begin
        cmpsr_busy_sync1 <= cmpsr_busy_out;
        cmpsr_busy_sync  <= cmpsr_busy_sync1;
    end
end

rx_msg_handler u_rx_msg_handler (
    .clk                    (clk),
    .rst_n                  (rst_n),
    
    // From CDC Sync (classifier outputs, synchronized to sys clk)
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
    
    // Handshake back to CDC/Classifier
    .seq_ready              (sys_seq_ready),
    .burst_done             (sys_burst_done),
    
    // Configuration from RGF_IMG
    .hw_img_width           (hw_img_width[7:0]),
    .hw_img_height          (hw_img_height[7:0]),
    
    // RGF Interface outputs
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
    
    // RGF read path
    .rgf_read_data          (rgf_read_data_combined),
    .cmpsr_busy             (cmpsr_busy_sync),         // CDC synced from 176MHz
    .rgf_addr_to_cmpsr      (rgf_addr_to_cmpsr_out),
    .rgf_data_to_cmpsr      (rgf_data_to_cmpsr_out),
    .rgf_start_req_to_cmpsr (rgf_start_req_to_cmpsr_out),
    .rgf_read_active        (rgf_read_active_out),
    
    // SRAM Write Interface - Single Pixel Sequencer
    .sing_red_write_data    (seq_sing_red_data_out),
    .sing_green_write_data  (seq_sing_green_data_out),
    .sing_blue_write_data   (seq_sing_blue_data_out),
    .sing_sram_r_wr_en      (seq_sing_r_wr_en_out),
    .sing_sram_r_addr       (seq_sing_r_addr_out),
    .sing_sram_g_wr_en      (seq_sing_g_wr_en_out),
    .sing_sram_g_addr       (seq_sing_g_addr_out),
    .sing_sram_b_wr_en      (seq_sing_b_wr_en_out),
    .sing_sram_b_addr       (seq_sing_b_addr_out),
    .sing_seq_busy          (seq_sing_busy_out),
    .sing_seq_done          (seq_sing_done_out),
    
    // SRAM Write Interface - Burst Sequencer
    .burst_red_write_data   (seq_burst_red_wr_data_out),
    .burst_green_write_data (seq_burst_green_wr_data_out),
    .burst_blue_write_data  (seq_burst_blue_wr_data_out),
    .burst_sram_r_wr_en     (seq_burst_r_wr_en_out),
    .burst_sram_r_addr      (seq_burst_r_addr_out),
    .burst_sram_g_wr_en     (seq_burst_g_wr_en_out),
    .burst_sram_g_addr      (seq_burst_g_addr_out),
    .burst_sram_b_wr_en     (seq_burst_b_wr_en_out),
    .burst_sram_b_addr      (seq_burst_b_addr_out),
    .burst_seq_busy         (seq_burst_busy_out),
    .burst_seq_done_out     (seq_burst_done_out)
);



// Instantiate Image RGF module
    logic [31:0]  img_rgf_data_out;
    logic [9:0]   hw_img_height;
    logic [9:0]   hw_img_width;
    logic         hw_img_ready_in_PC;
    logic [9:0]   hw_row_cnt;
    logic [9:0]   hw_col_cnt;
    logic         hw_img_transfer_complete;
    logic         hw_img_ready_in_SRAM;
    logic         hw_start_image_read;


always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        hw_img_ready_in_SRAM <= 1'b0;
    end else begin
        if (seq_sing_busy_out || seq_burst_busy_out) begin
            hw_img_ready_in_SRAM <= 1'b0;
        end
        else if (seq_sing_done_out || seq_burst_done_out) begin
            hw_img_ready_in_SRAM <= 1'b1;
        end
    end
end

RGF_IMG #(
    .ADDR_WIDTH (16),
    .DATA_WIDTH (32)
    ) u_RGF_IMG (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (rgf_offset_addr),
        .wr_en            (rgf_wr_en),
        .rd_en            (rgf_rd_en),
        .wdata            (rgf_wr_data),
        .addr_decoder_leg (rgf_leg_img),
        .rdata            (img_rgf_data_out),
        .hw_img_height            (hw_img_height),
        .hw_img_width             (hw_img_width),
        .hw_img_ready_in_PC       (hw_img_ready_in_PC),
        .hw_row_cnt               (hw_row_cnt),
        .hw_col_cnt               (hw_col_cnt),
        .hw_img_transfer_complete (hw_img_transfer_complete),
        .hw_img_ready_in_SRAM     (hw_img_ready_in_SRAM),
        .hw_start_image_read      (hw_start_image_read)
);

//==============================================================================
// 7. SRAM Red Instance
//==============================================================================
// Red Channel Memory (Inputs driven by MUX of Single/Burst Logic)
//------------------------------------------------------------------------------

logic        sram_red_wr_en_in;
logic [13:0] sram_red_wr_addr_in;
logic [31:0] sram_red_wr_data_in;
logic        sram_red_rd_en_in;
logic [13:0] sram_red_rd_addr_in;
logic [31:0] sram_red_rd_data_out;

// SRAM Red Write Path - MUX between single and burst RX sequencers
assign sram_red_wr_en_in   = seq_sing_r_wr_en_out | seq_burst_r_wr_en_out;
assign sram_red_wr_addr_in = seq_burst_r_wr_en_out ? seq_burst_r_addr_out : seq_sing_r_addr_out;
assign sram_red_wr_data_in = seq_burst_r_wr_en_out ? seq_burst_red_wr_data_out : seq_sing_red_data_out;

// SRAM Red Read Path - Connect to TX Sequencer (14-bit addr)
assign sram_red_rd_en_in   = seq_tx_r_rd_en_out;
assign sram_red_rd_addr_in = seq_tx_r_addr_out;

sram_red #(
    .ADDR_W(14),    // 2^14 = 16384 addresses for 256x256 image (4 pixels/word)
    .DATA_W(32)
) u_sram_red (
    .clk     (clk),
    .wr_en   (sram_red_wr_en_in),
    .wr_addr (sram_red_wr_addr_in),
    .wr_data (sram_red_wr_data_in),
    .rd_en   (sram_red_rd_en_in),
    .rd_addr (sram_red_rd_addr_in),
    .rd_data (sram_red_rd_data_out)
);


//==============================================================================
// 8. SRAM Green Instance
//==============================================================================
// Green Channel Memory
//------------------------------------------------------------------------------

logic        sram_green_wr_en_in;
logic [13:0] sram_green_wr_addr_in;
logic [31:0] sram_green_wr_data_in;
logic        sram_green_rd_en_in;
logic [13:0] sram_green_rd_addr_in;
logic [31:0] sram_green_rd_data_out;

// SRAM Green Write Path - MUX between single and burst RX sequencers
assign sram_green_wr_en_in   = seq_sing_g_wr_en_out | seq_burst_g_wr_en_out;
assign sram_green_wr_addr_in = seq_burst_g_wr_en_out ? seq_burst_g_addr_out : seq_sing_g_addr_out;
assign sram_green_wr_data_in = seq_burst_g_wr_en_out ? seq_burst_green_wr_data_out : seq_sing_green_data_out;

// SRAM Green Read Path - Connect to TX Sequencer (14-bit addr)
assign sram_green_rd_en_in   = seq_tx_g_rd_en_out;
assign sram_green_rd_addr_in = seq_tx_g_addr_out;

sram_green #(
    .ADDR_W(14),    // 2^14 = 16384 addresses for 256x256 image (4 pixels/word)
    .DATA_W(32)
) u_sram_green (
    .clk     (clk),
    .wr_en   (sram_green_wr_en_in),
    .wr_addr (sram_green_wr_addr_in),
    .wr_data (sram_green_wr_data_in),
    .rd_en   (sram_green_rd_en_in),
    .rd_addr (sram_green_rd_addr_in),
    .rd_data (sram_green_rd_data_out)
);


//==============================================================================
// 9. SRAM Blue Instance
//==============================================================================
// Blue Channel Memory
//------------------------------------------------------------------------------

logic        sram_blue_wr_en_in;
logic [13:0] sram_blue_wr_addr_in;
logic [31:0] sram_blue_wr_data_in;
logic        sram_blue_rd_en_in;
logic [13:0] sram_blue_rd_addr_in;
logic [31:0] sram_blue_rd_data_out;

// SRAM Blue Write Path - MUX between single and burst RX sequencers
assign sram_blue_wr_en_in   = seq_sing_b_wr_en_out | seq_burst_b_wr_en_out;
assign sram_blue_wr_addr_in = seq_burst_b_wr_en_out ? seq_burst_b_addr_out : seq_sing_b_addr_out;
assign sram_blue_wr_data_in = seq_burst_b_wr_en_out ? seq_burst_blue_wr_data_out : seq_sing_blue_data_out;

// SRAM Blue Read Path - Connect to TX Sequencer (14-bit addr)
assign sram_blue_rd_en_in   = seq_tx_b_rd_en_out;
assign sram_blue_rd_addr_in = seq_tx_b_addr_out;

sram_blue #(
    .ADDR_W(14),    // 2^14 = 16384 addresses for 256x256 image (4 pixels/word)
    .DATA_W(32)
) u_sram_blue (
    .clk     (clk),
    .wr_en   (sram_blue_wr_en_in),
    .wr_addr (sram_blue_wr_addr_in),
    .wr_data (sram_blue_wr_data_in),
    .rd_en   (sram_blue_rd_en_in),
    .rd_addr (sram_blue_rd_addr_in),
    .rd_data (sram_blue_rd_data_out)
);


//==============================================================================
// 10. Sequencer Tx Image Burst Instance
//==============================================================================
// Reads SRAM and sends burst packets
//------------------------------------------------------------------------------

logic        seq_tx_busy_in;
logic        seq_tx_start_in;
logic        seq_tx_burst_en_in;
logic        seq_tx_img_complete_in;

logic [13:0] seq_tx_r_addr_out;
logic [13:0] seq_tx_g_addr_out;
logic [13:0] seq_tx_b_addr_out;
logic        seq_tx_r_rd_en_out;
logic        seq_tx_g_rd_en_out;
logic        seq_tx_b_rd_en_out;
logic [95:0] seq_tx_pixel_packet_out;
logic        seq_tx_packet_ready_out;
logic        seq_tx_active_out;
logic [16:0] seq_tx_counter_out;

Sequencer_Burst_Tx #(
    .RAM_ADDR_WIDTH(14),
    .RAM_DATA_WIDTH(32),
    .FIFO_WIDTH    (32),
    .FIFO_DEPTH    (16),
    .PIXEL_COUNT   (65536)
) u_seq_tx_image_burst (
    .wr_clk             (clk),              // SRAM side: 100MHz system clock
    .rd_clk             (CLK_176MHZ),       // TX side: 176MHz fast clock
    .rst_n              (rst_n),
    .red_RAM_data       (sram_red_rd_data_out),     // Connects to SRAM Red
    .green_RAM_data     (sram_green_rd_data_out),   // Connects to SRAM Green
    .blue_RAM_data      (sram_blue_rd_data_out),    // Connects to SRAM Blue
    .red_RAM_addr       (seq_tx_r_addr_out),
    .green_RAM_addr     (seq_tx_g_addr_out),
    .blue_RAM_addr      (seq_tx_b_addr_out),
    .red_read_en        (seq_tx_r_rd_en_out),
    .green_read_en      (seq_tx_g_rd_en_out),
    .blue_read_en       (seq_tx_b_rd_en_out),
    .tx_busy            (seq_tx_busy_in),
    .pixel_packet       (seq_tx_pixel_packet_out),
    .pixel_packet_ready (seq_tx_packet_ready_out),
    .img_ctrl_start     (seq_tx_start_in),
    .activate_burst     (seq_tx_burst_en_in),
    .img_complete       (seq_tx_img_complete_in),
    .frame_tx_active    (seq_tx_active_out),
    .pixel_counter      (seq_tx_counter_out)
);


//==============================================================================
// 11. Message Composer Instance
//==============================================================================
// Composes RGF read responses or Image Packets into messages
//------------------------------------------------------------------------------

logic        cmpsr_rgf_read_in;
logic [31:0] cmpsr_rgf_addr_in;
logic [31:0] cmpsr_rgf_data_in;
logic        cmpsr_img_single_in;
logic [13:0] cmpsr_img_row_in;
logic [13:0] cmpsr_img_col_in;
logic [23:0] cmpsr_pixel_cell_in;
logic        cmpsr_img_burst_in;
logic [31:0] cmpsr_red_burst_in;
logic [31:0] cmpsr_green_burst_in;
logic [31:0] cmpsr_blue_burst_in;
logic        cmpsr_start_req_in;
logic        cmpsr_tx_mac_busy_in;

logic        cmpsr_tx_req_out;
logic        cmpsr_busy_out;
logic [127:0] cmpsr_data_out;

//------------------------------------------------------------------------------
// CDC: RGF Manager (100MHz) → Message Composer (176MHz)
//------------------------------------------------------------------------------
// Synchronize control signals from 100MHz to 176MHz domain

// 2-FF sync for rgf_read_active
logic rgf_read_sync1, rgf_read_sync2;
always_ff @(posedge CLK_176MHZ or negedge rst_n_176MHZ) begin
    if (!rst_n_176MHZ) begin
        rgf_read_sync1 <= 1'b0;
        rgf_read_sync2 <= 1'b0;
    end else begin
        rgf_read_sync1 <= rgf_read_active_out;
        rgf_read_sync2 <= rgf_read_sync1;
    end
end

// 2-FF sync for rgf_start_req
logic rgf_start_req_sync1, rgf_start_req_sync2;
always_ff @(posedge CLK_176MHZ or negedge rst_n_176MHZ) begin
    if (!rst_n_176MHZ) begin
        rgf_start_req_sync1 <= 1'b0;
        rgf_start_req_sync2 <= 1'b0;
    end else begin
        rgf_start_req_sync1 <= rgf_start_req_to_cmpsr_out;
        rgf_start_req_sync2 <= rgf_start_req_sync1;
    end
end

// Register data signals in 176MHz domain (stable when start_req is high)
logic [31:0] rgf_addr_sync, rgf_data_sync;
always_ff @(posedge CLK_176MHZ or negedge rst_n_176MHZ) begin
    if (!rst_n_176MHZ) begin
        rgf_addr_sync <= 32'd0;
        rgf_data_sync <= 32'd0;
    end else begin
        rgf_addr_sync <= rgf_addr_to_cmpsr_out;
        rgf_data_sync <= rgf_data_to_cmpsr_out;
    end
end

msg_compposer u_msg_composer (
    .clk                   (CLK_176MHZ),
    .rst_n                 (rst_n_176MHZ),
    .now_rgf_read          (rgf_read_sync2),               // CDC synced
    .rgf_raw_address       (rgf_addr_sync),                // CDC synced
    .rgf_data              (rgf_data_sync),                // CDC synced
    .now_image_read_single (cmpsr_img_single_in),
    .img_row_counter       (cmpsr_img_row_in),
    .img_col_counter       (cmpsr_img_col_in),
    .pixel_cell            (cmpsr_pixel_cell_in),
    .now_image_read_burst  (cmpsr_img_burst_in),
    .red_burst_data        (cmpsr_red_burst_in),
    .green_burst_data      (cmpsr_green_burst_in),
    .blue_burst_data       (cmpsr_blue_burst_in),
    .to_cmpsr_start_req    (rgf_start_req_sync2 | seq_tx_packet_ready_out),   // CDC synced RGF or Burst TX
    .tx_mac_busy           (cmpsr_tx_mac_busy_in),
    .to_mac_tx_start_req   (cmpsr_tx_req_out),
    .cmpsr_busy            (cmpsr_busy_out),
    .data_out_msg_cmps     (cmpsr_data_out)
);


//==============================================================================
// 12. UART Tx MAC Instance
//==============================================================================
// Serializes 128-bit messages to bytes
//------------------------------------------------------------------------------

logic        tx_mac_start_req_in;
logic [127:0] tx_mac_read_data_in;
logic        tx_mac_busy_in;

logic        tx_mac_start_pulse_out;
logic [7:0]  tx_mac_data_out;
logic        tx_mac_active_out;

uart_tx_mac_fsm u_uart_tx_mac (
    .clk          (CLK_176MHZ),
    .rst_n        (rst_n_176MHZ),
    .start_tx_req (cmpsr_tx_req_out),       // Connects to Composer
    .read_data    (cmpsr_data_out),         // Connects to Composer
    .tx_busy      (tx_mac_busy_in),
    .tx_start     (tx_mac_start_pulse_out),
    .tx_data      (tx_mac_data_out),
    .resp_active  (tx_mac_active_out)
);


//==============================================================================
// 13. UART Tx PHY Instance
//==============================================================================
// Serializes bytes to bits (Tx Line)
//------------------------------------------------------------------------------

logic        tx_phy_start_in;
logic [7:0]  tx_phy_data_in;
logic        tx_phy_out;
logic        tx_phy_busy_out;
uart_state_t tx_phy_state_out;

uart_tx #(
    .CLK_FREQ (176_000_000),   // 176MHz fast clock (matches RX)
    .BAUD_RATE(5_500_000)      // Match RX baud rate
) u_uart_tx_phy (
    .clk      (CLK_176MHZ),
    .rst_n    (rst_n_176MHZ),
    .tx_start (tx_mac_start_pulse_out),     // Connects to TX MAC
    .tx_data  (tx_mac_data_out),            // Connects to TX MAC
    .tx       (tx_phy_out),
    .tx_busy  (tx_phy_busy_out),
    .tx_state (tx_phy_state_out)
);
    
    // Instantiate LED RGF module
    logic [31:0]     led_rgf_data_out;
    logic [1:0]      hw_led_select;
    logic            hw_led16_on_off;
    logic            hw_led17_on_off;
    logic            hw_led16_cie_on_off;
    logic            hw_led17_cie_on_off;
    logic [1:0]      hw_pattern_mode;

    RGF_LED #(
        .ADDR_WIDTH (16),
        .DATA_WIDTH (32)
    ) u_RGF_LED (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (rgf_offset_addr),
        .wr_en            (rgf_wr_en),
        .rd_en            (rgf_rd_en),
        .wdata            (rgf_wr_data),
        .addr_decoder_leg (rgf_leg_led),
        .rdata            (led_rgf_data_out),
        .hw_led_select    (hw_led_select),
        .hw_led16_on_off  (hw_led16_on_off),
        .hw_led17_on_off  (hw_led17_on_off),
        .hw_led16_cie_on_off (hw_led16_cie_on_off),
        .hw_led17_cie_on_off (hw_led17_cie_on_off),
        .hw_pattern_mode  (hw_pattern_mode)
    );

    // Instantiate Counter RGF module
    logic [31:0]  cnt_rgf_data_out;
    logic [15:0]  hw_uart_rx_packets_cnt;
    logic [15:0]  hw_uart_tx_packets_cnt;
    logic [15:0]  hw_color_msg_cnt;
    logic [15:0]  hw_config_msg_cnt;

    RGF_CNT #(
        .ADDR_WIDTH (16),
        .DATA_WIDTH (32)
    ) u_RGF_CNT (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (rgf_offset_addr),
        .wr_en            (rgf_wr_en),
        .rd_en            (rgf_rd_en),
        .wdata            (rgf_wr_data),
        .addr_decoder_leg (rgf_leg_ctrl),
        .rdata            (cnt_rgf_data_out),
        .hw_uart_rx_packets_cnt (hw_uart_rx_packets_cnt),
        .hw_uart_tx_packets_cnt (hw_uart_tx_packets_cnt),
        .hw_color_msg_cnt       (hw_color_msg_cnt),
        .hw_config_msg_cnt      (hw_config_msg_cnt)
    );

    // Instantiate PWM RGF module
    logic [31:0]  pwm_rgf_data_out;
    logic [15:0]  hw_pwm_time_slots;
    logic [15:0]  hw_pwm_sweep_time;
    logic [12:0]  hw_red_output_freq;
    logic [1:0]   hw_red_magnitude;
    logic [8:0]   hw_red_init_phase;
    logic [12:0]  hw_green_output_freq;
    logic [1:0]   hw_green_magnitude;
    logic [8:0]   hw_green_init_phase;
    logic [12:0]  hw_blue_output_freq;
    logic [1:0]   hw_blue_magnitude;
    logic [8:0]   hw_blue_init_phase;

    RGF_PWM #(
        .ADDR_WIDTH (16),
        .DATA_WIDTH (32)
    ) u_RGF_PWM (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (rgf_offset_addr),
        .wr_en            (rgf_wr_en),
        .rd_en            (rgf_rd_en),
        .wdata            (rgf_wr_data),
        .addr_decoder_leg (rgf_leg_pwm),
        .rdata            (pwm_rgf_data_out),
        .hw_pwm_time_slots    (hw_pwm_time_slots),
        .hw_pwm_sweep_time    (hw_pwm_sweep_time),
        .hw_red_output_freq   (hw_red_output_freq),
        .hw_red_magnitude     (hw_red_magnitude),
        .hw_red_init_phase    (hw_red_init_phase),
        .hw_green_output_freq (hw_green_output_freq),
        .hw_green_magnitude   (hw_green_magnitude),
        .hw_green_init_phase  (hw_green_init_phase),
        .hw_blue_output_freq  (hw_blue_output_freq),
        .hw_blue_magnitude    (hw_blue_magnitude),
        .hw_blue_init_phase   (hw_blue_init_phase)
    );

    // Instantiate System RGF module
    logic [31:0]  sys_rgf_data_out;
    logic         hw_sw_reset;
    logic         hw_enable;

    RGF_SYS #(
        .ADDR_WIDTH (16),
        .DATA_WIDTH (32)
    ) u_RGF_SYS (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (rgf_offset_addr),
        .wr_en            (rgf_wr_en),
        .rd_en            (rgf_rd_en),
        .wdata            (rgf_wr_data),
        .addr_decoder_leg (rgf_leg_sys),
        .rdata            (sys_rgf_data_out),
        .hw_sw_reset      (hw_sw_reset),
        .hw_enable        (hw_enable)
    );

    // NOTE: RGF_IMG is instantiated earlier at line ~433 using rgf_manager signals


    // Instantiate Sequence TX Image FIFO RGF module
    logic [31:0]  seq_tx_img_fifo_rgf_data_out;
    // -- FIFO Sizes Outputs --
    logic [11:0] hw_fifo_depth;
    logic [5:0]  hw_single_cell_width;

    // -- Threshold LVL Outputs --
    logic [10:0] hw_almost_empty_level;
    logic [10:0] hw_almost_full_level;

    RGF_Seq_TX_IMG_FIFO #(
        .ADDR_WIDTH (16),
        .DATA_WIDTH (32)
    ) u_RGF_Seq_TX_IMG_FIFO (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (rgf_offset_addr),
        .wr_en            (rgf_wr_en),
        .rd_en            (rgf_rd_en),
        .wdata            (rgf_wr_data),
        .addr_decoder_leg (rgf_leg_tx_fifo),
        .rdata            (seq_tx_img_fifo_rgf_data_out),
        .hw_fifo_depth            (hw_fifo_depth),
        .hw_single_cell_width     (hw_single_cell_width),
        .hw_almost_empty_level    (hw_almost_empty_level),
        .hw_almost_full_level     (hw_almost_full_level)
    );


    // ============================================================
    // RGB LED Outputs (directly tied low - no pwm_rgb module)
    // ============================================================
    assign R_16_out = 1'b0;
    assign G_16_out = 1'b0;
    assign B_16_out = 1'b0;
    assign R_17_out = 1'b0;
    assign G_17_out = 1'b0;
    assign B_17_out = 1'b0;

//==============================================================================
// TX Path Control Logic
//==============================================================================
// Connect TX control signals and feedback loops
//------------------------------------------------------------------------------

// TX PHY output to top-level port
assign tx = tx_phy_out;

// TX MAC busy feedback - TX PHY busy feeds back to TX MAC
assign tx_mac_busy_in = tx_phy_busy_out;

// Message Composer TX MAC busy - TX MAC active feeds to composer
assign cmpsr_tx_mac_busy_in = tx_mac_active_out;

// TX Sequencer Control Signals
// ============================================================================
// The TX sequencer needs img_ctrl_start to stay HIGH during the entire transfer.
// We latch on the MSG_START_BURST_RD message and clear when transfer completes.

// Detect MSG_START_BURST_RD trigger (pulse)
logic seq_tx_trigger;
assign seq_tx_trigger = (sys_classified_type == MSG_START_BURST_RD) && sys_data_available;

// ============================================================================
// Synchronize frame_tx_active from 176MHz (rd_clk) to 100MHz (clk)
// CRITICAL: seq_tx_active_out (frame_tx_active) is in the 176MHz domain.
// Using a multi-bit counter (pixel_counter) directly across clock domains
// causes CDC violations and random premature stop (e.g., stalling at ~48%).
// Instead, synchronize the single-bit frame_tx_active and detect its falling edge.
// ============================================================================
logic seq_tx_active_sync1, seq_tx_active_sync2, seq_tx_active_sync3;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        seq_tx_active_sync1 <= 1'b0;
        seq_tx_active_sync2 <= 1'b0;
        seq_tx_active_sync3 <= 1'b0;
    end else begin
        seq_tx_active_sync1 <= seq_tx_active_out;  // 1st FF (may be metastable)
        seq_tx_active_sync2 <= seq_tx_active_sync1; // 2nd FF (stable)
        seq_tx_active_sync3 <= seq_tx_active_sync2; // 3rd FF (for edge detect)
    end
end

// Detect falling edge of synchronized frame_tx_active = TX transfer complete
logic seq_tx_frame_done;
assign seq_tx_frame_done = seq_tx_active_sync3 && !seq_tx_active_sync2;

// Latch the start signal - stays high during entire transfer
logic seq_tx_active_latch;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        seq_tx_active_latch <= 1'b0;
    end else begin
        if (seq_tx_trigger && hw_img_ready_in_SRAM) begin
            seq_tx_active_latch <= 1'b1;  // Start TX (only if image is ready)
        end else if (seq_tx_frame_done) begin
            seq_tx_active_latch <= 1'b0;  // Clear when TX frame transfer complete (properly synced)
        end
    end
end

// img_ctrl_start - stays high during transfer
assign seq_tx_start_in = seq_tx_active_latch;

// activate_burst - same as start for continuous operation
assign seq_tx_burst_en_in = seq_tx_active_latch;

// TX sequencer is busy when TX MAC is active
assign seq_tx_busy_in = tx_mac_active_out;

// Image complete signal - must be high before TX can start
assign seq_tx_img_complete_in = hw_img_ready_in_SRAM;

// Message Composer Burst Image Inputs
// pixel_packet format: {P3[R,G,B], P2[R,G,B], P1[R,G,B], P0[R,G,B]} = 96 bits
// [95:88]=R3, [87:80]=G3, [79:72]=B3
// [71:64]=R2, [63:56]=G2, [55:48]=B2
// [47:40]=R1, [39:32]=G1, [31:24]=B1
// [23:16]=R0, [15:8]=G0,  [7:0]=B0
// Repack by color channel for message composer:
assign cmpsr_img_burst_in     = seq_tx_packet_ready_out;
assign cmpsr_red_burst_in     = {seq_tx_pixel_packet_out[95:88],   // R3
                                 seq_tx_pixel_packet_out[71:64],   // R2
                                 seq_tx_pixel_packet_out[47:40],   // R1
                                 seq_tx_pixel_packet_out[23:16]};  // R0
assign cmpsr_green_burst_in   = {seq_tx_pixel_packet_out[87:80],   // G3
                                 seq_tx_pixel_packet_out[63:56],   // G2
                                 seq_tx_pixel_packet_out[39:32],   // G1
                                 seq_tx_pixel_packet_out[15:8]};   // G0
assign cmpsr_blue_burst_in    = {seq_tx_pixel_packet_out[79:72],   // B3
                                 seq_tx_pixel_packet_out[55:48],   // B2
                                 seq_tx_pixel_packet_out[31:24],   // B1
                                 seq_tx_pixel_packet_out[7:0]};    // B0

    // ============================================================
    // 7-Segment Display (directly tied off - no seg7 module)
    // ============================================================
    assign seg7_an  = 8'hFF;  // All anodes off
    assign seg7_seg = 7'h7F;  // All segments off

    // ============================================================
    // LED Debug Outputs - TX Path Diagnostics
    // ============================================================
    always_comb begin
        led = 16'b0; // Default all LEDs off
        
        // TX Path Debug (most important for current issue)
        led[0] = hw_img_ready_in_SRAM;     // LED 0: Image stored in SRAM
        led[1] = seq_tx_trigger;           // LED 1: MSG_START_BURST_RD received
        led[2] = seq_tx_active_latch;      // LED 2: TX sequencer active
        led[3] = seq_tx_active_out;        // LED 3: TX sequencer frame active
        led[4] = seq_tx_packet_ready_out;  // LED 4: Pixel packet ready
        led[5] = cmpsr_tx_req_out;         // LED 5: Composer TX request
        led[6] = tx_mac_active_out;        // LED 6: TX MAC active
        led[7] = tx_phy_busy_out;          // LED 7: TX PHY busy
        
        // RX Path Debug
        led[8]  = sys_data_available;      // LED 8: Data available (CDC synced)
        led[9]  = mac_msg_valid_out;       // LED 9: Valid message from MAC
        led[10] = sys_burst_on;            // LED 10: Burst mode active
        led[11] = seq_burst_done_out;      // LED 11: RX burst sequencer done
        led[12] = rx_phy_valid_out;        // LED 12: PHY received valid byte
        led[13] = class_data_avail_out;// LED 13: Classifier output valid
        
        // Additional diagnostics
        led[14] = seq_burst_busy_out;      // LED 14: Burst sequencer busy (processing)
        led[15] = class_burst_on_out;      // LED 15: Burst ON (from classifier, pre-CDC)
    end

endmodule