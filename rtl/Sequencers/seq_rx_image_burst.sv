import parser_pkg::*;

module seq_rx_image_burst (
    // Inputs //gm  
    input  logic clk,
    input  logic rst_n,
    input  msg_type_e   Msg_Type,
    input  logic [15:0] img_height, // Image height from PC (16-bit for 256+)
    input  logic [15:0] img_width, // Image width from PC (16-bit for 256+)
    input  logic [31:0] red_burst, // Pixel data sent from PC for burst write
    input  logic [31:0] green_burst, // Pixel data sent from PC for burst write
    input  logic [31:0] blue_burst, // Pixel data sent from PC for burst write
    input  logic new_msg_valid,


    output logic [31:0] red_wr_data, // Data to be written to SRAM from PC
    output logic [31:0] green_wr_data, // Data to be written to SRAM from PC
    output logic [31:0] blue_wr_data, // Data to be written to SRAM from PC
    output logic sram_r_wr_en, // Write enable for SRAM RGB data
    output logic [13:0] sram_r_addr_wr, // SRAM address (14-bit for 16K entries)
    output logic sram_g_wr_en, // Write enable for SRAM RGB data
    output logic [13:0] sram_g_addr_wr, // SRAM address (14-bit for 16K entries)
    output logic sram_b_wr_en, // Write enable for SRAM RGB data
    output logic [13:0] sram_b_addr_wr, // SRAM address (14-bit for 16K entries)
    output logic rx_seq_burst_busy, // Signal to indicate if burst write is in progress
    output logic rx_seq_burst_dn, // Signal to indicate burst write is done
    output logic got_msg_from_class
);

    
    // ----------------------------------------------------------------------
    // State Machine for Rx Image Single Write Management
    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    // State Definition (using Enum)
    // ----------------------------------------------------------------------
    // Using enum is best for debug in waveform viewers
    typedef enum logic [3:0] {
        IDLE, // Idle state
        CLASS_HS,
        WAIT_TO_PIXEL, // Wait for pixel data from PC
        WRITE_TO_SRAM, // Write data to SRAM
        CHECK_ROW_COUNT, // Check if we've completed all rows
        CMPLTD // All pixels written
    } state_t;

    state_t current_state_rx_burst, next_state_rx_burst;

    logic [14:0] row_counter; // Count of rows written (15-bit: must reach 16384 for 256x256)

    logic Start_IMG_Write_Burst; // Flag to indicate start of image burst write
    logic New_Pixels_Write_Burst; // Flag to indicate new pixel data for burst write
    logic [31:0] max_rows; // Maximum number of rows based on image height

    // ======================================================================
    // CRITICAL FIX: Latch image dimensions from MSG_START_BURST_WR
    // ======================================================================
    // The CDC updates ALL data fields (including height/width) on EVERY message.
    // When MSG_BURST_PIXEL_WR arrives, the classifier sets height=0, width=0
    // (default values since pixel messages don't carry dimensions).
    // The CDC overwrites sys_parsed_height/width to 0, which makes max_rows=0,
    // causing the sequencer to complete after just 1 pixel message.
    //
    // Fix: Latch height/width when the START message arrives. Subsequent pixel
    // messages cannot overwrite these latched values.
    // ======================================================================
    logic [15:0] latched_img_height;
    logic [15:0] latched_img_width;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_img_height <= 16'd0;
            latched_img_width  <= 16'd0;
        end else if (Start_IMG_Write_Burst) begin
            latched_img_height <= img_height;
            latched_img_width  <= img_width;
        end
    end

    // Use LATCHED dimensions for max_rows (immune to CDC overwriting on pixel messages)
    assign max_rows = (32'(latched_img_height) * 32'(latched_img_width)) / 4;
    
    // Edge detection on new_msg_valid to prevent re-triggering
    // sys_data_available stays HIGH for several cycles due to CDC round-trip delay.
    // Without edge detection, the sequencer would re-trigger 2-3 times per message,
    // advancing the row_counter too fast and corrupting the image.
    logic new_msg_valid_d;
    logic new_msg_valid_pulse;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            new_msg_valid_d <= 1'b0;
        else
            new_msg_valid_d <= new_msg_valid;
    end
    
    assign new_msg_valid_pulse = new_msg_valid && !new_msg_valid_d;
    
    assign Start_IMG_Write_Burst = ((Msg_Type == MSG_START_BURST_WR) && new_msg_valid_pulse) ? 1'b1 : 1'b0;
    assign New_Pixels_Write_Burst = ((Msg_Type == MSG_BURST_PIXEL_WR) && new_msg_valid_pulse) ? 1'b1 : 1'b0;

    // Latch burst data when new pixel message is detected
    // This ensures data is stable for the WRITE_TO_SRAM state
    logic [31:0] red_burst_latched;
    logic [31:0] green_burst_latched;
    logic [31:0] blue_burst_latched;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            red_burst_latched   <= 32'd0;
            green_burst_latched <= 32'd0;
            blue_burst_latched  <= 32'd0;
        end else if (New_Pixels_Write_Burst) begin
            // Latch data when new pixel message arrives
            red_burst_latched   <= red_burst;
            green_burst_latched <= green_burst;
            blue_burst_latched  <= blue_burst;
        end
    end

    // Use latched data for SRAM writes
    assign red_wr_data = red_burst_latched;
    assign green_wr_data = green_burst_latched;
    assign blue_wr_data = blue_burst_latched;

    // ----------------------------------------------------------------------
    // State Machine for Rx Image Burst Write Management
    // ----------------------------------------------------------------------   

    // ----------------------------------------------------------------------
    // Block 1: Sequential Logic (State Memory)
    // ----------------------------------------------------------------------
    // This block ONLY updates the current_state flip-flop.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_rx_burst <= IDLE;
        end else begin
            current_state_rx_burst <= next_state_rx_burst;
            case (current_state_rx_burst)
                IDLE: begin
                    row_counter <= 32'd0; // Reset row counter in IDLE
                end
                WRITE_TO_SRAM: begin
                    if (row_counter < max_rows) 
                        row_counter <= row_counter + 1; // Increment row counter after writing to SRAM
                end
            endcase
        end
    end

    // ----------------------------------------------------------------------
    // Block 2: Next State Logic (Combinational)
    // ----------------------------------------------------------------------
    // This block calculates "what is the NEXT step?" based on inputs.
    always_comb begin
        // Default assignment prevents unintended latches
        next_state_rx_burst = current_state_rx_burst;

        case (current_state_rx_burst)
            IDLE: begin
                // Wait for start command
                if (Start_IMG_Write_Burst) 
                    next_state_rx_burst = CLASS_HS; // Move to wait for pixel data
                else           
                    next_state_rx_burst = IDLE;
            end

            CLASS_HS: begin
                next_state_rx_burst = WAIT_TO_PIXEL; // Move to wait for pixel data
            end

            WAIT_TO_PIXEL: begin
                // Wait for new pixel data from PC
                if (New_Pixels_Write_Burst) 
                    next_state_rx_burst = WRITE_TO_SRAM; // Move to write to SRAM
                else 
                    next_state_rx_burst = WAIT_TO_PIXEL; // Stay in wait state
            end

            WRITE_TO_SRAM: begin
                next_state_rx_burst = CHECK_ROW_COUNT; // Move to check row count after writing to SRAM
            end

            CHECK_ROW_COUNT: begin
                // Check if we've completed all rows
                if (row_counter < max_rows) 
                    next_state_rx_burst = WAIT_TO_PIXEL; // Move to wait for next pixel data
                else 
                    next_state_rx_burst = CMPLTD; // Move to completed state
            end

            CMPLTD: begin
                // All pixels written, stay in completed state
                next_state_rx_burst = IDLE; // Reset to IDLE after completion (or could stay in CMPLTD if we want to indicate done state)
            end

            default: next_state_rx_burst = IDLE; // Recovery
        endcase
    end

    // ----------------------------------------------------------------------
    // Block 3: Output Logic (Combinational)
    // ----------------------------------------------------------------------
    // This block defines the outputs for the PRESENT state (Moore/Mealy).
    always_comb begin
        // 1. Set Defaults (CRITICAL to avoid latches)


        // 2. Override based on specific states
        case (current_state_rx_burst)
            IDLE: begin
                sram_r_wr_en = 1'b0;
                sram_g_wr_en = 1'b0;
                sram_b_wr_en = 1'b0;
                sram_r_addr_wr = 14'd0;
                sram_g_addr_wr = 14'd0;
                sram_b_addr_wr = 14'd0;
                rx_seq_burst_busy = 1'b0;
                rx_seq_burst_dn = 1'b0;
                got_msg_from_class = 1'b0;
            end
            CLASS_HS: begin
                sram_r_wr_en = 1'b0;
                sram_g_wr_en = 1'b0;
                sram_b_wr_en = 1'b0;
                sram_r_addr_wr = 14'd0;
                sram_g_addr_wr = 14'd0;
                sram_b_addr_wr = 14'd0;
                rx_seq_burst_busy = 1'b1; // Indicate burst write in progress
                rx_seq_burst_dn = 1'b0;
                got_msg_from_class = 1'b1;
            end
            WAIT_TO_PIXEL: begin
                sram_r_wr_en = 1'b0;
                sram_g_wr_en = 1'b0;
                sram_b_wr_en = 1'b0;
                sram_r_addr_wr = 14'd0;
                sram_g_addr_wr = 14'd0;
                sram_b_addr_wr = 14'd0;
                rx_seq_burst_busy = 1'b1; // Indicate burst write in progress
                rx_seq_burst_dn = 1'b0;
                got_msg_from_class = 1'b0;
            end
            WRITE_TO_SRAM: begin
                sram_r_wr_en = 1'b1; // Enable write to SRAM for red data
                sram_g_wr_en = 1'b1; // Enable write to SRAM for green data
                sram_b_wr_en = 1'b1; // Enable write to SRAM for blue data
                sram_r_addr_wr = row_counter[13:0]; // 14-bit SRAM address from 15-bit counter
                sram_g_addr_wr = row_counter[13:0]; // 14-bit SRAM address from 15-bit counter
                sram_b_addr_wr = row_counter[13:0]; // 14-bit SRAM address from 15-bit counter
                rx_seq_burst_busy = 1'b1; // Indicate burst write in progress
                rx_seq_burst_dn = 1'b0;
                got_msg_from_class = 1'b1;
            end
            CHECK_ROW_COUNT: begin
                sram_r_wr_en = 1'b0;
                sram_g_wr_en = 1'b0;
                sram_b_wr_en = 1'b0;
                sram_r_addr_wr = 14'd0;
                sram_g_addr_wr = 14'd0;
                sram_b_addr_wr = 14'd0;
                rx_seq_burst_busy = 1'b1; // Indicate burst write in progress
                rx_seq_burst_dn = 1'b0;
                // Keep got_msg_from_class high for 2 cycles total (WRITE_TO_SRAM + CHECK_ROW_COUNT)
                // This ensures the 176MHz CDC captures the pulse reliably
                got_msg_from_class = 1'b1;
            end
            CMPLTD: begin
                sram_r_wr_en = 1'b0;
                sram_g_wr_en = 1'b0;
                sram_b_wr_en = 1'b0;
                sram_r_addr_wr = 14'd0;
                sram_g_addr_wr = 14'd0;
                sram_b_addr_wr = 14'd0;
                rx_seq_burst_busy = 1'b0; // Indicate burst write is done
                rx_seq_burst_dn = 1'b1; // Indicate burst write is done
                got_msg_from_class = 1'b0;
            end
            default: begin
                sram_r_wr_en = 1'b0;
                sram_g_wr_en = 1'b0;
                sram_b_wr_en = 1'b0;
                sram_r_addr_wr = 14'd0;
                sram_g_addr_wr = 14'd0;
                sram_b_addr_wr = 14'd0;
                rx_seq_burst_busy = 1'b0;
                rx_seq_burst_dn = 1'b0;
                got_msg_from_class = 1'b0;
            end
        endcase
    end
endmodule