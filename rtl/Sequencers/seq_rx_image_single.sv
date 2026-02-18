import parser_pkg::*;

module seq_rx_image_single (
    // Inputs //gm  
    input  logic clk,
    input  logic rst_n,
    input  logic [7:0] img_width, // Image width from RGF
    input  logic [7:0] img_height, // Image height from RGF
    input  msg_type_e   Msg_Type, 
    input  logic [7:0] red_pixel, // RGB pixel data from PC to be written to SRAM
    input  logic [7:0] green_pixel, // RGB pixel data from PC to be written to SRAM
    input  logic [7:0] blue_pixel, // RGB pixel data from PC to be written to SRAM
    input  logic new_msg_valid,


    // Outputs
    output logic [31:0] red_write_data, // Data to be written to SRAM from PC
    output logic [31:0] green_write_data, // Data to be written to SRAM from PC
    output logic [31:0] blue_write_data, // Data to be written to SRAM from PC
    output logic sram_r_wr_en, // Write enable for SRAM RGB data
    output logic [13:0] sram_r_addr_wr, // SRAM address (14-bit for 16K entries)
    output logic sram_g_wr_en, // Write enable for SRAM RGB data
    output logic [13:0] sram_g_addr_wr, // SRAM address (14-bit for 16K entries)
    output logic sram_b_wr_en, // Write enable for SRAM RGB data
    output logic [13:0] sram_b_addr_wr, // SRAM address (14-bit for 16K entries)
    output logic rx_seq_single_busy, // Signal to indicate if single write is in progress
    output logic rx_seq_single_dn, // Signal to indicate single write is done
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
        LOAD_BUFFER, // Load pixel data into internal buffer
        CHECK_PIXEL_COUNT, // Check if we've loaded enough pixels for one write
        CHECK_ROW_COUNT, // Check if we've completed all rows
        WRITE_TO_SRAM, // Write buffered data to SRAM
        INC_ROW, // Increment row count
        WAIT_TO_PIXEL, // Wait for next pixel data
        CMPLTD // All pixels written
    } state_t;

    state_t current_state_rx_single, next_state_rx_single;

    logic [31:0] red_buffer; // Buffer to hold red channel data for one write
    logic [31:0] green_buffer; // Buffer to hold green channel data for one write
    logic [31:0] blue_buffer; // Buffer to hold blue channel data for one write

    logic [2:0] pixel_counter; // Count of pixels loaded into buffer (max 4 for 32-bit RGB)
    logic [13:0] row_counter; // Count of rows written (14-bit for 16K max)
    logic [4:0] pixel_index_start; // Index for pixel within a row (assuming max 32 pixels per row for 32-bit RGB)

    logic New_Pixel_Sent_From_PC; // Flag to indicate new pixel data has been sent from PC
    logic [31:0] max_rows; // Maximum number of rows based on image height

    assign max_rows = (img_height*img_width)/4; // Calculate max rows based on image dimensions and 4 pixels per write (32 bits)
    assign New_Pixel_Sent_From_PC = ((Msg_Type == MSG_SINGLE_PIXEL_WR) && new_msg_valid) ? 1'b1 : 1'b0; // Set flag when new pixel data is sent from PC

    always_comb begin : pixel_index
    case (pixel_counter)
        3'd0: pixel_index_start = 5'd31;
        3'd1: pixel_index_start = 5'd23;
        3'd2: pixel_index_start = 5'd15;
        3'd3: pixel_index_start = 5'd7;
        default: begin
            pixel_index_start = 5'd0;
        end
    endcase
        
    end


    assign red_write_data = red_buffer;
    assign green_write_data = green_buffer;
    assign blue_write_data = blue_buffer;


    // ----------------------------------------------------------------------
    // State Machine for single pixel write management
    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    // Block 1: Sequential Logic (State Memory)
    // ----------------------------------------------------------------------
    // This block ONLY updates the current_state flip-flop.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_rx_single <= IDLE;
        end else begin
            current_state_rx_single <= next_state_rx_single;
            case (current_state_rx_single)
                IDLE: begin
                    row_counter <= 3'd0;
                    pixel_counter <= 3'd0;
                end
                LOAD_BUFFER: begin
                red_buffer[pixel_index_start -: 8] <= red_pixel; // Load red pixel data into buffer
                green_buffer[pixel_index_start -: 8] <= green_pixel; // Load green pixel data into buffer
                blue_buffer[pixel_index_start -: 8] <= blue_pixel; // Load blue pixel data into buffer
                pixel_counter <= pixel_counter + 1;
                end
                INC_ROW: begin
                    row_counter <= row_counter + 1;
                    pixel_counter <= 3'd0; // Reset pixel counter for new row
                end
                default: begin
                    row_counter <= row_counter;
                    pixel_counter <= pixel_counter;
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
        next_state_rx_single = current_state_rx_single;

        case (current_state_rx_single)
            IDLE: begin
                // Wait for start command
                if (New_Pixel_Sent_From_PC) 
                    next_state_rx_single = LOAD_BUFFER; // Move to load buffer
                else           
                    next_state_rx_single = IDLE;
            end
            LOAD_BUFFER: begin
                // Load pixel data into buffer
                next_state_rx_single = CHECK_PIXEL_COUNT; // Move to check pixel count
            end
            CHECK_PIXEL_COUNT: begin
                // Check if we've loaded enough pixels for one write
                if (pixel_counter < 3'd4)
                    next_state_rx_single = WAIT_TO_PIXEL; // Move to check row count
                else
                    next_state_rx_single = WRITE_TO_SRAM; // Move to write to SRAM
            end
            CHECK_ROW_COUNT: begin
                // Check if we've completed all rows
                if (row_counter < max_rows) 
                    next_state_rx_single = WAIT_TO_PIXEL; // Move to wait for next pixel
                else 
                    next_state_rx_single = CMPLTD; // Move to completed
            end
            WRITE_TO_SRAM: begin
                // Write buffered data to SRAM
                next_state_rx_single = INC_ROW; // Move to increment row count
            end
            INC_ROW: begin
                // Increment row count and reset pixel counter
                next_state_rx_single = CHECK_ROW_COUNT; // Move back to check row count
            end
            WAIT_TO_PIXEL: begin
                // Wait for next pixel data from PC
                if (New_Pixel_Sent_From_PC) 
                    next_state_rx_single = LOAD_BUFFER; // Move to load buffer
                else 
                    next_state_rx_single = WAIT_TO_PIXEL; // Stay in wait state
            end
            CMPLTD: begin
                // All pixels written, stay in completed state
                next_state_rx_single = IDLE;
            end
            default: next_state_rx_single = IDLE; // Recovery
        endcase
    end

    // ----------------------------------------------------------------------
    // Block 3: Output Logic
    // ----------------------------------------------------------------------
    always_comb begin
        // 1. Set Defaults to prevent latches
        sram_r_wr_en       = 1'b0;
        sram_g_wr_en       = 1'b0;
        sram_b_wr_en       = 1'b0;
        sram_r_addr_wr     = 14'd0;
        sram_g_addr_wr     = 14'd0;
        sram_b_addr_wr     = 14'd0;
        rx_seq_single_busy = 1'b0;
        rx_seq_single_dn   = 1'b0;
        got_msg_from_class = 1'b0;

        // 2. Override based on specific states
        case (current_state_rx_single)
            IDLE: begin
                // All signals are 0 (default)
            end
    
            LOAD_BUFFER: begin
                rx_seq_single_busy = 1'b1;
                got_msg_from_class = 1'b1;
            end

            CHECK_PIXEL_COUNT, CHECK_ROW_COUNT, WAIT_TO_PIXEL: begin
                // Keep busy signal high during wait/check states
                rx_seq_single_busy = 1'b1;
            end

            WRITE_TO_SRAM: begin
                sram_r_wr_en       = 1'b1;
                sram_g_wr_en       = 1'b1;
                sram_b_wr_en       = 1'b1;
                sram_r_addr_wr     = row_counter;
                sram_g_addr_wr     = row_counter;
                sram_b_addr_wr     = row_counter;
                rx_seq_single_busy = 1'b1;
            end

            INC_ROW: begin
                rx_seq_single_busy = 1'b1;
            end

            CMPLTD: begin
                rx_seq_single_dn   = 1'b1;
            end
            
            default: begin
                // Defaults apply
            end
        endcase
    end
endmodule