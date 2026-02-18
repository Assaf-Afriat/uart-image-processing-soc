module msg_compposer (
    // Inputs
    input  logic         clk,
    input  logic         rst_n,
    
    //--- RGF Manager Interface ---
    input  logic         now_rgf_read, // Indicates that an RGF read operation is in progress
    input  logic [31:0]  rgf_raw_address, // Address being accessed in RGF
    input  logic [31:0]  rgf_data, // Data read from RGF

    //--- Image Read Single Interface ---
    input  logic         now_image_read_single, // Indicates that a single image read operation is in progress
    input  logic [13:0]  img_row_counter, // Row counter for image read
    input  logic [13:0]  img_col_counter, // Column counter for image read
    input  logic [23:0]  pixel_cell, // Pixel data for current cell

    //--- Image Read Burst Interface ---
    input  logic         now_image_read_burst, // Indicates that a burst image read operation is in progress
    input  logic [31:0]  red_burst_data, // Red channel data for burst read
    input  logic [31:0]  green_burst_data, // Green channel data for burst read
    input  logic [31:0]  blue_burst_data, // Blue channel data for burst read
    input  logic to_cmpsr_start_req,
    input  logic tx_mac_busy,

    // Outputs
    output logic [127:0] data_out_msg_cmps, // Data output from message composer
    output logic         to_mac_tx_start_req, // Start request to MAC TX module
    output logic         cmpsr_busy // Signal to indicate that message composer is busy
);

    // Internal signals
    logic [127:0] img_single_msg;
    logic [127:0] img_burst_msg;
    logic [127:0] rgf_msg;

    logic [7:0] base_address_rgf;
    logic [23:0] offset_address_rgf;

    assign to_mac_tx_start_req = to_cmpsr_start_req; // Forward the start request to MAC TX module
    // Message Composer Busy Logic
    assign cmpsr_busy = tx_mac_busy; // Message composer is busy when MAC TX is busy

    // rgf_raw_address = {8'b0, base_addr[7:0], offset[15:0]} (packed by rgf_manager)
    assign base_address_rgf = rgf_raw_address[23:16]; // Extract base address for RGF
    assign offset_address_rgf = {8'b0, rgf_raw_address[15:0]}; // Extract 16-bit offset address for RGF
    
    // Compose Image Message
    always_comb begin
       
        img_single_msg = {
                8'h7B,                          // '{' 
                8'h49,                          // 'R' 
                8'h00,                          // Padding/Separator 
                {2'b00, img_row_counter[13:8]},          // Row High Byte
                img_row_counter[7:0],           // Row Low Byte
                8'h2C,                          // ','
                8'h43,                          // 'C'
                8'h00,                          // Padding/Separator
                {2'b00, img_col_counter[13:8]},          // Column High Byte
                img_col_counter[7:0],           // Column Low Byte
                8'h2C,                          // ','
                8'h50,                          // 'P'
                pixel_cell[23:16],              // Rn
                pixel_cell[15:8],               // Gn
                pixel_cell[7:0],                // Bn
                8'h7D                           // '}' 
        };


        rgf_msg = {
                8'h7B,                          // '{'
                8'h52,                          // 'R'
                base_address_rgf,                    // Address Block
                offset_address_rgf[15:8],       // Offset High Byte
                offset_address_rgf[7:0],        // Offset Low Byte
                8'h2C,                          // ','
                8'h56,                          // 'V'
                8'h00,                          // Padding/Separator
                rgf_data[31:24],                // Data MSB
                rgf_data[23:16],
                8'h2C,                          // ','
                8'h56,                          // 'V'
                8'h00,                          // Padding/Separator
                rgf_data[15:8],
                rgf_data[7:0],                  // Data LSB
                8'h7D                           // '}'
        };

        // red_burst_data   = {R0, R1, R2, R3}  [31:24]=R0, [7:0]=R3
        // green_burst_data = {G0, G1, G2, G3}  [31:24]=G0, [7:0]=G3
        // blue_burst_data  = {B0, B1, B2, B3}  [31:24]=B0, [7:0]=B3
        // img_burst_msg format: {R0,G0,B0,R1, G1,B1,R2,G2, B2,R3,G3,B3}
        // Interleaved pixel order - read MSB ([31:24]=P0) first

        img_burst_msg = {
                8'h7B,                          // '{' - byte 0
                red_burst_data[31:24],          // R0  - byte 1
                green_burst_data[31:24],        // G0  - byte 2
                blue_burst_data[31:24],         // B0  - byte 3
                red_burst_data[23:16],          // R1  - byte 4
                8'h2C,                          // ',' - byte 5
                green_burst_data[23:16],        // G1  - byte 6
                blue_burst_data[23:16],         // B1  - byte 7
                red_burst_data[15:8],           // R2  - byte 8
                green_burst_data[15:8],         // G2  - byte 9
                8'h2C,                          // ',' - byte 10
                blue_burst_data[15:8],          // B2  - byte 11
                red_burst_data[7:0],            // R3  - byte 12
                green_burst_data[7:0],          // G3  - byte 13
                blue_burst_data[7:0],           // B3  - byte 14
                8'h7D                           // '}' - byte 15
        };

        if (now_image_read_single) begin
            data_out_msg_cmps = img_single_msg;
        end
        else if (now_image_read_burst) begin
            data_out_msg_cmps = img_burst_msg;
        end
        else if (now_rgf_read) begin
            data_out_msg_cmps = rgf_msg;
        end
        else begin
            data_out_msg_cmps = 128'h0;
        end

    end

endmodule