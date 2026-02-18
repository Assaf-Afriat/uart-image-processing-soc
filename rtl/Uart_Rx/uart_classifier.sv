// ============================================================================
// UART Parser + Classifier
// ============================================================================
//
// Parser (Combinational):
//   - Receives msg_data and msg_type from MAC
//   - Validates full message correctness based on msg_type
//   - Checks: digits where expected, correct field letters, value ranges
//
// Classifier (Registered):
//   - Refines message type based on address range (RGF vs Pixel)
//   - Extracts values from validated messages
//   - Holds values until sequencer consumes them
//
// Message Types:
//   1. MSG_RGF_WRITE       - {W<addr>,V<DH>,V<DL>} addr <= 31 (RGF range)
//   2. MSG_RGF_READ        - {R<addr>} short read
//   3. MSG_SINGLE_PIXEL_WR - {W<addr>,P<rgb>} addr > 31 (Pixel range)
//   4. MSG_START_BURST_WR  - {I<X>,H<H>,W<W>} start burst write
//   5. MSG_BURST_PIXEL_WR  - Pixel data during burst mode
//   6. MSG_START_BURST_RD  - {R<addr>,H<H>,W<W>} start burst read
//
// ============================================================================

import parser_pkg::*;

module uart_classifier (
    input  logic         clk,
    input  logic         rst_n,
    
    // Interface from MAC
    input  logic [127:0] msg_data,
    input  logic         msg_valid,
    input  msg_type_e    msg_type,
    
    // Parsed/Classified outputs
    output logic         valid_msg,          // Message passed validation
    output msg_type_e    classified_type,    // Validated message type
    output logic [7:0]   parsed_addr,        // Parsed address (0-255)
    output logic [15:0]  parsed_offset_addr, // Parsed offset address (raw bytes: {byte3, byte4})
    output logic [15:0]   parsed_data_high,   // High 16 bits of data (RGF write)
    output logic [15:0]   parsed_data_low,    // Low 16 bits of data (RGF write)
    output logic [31:0]   parsed_height,      // Image height (burst)
    output logic [31:0]   parsed_width,       // Image width (burst)
    
    // Single Pixel outputs (MSG_SINGLE_PIXEL_WR)
    output logic [7:0]   pixel_r,            // Pixel Red
    output logic [7:0]   pixel_g,            // Pixel Green
    output logic [7:0]   pixel_b,            // Pixel Blue
    
    // Burst Pixel outputs (MSG_BURST_PIXEL_WR) - 4 pixels per message
    // Grouped by color channel for direct SRAM writes
    output logic [31:0]  burst_red,          // {R3, R2, R1, R0} -> Red SRAM
    output logic [31:0]  burst_green,        // {G3, G2, G1, G0} -> Green SRAM
    output logic [31:0]  burst_blue,         // {B3, B2, B1, B0} -> Blue SRAM
    
    // Handshake with sequencer
    input  logic         seq_ready,          // Sequencer ready to receive
    output logic         data_available,     // Data available for sequencer
    
    // Burst mode control
    output logic         burst_on,           // Burst mode active -> to MAC
    input  logic         burst_done          // End of image from sequencer
);

    // ========================================================================
    // Extract bytes from buffer for readability
    // ========================================================================
    logic [7:0] byte0, byte1, byte2, byte3, byte4, byte5;
    logic [7:0] byte6, byte7, byte8, byte9, byte10;
    logic [7:0] byte11, byte12, byte13, byte14, byte15;
    
    assign byte0  = msg_data[7:0];      // '{'
    assign byte1  = msg_data[15:8];     // Opcode
    assign byte2  = msg_data[23:16];    // Field 1 digit 0
    assign byte3  = msg_data[31:24];    // Field 1 digit 1
    assign byte4  = msg_data[39:32];    // Field 1 digit 2
    assign byte5  = msg_data[47:40];    // ',' or '}'
    assign byte6  = msg_data[55:48];    // Field 2 letter
    assign byte7  = msg_data[63:56];    // Field 2 digit 0
    assign byte8  = msg_data[71:64];    // Field 2 digit 1
    assign byte9  = msg_data[79:72];    // Field 2 digit 2
    assign byte10 = msg_data[87:80];    // ','
    assign byte11 = msg_data[95:88];    // Field 3 letter
    assign byte12 = msg_data[103:96];   // Field 3 digit 0
    assign byte13 = msg_data[111:104];  // Field 3 digit 1
    assign byte14 = msg_data[119:112];  // Field 3 digit 2
    assign byte15 = msg_data[127:120];  // '}'

    // ========================================================================
    // PARSER: Combinational Message Validation
    // ========================================================================
    // Address fields are RAW BYTE VALUES, not ASCII digits
    // Format: {<opcode><addr><offset_hi><offset_lo>...}
    //   - addr = byte2 (8-bit address)
    //   - offset_addr = {byte3, byte4} (16-bit offset)
    
    logic parser_valid;
    logic [7:0]  p_addr;
    logic [15:0] p_offset_addr;
    logic [15:0] p_data_high, p_data_low;  // 16-bit data fields
    logic [23:0] p_height, p_width;  // 3 bytes each for height/width
    logic [7:0]  p_pixel_r, p_pixel_g, p_pixel_b;
    // Burst mode packs 4 pixels: P0(R,G,B), P1(R,G,B), P2(R,G,B), P3(R,G,B)
    logic [7:0] p_burst_p0_r, p_burst_p0_g, p_burst_p0_b, p_burst_p1_r;
    logic [7:0] p_burst_p1_g, p_burst_p1_b, p_burst_p2_r, p_burst_p2_g;
    logic [7:0] p_burst_p2_b, p_burst_p3_r, p_burst_p3_g, p_burst_p3_b;
    
    always_comb begin
        parser_valid   = 1'b0;
        p_addr         = 8'd0;
        p_offset_addr  = 16'd0;
        p_data_high    = 16'd0;
        p_data_low     = 16'd0;
        p_height       = 24'd0;
        p_width        = 24'd0;
        p_pixel_r      = 8'd0;
        p_pixel_g      = 8'd0;
        p_pixel_b      = 8'd0;
        p_burst_p0_r   = 8'd0; p_burst_p0_g = 8'd0; p_burst_p0_b = 8'd0; p_burst_p1_r = 8'd0;
        p_burst_p1_g   = 8'd0; p_burst_p1_b = 8'd0; p_burst_p2_r = 8'd0; p_burst_p2_g = 8'd0;
        p_burst_p2_b   = 8'd0; p_burst_p3_r = 8'd0; p_burst_p3_g = 8'd0; p_burst_p3_b = 8'd0;
        
        case (msg_type)
            // ================================================================
            // All address fields use RAW BYTE VALUES (not ASCII digits)
            // Format: byte2=addr[7:0], byte3=offset[15:8], byte4=offset[7:0]
            // ================================================================
            
            // ----------------------------------------------------------------
            // MSG_RGF_WRITE: {W<addr><off_hi><off_lo>,V<dh>,V<dl>}
            // Raw bytes: byte2=addr, byte3/4=offset, byte7=data_hi, byte12=data_lo
            // ----------------------------------------------------------------
            MSG_RGF_WRITE: begin
                if (byte0 == CHAR_OPEN &&
                    byte1 == OP_W &&
                    byte5 == CHAR_COMMA &&
                    byte6 == CHAR_V &&
                    byte10 == CHAR_COMMA &&
                    byte11 == CHAR_V &&
                    byte15 == CHAR_CLOSE)
                begin
                    parser_valid  = 1'b1;
                    p_addr        = byte2;                    // addr[7:0]
                    p_offset_addr = {byte3, byte4};           // offset[15:0]
                    p_data_high   = {byte7, byte8};          // data high 16 bits
                    p_data_low    = {byte12, byte13};        // data low 16 bits
                end
            end
            
            // ----------------------------------------------------------------
            // MSG_SINGLE_PIXEL_WR: {W<addr><off_hi><off_lo>,P<R><G><B>}
            // Raw bytes for pixel RGB
            // ----------------------------------------------------------------
            MSG_SINGLE_PIXEL_WR: begin
                if (byte0 == CHAR_OPEN &&
                    byte1 == OP_W &&
                    byte5 == CHAR_COMMA &&
                    byte6 == CHAR_P)
                begin
                    parser_valid  = 1'b1;
                    p_addr        = byte2;                    // addr[7:0]
                    p_offset_addr = {byte3, byte4};           // offset[15:0]
                    p_pixel_r     = byte7;                    // Red
                    p_pixel_g     = byte8;                    // Green
                    p_pixel_b     = byte9;                    // Blue
                end
            end
            
            // ----------------------------------------------------------------
            // MSG_RGF_READ: {R<addr><off_hi><off_lo>}
            // 6-byte message with raw bytes
            // ----------------------------------------------------------------
            MSG_RGF_READ: begin
                if (byte0 == CHAR_OPEN &&
                    byte1 == OP_R &&
                    byte5 == CHAR_CLOSE)
                begin
                    parser_valid  = 1'b1;
                    p_addr        = byte2;                    // addr[7:0]
                    p_offset_addr = {byte3, byte4};           // offset[15:0]
                end
            end
            
            // ----------------------------------------------------------------
            // MSG_START_BURST_WR: {I<x><x><x>,H<h2><h1><h0>,W<w2><w1><w0>}
            // Raw bytes for height and width (3 bytes each)
            // ----------------------------------------------------------------
            MSG_START_BURST_WR: begin
                if (byte0 == CHAR_OPEN &&
                    byte1 == OP_I &&
                    byte5 == CHAR_COMMA &&
                    byte6 == CHAR_H &&
                    byte10 == CHAR_COMMA &&
                    byte11 == CHAR_W &&
                    byte15 == CHAR_CLOSE)
                begin
                    parser_valid = 1'b1;
                    p_height     = {byte7, byte8, byte9};     // Height (3 bytes)
                    p_width      = {byte12, byte13, byte14};  // Width (3 bytes)
                end
            end
            
            // ----------------------------------------------------------------
            // MSG_BURST_PIXEL_WR: {<R0,G0,B0,R1>,<G1,B1,R2,G2>,<B2,R3,G3,B3>}
            // 4 pixels packed: P0(R,G,B), P1(R,G,B), P2(R,G,B), P3(R,G,B)
            // Raw byte values (not ASCII)
            // ----------------------------------------------------------------
            MSG_BURST_PIXEL_WR: begin
                if (byte0 == CHAR_OPEN &&
                    byte5 == CHAR_COMMA &&
                    byte10 == CHAR_COMMA &&
                    byte15 == CHAR_CLOSE)
                begin
                    parser_valid = 1'b1;
                    // Extract 4 pixels (raw bytes, not ASCII)
                    // Group 1: bytes 1,2,3,4 = R0,G0,B0,R1
                    p_burst_p0_r = byte1;
                    p_burst_p0_g = byte2;
                    p_burst_p0_b = byte3;
                    p_burst_p1_r = byte4;
                    // Group 2: bytes 6,7,8,9 = G1,B1,R2,G2
                    p_burst_p1_g = byte6;
                    p_burst_p1_b = byte7;
                    p_burst_p2_r = byte8;
                    p_burst_p2_g = byte9;
                    // Group 3: bytes 11,12,13,14 = B2,R3,G3,B3
                    p_burst_p2_b = byte11;
                    p_burst_p3_r = byte12;
                    p_burst_p3_g = byte13;
                    p_burst_p3_b = byte14;
                end
            end
            
            // ----------------------------------------------------------------
            // MSG_START_BURST_RD: {R<addr><off_hi><off_lo>,H<h>,W<w>}
            // Raw bytes for address, offset, height, width
            // ----------------------------------------------------------------
            MSG_START_BURST_RD: begin
                if (byte0 == CHAR_OPEN &&
                    byte1 == OP_R &&
                    byte5 == CHAR_COMMA &&
                    byte6 == CHAR_H &&
                    byte10 == CHAR_COMMA &&
                    byte11 == CHAR_W &&
                    byte15 == CHAR_CLOSE)
                begin
                    parser_valid  = 1'b1;
                    p_addr        = byte2;                    // addr[7:0]
                    p_offset_addr = {byte3, byte4};           // offset[15:0]
                    p_height      = {byte7,byte8,byte9};                    // Height
                    p_width       = {byte12,byte13,byte14};                   // Width
                end
            end
            
            default: begin
                parser_valid = 1'b0;
            end
        endcase
    end

    // ========================================================================
    // CLASSIFIER: Registered Storage
    // ========================================================================
    logic data_valid_reg; // Data valid signal for sequencer
    logic burst_on_reg;  // Burst mode active signal for MAC
    
    
    // Registered outputs for single pixel
    logic [7:0] pixel_r_reg, pixel_g_reg, pixel_b_reg;
    
    // Registered outputs for burst pixels - grouped by color channel
    logic [31:0] burst_red_reg, burst_green_reg, burst_blue_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_msg          <= 1'b0;
            classified_type    <= MSG_NONE;
            parsed_addr        <= 8'd0;
            parsed_offset_addr <= 16'd0;
            parsed_data_high   <= 16'd0;
            parsed_data_low    <= 16'd0;
            parsed_height      <= 32'd0;
            parsed_width       <= 32'd0;
            pixel_r_reg        <= 8'd0;
            pixel_g_reg        <= 8'd0;
            pixel_b_reg        <= 8'd0;
            burst_red_reg      <= 32'd0;
            burst_green_reg    <= 32'd0;
            burst_blue_reg     <= 32'd0;
            data_valid_reg     <= 1'b0;
            burst_on_reg       <= 1'b0;
        end else begin
            // Burst mode control
            // Set when MSG_START_BURST_WR is validated
            if (msg_valid && parser_valid && msg_type == MSG_START_BURST_WR) begin
                burst_on_reg <= 1'b1;
            end
            // Clear when sequencer signals end of image
            else if (burst_done) begin
                burst_on_reg <= 1'b0;
            end
            
            // Sample new message when valid from MAC and parser approves
            if (msg_valid && parser_valid) begin
                valid_msg          <= 1'b1;
                classified_type    <= msg_type;
                parsed_addr        <= p_addr;
                parsed_offset_addr <= p_offset_addr;
                parsed_data_high   <= p_data_high;
                parsed_data_low    <= p_data_low;
                parsed_height      <= {8'd0, p_height};
                parsed_width       <= {8'd0, p_width};
                
                // Single pixel RGB
                pixel_r_reg        <= p_pixel_r;
                pixel_g_reg        <= p_pixel_g;
                pixel_b_reg        <= p_pixel_b;
                
                // Burst pixels - grouped by color for SRAM writes
                burst_red_reg   <= {p_burst_p0_r, p_burst_p1_r, p_burst_p2_r, p_burst_p3_r};
                burst_green_reg <= {p_burst_p0_g, p_burst_p1_g, p_burst_p2_g, p_burst_p3_g};
                burst_blue_reg  <= {p_burst_p0_b, p_burst_p1_b, p_burst_p2_b, p_burst_p3_b};
                
                data_valid_reg   <= 1'b1;
            end
            // Clear when sequencer consumes the data
            else if (seq_ready && data_valid_reg) begin
                data_valid_reg  <= 1'b0;
                valid_msg       <= 1'b0;
            end
        end
    end
    
    assign burst_on = burst_on_reg;
    
    assign data_available = data_valid_reg;
    
    // Output assignments
    assign pixel_r     = pixel_r_reg;
    assign pixel_g     = pixel_g_reg;
    assign pixel_b     = pixel_b_reg;
    assign burst_red   = burst_red_reg;
    assign burst_green = burst_green_reg;
    assign burst_blue  = burst_blue_reg;

endmodule
