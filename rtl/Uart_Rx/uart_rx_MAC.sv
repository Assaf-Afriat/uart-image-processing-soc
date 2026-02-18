// ============================================================================
// UART RX MAC Layer (Media Access Control)
// ============================================================================
//
// Receives bytes from UART RX PHY and builds/validates messages.
// Drops invalid messages immediately - only valid messages reach the parser.
//
// Normal Mode Validation:
//   Byte 0:  Wait for '{'
//   Byte 1:  Letter {W, R, I}
//   Byte 2,3,4: Don't care
//   Byte 5:  '}' (short msg end) OR ','
//   Byte 6:  Letter {V, P, C, H}
//   Byte 7,8,9: Don't care
//   Byte 10: '}' (medium msg end) OR ','
//   Byte 11: Letter {V, P, W}
//   Byte 12,13,14: Don't care
//   Byte 15: '}'
//
// Burst Mode Validation (when burst_mode active):
//   Byte 0:  '{'
//   Byte 1,2,3,4: Don't care
//   Byte 5:  ','
//   Byte 6,7,8,9: Don't care
//   Byte 10: ','
//   Byte 11,12,13,14: Don't care
//   Byte 15: '}'
//
// ============================================================================

import parser_pkg::*;

module uart_rx_MAC (
    input  logic        clk,
    input  logic        rst_n,
    
    // Interface from UART RX PHY
    input  logic [7:0]  rx_data,
    input  logic        rx_valid,
    
    // Message output to Parser
    output logic [127:0] msg_data,
    output logic         msg_valid,
    output msg_type_e    msg_type,
    
    // Burst mode control
    input  logic         burst_mode_in,    // Burst mode from classifier
    
    // Status
    output logic         frame_error
);

    // ========================================================================
    // Local Constants (field letter aliases for validation)
    // ========================================================================
    // Valid field letters at byte 6
    localparam logic [7:0] FL6_V = CHAR_V;
    localparam logic [7:0] FL6_P = CHAR_P;
    localparam logic [7:0] FL6_C = CHAR_C;
    localparam logic [7:0] FL6_H = CHAR_H;
    
    // Valid field letters at byte 11
    localparam logic [7:0] FL11_V = CHAR_V;
    localparam logic [7:0] FL11_P = CHAR_P;
    localparam logic [7:0] FL11_W = CHAR_W;

    // ========================================================================
    // FSM States
    // ========================================================================
    typedef enum logic [1:0] {
        S_IDLE,         // Waiting for '{'
        S_COLLECT,      // Collecting message bytes
        S_DONE          // Message complete
    } state_t;
    
    state_t state, next_state;
    
    // ========================================================================
    // Internal Registers
    // ========================================================================
    logic [127:0] buffer;       // Message buffer (16 bytes)
    logic [3:0]   byte_idx;     // Current byte index (0-15)
    logic [7:0]   opcode;       // Stored opcode (byte 1)
    logic         msg_ended_5;  // Message ended at byte 5
    logic         msg_ended_10; // Message ended at byte 10
    
    // ========================================================================
    // Character Classification
    // ========================================================================
    logic is_open;
    logic is_close;
    logic is_comma;
    logic is_valid_opcode;
    logic is_valid_field6;
    logic is_valid_field11;
    
    assign is_open  = (rx_data == CHAR_OPEN);
    assign is_close = (rx_data == CHAR_CLOSE);
    assign is_comma = (rx_data == CHAR_COMMA);
    
    // Close brace is only valid as a message terminator at positions 5, 10, or 15.
    // At other positions, 0x7D is treated as raw data (e.g. pixel value 125).
    logic close_at_end;
    assign close_at_end = is_close && (byte_idx == 4'd5 || byte_idx == 4'd10 || byte_idx == 4'd15);
    
    // Byte 1: W, R, I
    assign is_valid_opcode = (rx_data == OP_W) || (rx_data == OP_R) || (rx_data == OP_I);
    
    // Byte 6: V, P, C, H
    assign is_valid_field6 = (rx_data == FL6_V) || (rx_data == FL6_P) || 
                             (rx_data == FL6_C) || (rx_data == FL6_H);
    
    // Byte 11: V, P, W
    assign is_valid_field11 = (rx_data == FL11_V) || (rx_data == FL11_P) || (rx_data == FL11_W);

    // ========================================================================
    // Byte Validation
    // ========================================================================
    logic byte_ok; // Byte valid signal
    
    always_comb begin
        byte_ok = 1'b0;
        
        if (burst_mode_in) begin
            // ============================================================
            // BURST MODE: Different validation
            // ============================================================
            case (byte_idx)
                4'd0:  byte_ok = is_open;                          // '{'
                4'd1:  byte_ok = 1'b1;                             // Don't care
                4'd2:  byte_ok = 1'b1;                             // Don't care
                4'd3:  byte_ok = 1'b1;                             // Don't care
                4'd4:  byte_ok = 1'b1;                             // Don't care
                4'd5:  byte_ok = is_comma;                         // ','
                4'd6:  byte_ok = 1'b1;                             // Don't care
                4'd7:  byte_ok = 1'b1;                             // Don't care
                4'd8:  byte_ok = 1'b1;                             // Don't care
                4'd9:  byte_ok = 1'b1;                             // Don't care
                4'd10: byte_ok = is_comma;                         // ','
                4'd11: byte_ok = 1'b1;                             // Don't care
                4'd12: byte_ok = 1'b1;                             // Don't care
                4'd13: byte_ok = 1'b1;                             // Don't care
                4'd14: byte_ok = 1'b1;                             // Don't care
                4'd15: byte_ok = is_close;                         // '}'
                default: byte_ok = 1'b0;
            endcase
        end else begin
            // ============================================================
            // NORMAL MODE: Full validation
            // ============================================================
            case (byte_idx)
                4'd0:  byte_ok = is_open;                          // '{'
                4'd1:  byte_ok = is_valid_opcode;                  // W, R, I
                4'd2:  byte_ok = 1'b1;                             // Don't care
                4'd3:  byte_ok = 1'b1;                             // Don't care
                4'd4:  byte_ok = 1'b1;                             // Don't care
                4'd5:  byte_ok = is_close || is_comma;             // '}' or ','
                4'd6:  byte_ok = is_valid_field6;                  // V, P, C, H
                4'd7:  byte_ok = 1'b1;                             // Don't care
                4'd8:  byte_ok = 1'b1;                             // Don't care
                4'd9:  byte_ok = 1'b1;                             // Don't care
                4'd10: byte_ok = is_close || is_comma;             // '}' or ','
                4'd11: byte_ok = is_valid_field11;                 // V, P, W
                4'd12: byte_ok = 1'b1;                             // Don't care
                4'd13: byte_ok = 1'b1;                             // Don't care
                4'd14: byte_ok = 1'b1;                             // Don't care
                4'd15: byte_ok = is_close;                         // '}'
                default: byte_ok = 1'b0;
            endcase
        end
    end

    // ========================================================================
    // FSM State Register
    // ========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // ========================================================================
    // FSM Next State Logic
    // ========================================================================
    always_comb begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (rx_valid && is_open)
                    next_state = S_COLLECT;
            end
            
            S_COLLECT: begin
                if (rx_valid) begin
                    if (!byte_ok) begin
                        // Invalid byte
                        if (is_open)
                            next_state = S_COLLECT;  // Restart with new '{'
                        else
                            next_state = S_IDLE;     // Drop message
                    end else if (close_at_end) begin
                        // Valid close at expected position - message complete
                        next_state = S_DONE;
                    end
                    // else: continue collecting (0x7D at non-end positions is data)
                end
            end
            
            S_DONE: begin
                next_state = S_IDLE;
            end
            
            default: next_state = S_IDLE;
        endcase
    end

    // ========================================================================
    // Data Path
    // ========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buffer       <= '0;
            byte_idx     <= '0;
            opcode       <= '0;
            frame_error  <= 1'b0;
            msg_ended_5  <= 1'b0;
            msg_ended_10 <= 1'b0;
        end else begin
            frame_error <= 1'b0;
            
            case (state)
                S_IDLE: begin
                    if (rx_valid && is_open) begin
                        buffer <= '0;
                        buffer[7:0] <= rx_data;  // Store '{'
                        byte_idx <= 4'd1;
                        msg_ended_5 <= 1'b0;
                        msg_ended_10 <= 1'b0;
                    end
                end
                
                S_COLLECT: begin
                    if (rx_valid) begin
                        if (byte_ok) begin
                            // Store byte at current position
                            case (byte_idx)
                                4'd1:  begin 
                                    buffer[15:8] <= rx_data;
                                    if (!burst_mode_in) opcode <= rx_data;
                                end
                                4'd2:  buffer[23:16]   <= rx_data;
                                4'd3:  buffer[31:24]   <= rx_data;
                                4'd4:  buffer[39:32]   <= rx_data;
                                4'd5:  begin
                                    buffer[47:40] <= rx_data;
                                    if (close_at_end) msg_ended_5 <= 1'b1;
                                end
                                4'd6:  buffer[55:48]   <= rx_data;
                                4'd7:  buffer[63:56]   <= rx_data;
                                4'd8:  buffer[71:64]   <= rx_data;
                                4'd9:  buffer[79:72]   <= rx_data;
                                4'd10: begin
                                    buffer[87:80] <= rx_data;
                                    if (close_at_end) msg_ended_10 <= 1'b1;
                                end
                                4'd11: buffer[95:88]   <= rx_data;
                                4'd12: buffer[103:96]  <= rx_data;
                                4'd13: buffer[111:104] <= rx_data;
                                4'd14: buffer[119:112] <= rx_data;
                                4'd15: buffer[127:120] <= rx_data;
                                default: ;
                            endcase
                            
                            // Increment byte index (unless message ends at valid position)
                            if (!close_at_end)
                                byte_idx <= byte_idx + 1;
                                
                        end else if (is_open) begin
                            // Restart with new '{'
                            buffer <= '0;
                            buffer[7:0] <= rx_data;
                            byte_idx <= 4'd1;
                            opcode <= '0;
                            msg_ended_5 <= 1'b0;
                            msg_ended_10 <= 1'b0;
                        end else begin
                            // Invalid byte - drop message
                            frame_error <= 1'b1;
                            buffer <= '0;
                            byte_idx <= '0;
                            opcode <= '0;
                        end
                    end
                end
                
                S_DONE: begin
                    // Reset for next message
                    byte_idx <= '0;
                    opcode <= '0;
                end
                
                default: ;
            endcase
        end
    end

    // ========================================================================
    // Message Type Decode
    // ========================================================================
    // Differentiate W messages by byte 6: V=RGF Write, P=Single Pixel Write
    msg_type_e decoded_type;
    logic [7:0] field_byte6;
    
    assign field_byte6 = buffer[55:48];  // Byte 6 contains field letter
    
    always_comb begin
        decoded_type = MSG_NONE;
        
        if (burst_mode_in) begin
            // In burst mode, all messages are burst pixel data
            decoded_type = MSG_BURST_PIXEL_WR;
        end else begin
            case (opcode)
                OP_W: begin
                    // Differentiate by field letter at byte 6
                    if (field_byte6 == CHAR_V)
                        decoded_type = MSG_RGF_WRITE;       // {W<addr>,V<DH>,V<DL>}
                    else if (field_byte6 == CHAR_P)
                        decoded_type = MSG_SINGLE_PIXEL_WR; // {W<addr>,P<R,G,B>}
                    else
                        decoded_type = MSG_INVALID;
                end
                OP_I: decoded_type = MSG_START_BURST_WR;
                OP_R: begin
                    if (msg_ended_5)
                        decoded_type = MSG_RGF_READ;        // {R<addr>}
                    else
                        decoded_type = MSG_START_BURST_RD;  // {R<addr>,H<>,W<>}
                end
                default: decoded_type = MSG_INVALID;
            endcase
        end
    end

    // ========================================================================
    // Output Logic
    // ========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msg_data  <= '0;
            msg_valid <= 1'b0;
            msg_type  <= MSG_NONE;
        end else begin
            msg_valid <= 1'b0;
            
            if (state == S_DONE) begin
                msg_data  <= buffer;
                msg_valid <= 1'b1;
                msg_type  <= decoded_type;
            end
        end
    end

endmodule
