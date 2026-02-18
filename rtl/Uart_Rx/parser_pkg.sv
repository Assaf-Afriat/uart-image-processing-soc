package parser_pkg;

    // ========================================================================
    // Message Type Enum
    // ========================================================================
    // Differentiation: RGF Write uses V (Value), Single Pixel uses P (Pixel)
    typedef enum logic [3:0] {
        MSG_NONE              = 4'd0,   // No message / Idle
        MSG_RGF_WRITE         = 4'd1,   // {W<addr>,V<0,DH1,DH0>,V<0,DL1,DL0>}
        MSG_RGF_READ          = 4'd2,   // {R<addr>}
        MSG_SINGLE_PIXEL_WR   = 4'd3,   // {W<addr>,P<R,G,B>}
        MSG_START_BURST_WR    = 4'd4,   // {I<0,0,0>,H<H2,H1,H0>,W<W2,W1,W0>}
        MSG_BURST_PIXEL_WR    = 4'd5,   // {<R0,G0,B0,R1>,<G1,B1,R2,G2>,<B2,R3,G3,B3>} 4 pixels
        MSG_START_BURST_RD    = 4'd6,   // {R<addr>,H<H2,H1,H0>,W<W2,W1,W0>}
        MSG_INVALID           = 4'd15   // Invalid/malformed message
    } msg_type_e;

    // ========================================================================
    // Frame Delimiters
    // ========================================================================
    localparam logic [7:0] CHAR_OPEN  = 8'h7B;  // '{'
    localparam logic [7:0] CHAR_CLOSE = 8'h7D;  // '}'
    localparam logic [7:0] CHAR_COMMA = 8'h2C;  // ','

    // ========================================================================
    // Opcode Characters (Byte 1)
    // ========================================================================
    localparam logic [7:0] OP_W = 8'h57;  // 'W' - Register Write
    localparam logic [7:0] OP_R = 8'h52;  // 'R' - Register Read / Burst Read
    localparam logic [7:0] OP_I = 8'h49;  // 'I' - Image Burst Write

    // ========================================================================
    // Field Letter Characters
    // ========================================================================
    localparam logic [7:0] CHAR_V = 8'h56;  // 'V' - Value
    localparam logic [7:0] CHAR_H = 8'h48;  // 'H' - Height
    localparam logic [7:0] CHAR_W = 8'h57;  // 'W' - Width
    localparam logic [7:0] CHAR_P = 8'h50;  // 'P' - Pixel/PWM
    localparam logic [7:0] CHAR_C = 8'h43;  // 'C' - Column
    localparam logic [7:0] CHAR_L = 8'h4C;  // 'L' - LED

    // ========================================================================
    // Message Lengths (bytes including '{' and '}')
    // ========================================================================
    localparam int MSG_LEN_SHORT = 6;   // {R000} or {L000}
    localparam int MSG_LEN_FULL  = 16;  // {W000,V000,V000} or {I000,H000,W000}

    // ========================================================================
    // Message Type Aliases (for backward compatibility)
    // ========================================================================
    localparam msg_type_e MSG_WRITE_RGF = MSG_RGF_WRITE;
    localparam msg_type_e MSG_READ_RGF  = MSG_RGF_READ;

endpackage