// -------------------------------------------------------------------------
// File: sys_rgf.sv
// Description: System Register File using Start/End bit parameters
// -------------------------------------------------------------------------

module RGF_SYS #(
    parameter int ADDR_WIDTH = 4,
    parameter int DATA_WIDTH = 32
) (
    // System Signals
    input  logic                    clk,
    input  logic                    rst_n,

    // Bus Interface
    input  logic [ADDR_WIDTH-1:0]   addr,
    input  logic                    wr_en,
    input  logic                    rd_en,
    input  logic [DATA_WIDTH-1:0]   wdata,
    input  logic                    addr_decoder_leg,
    output logic [DATA_WIDTH-1:0]   rdata,

    // Hardware Outputs
    // Width calculated as: (END - START + 1) (END is MSB, START is LSB)
    output logic         hw_sw_reset,
    output logic         hw_enable
);

    // ---------------------------------------------------------------------
    // Local Parameters: Address Map
    // ---------------------------------------------------------------------
    localparam logic [ADDR_WIDTH-1:0] ADDR_CTRL = 'h0;

    // ---------------------------------------------------------------------
    // Local Parameters: Bit Fields (Start & End)
    // ---------------------------------------------------------------------
    
    // -- CTRL Register Fields --
    localparam int CTRL_SW_RESET_START = 0;
    localparam int CTRL_SW_RESET_END   = 0;
    
    localparam int CTRL_ENABLE_START   = 1;
    localparam int CTRL_ENABLE_END     = 1;
    
    // Reserved: Fill from MSB down to the last used bit + 1
    localparam int CTRL_RES_START      = 2;
    localparam int CTRL_RES_END        = DATA_WIDTH - 1; // 31

    // -------------------------------------------------------------------------
    // Typedef Definitions
    // -------------------------------------------------------------------------
    // In packed structs, we define fields from MSB to LSB.
    // The width of each field is [END - START : 0] (END is MSB, START is LSB).
    
    typedef struct packed {
        logic [CTRL_RES_END - CTRL_RES_START : 0]          reserved;
        logic [CTRL_ENABLE_END - CTRL_ENABLE_START : 0]    enable;
        logic [CTRL_SW_RESET_END - CTRL_SW_RESET_START : 0] sw_reset;
    } ctrl_reg_t;

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    ctrl_reg_t  r_ctrl;

    // -------------------------------------------------------------------------
    // Write Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize with default values from specification
            r_ctrl.sw_reset <= 1'h0;
            r_ctrl.enable   <= 1'h0;
        end
        else if (wr_en && addr_decoder_leg) begin
            case (addr)
                ADDR_CTRL: begin
                    // Syntax: wdata[END_BIT : START_BIT] (END is MSB, START is LSB)
                    // This is very readable and matches the datasheet definition directly.
                    r_ctrl.sw_reset <= wdata[CTRL_SW_RESET_END : CTRL_SW_RESET_START];
                    r_ctrl.enable   <= wdata[CTRL_ENABLE_END : CTRL_ENABLE_START];
                end
                
                default: ;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Read Logic
    // -------------------------------------------------------------------------
    always_comb begin
        rdata = '0;
        if (rd_en && addr_decoder_leg) begin
            case (addr)
                ADDR_CTRL: rdata = 32'(r_ctrl);
                default:   rdata = '0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Output Assignments
    // -------------------------------------------------------------------------
    assign hw_sw_reset = r_ctrl.sw_reset;
    assign hw_enable   = r_ctrl.enable;

endmodule
