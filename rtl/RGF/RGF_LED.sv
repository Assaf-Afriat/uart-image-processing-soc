// -------------------------------------------------------------------------
// File: led_rgf.sv
// Description: LED Register File using Start/End bit parameters
// -------------------------------------------------------------------------

module RGF_LED #(
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
    // Width calculated as: (START - END + 1)
    output logic [1:0] hw_led_select,
    output logic       hw_led16_on_off,
    output logic       hw_led17_on_off,
    output logic       hw_led16_cie_on_off,
    output logic       hw_led17_cie_on_off,
    output logic [1:0] hw_pattern_mode
);

    // ---------------------------------------------------------------------
    // Local Parameters: Address Map
    // ---------------------------------------------------------------------
    localparam logic [ADDR_WIDTH-1:0] ADDR_LED_CTRL        = 'h0;
    localparam logic [ADDR_WIDTH-1:0] ADDR_LED_PATTERN_CFG = 'h4;

    // ---------------------------------------------------------------------
    // Local Parameters: Bit Fields (Start & End)
    // ---------------------------------------------------------------------
    
    // -- LED_CTRL Register Fields --
    localparam int LED_CTRL_SELECT_START    = 0;
    localparam int LED_CTRL_SELECT_END      = 1;
    
    localparam int LED_CTRL_L16_EN_START    = 2;
    localparam int LED_CTRL_L16_EN_END      = 2;
    
    localparam int LED_CTRL_L17_EN_START    = 3;
    localparam int LED_CTRL_L17_EN_END      = 3;

    localparam int LED_CTRL_L16_CIE_START   = 4;
    localparam int LED_CTRL_L16_CIE_END     = 4;

    localparam int LED_CTRL_L17_CIE_START   = 5;
    localparam int LED_CTRL_L17_CIE_END     = 5;

    // Reserved: Fill from MSB down to the last used bit + 1
    localparam int LED_CTRL_RES_START       = 6;
    localparam int LED_CTRL_RES_END         = DATA_WIDTH - 1; // 31

    // -- LED_Pattern_CFG Register Fields --
    localparam int PATT_CFG_MODE_START      = 0;
    localparam int PATT_CFG_MODE_END        = 1;

    localparam int PATT_CFG_RES_START       = 2;
    localparam int PATT_CFG_RES_END         = DATA_WIDTH - 1; // 31

    // -------------------------------------------------------------------------
    // Typedef Definitions
    // -------------------------------------------------------------------------
    // In packed structs, we define fields from MSB to LSB.
    // The width of each field is [START - END : 0].
    
    typedef struct packed {
        logic [LED_CTRL_RES_END - LED_CTRL_RES_START : 0]           reserved;
        logic [LED_CTRL_L17_CIE_END - LED_CTRL_L17_CIE_START : 0]   led17_cie_on_off;
        logic [LED_CTRL_L16_CIE_END - LED_CTRL_L16_CIE_START : 0]   led16_cie_on_off;
        logic [LED_CTRL_L17_EN_END - LED_CTRL_L17_EN_START : 0]     led17_on_off;
        logic [LED_CTRL_L16_EN_END - LED_CTRL_L16_EN_START : 0]     led16_on_off;
        logic [LED_CTRL_SELECT_END - LED_CTRL_SELECT_START : 0]     led_select;
    } led_ctrl_reg_t;

    typedef struct packed {
        logic [PATT_CFG_RES_END - PATT_CFG_RES_START : 0]          reserved;
        logic [PATT_CFG_MODE_END - PATT_CFG_MODE_START : 0]        pattern_mode;
    } led_pattern_cfg_reg_t;

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    led_ctrl_reg_t          r_led_ctrl;
    led_pattern_cfg_reg_t   r_led_pattern_cfg;

    // -------------------------------------------------------------------------
    // Write Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_led_ctrl        <= led_ctrl_reg_t'(0);
            r_led_pattern_cfg <= led_pattern_cfg_reg_t'(0);
        end
        else if (wr_en && addr_decoder_leg) begin
            case (addr)
                ADDR_LED_CTRL: begin
                    // Syntax: wdata[START_BIT : END_BIT]
                    // This is very readable and matches the datasheet definition directly.
                    r_led_ctrl.led_select       <= wdata[LED_CTRL_SELECT_END  : LED_CTRL_SELECT_START];
                    r_led_ctrl.led16_on_off     <= wdata[LED_CTRL_L16_EN_END  : LED_CTRL_L16_EN_START];
                    r_led_ctrl.led17_on_off     <= wdata[LED_CTRL_L17_EN_END  : LED_CTRL_L17_EN_START];
                    r_led_ctrl.led16_cie_on_off <= wdata[LED_CTRL_L16_CIE_END : LED_CTRL_L16_CIE_START];
                    r_led_ctrl.led17_cie_on_off <= wdata[LED_CTRL_L17_CIE_END : LED_CTRL_L17_CIE_START];
                end

                ADDR_LED_PATTERN_CFG: begin
                    r_led_pattern_cfg.pattern_mode <= wdata[PATT_CFG_MODE_END : PATT_CFG_MODE_START];
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
                ADDR_LED_CTRL:        rdata = 32'(r_led_ctrl);
                ADDR_LED_PATTERN_CFG: rdata = 32'(r_led_pattern_cfg);
                default:              rdata = '0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Output Assignments
    // -------------------------------------------------------------------------
    assign hw_led_select       = r_led_ctrl.led_select;
    assign hw_led16_on_off     = r_led_ctrl.led16_on_off;
    assign hw_led17_on_off     = r_led_ctrl.led17_on_off;
    assign hw_led16_cie_on_off = r_led_ctrl.led16_cie_on_off;
    assign hw_led17_cie_on_off = r_led_ctrl.led17_cie_on_off;
    
    assign hw_pattern_mode     = r_led_pattern_cfg.pattern_mode;

endmodule