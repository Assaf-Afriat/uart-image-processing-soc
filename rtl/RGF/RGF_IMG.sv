// -------------------------------------------------------------------------
// File: img_rgf.sv
// Description: Image Register File using Start/End bit parameters
// -------------------------------------------------------------------------

module RGF_IMG #(
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
    //s
    // -- IMG Status Outputs --
    output logic [9:0] hw_img_height,
    output logic [9:0] hw_img_width,
    output logic       hw_img_ready_in_PC,

    // -- IMG TX Monitor Outputs --
    input logic [9:0] hw_row_cnt,
    input logic [9:0] hw_col_cnt,
    input logic       hw_img_transfer_complete,
    input logic       hw_img_ready_in_SRAM,

    // -- IMG CTRL Outputs --
    output logic       hw_start_image_read
);

    // ---------------------------------------------------------------------
    // Local Parameters: Address Map
    // ---------------------------------------------------------------------
    localparam logic [ADDR_WIDTH-1:0] ADDR_IMG_STATUS     = 'h0;
    localparam logic [ADDR_WIDTH-1:0] ADDR_IMG_TX_MONITOR = 'h4;
    localparam logic [ADDR_WIDTH-1:0] ADDR_IMG_CTRL       = 'h8;

    // ---------------------------------------------------------------------
    // Local Parameters: Bit Fields (Start & End)
    // ---------------------------------------------------------------------
    
    // -- IMG Status Register Fields --
    localparam int STATUS_HEIGHT_START      = 0;
    localparam int STATUS_HEIGHT_END        = 9;  // 10 bits

    localparam int STATUS_WIDTH_START       = 10;
    localparam int STATUS_WIDTH_END         = 19; // 10 bits

    localparam int STATUS_READY_START       = 20;
    localparam int STATUS_READY_END         = 20; // 1 bit

    localparam int STATUS_RES_START         = 21;
    localparam int STATUS_RES_END           = DATA_WIDTH - 1;

    // -- IMG TX Monitor Register Fields --
    localparam int MON_ROW_CNT_START        = 0;
    localparam int MON_ROW_CNT_END          = 9;  // 10 bits

    localparam int MON_COL_CNT_START        = 10;
    localparam int MON_COL_CNT_END          = 19; // 10 bits

    localparam int MON_TRANS_COMP_START     = 20;
    localparam int MON_TRANS_COMP_END       = 20; // 1 bit

    localparam int MON_TRANS_ERR_START      = 21;
    localparam int MON_TRANS_ERR_END        = 21; // 1 bit

    localparam int MON_RES_START            = 22;
    localparam int MON_RES_END              = DATA_WIDTH - 1;

    // -- IMG CTRL Register Fields --
    localparam int CTRL_START_READ_START    = 0;
    localparam int CTRL_START_READ_END      = 0;  // 1 bit

    localparam int CTRL_RES_START           = 1;
    localparam int CTRL_RES_END             = DATA_WIDTH - 1;

    // -------------------------------------------------------------------------
    // Typedef Definitions
    // -------------------------------------------------------------------------
    // In packed structs, we define fields from MSB to LSB.
    
    typedef struct packed {
        logic [STATUS_RES_END - STATUS_RES_START : 0]           reserved;
        logic [STATUS_READY_END - STATUS_READY_START : 0]       img_ready_in_PC;
        logic [STATUS_WIDTH_END - STATUS_WIDTH_START : 0]       img_width;
        logic [STATUS_HEIGHT_END - STATUS_HEIGHT_START : 0]     img_height;
    } img_status_reg_t;

    typedef struct packed {
        logic [MON_RES_END - MON_RES_START : 0]                 reserved;
        logic [MON_TRANS_ERR_END - MON_TRANS_ERR_START : 0]     img_ready_in_SRAM;
        logic [MON_TRANS_COMP_END - MON_TRANS_COMP_START : 0]   transfer_complete;
        logic [MON_COL_CNT_END - MON_COL_CNT_START : 0]         column_cnt;
        logic [MON_ROW_CNT_END - MON_ROW_CNT_START : 0]         row_cnt;
    } img_tx_monitor_reg_t;

    typedef struct packed {
        logic [CTRL_RES_END - CTRL_RES_START : 0]               reserved;
        logic [CTRL_START_READ_END - CTRL_START_READ_START : 0] start_image_read;
    } img_ctrl_reg_t;

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    img_status_reg_t        r_img_status;
    img_tx_monitor_reg_t    r_img_tx_monitor;
    img_ctrl_reg_t          r_img_ctrl;

    // -------------------------------------------------------------------------
    // Write Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_img_status     <= img_status_reg_t'(0);
            r_img_tx_monitor <= img_tx_monitor_reg_t'(0);
            r_img_ctrl       <= img_ctrl_reg_t'(0);
        end
        else if (wr_en && addr_decoder_leg) begin
            case (addr)
                ADDR_IMG_STATUS: begin
                    r_img_status.img_height <= wdata[STATUS_HEIGHT_END : STATUS_HEIGHT_START];
                    r_img_status.img_width  <= wdata[STATUS_WIDTH_END  : STATUS_WIDTH_START];
                    r_img_status.img_ready_in_PC  <= wdata[STATUS_READY_END  : STATUS_READY_START];
                end

                ADDR_IMG_TX_MONITOR: begin
                    // Note: TX Monitor will be read-only from PC
                end

                ADDR_IMG_CTRL: begin
                    r_img_ctrl.start_image_read <= wdata[CTRL_START_READ_END : CTRL_START_READ_START];
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
                ADDR_IMG_STATUS:     rdata = 32'(r_img_status);
                ADDR_IMG_TX_MONITOR: rdata = 32'(r_img_tx_monitor);
                ADDR_IMG_CTRL:       rdata = 32'(r_img_ctrl);
                default:             rdata = '0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Output Assignments
    // -------------------------------------------------------------------------
    // IMG Status
    assign hw_img_height            = r_img_status.img_height;
    assign hw_img_width             = r_img_status.img_width;
    assign hw_img_ready_in_PC       = r_img_status.img_ready_in_PC;

    // IMG TX Monitor
    assign r_img_tx_monitor.row_cnt = hw_row_cnt;
    assign r_img_tx_monitor.column_cnt = hw_col_cnt;
    assign r_img_tx_monitor.transfer_complete = hw_img_transfer_complete;
    assign r_img_tx_monitor.img_ready_in_SRAM = hw_img_ready_in_SRAM; // Assuming img_ready_in_SRAM indicates an error for demonstration

    // IMG CTRL
    assign hw_start_image_read      = r_img_ctrl.start_image_read;

endmodule