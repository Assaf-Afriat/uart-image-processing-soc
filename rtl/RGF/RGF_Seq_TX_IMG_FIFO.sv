// -------------------------------------------------------------------------
// File: fifo_rgf.sv
// Description: FIFO Register File using Start/End bit parameters
// -------------------------------------------------------------------------

module RGF_Seq_TX_IMG_FIFO #(
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
    // -- FIFO Sizes Outputs --
    output logic [11:0] hw_fifo_depth,
    output logic [5:0]  hw_single_cell_width,

    // -- Threshold LVL Outputs --
    output logic [10:0] hw_almost_empty_level,
    output logic [10:0] hw_almost_full_level
);

    // ---------------------------------------------------------------------
    // Local Parameters: Address Map
    // ---------------------------------------------------------------------
    localparam logic [ADDR_WIDTH-1:0] ADDR_FIFO_SIZES    = 'h0;
    localparam logic [ADDR_WIDTH-1:0] ADDR_THRESHOLD_LVL = 'h4;

    // ---------------------------------------------------------------------
    // Local Parameters: Bit Fields (Start & End)
    // ---------------------------------------------------------------------
    
    // -- FIFO Sizes Register Fields --
    localparam int SIZES_DEPTH_START        = 0;
    localparam int SIZES_DEPTH_END          = 11; // 12 bits

    localparam int SIZES_WIDTH_START        = 12;
    localparam int SIZES_WIDTH_END          = 17; // 6 bits

    localparam int SIZES_RES_START          = 18;
    localparam int SIZES_RES_END            = DATA_WIDTH - 1;

    // -- Threshold LVL Register Fields --
    localparam int THRESH_EMPTY_START       = 0;
    localparam int THRESH_EMPTY_END         = 10; // 11 bits

    localparam int THRESH_FULL_START        = 11;
    localparam int THRESH_FULL_END          = 21; // 11 bits

    localparam int THRESH_RES_START         = 22;
    localparam int THRESH_RES_END           = DATA_WIDTH - 1;

    // -------------------------------------------------------------------------
    // Typedef Definitions
    // -------------------------------------------------------------------------
    // In packed structs, we define fields from MSB to LSB.
    
    typedef struct packed {
        logic [SIZES_RES_END - SIZES_RES_START : 0]       reserved;
        logic [SIZES_WIDTH_END - SIZES_WIDTH_START : 0]   single_cell_width;
        logic [SIZES_DEPTH_END - SIZES_DEPTH_START : 0]   fifo_depth;
    } fifo_sizes_reg_t;

    typedef struct packed {
        logic [THRESH_RES_END - THRESH_RES_START : 0]     reserved;
        logic [THRESH_FULL_END - THRESH_FULL_START : 0]   almost_full_level;
        logic [THRESH_EMPTY_END - THRESH_EMPTY_START : 0] almost_empty_level;
    } threshold_lvl_reg_t;

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    fifo_sizes_reg_t     r_fifo_sizes;
    threshold_lvl_reg_t  r_threshold_lvl;

    // -------------------------------------------------------------------------
    // Write Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize with Default Values from the table
            
            // FIFO Sizes Defaults: Depth=0x400, Width=0x20
            r_fifo_sizes.fifo_depth        <= 12'h400; 
            r_fifo_sizes.single_cell_width <= 6'h20;
            r_fifo_sizes.reserved          <= '0;

            // Threshold Defaults: Empty=0x40, Full=0x40
            r_threshold_lvl.almost_empty_level <= 11'h40;
            r_threshold_lvl.almost_full_level  <= 11'h40;
            r_threshold_lvl.reserved           <= '0;
        end
        else if (wr_en && addr_decoder_leg) begin
            case (addr)
                ADDR_FIFO_SIZES: begin
                    r_fifo_sizes.fifo_depth        <= wdata[SIZES_DEPTH_END : SIZES_DEPTH_START];
                    r_fifo_sizes.single_cell_width <= wdata[SIZES_WIDTH_END : SIZES_WIDTH_START];
                end

                ADDR_THRESHOLD_LVL: begin
                    r_threshold_lvl.almost_empty_level <= wdata[THRESH_EMPTY_END : THRESH_EMPTY_START];
                    r_threshold_lvl.almost_full_level  <= wdata[THRESH_FULL_END  : THRESH_FULL_START];
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
                ADDR_FIFO_SIZES:    rdata = 32'(r_fifo_sizes);
                ADDR_THRESHOLD_LVL: rdata = 32'(r_threshold_lvl);
                default:            rdata = '0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Output Assignments
    // -------------------------------------------------------------------------
    // FIFO Sizes
    assign hw_fifo_depth        = r_fifo_sizes.fifo_depth;
    assign hw_single_cell_width = r_fifo_sizes.single_cell_width;

    // Threshold Levels
    assign hw_almost_empty_level = r_threshold_lvl.almost_empty_level;
    assign hw_almost_full_level  = r_threshold_lvl.almost_full_level;

endmodule