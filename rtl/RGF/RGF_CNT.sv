// -------------------------------------------------------------------------
// File: cnt_rgf.sv
// Description: Counter Register File using Start/End bit parameters
// -------------------------------------------------------------------------

module RGF_CNT #(
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
    output logic [31:0]  hw_uart_rx_packets_cnt,
    output logic [31:0]  hw_uart_tx_packets_cnt,
    output logic [31:0]  hw_color_msg_cnt,
    output logic [31:0]  hw_config_msg_cnt
);

    // ---------------------------------------------------------------------
    // Local Parameters: Address Map
    // ---------------------------------------------------------------------
    localparam logic [ADDR_WIDTH-1:0] ADDR_UART_RX_PACKETS_CNT = 'h0;
    localparam logic [ADDR_WIDTH-1:0] ADDR_UART_TX_PACKETS_CNT = 'h4;
    localparam logic [ADDR_WIDTH-1:0] ADDR_COLOR_MSG_CNT       = 'h8;
    localparam logic [ADDR_WIDTH-1:0] ADDR_CONFIG_MSG_CNT      = 'hC;

    // ---------------------------------------------------------------------
    // Local Parameters: Bit Fields (Start & End)
    // ---------------------------------------------------------------------
    
    // -- UART RX Packets Counter Register Fields --
    localparam int UART_RX_PACKETS_CNT_START = 0;
    localparam int UART_RX_PACKETS_CNT_END   = 31;

    // -- UART TX Packets Counter Register Fields --
    localparam int UART_TX_PACKETS_CNT_START = 0;
    localparam int UART_TX_PACKETS_CNT_END   = 31;

    // -- Color Messages Counter Register Fields --
    localparam int COLOR_MSG_CNT_START = 0;
    localparam int COLOR_MSG_CNT_END   = 31;

    // -- Config Messages Counter Register Fields --
    localparam int CONFIG_MSG_CNT_START = 0;
    localparam int CONFIG_MSG_CNT_END   = 31;

    // -------------------------------------------------------------------------
    // Typedef Definitions
    // -------------------------------------------------------------------------
    // In packed structs, we define fields from MSB to LSB.
    // The width of each field is [END - START : 0] (END is MSB, START is LSB).
    
    typedef struct packed {
        logic [UART_RX_PACKETS_CNT_END - UART_RX_PACKETS_CNT_START : 0]  uart_rx_packets_cnt;
    } uart_rx_packets_cnt_reg_t;

    typedef struct packed {
        logic [UART_TX_PACKETS_CNT_END - UART_TX_PACKETS_CNT_START : 0]  uart_tx_packets_cnt;
    } uart_tx_packets_cnt_reg_t;

    typedef struct packed {
        logic [COLOR_MSG_CNT_END - COLOR_MSG_CNT_START : 0]  color_msg_cnt;
    } color_msg_cnt_reg_t;

    typedef struct packed {
        logic [CONFIG_MSG_CNT_END - CONFIG_MSG_CNT_START : 0]  config_msg_cnt;
    } config_msg_cnt_reg_t;

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    uart_rx_packets_cnt_reg_t  r_uart_rx_packets_cnt;
    uart_tx_packets_cnt_reg_t  r_uart_tx_packets_cnt;
    color_msg_cnt_reg_t        r_color_msg_cnt;
    config_msg_cnt_reg_t       r_config_msg_cnt;

    // -------------------------------------------------------------------------
    // Write Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize with default values from specification
            r_uart_rx_packets_cnt.uart_rx_packets_cnt <= 32'h00000000;
            r_uart_tx_packets_cnt.uart_tx_packets_cnt <= 32'h00000000;
            r_color_msg_cnt.color_msg_cnt             <= 32'h00000000;
            r_config_msg_cnt.config_msg_cnt           <= 32'h00000000;
        end
        else if (wr_en && addr_decoder_leg) begin
            case (addr)
                ADDR_UART_RX_PACKETS_CNT: begin
                    // Syntax: wdata[END_BIT : START_BIT] (END is MSB, START is LSB)
                    // This is very readable and matches the datasheet definition directly.
                    r_uart_rx_packets_cnt.uart_rx_packets_cnt <= wdata[UART_RX_PACKETS_CNT_END : UART_RX_PACKETS_CNT_START];
                end

                ADDR_UART_TX_PACKETS_CNT: begin
                    r_uart_tx_packets_cnt.uart_tx_packets_cnt <= wdata[UART_TX_PACKETS_CNT_END : UART_TX_PACKETS_CNT_START];
                end

                ADDR_COLOR_MSG_CNT: begin
                    r_color_msg_cnt.color_msg_cnt <= wdata[COLOR_MSG_CNT_END : COLOR_MSG_CNT_START];
                end

                ADDR_CONFIG_MSG_CNT: begin
                    r_config_msg_cnt.config_msg_cnt <= wdata[CONFIG_MSG_CNT_END : CONFIG_MSG_CNT_START];
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
                ADDR_UART_RX_PACKETS_CNT: rdata = 32'(r_uart_rx_packets_cnt);
                ADDR_UART_TX_PACKETS_CNT: rdata = 32'(r_uart_tx_packets_cnt);
                ADDR_COLOR_MSG_CNT:       rdata = 32'(r_color_msg_cnt);
                ADDR_CONFIG_MSG_CNT:       rdata = 32'(r_config_msg_cnt);
                default:                  rdata = '0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Output Assignments
    // -------------------------------------------------------------------------
    assign hw_uart_rx_packets_cnt = r_uart_rx_packets_cnt.uart_rx_packets_cnt;
    assign hw_uart_tx_packets_cnt = r_uart_tx_packets_cnt.uart_tx_packets_cnt;
    assign hw_color_msg_cnt        = r_color_msg_cnt.color_msg_cnt;
    assign hw_config_msg_cnt       = r_config_msg_cnt.config_msg_cnt;

endmodule
