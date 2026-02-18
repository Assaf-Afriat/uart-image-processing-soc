// -------------------------------------------------------------------------
// File: pwm_rgf.sv
// Description: PWM Register File using Start/End bit parameters
// -------------------------------------------------------------------------

module RGF_PWM #(
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
    output logic [DATA_WIDTH-1:0]    rdata,

    // Hardware Outputs
    // Width calculated as: (END - START + 1) (END is MSB, START is LSB)
    output logic [15:0] hw_pwm_time_slots,
    output logic [15:0]   hw_pwm_sweep_time,
    output logic [12:0]       hw_red_output_freq,
    output logic [1:0]           hw_red_magnitude,
    output logic [8:0]        hw_red_init_phase,
    output logic [12:0]  hw_green_output_freq,
    output logic [1:0]       hw_green_magnitude,
    output logic [8:0]   hw_green_init_phase,
    output logic [12:0]    hw_blue_output_freq,
    output logic [1:0]        hw_blue_magnitude,
    output logic [8:0]      hw_blue_init_phase
);

    // ---------------------------------------------------------------------
    // Local Parameters: Address Map
    // ---------------------------------------------------------------------
    localparam logic [ADDR_WIDTH-1:0] ADDR_PWM_CFG   = 'h0;
    localparam logic [ADDR_WIDTH-1:0] ADDR_RED_LUT   = 'h4;
    localparam logic [ADDR_WIDTH-1:0] ADDR_GREEN_LUT = 'h8;
    localparam logic [ADDR_WIDTH-1:0] ADDR_BLUE_LUT  = 'hC;

    // ---------------------------------------------------------------------
    // Local Parameters: Bit Fields (Start & End)
    // ---------------------------------------------------------------------
    
    // -- PWM_CFG Register Fields --
    localparam int PWM_CFG_TIME_SLOTS_START  = 0;
    localparam int PWM_CFG_TIME_SLOTS_END    = 15;
    
    localparam int PWM_CFG_SWEEP_TIME_START  = 16;
    localparam int PWM_CFG_SWEEP_TIME_END    = 31;

    // -- Red_LUT Register Fields --
    localparam int RED_LUT_OUTPUT_FREQ_START = 0;
    localparam int RED_LUT_OUTPUT_FREQ_END   = 12;
    
    localparam int RED_LUT_MAGNITUDE_START   = 13;
    localparam int RED_LUT_MAGNITUDE_END     = 14;
    
    localparam int RED_LUT_INIT_PHASE_START  = 15;
    localparam int RED_LUT_INIT_PHASE_END    = 23;
    
    // Reserved: Fill from MSB down to the last used bit + 1
    localparam int RED_LUT_RES_START         = DATA_WIDTH - 1; // 31
    localparam int RED_LUT_RES_END           = 24;

    // -- Green_LUT Register Fields --
    localparam int GREEN_LUT_OUTPUT_FREQ_START = 0;
    localparam int GREEN_LUT_OUTPUT_FREQ_END   = 12;
    
    localparam int GREEN_LUT_MAGNITUDE_START   = 13;
    localparam int GREEN_LUT_MAGNITUDE_END     = 14;
    
    localparam int GREEN_LUT_INIT_PHASE_START  = 15;
    localparam int GREEN_LUT_INIT_PHASE_END    = 23;
    
    // Reserved: Fill from MSB down to the last used bit + 1
    localparam int GREEN_LUT_RES_START          = DATA_WIDTH - 1; // 31
    localparam int GREEN_LUT_RES_END            = 24;

    // -- Blue_LUT Register Fields --
    localparam int BLUE_LUT_OUTPUT_FREQ_START = 0;
    localparam int BLUE_LUT_OUTPUT_FREQ_END   = 12;
    
    localparam int BLUE_LUT_MAGNITUDE_START   = 13;
    localparam int BLUE_LUT_MAGNITUDE_END     = 14;
    
    localparam int BLUE_LUT_INIT_PHASE_START  = 15;
    localparam int BLUE_LUT_INIT_PHASE_END    = 23;
    
    // Reserved: Fill from MSB down to the last used bit + 1
    localparam int BLUE_LUT_RES_START         = DATA_WIDTH - 1; // 31
    localparam int BLUE_LUT_RES_END            = 24;

    // -------------------------------------------------------------------------
    // Typedef Definitions
    // -------------------------------------------------------------------------
    // In packed structs, we define fields from MSB to LSB.
    // The width of each field is [END - START : 0] (END is MSB, START is LSB).
    
    typedef struct packed {
        logic [PWM_CFG_SWEEP_TIME_END - PWM_CFG_SWEEP_TIME_START : 0]    sweep_time;
        logic [PWM_CFG_TIME_SLOTS_END - PWM_CFG_TIME_SLOTS_START : 0]    time_slots;
    } pwm_cfg_reg_t;

    typedef struct packed {
        logic [RED_LUT_RES_END - RED_LUT_RES_START : 0]                   reserved;
        logic [RED_LUT_INIT_PHASE_END - RED_LUT_INIT_PHASE_START : 0]     init_phase;
        logic [RED_LUT_MAGNITUDE_END - RED_LUT_MAGNITUDE_START : 0]       magnitude;
        logic [RED_LUT_OUTPUT_FREQ_END - RED_LUT_OUTPUT_FREQ_START : 0]   output_freq;
    } red_lut_reg_t;

    typedef struct packed {
        logic [GREEN_LUT_RES_END - GREEN_LUT_RES_START : 0]               reserved;
        logic [GREEN_LUT_INIT_PHASE_END - GREEN_LUT_INIT_PHASE_START : 0] init_phase;
        logic [GREEN_LUT_MAGNITUDE_END - GREEN_LUT_MAGNITUDE_START : 0]   magnitude;
        logic [GREEN_LUT_OUTPUT_FREQ_END - GREEN_LUT_OUTPUT_FREQ_START : 0] output_freq;
    } green_lut_reg_t;

    typedef struct packed {
        logic [BLUE_LUT_RES_END - BLUE_LUT_RES_START : 0]                 reserved;
        logic [BLUE_LUT_INIT_PHASE_END - BLUE_LUT_INIT_PHASE_START : 0]   init_phase;
        logic [BLUE_LUT_MAGNITUDE_END - BLUE_LUT_MAGNITUDE_START : 0]     magnitude;
        logic [BLUE_LUT_OUTPUT_FREQ_END - BLUE_LUT_OUTPUT_FREQ_START : 0] output_freq;
    } blue_lut_reg_t;

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    pwm_cfg_reg_t    r_pwm_cfg;
    red_lut_reg_t    r_red_lut;
    green_lut_reg_t  r_green_lut;
    blue_lut_reg_t   r_blue_lut;

    // -------------------------------------------------------------------------
    // Write Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize with default values from specification
            r_pwm_cfg.time_slots  <= 16'h0000;
            r_pwm_cfg.sweep_time  <= 16'h0000;
            r_red_lut.output_freq <= 13'h000A;
            r_red_lut.magnitude   <= 2'h0;
            r_red_lut.init_phase  <= 9'h000;
            r_green_lut.output_freq <= 13'h000A;
            r_green_lut.magnitude   <= 2'h0;
            r_green_lut.init_phase  <= 9'h000;
            r_blue_lut.output_freq <= 13'h000A;
            r_blue_lut.magnitude   <= 2'h0;
            r_blue_lut.init_phase  <= 9'h000;
        end
        else if (wr_en && addr_decoder_leg) begin
            case (addr)
                ADDR_PWM_CFG: begin
                    // Syntax: wdata[END_BIT : START_BIT] (END is MSB, START is LSB)
                    // This is very readable and matches the datasheet definition directly.
                    r_pwm_cfg.time_slots <= wdata[PWM_CFG_TIME_SLOTS_END : PWM_CFG_TIME_SLOTS_START];
                    r_pwm_cfg.sweep_time <= wdata[PWM_CFG_SWEEP_TIME_END : PWM_CFG_SWEEP_TIME_START];
                end

                ADDR_RED_LUT: begin
                    r_red_lut.output_freq <= wdata[RED_LUT_OUTPUT_FREQ_END : RED_LUT_OUTPUT_FREQ_START];
                    r_red_lut.magnitude   <= wdata[RED_LUT_MAGNITUDE_END : RED_LUT_MAGNITUDE_START];
                    r_red_lut.init_phase  <= wdata[RED_LUT_INIT_PHASE_END : RED_LUT_INIT_PHASE_START];
                end

                ADDR_GREEN_LUT: begin
                    r_green_lut.output_freq <= wdata[GREEN_LUT_OUTPUT_FREQ_END : GREEN_LUT_OUTPUT_FREQ_START];
                    r_green_lut.magnitude   <= wdata[GREEN_LUT_MAGNITUDE_END : GREEN_LUT_MAGNITUDE_START];
                    r_green_lut.init_phase  <= wdata[GREEN_LUT_INIT_PHASE_END : GREEN_LUT_INIT_PHASE_START];
                end

                ADDR_BLUE_LUT: begin
                    r_blue_lut.output_freq <= wdata[BLUE_LUT_OUTPUT_FREQ_END : BLUE_LUT_OUTPUT_FREQ_START];
                    r_blue_lut.magnitude   <= wdata[BLUE_LUT_MAGNITUDE_END : BLUE_LUT_MAGNITUDE_START];
                    r_blue_lut.init_phase  <= wdata[BLUE_LUT_INIT_PHASE_END : BLUE_LUT_INIT_PHASE_START];
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
                ADDR_PWM_CFG:    rdata = 32'(r_pwm_cfg);
                ADDR_RED_LUT:    rdata = 32'(r_red_lut);
                ADDR_GREEN_LUT:  rdata = 32'(r_green_lut);
                ADDR_BLUE_LUT:   rdata = 32'(r_blue_lut);
                default:         rdata = '0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Output Assignments
    // -------------------------------------------------------------------------
    assign hw_pwm_time_slots    = r_pwm_cfg.time_slots;
    assign hw_pwm_sweep_time    = r_pwm_cfg.sweep_time;
    assign hw_red_output_freq    = r_red_lut.output_freq;
    assign hw_red_magnitude     = r_red_lut.magnitude;
    assign hw_red_init_phase    = r_red_lut.init_phase;
    assign hw_green_output_freq  = r_green_lut.output_freq;
    assign hw_green_magnitude    = r_green_lut.magnitude;
    assign hw_green_init_phase   = r_green_lut.init_phase;
    assign hw_blue_output_freq   = r_blue_lut.output_freq;
    assign hw_blue_magnitude     = r_blue_lut.magnitude;
    assign hw_blue_init_phase     = r_blue_lut.init_phase;

endmodule
