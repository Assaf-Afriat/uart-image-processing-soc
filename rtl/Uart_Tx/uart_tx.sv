// UART Transmitter Module
// Parameterizable PLL clock frequency and baud rate
// Baud divisor is calculated as: CLK_FREQ / BAUD_RATE

module uart_tx #(
    parameter int CLK_FREQ = 450_000_000,  // PLL clock frequency in Hz (default: 450 MHz)
    parameter int BAUD_RATE = 5_000_000    // Baud rate (default: 5M baud)
) (
    input  logic        clk,           // PLL clock (frequency specified by CLK_FREQ)
    input  logic        rst_n,         // Active low reset
    input  logic        tx_start,      // Start transmission
    input  logic [7:0]  tx_data,       // Data to transmit
    output logic        tx,            // Serial output
    output logic        tx_busy,       // Transmission in progress
    output uart_state_t tx_state       // FSM state for debugging
);

    // Baud rate calculation: CLK_FREQ / BAUD_RATE
    // Example: 450MHz / 5M baud = 90 (exact, no rounding error!)
    localparam int BAUD_DIVISOR = CLK_FREQ / BAUD_RATE;
    localparam int BAUD_COUNTER_WIDTH = $clog2(BAUD_DIVISOR);

    import uart_types::*;

    uart_state_t fsm_state;
    logic [BAUD_COUNTER_WIDTH-1:0] baud_counter;
    logic [2:0] bit_counter;
    logic [7:0] data_reg;
    logic baud_tick;

    // Instantiate FSM
    uart_tx_fsm u_fsm (
        .clk         (clk),
        .rst_n       (rst_n),
        .tx_start    (tx_start),
        .baud_tick   (baud_tick),
        .state       (fsm_state),
        .bit_counter (bit_counter),
        .tx_busy     (tx_busy)
    );

    // Baud rate generator
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_counter <= '0;
            baud_tick <= 1'b0;
        end else begin
            if (fsm_state != IDLE) begin
                if (baud_counter == BAUD_DIVISOR - 1) begin
                    baud_counter <= '0;
                    baud_tick <= 1'b1;
                end else begin
                    baud_counter <= baud_counter + 1;
                    baud_tick <= 1'b0;
                end
            end else begin
                baud_counter <= '0;
                baud_tick <= 1'b0;
            end
        end
    end

    // Data register and shifting logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_reg <= '0;
        end else begin
            if (tx_start && fsm_state == IDLE) begin
                data_reg <= tx_data;
            end else if (baud_tick && fsm_state == DATA_BITS) begin
                data_reg <= {1'b0, data_reg[7:1]};  // Shift right
            end
        end
    end

    // Output logic
    always_comb begin
        tx = 1'b1;  // Default idle high
        
        case (fsm_state)
            IDLE: begin
                tx = 1'b1;
            end
            
            START_BIT: begin
                tx = 1'b0;
            end
            
            DATA_BITS: begin
                tx = data_reg[0];  // LSB first
            end
            
            STOP_BIT: begin
                tx = 1'b1;
            end
            
            default: begin
                tx = 1'b1;
            end
        endcase
    end
    
    // Expose FSM state for debugging
    assign tx_state = fsm_state;

endmodule
