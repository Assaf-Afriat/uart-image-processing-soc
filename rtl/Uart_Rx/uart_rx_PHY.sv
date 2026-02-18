// ============================================================================
// UART RX PHY Layer (Physical Layer)
// Handles bit-level reception: Start bit, 8 data bits, Stop bit
// Uses 32 samples per bit with mid-bit sampling (tick 16)
// ============================================================================
//
// Error Handling:
// - On framing error: assert rx_error, increment faulty frame counter
// - Do not forward faulty frames to MAC layer
//
// ============================================================================

import uart_types::*;

module uart_rx #(
    parameter int CLK_FREQ    = 176_000_000,  // Clock frequency in Hz
    parameter int BAUD_RATE   = 5_500_000     // Baud rate
) (
    // Inputs
    input  logic        clk,           // System clock
    input  logic        rst_n,         // Active low reset
    input  logic        rx,            // Serial input
    // Outputs
    output logic [7:0]  rx_data,       // Received data
    output logic        rx_valid,      // Data valid pulse (only asserted for good frames)
    output logic        rx_error,      // Framing error (stop bit not high)
    output logic        faulty_frame_inc, // Pulse to increment faulty frame counter in RGF
    output uart_state_t rx_state       // FSM state for debugging
);

    // Timing calculation
    // CLKS_PER_BIT = CLK_FREQ / BAUD_RATE = 176,000,000 / 5,500,000 = 32 clock cycles per bit
    // Sample at the middle of the bit (clock 16)
    localparam int CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;  // 32 clocks per bit
    localparam int SAMPLE_POINT = CLKS_PER_BIT / 2;      // 16 - sample at middle
    localparam int COUNTER_WIDTH = $clog2(CLKS_PER_BIT); // 5-bit counter for 0-31
    
    // FSM state
    uart_state_t fsm_state, next_fsm_state;
    
    // Counters and registers
    logic [COUNTER_WIDTH-1:0] clk_counter;  // Counts 0 to CLKS_PER_BIT-1 (0-31)
    logic [2:0] bit_counter;                // Counts 0-7 (8 data bits)
    logic [7:0] data_reg;                   // Shift register for received data
    logic sample_tick;                      // Pulse at middle of bit (clk 16)
    
    // Synchronizer for async RX signal (triple flop)
    logic rx_sync1, rx_sync2, rx_sync3;
    
    // Edge detection
    logic rx_prev;
    logic falling_edge;
    
    // Error flags (registered)
    logic frame_error_flag;

    // ============================================================================
    // Triple Flop Synchronizer for async RX signal
    // ============================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
            rx_sync3 <= 1'b1;
        end else begin
            rx_sync1 <= rx;
            rx_sync2 <= rx_sync1;
            rx_sync3 <= rx_sync2;
        end
    end

    // ============================================================================
    // Start Bit Detection - Falling edge detection
    // ============================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_prev <= 1'b1;
            falling_edge <= 1'b0;
        end else begin
            rx_prev <= rx_sync3;
            falling_edge <= (rx_prev == 1'b1 && rx_sync3 == 1'b0);
        end
    end
    
    // Start bit valid detection
    logic start_detected;
    assign start_detected = falling_edge && (fsm_state == IDLE);

    // ============================================================================
    // Clock Counter - Counts 0 to 31 within each bit period
    // Generates sample_tick at the middle of the bit (clock 16)
    // ============================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_counter <= '0;
            sample_tick <= 1'b0;
        end else begin
            sample_tick <= 1'b0;
            
            if (start_detected) begin
                // Reset counter when start bit detected
                clk_counter <= '0;
            end else if (fsm_state == IDLE) begin
                // Keep counter at 0 while idle
                clk_counter <= '0;
            end else begin
                // Count clock cycles within each bit
                if (clk_counter == CLKS_PER_BIT - 1) begin
                    // Wrap at end of bit period
                    clk_counter <= '0;
                end else begin
                    clk_counter <= clk_counter + 1;
                end
                
                // Generate sample_tick at the middle of the bit
                if (clk_counter == SAMPLE_POINT - 1) begin
                    sample_tick <= 1'b1;
                end
            end
        end
    end

    // ============================================================================
    // FSM State Register
    // ============================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state <= IDLE;
            bit_counter <= '0;
        end else begin
            fsm_state <= next_fsm_state;
            
            // Update bit counter on sample_tick
            if (sample_tick) begin
                case (fsm_state)
                    START_BIT: begin
                        bit_counter <= '0;
                    end
                    DATA_BITS: begin
                        bit_counter <= bit_counter + 1;
                    end
                    default: begin
                        // Keep counter value
                    end
                endcase
            end
        end
    end

    // ============================================================================
    // FSM Next State Logic
    // ============================================================================
    always_comb begin
        next_fsm_state = fsm_state;
        
        case (fsm_state)
            IDLE: begin
                if (start_detected) begin
                    next_fsm_state = START_BIT;
                end
            end
            
            START_BIT: begin
                if (sample_tick) begin
                    // Verify start bit is still low
                    if (rx_sync3 == 1'b0) begin
                        next_fsm_state = DATA_BITS;
                    end else begin
                        // False start, go back to idle
                        next_fsm_state = IDLE;
                    end
                end
            end
            
            DATA_BITS: begin
                if (sample_tick && bit_counter == 7) begin
                    // After 8 data bits, go to STOP_BIT
                    next_fsm_state = STOP_BIT;
                end
            end
            
            STOP_BIT: begin
                if (sample_tick) begin
                    next_fsm_state = IDLE;
                end
            end
            
            default: begin
                next_fsm_state = IDLE;
            end
        endcase
    end

    // ============================================================================
    // Data Register and Error Detection
    // ============================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_reg <= '0;
            rx_data <= '0;
            rx_valid <= 1'b0;
            rx_error <= 1'b0;
            faulty_frame_inc <= 1'b0;
            frame_error_flag <= 1'b0;
        end else begin
            // Default: clear pulses
            rx_valid <= 1'b0;
            rx_error <= 1'b0;
            faulty_frame_inc <= 1'b0;
            
            if (sample_tick) begin
                case (fsm_state)
                    START_BIT: begin
                        // Reset error flags at start of new frame
                        frame_error_flag <= 1'b0;
                        // Check for false start
                        if (rx_sync3) begin
                            frame_error_flag <= 1'b1;
                        end
                    end
                    
                    DATA_BITS: begin
                        // Shift in received bit (LSB first)
                        data_reg <= {rx_sync3, data_reg[7:1]};
                    end
                    
                    STOP_BIT: begin
                        // Check stop bit (should be high)
                        if (!rx_sync3) begin
                            frame_error_flag <= 1'b1;
                        end
                        
                        // Determine final outcome
                        if (!frame_error_flag && rx_sync3) begin
                            // Good frame: forward to MAC
                            rx_data <= data_reg;
                            rx_valid <= 1'b1;
                        end else begin
                            // Bad frame: don't forward, signal error
                            rx_error <= 1'b1;
                            faulty_frame_inc <= 1'b1;  // Increment faulty frame counter
                        end
                    end
                    
                    default: begin
                        // Do nothing
                    end
                endcase
            end
        end
    end

    // Output state for debugging
    assign rx_state = fsm_state;

endmodule
