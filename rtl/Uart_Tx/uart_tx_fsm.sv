// UART Transmitter FSM Module
// Handles state machine logic for UART transmission

import uart_types::*;

module uart_tx_fsm (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        tx_start,
    input  logic        baud_tick,
    output uart_state_t state,
    output logic [2:0]  bit_counter,
    output logic        tx_busy
);

    uart_state_t current_state, next_state;
    logic [2:0] bit_cnt;

    // State register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            bit_cnt <= '0;
        end else begin
            current_state <= next_state;
            
            if (baud_tick) begin
                case (current_state)
                    START_BIT: begin
                        bit_cnt <= '0;
                    end
                    DATA_BITS: begin
                        bit_cnt <= bit_cnt + 1;
                    end
                    default: begin
                        bit_cnt <= bit_cnt;
                    end
                endcase
            end
        end
    end

    // Next state logic
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (tx_start) begin
                    next_state = START_BIT;
                end
            end
            
            START_BIT: begin
                if (baud_tick) begin
                    next_state = DATA_BITS;
                end
            end
            
            DATA_BITS: begin
                if (baud_tick && bit_cnt == 7) begin
                    next_state = STOP_BIT;
                end
            end
            
            STOP_BIT: begin
                if (baud_tick) begin
                    next_state = IDLE;
                end
            end
            
            default: begin
                next_state = IDLE;
            end
        endcase
    end

    // Outputs
    assign state = current_state;
    assign bit_counter = bit_cnt;
    assign tx_busy = (current_state != IDLE);

endmodule

