import parser_pkg::*;

module rgf_manager (
    // Inputs //gm  
    input  logic clk,
    input  logic rst_n,
    input  logic [7:0] raw_base_address,
    input  logic [15:0] raw_offset_address,
    input  logic [15:0] raw_data_high,
    input  logic [15:0] raw_data_low,
    input  msg_type_e   Msg_Type,
    input  logic [31:0] read_data_from_rgf, // Data read from RGF to be sent back to PC
    input  logic cmpsr_busy,
    input  logic now_image_read_single,
    input  logic now_image_read_burst,
    input  logic new_msg_valid,

    // Outputs
    output logic rgf_leg_img,
    output logic rgf_leg_tx_fifo,
    output logic rgf_leg_led,
    output logic rgf_leg_sys,
    output logic rgf_leg_ctrl,
    output logic rgf_leg_pwm,
    output logic [15:0] offset_address,
    output logic [31:0] wr_data,
    output logic wr_en,
    output logic rd_en,
    output logic [31:0] raw_address_to_cmpsr,
    output logic [31:0] data_out_to_cmpsr,
    output logic start_request_to_cmpsr,
    output logic rgf_read,
    output logic got_msg_from_class
);
    // ----------------------------------------------------------------------
    // Local Parameters: Address Map

    assign offset_address = raw_offset_address; 

    assign wr_data = {raw_data_high, raw_data_low}; // Data to be written to RGF from PC (zero-extended to 32-bit)

    // Write enable: asserted for one cycle when RGF write message arrives
    assign wr_en = ((Msg_Type == MSG_WRITE_RGF) && new_msg_valid) ? 1'b1 : 1'b0;

    // Read enable: asserted when RGF read message arrives AND during entire read FSM.
    // The handshake clears data_available (new_msg_valid) before the read FSM finishes,
    // so rd_en must also be driven by the active read state machine to keep RGF outputs valid.
    assign rd_en = ((Msg_Type == MSG_READ_RGF) && new_msg_valid) || (current_state_rgf != IDLE);
    
    // Handshake: acknowledge when we receive a message we handle (read or write)
    // REGISTERED acknowledgement to ensure data_available stays high long enough
    // for the system to observe it before the handshake clears it
    logic write_ack_reg;
    logic write_ack_pulse;
    
    // Detect rising edge of write condition
    assign write_ack_pulse = (Msg_Type == MSG_WRITE_RGF) && new_msg_valid;
    
    // Register the acknowledgement with a small delay
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_ack_reg <= 1'b0;
        end else begin
            write_ack_reg <= write_ack_pulse;
        end
    end

    assign raw_address_to_cmpsr = {raw_base_address, raw_offset_address}; // Forward the raw address to tx msg

    assign data_out_to_cmpsr = read_data_from_rgf; // Data read from RGF to be sent back to PC

    // decoder for RGF address
    assign rgf_leg_led  = raw_base_address == 8'h10 ? 1'b1 : 1'b0;
    assign rgf_leg_sys  = raw_base_address == 8'h20 ? 1'b1 : 1'b0;
    assign rgf_leg_ctrl = raw_base_address == 8'h30 ? 1'b1 : 1'b0;
    assign rgf_leg_pwm  = raw_base_address == 8'h40 ? 1'b1 : 1'b0;
    assign rgf_leg_img  = raw_base_address == 8'h50 ? 1'b1 : 1'b0;
    assign rgf_leg_tx_fifo = raw_base_address == 8'h60 ? 1'b1 : 1'b0;

    // ----------------------------------------------------------------------
    // State Machine for RGF Read Management
    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    // State Definition (using Enum)
    // ----------------------------------------------------------------------
    // Using enum is best for debug in waveform viewers
    typedef enum logic [2:0] {
        IDLE, // Idle state
        WAIT_READ_DN,
        WAIT_TO_START,
        WAIT_TO_FINISH
    } state_t;

    state_t current_state_rgf, next_state_rgf;

    logic [2:0] read_dn_tick; // Counter for read done ticks

    // ----------------------------------------------------------------------
    // Block 1: Sequential Logic (State Memory)
    // ----------------------------------------------------------------------
    // This block ONLY updates the current_state flip-flop.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_rgf <= IDLE;
        end else begin
            current_state_rgf <= next_state_rgf;
            case (current_state_rgf)
                IDLE: begin
                    read_dn_tick <= 3'd0; // Reset read done tick counter
                end
                WAIT_READ_DN: begin
                    read_dn_tick <= read_dn_tick + 1;
                end
                WAIT_TO_START: begin
                    read_dn_tick <= 3'd0; // Reset read done tick counter
                end
                WAIT_TO_FINISH: begin
                    read_dn_tick <= 3'd0; // Reset read done tick counter
                end
            endcase
        end
    end

    // ----------------------------------------------------------------------
    // Block 2: Next State Logic (Combinational)
    // ----------------------------------------------------------------------
    // This block calculates "what is the NEXT step?" based on inputs.
    always_comb begin
        // Default assignment prevents unintended latches
        next_state_rgf = current_state_rgf;

        case (current_state_rgf)
            IDLE: begin
                // Wait for start command
                if (Msg_Type == MSG_READ_RGF && new_msg_valid && !cmpsr_busy && !now_image_read_single && !now_image_read_burst) 
                    next_state_rgf = WAIT_READ_DN; // Move to wait for read done
                else           
                    next_state_rgf = IDLE;
            end

            WAIT_READ_DN: begin
                // If read is done, move to next state
                if (read_dn_tick == 3'd4) begin // Assuming 4 ticks for read done
                    next_state_rgf = WAIT_TO_START; // Move to wait for start
                end else begin
                    next_state_rgf = WAIT_READ_DN;
                end
            end

            WAIT_TO_START: begin
                // Wait for start signal from comparator
                if (cmpsr_busy) begin
                    next_state_rgf = WAIT_TO_FINISH; // Move to wait for finish
                end else begin
                    next_state_rgf = WAIT_TO_START;
                end
            end

            WAIT_TO_FINISH: begin
                if (!cmpsr_busy) 
                    next_state_rgf = IDLE;
                else 
                    next_state_rgf = WAIT_TO_FINISH;
            end
            default: next_state_rgf = IDLE; // Recovery
        endcase
    end

    // ----------------------------------------------------------------------
    // Block 3: Output Logic (Combinational)
    // ----------------------------------------------------------------------
    // This block defines the outputs for the PRESENT state (Moore/Mealy).
    // got_msg_from_class is asserted when:
    //   - We receive a WRITE message (immediate ack via write_ack)
    //   - We're processing a READ message (state machine ack)
    always_comb begin
        // 1. Set Defaults (CRITICAL to avoid latches)
        rgf_read               = 1'b0;
        start_request_to_cmpsr = 1'b0;
        
        // got_msg_from_class includes both:
        // - Registered write acknowledgement (1-cycle delayed)
        // - State machine read acknowledgement
        got_msg_from_class = write_ack_reg;  // Start with registered write ack

        // 2. Override based on specific states
        case (current_state_rgf)
            IDLE: begin
                rgf_read               = 1'b0;
                start_request_to_cmpsr = 1'b0;
                // write_ack_reg already included in default above
            end
    
            WAIT_READ_DN: begin
                rgf_read               = 1'b1;
                start_request_to_cmpsr = 1'b0;
                got_msg_from_class     = 1'b1;  // Read ack from state machine
            end

            WAIT_TO_START: begin
                rgf_read               = 1'b1;
                start_request_to_cmpsr = 1'b1;
            end

            WAIT_TO_FINISH: begin
                rgf_read               = 1'b1;
                start_request_to_cmpsr = 1'b0;
            end

            default: begin
                rgf_read               = 1'b0;
                start_request_to_cmpsr = 1'b0;
            end
        endcase
    end
endmodule