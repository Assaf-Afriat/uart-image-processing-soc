// -------------------------------------------------------------------------
// File: read_responder_fsm.sv
// Description: FSM to handle Read Response transmission
// Sends 128-bit data (16 bytes) over UART when triggered by a pulse.
// -------------------------------------------------------------------------

module uart_tx_mac_fsm (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start_tx_req,   // TRIGGER: Pulse from msg_compposer
    input  logic [127:0] read_data,     // Data to send (16 bytes)
    input  logic        tx_busy,        // UART TX busy signal
    
    output logic        tx_start,       // Start transmission pulse
    output logic [7:0]  tx_data,        // Byte to transmit
    output logic        resp_active     // High when FSM is sending data
);
    // FSM States
    typedef enum logic [2:0] {
        IDLE,
        LATCH_DATA,      // Capture the 128-bit data and reset counter
        SEND_BYTE,       // Put current byte on line and trigger TX
        WAIT_TX_DONE     // Wait for TX to finish
    } fsm_state_t;

    fsm_state_t current_state, next_state;

    // Internal signals
    logic [127:0] latched_data;
    logic [3:0]   byte_counter; // Counts 15 down to 0 (16 bytes)
    
    // Start Pulse Detection
    logic start_req_prev;
    logic start_trigger;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) start_req_prev <= 1'b0;
        else        start_req_prev <= start_tx_req;
    end
    assign start_trigger = start_tx_req && !start_req_prev;

    // ---------------------------------------------------------------------
    // Process 1: State Register & Data/Counter Path (Sequential)
    // ---------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            latched_data  <= '0;
            byte_counter  <= 4'd15; // Start from MSB (Byte 15)
        end else begin
            current_state <= next_state;
            case (current_state)
                IDLE: begin
                    // CRITICAL: Latch data immediately when trigger detected
                    // This is needed because read_data is only valid for 1 cycle
                    if (start_trigger) begin
                        latched_data <= read_data; // Capture inputs NOW
                        byte_counter <= 4'd15;     // Reset counter to MSB
                    end
                end

                LATCH_DATA: begin
                    // Data already latched in IDLE state
                    // Just keep byte_counter at 15
                end

                WAIT_TX_DONE: begin
                    // If transmission finished, decrement counter for next loop
                    if (!tx_busy) begin
                        if (byte_counter != 0) begin
                            byte_counter <= byte_counter - 1;
                        end
                    end
                end
            endcase
        end
    end

    // ---------------------------------------------------------------------
    // Process 2: Next State Logic (Combinational)
    // ---------------------------------------------------------------------
    always_comb begin
        next_state = current_state;
        case (current_state)
            IDLE: begin
                if (start_trigger) begin
                    next_state = LATCH_DATA;
                end
            end

            LATCH_DATA: begin
                next_state = SEND_BYTE;
            end

            SEND_BYTE: begin
                // Pulse generated, move to wait
                next_state = WAIT_TX_DONE;
            end

            WAIT_TX_DONE: begin
                if (!tx_busy) begin
                    // Current byte finished. Check if we have more.
                    if (byte_counter == 0) begin
                        next_state = IDLE; // All 16 bytes sent
                    end else begin
                        next_state = SEND_BYTE; // Send next byte
                    end
                end
            end

            default: next_state = IDLE;
        endcase
    end

    // ---------------------------------------------------------------------
    // Process 3: Output Logic (Combinational)
    // ---------------------------------------------------------------------
    always_comb begin
        // Defaults
        tx_start    = 1'b0;
        tx_data     = 8'h00;
        resp_active = 1'b1; // Active mostly

        case (current_state)
            IDLE: begin
                resp_active = 1'b0;
            end

            LATCH_DATA: begin
                // Latching internal registers
            end

            SEND_BYTE: begin
                // Select the byte based on the counter using Part Select
                tx_data  = latched_data[byte_counter*8 +: 8];
                tx_start = 1'b1;
            end

            WAIT_TX_DONE: begin
                // Keep data stable while waiting
                tx_data = latched_data[byte_counter*8 +: 8];
            end
            
            default: begin
                resp_active = 1'b0;
            end
        endcase
    end

endmodule