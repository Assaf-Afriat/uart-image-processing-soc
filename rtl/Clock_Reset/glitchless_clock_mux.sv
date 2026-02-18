module glitchless_clock_mux (
    input  logic clk1,         
    input  logic clk2,         
    input  logic clk1_locked,  
    input  logic clk2_locked,  
    input  logic sel,          
    input  logic rst_n1,       
    input  logic rst_n2,       
    output logic clk_out,      
    output logic rst_n_out     
);
    logic q1_en, q2_en;
    logic q1_meta, q1_sync;
    logic q2_meta, q2_sync;

    // PROTECTION: Enable signals must update on NEGEDGE to avoid runt pulses
    
    // ----------------------------------------------------------------------
    // CLK 1 CONTROL (Logic runs on NEGATIVE edge)
    // ----------------------------------------------------------------------
    always_ff @(negedge clk1 or negedge rst_n1) begin
        if (!rst_n1) begin
            q1_en    <= 1'b0;
            q1_meta  <= 1'b0;
            q1_sync  <= 1'b0;
        end else begin
            q1_meta <= ~q2_en;   // Sampling the OTHER clock's enable
            q1_sync <= q1_meta;

            if (!clk1_locked) begin
                q1_en <= 1'b0;
            end else begin
                // Turn on if SEL=0 AND other clock is off
                if (!sel && q1_sync)
                    q1_en <= 1'b1;
                // Turn off if SEL=1
                else if (sel)
                    q1_en <= 1'b0;
            end
        end
    end

    // ----------------------------------------------------------------------
    // CLK 2 CONTROL (Logic runs on NEGATIVE edge)
    // ----------------------------------------------------------------------
    always_ff @(negedge clk2 or negedge rst_n2) begin
        if (!rst_n2) begin
            q2_en    <= 1'b0;
            q2_meta  <= 1'b0;
            q2_sync  <= 1'b0;
        end else begin
            q2_meta <= ~q1_en;   // Sampling the OTHER clock's enable
            q2_sync <= q2_meta;

            if (!clk2_locked) begin
                q2_en <= 1'b0;
            end else begin
                // Turn on if SEL=1 AND other clock is off
                if (sel && q2_sync)
                    q2_en <= 1'b1;
                // Turn off if SEL=0
                else if (!sel)
                    q2_en <= 1'b0;
            end
        end
    end

    logic clk1_gated, clk2_gated;
    assign clk1_gated = clk1 & q1_en;
    assign clk2_gated = clk2 & q2_en;

    assign clk_out = clk1_gated | clk2_gated;
    
    // Reset logic remains the same
    assign rst_n_out = q2_en ? rst_n2 : (q1_en ? rst_n1 : (sel ? rst_n2 : rst_n1));

endmodule