// Reset Synchronizer Module
// Synchronizes an asynchronous reset signal to a destination clock domain
// 
// Features:
// - Async reset assertion (immediate)
// - Synchronized reset deassertion (2FF synchronizer)
// - Prevents metastability during reset release
//
// Usage:
//   reset_synchronizer u_rst_sync (
//       .clk_dst(clk_tx),      // Destination clock domain
//       .rst_n_async(rst_n),   // Async reset input (active low)
//       .rst_n_sync(rst_n_tx)  // Synchronized reset output (active low)
//   );

module reset_synchronizer (
    input  logic clk_dst,        // Destination clock domain
    input  logic rst_n_async,     // Async reset input (active low)
    output logic rst_n_sync       // Synchronized reset output (active low)
);

    // Two flip-flops for synchronization
    logic rst_n_ff1, rst_n_ff2;
    
    // Reset synchronizer: async assertion, synchronized deassertion
    // The reset is asserted immediately (async), but released synchronously
    always_ff @(posedge clk_dst or negedge rst_n_async) begin
        if (!rst_n_async) begin
            // Async reset assertion (immediate)
            rst_n_ff1 <= 1'b0;
            rst_n_ff2 <= 1'b0;
        end else begin
            // Synchronized reset release (2FF synchronizer)
            rst_n_ff1 <= 1'b1;      // First FF: synchronize to clk_dst
            rst_n_ff2 <= rst_n_ff1; // Second FF: remove metastability
        end
    end
    
    // Output synchronized reset
    assign rst_n_sync = rst_n_ff2;

endmodule
