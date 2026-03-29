module debouncer #(
    parameter int CLK_HZ    = 50_000_000,
    parameter int SETTLE_MS = 20
)(
    input  logic clk,
    input  logic rst_n,
    input  logic noisy_in,
    output logic clean_out
);

    // ------------------------------------------------------------
    // Derived parameters
    // ------------------------------------------------------------
    localparam int COUNT_MAX = (CLK_HZ / 1000) * SETTLE_MS;
    localparam int CNT_W     = $clog2(COUNT_MAX + 1);

    // ------------------------------------------------------------
    // Internal signals
    // ------------------------------------------------------------
    logic sync_ff1, sync_ff2;     // optional 2-FF sync (recommended)
    logic stable_state;
    logic [CNT_W-1:0] counter;

    // ------------------------------------------------------------
    // Input Synchronizer (prevents metastability)
    // ------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_ff1 <= 1'b0;
            sync_ff2 <= 1'b0;
        end else begin
            sync_ff1 <= noisy_in;
            sync_ff2 <= sync_ff1;
        end
    end

    // ------------------------------------------------------------
    // Debounce Logic
    // ------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter      <= '0;
            stable_state <= 1'b0;
            clean_out    <= 1'b0;
        end else begin
            if (sync_ff2 == stable_state) begin
                // No change → reset counter
                counter <= '0;
            end else begin
                // Input different → count stability time
                if (counter == COUNT_MAX - 1) begin
                    stable_state <= sync_ff2;
                    clean_out    <= sync_ff2;
                    counter      <= '0;
                end else begin
                    counter <= counter + 1;
                end
            end
        end
    end

endmodule