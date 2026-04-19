// =============================================================================
// File        : watchdog_timer.sv
// Module      : watchdog_timer
// Description : Hardware watchdog timer (100-2000ms, configurable).
//               Default timeout: 500ms @ 50MHz = 25,000,000 cycles.
//               Kicked at 100Hz by MOD_8 FSM.
//               Expiry → WDT_EXPIRED flag → triggers EMERGENCY.
//               Configuration only allowed in DISARMED state.
// =============================================================================

`timescale 1ns/1ps

module watchdog_timer #(
    parameter int CLK_HZ      = 50_000_000,
    parameter int DEFAULT_MS  = 500,  // 500 ms default
    parameter int MIN_MS      = 100,
    parameter int MAX_MS      = 2000
)(
    input  logic clk,
    input  logic rst_n,

    // Kick input (from FSM, 100Hz)
    input  logic kick,

    // Configuration (only valid in DISARMED)
    input  logic        config_en,   // 1 = write new timeout
    input  logic [10:0] timeout_ms,  // 100-2000 ms

    // Armed flag (config blocked when armed)
    input  logic armed,

    // Status
    output logic wdt_expired,
    output logic [31:0] wdt_cnt_out  // debug counter value
);

    localparam int DEFAULT_CNT = (CLK_HZ / 1000) * DEFAULT_MS;
    localparam int MAX_CNT     = (CLK_HZ / 1000) * MAX_MS;
    localparam int CNT_W       = $clog2(MAX_CNT + 1);

    logic [CNT_W-1:0] timeout_cnt;  // configured timeout in clock cycles
    logic [CNT_W-1:0] cnt;

    // ---- Configuration register (DISARMED only) ----------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timeout_cnt <= DEFAULT_CNT[CNT_W-1:0];
        end else if (config_en && !armed) begin
            // Clamp to valid range
            logic [10:0] ms_clamped;
            ms_clamped = (timeout_ms < MIN_MS) ? MIN_MS[10:0] :
                         (timeout_ms > MAX_MS) ? MAX_MS[10:0] : timeout_ms;
            timeout_cnt <= (CLK_HZ / 1000) * ms_clamped;
        end
    end

    // ---- Countdown / kick logic --------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt         <= '0;
            wdt_expired <= 1'b0;
        end else begin
            if (kick) begin
                cnt         <= '0;
                wdt_expired <= 1'b0;
            end else if (!wdt_expired) begin
                if (cnt >= timeout_cnt) begin
                    wdt_expired <= 1'b1;
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end
        end
    end

    assign wdt_cnt_out = {{(32-CNT_W){1'b0}}, cnt};

endmodule