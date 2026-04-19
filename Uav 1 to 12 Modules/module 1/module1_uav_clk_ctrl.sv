// =============================================================================
// File        : uav_clk_ctrl.sv
// Module      : uav_clk_ctrl
// Description : UAV MOD_1 - System Clock & Control Strobes
//               Integrates PLL lock detection, CE strobe generation
//               (1 kHz, 100 Hz, 50 Hz, 10 Hz) and metastability
//               synchronizers for all asynchronous inputs.
//               Target: Xilinx Artix-7 XC7A35T @ 50 MHz
// =============================================================================

`timescale 1ns/1ps

module uav_clk_ctrl #(
    parameter int CLK_HZ        = 50_000_000,  // 50 MHz system clock
    parameter int PLL_LOCK_CNTS = 1000,        // cycles to confirm PLL lock
    parameter int SYNC_STAGES   = 2            // metastability FF stages
)(
    // Primary clock & reset
    input  logic clk,           // 50 MHz system clock from PLL output
    input  logic pll_locked,    // PLL lock indicator (async)
    input  logic ext_rst_n,     // External reset (active-low, async)

    // Asynchronous inputs to be synchronised
    input  logic arm_btn_async, // Arming button (async)
    input  logic mode_async,    // Flight mode select (async)

    // Synchronised outputs
    output logic rst_n,         // System reset (synchronised, active-low)
    output logic arm_btn_sync,  // Synchronised arm button
    output logic mode_sync,     // Synchronised mode select

    // CE strobes
    output logic ce_1khz,       // 1 kHz  – inner PID loop (MOD_2)
    output logic ce_100hz,      // 100 Hz – outer PID / EKF (MOD_2/4/5)
    output logic ce_50hz,       // 50 Hz  – attitude telemetry
    output logic ce_10hz,       // 10 Hz  – GPS / navigation

    // Status
    output logic pll_lock_stable // PLL lock confirmed for ≥ PLL_LOCK_CNTS cycles
);

    // -------------------------------------------------------------------------
    // PLL lock debounce / confirmation
    // -------------------------------------------------------------------------
    logic pll_locked_sync;
    logic [$clog2(PLL_LOCK_CNTS+1)-1:0] lock_cnt;

    // Synchronise async PLL lock signal
    synchronizer #(.STAGES(SYNC_STAGES), .WIDTH(1)) u_pll_sync (
        .dst_clk  (clk),
        .async_in (pll_locked),
        .sync_out (pll_locked_sync)
    );

    always_ff @(posedge clk or negedge ext_rst_n) begin
        if (!ext_rst_n) begin
            lock_cnt        <= '0;
            pll_lock_stable <= 1'b0;
        end else begin
            if (!pll_locked_sync) begin
                lock_cnt        <= '0;
                pll_lock_stable <= 1'b0;
            end else if (lock_cnt < PLL_LOCK_CNTS) begin
                lock_cnt        <= lock_cnt + 1'b1;
                pll_lock_stable <= 1'b0;
            end else begin
                pll_lock_stable <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Synchronised reset: deassert only when PLL is stable
    // -------------------------------------------------------------------------
    logic rst_n_int;
    assign rst_n_int = ext_rst_n & pll_lock_stable;

    // Two-stage reset synchroniser (release synchronously)
    logic rst_ff1, rst_ff2;
    always_ff @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            rst_ff1 <= 1'b0;
            rst_ff2 <= 1'b0;
        end else begin
            rst_ff1 <= 1'b1;
            rst_ff2 <= rst_ff1;
        end
    end
    assign rst_n = rst_ff2;

    // -------------------------------------------------------------------------
    // Async-input synchronisers
    // -------------------------------------------------------------------------
    synchronizer #(.STAGES(SYNC_STAGES), .WIDTH(1)) u_arm_sync (
        .dst_clk  (clk),
        .async_in (arm_btn_async),
        .sync_out (arm_btn_sync)
    );

    synchronizer #(.STAGES(SYNC_STAGES), .WIDTH(1)) u_mode_sync (
        .dst_clk  (clk),
        .async_in (mode_async),
        .sync_out (mode_sync)
    );

    // -------------------------------------------------------------------------
    // CE strobe generators (reuse tick_gen.sv)
    // -------------------------------------------------------------------------
    tick_gen #(.CLK_HZ(CLK_HZ), .TICK_HZ(1000)) u_tick_1k (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (ce_1khz)
    );

    tick_gen #(.CLK_HZ(CLK_HZ), .TICK_HZ(100)) u_tick_100 (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (ce_100hz)
    );

    tick_gen #(.CLK_HZ(CLK_HZ), .TICK_HZ(50)) u_tick_50 (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (ce_50hz)
    );

    tick_gen #(.CLK_HZ(CLK_HZ), .TICK_HZ(10)) u_tick_10 (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (ce_10hz)
    );

endmodule
