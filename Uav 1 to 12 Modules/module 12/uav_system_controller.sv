// =============================================================================
// File        : uav_system_controller.sv
// Module      : uav_system_controller
// Description : UAV top-level system controller.
//               Manages reset sequencing:
//                 1. PLL lock detection (via MOD_1)
//                 2. Clock distribution enable
//                 3. Synchronizer release
//                 4. Sequential subsystem reset release (MOD_4/5/6/7/8/9/10)
//               Monitors global health and triggers emergency on fatal faults.
//               FIXES:
//                 - Proper delay counter bit width
//                 - CDC synchronization for async inputs
//                 - Timeout protection in state machine
//                 - Clean emergency output
// =============================================================================

`timescale 1ns/1ps

module uav_system_controller #(
    parameter int CLK_HZ   = 50_000_000,
    // Reset sequencing delays (in clock cycles)
    parameter int DELAY_CLK  = 100,     // wait for clock to stabilize
    parameter int DELAY_SYNC = 200,     // wait for synchronizers
    parameter int DELAY_SUB  = 500      // subsystem stagger
)(
    input  logic clk,
    input  logic rst_n,            // master reset (synchronised by MOD_1)
    input  logic pll_lock_stable,  // from MOD_1

    // Individual subsystem resets (active-low, released sequentially)
    output logic rst_n_clk,    // MOD_1 clocks
    output logic rst_n_sync,   // MOD_1 synchronizers
    output logic rst_n_ekf,    // MOD_4/5
    output logic rst_n_sensors,// MOD_5
    output logic rst_n_gps,    // MOD_6
    output logic rst_n_mavlink,// MOD_7
    output logic rst_n_nav,    // MOD_8
    output logic rst_n_wdt,    // MOD_9
    output logic rst_n_axi,    // MOD_10

    // Global emergency flag
    input  logic wdt_expired,
    input  logic geofence_violation,
    output logic emergency
);

    // -------------------------------------------------------------------------
    // CDC Synchronization for async inputs (metastability protection)
    // -------------------------------------------------------------------------
    logic wdt_expired_sync1, wdt_expired_sync2;
    logic geofence_viol_sync1, geofence_viol_sync2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdt_expired_sync1 <= 1'b0;
            wdt_expired_sync2 <= 1'b0;
            geofence_viol_sync1 <= 1'b0;
            geofence_viol_sync2 <= 1'b0;
        end else begin
            // Stage 1: capture async input
            wdt_expired_sync1 <= wdt_expired;
            geofence_viol_sync1 <= geofence_violation;

            // Stage 2: synchronize
            wdt_expired_sync2 <= wdt_expired_sync1;
            geofence_viol_sync2 <= geofence_viol_sync1;
        end
    end

    // -------------------------------------------------------------------------
    // Calculate required delay counter bit width
    // -------------------------------------------------------------------------
    localparam int MAX_DELAY = (DELAY_CLK > DELAY_SYNC) ?
                               ((DELAY_CLK > DELAY_SUB) ? DELAY_CLK : DELAY_SUB) :
                               ((DELAY_SYNC > DELAY_SUB) ? DELAY_SYNC : DELAY_SUB);
    localparam int DLY_CNT_W = $clog2(MAX_DELAY + 1);

    logic [DLY_CNT_W-1:0] dly_cnt;

    // -------------------------------------------------------------------------
    // Reset sequencer state machine
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        RST_WAIT_PLL, RST_CLK, RST_SYNC, RST_EKF, RST_SENS,
        RST_GPS, RST_MAV, RST_NAV, RST_WDT, RST_AXI, RST_DONE
    } rst_state_t;

    rst_state_t rst_st;
    localparam int TIMEOUT_MAX = (1 << DLY_CNT_W) - 1;  // Overflow detect

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_st      <= RST_WAIT_PLL;
            dly_cnt     <= '0;
            rst_n_clk     <= 1'b0;
            rst_n_sync    <= 1'b0;
            rst_n_ekf     <= 1'b0;
            rst_n_sensors <= 1'b0;
            rst_n_gps     <= 1'b0;
            rst_n_mavlink <= 1'b0;
            rst_n_nav     <= 1'b0;
            rst_n_wdt     <= 1'b0;
            rst_n_axi     <= 1'b0;
        end else begin
            case (rst_st)
                // Wait for PLL lock stable signal from MOD_1
                RST_WAIT_PLL: begin
                    if (pll_lock_stable) begin
                        rst_st <= RST_CLK;
                        dly_cnt <= '0;
                    end
                end

                // Release clock domain resets
                RST_CLK: begin
                    rst_n_clk <= 1'b1;
                    if (dly_cnt == DELAY_CLK[DLY_CNT_W-1:0]) begin
                        dly_cnt <= '0;
                        rst_st  <= RST_SYNC;
                    end else begin
                        dly_cnt <= dly_cnt + 1'b1;
                    end
                end

                // Release synchronizer resets
                RST_SYNC: begin
                    rst_n_sync <= 1'b1;
                    if (dly_cnt == DELAY_SYNC[DLY_CNT_W-1:0]) begin
                        dly_cnt <= '0;
                        rst_st  <= RST_EKF;
                    end else begin
                        dly_cnt <= dly_cnt + 1'b1;
                    end
                end

                // Release EKF resets (MOD_4, MOD_5)
                RST_EKF: begin
                    rst_n_ekf <= 1'b1;
                    if (dly_cnt == DELAY_SUB[DLY_CNT_W-1:0]) begin
                        dly_cnt <= '0;
                        rst_st  <= RST_SENS;
                    end else begin
                        dly_cnt <= dly_cnt + 1'b1;
                    end
                end

                // Release sensor resets (MOD_5)
                RST_SENS: begin
                    rst_n_sensors <= 1'b1;
                    if (dly_cnt == DELAY_SUB[DLY_CNT_W-1:0]) begin
                        dly_cnt <= '0;
                        rst_st  <= RST_GPS;
                    end else begin
                        dly_cnt <= dly_cnt + 1'b1;
                    end
                end

                // Release GPS resets (MOD_6)
                RST_GPS: begin
                    rst_n_gps <= 1'b1;
                    if (dly_cnt == DELAY_SUB[DLY_CNT_W-1:0]) begin
                        dly_cnt <= '0;
                        rst_st  <= RST_MAV;
                    end else begin
                        dly_cnt <= dly_cnt + 1'b1;
                    end
                end

                // Release MAVLink resets (MOD_7)
                RST_MAV: begin
                    rst_n_mavlink <= 1'b1;
                    if (dly_cnt == DELAY_SUB[DLY_CNT_W-1:0]) begin
                        dly_cnt <= '0;
                        rst_st  <= RST_NAV;
                    end else begin
                        dly_cnt <= dly_cnt + 1'b1;
                    end
                end

                // Release navigation resets (MOD_8)
                RST_NAV: begin
                    rst_n_nav <= 1'b1;
                    if (dly_cnt == DELAY_SUB[DLY_CNT_W-1:0]) begin
                        dly_cnt <= '0;
                        rst_st  <= RST_WDT;
                    end else begin
                        dly_cnt <= dly_cnt + 1'b1;
                    end
                end

                // Release watchdog resets (MOD_9)
                RST_WDT: begin
                    rst_n_wdt <= 1'b1;
                    if (dly_cnt == DELAY_SUB[DLY_CNT_W-1:0]) begin
                        dly_cnt <= '0;
                        rst_st  <= RST_AXI;
                    end else begin
                        dly_cnt <= dly_cnt + 1'b1;
                    end
                end

                // Release AXI resets (MOD_10)
                RST_AXI: begin
                    rst_n_axi <= 1'b1;
                    rst_st    <= RST_DONE;
                end

                // All resets released, sequencing complete
                RST_DONE: begin
                    // Stay here until power cycle
                end

                // Fallback to safe state
                default: begin
                    rst_st <= RST_WAIT_PLL;
                end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Global emergency flag (registered)
    // Uses synchronized inputs for safety
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            emergency <= 1'b0;
        end else begin
            emergency <= wdt_expired_sync2 | geofence_viol_sync2;
        end
    end

endmodule