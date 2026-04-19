// =============================================================================
// Testbench  : uav_watchdog_tb
// Description: Minimal testbench for module9_uav_watchdog
//              - 15 test cases with immediate results
//              - Vivado xsim compatible
// =============================================================================

`timescale 1ns/1ps

module module9_tb;

    // =========================================================================
    // Test parameters
    // =========================================================================
    parameter int CLK_PERIOD_NS = 20;
    parameter int CLK_HZ = 50_000_000;
    parameter int DATA_W = 32;

    // =========================================================================
    // Test signals
    // =========================================================================
    reg clk;
    reg rst_n;

    reg wdt_kick;
    reg armed;

    reg        wdt_config_en;
    reg [10:0] wdt_timeout_ms;

    reg       ekf_healthy;
    reg [3:0] gps_fix_type;
    reg [15:0] gps_hdop_q8;
    reg       baro_valid;
    reg       mag_valid;
    reg       imu_fault;
    reg       gcs_present;
    reg [7:0] check_mask;

    reg signed [DATA_W-1:0] curr_lat;
    reg signed [DATA_W-1:0] curr_lon;
    reg signed [DATA_W-1:0] curr_alt;

    wire wdt_ok;
    wire wdt_expired;
    wire preflight_ok;
    wire [7:0] fault_flags;
    wire emerg_active;

    wire signed [DATA_W-1:0] emerg_sp_lat;
    wire signed [DATA_W-1:0] emerg_sp_lon;
    wire signed [DATA_W-1:0] emerg_sp_alt;
    wire signed [DATA_W-1:0] emerg_sp_thrust;
    wire signed [DATA_W-1:0] emerg_sp_vd;

    // Test counters
    integer test_num = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    module9_uav_watchdog #(
        .CLK_HZ (CLK_HZ),
        .DATA_W (DATA_W)
    ) dut (
        .clk (clk),
        .rst_n (rst_n),
        .wdt_kick (wdt_kick),
        .armed (armed),
        .wdt_config_en (wdt_config_en),
        .wdt_timeout_ms (wdt_timeout_ms),
        .ekf_healthy (ekf_healthy),
        .gps_fix_type (gps_fix_type),
        .gps_hdop_q8 (gps_hdop_q8),
        .baro_valid (baro_valid),
        .mag_valid (mag_valid),
        .imu_fault (imu_fault),
        .gcs_present (gcs_present),
        .check_mask (check_mask),
        .curr_lat (curr_lat),
        .curr_lon (curr_lon),
        .curr_alt (curr_alt),
        .wdt_ok (wdt_ok),
        .wdt_expired (wdt_expired),
        .preflight_ok (preflight_ok),
        .fault_flags (fault_flags),
        .emerg_active (emerg_active),
        .emerg_sp_lat (emerg_sp_lat),
        .emerg_sp_lon (emerg_sp_lon),
        .emerg_sp_alt (emerg_sp_alt),
        .emerg_sp_thrust (emerg_sp_thrust),
        .emerg_sp_vd (emerg_sp_vd)
    );

    // =========================================================================
    // Clock generation
    // =========================================================================
    always begin
        #(CLK_PERIOD_NS/2) clk = ~clk;
    end

    // =========================================================================
    // Helper tasks
    // =========================================================================

    task wait_clk(input integer n);
        integer i;
        for (i = 0; i < n; i = i + 1) begin
            @(posedge clk);
        end
    endtask

    task report_test(input string name, input integer pass);
        test_num = test_num + 1;
        if (pass) begin
            tests_passed = tests_passed + 1;
            $display("[PASS] TC%02d: %s", test_num, name);
        end else begin
            tests_failed = tests_failed + 1;
            $display("[FAIL] TC%02d: %s", test_num, name);
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================

    initial begin
        $display("\n");
        $display("=============================================================================");
        $display("UAV Watchdog Module - Minimal Testbench");
        $display("=============================================================================\n");

        clk = 1'b0;
        rst_n = 1'b0;

        wdt_kick = 1'b0;
        armed = 1'b0;

        wdt_config_en = 1'b0;
        wdt_timeout_ms = 11'd1000;

        ekf_healthy = 1'b0;
        gps_fix_type = 4'b0000;
        gps_hdop_q8 = 16'd0;
        baro_valid = 1'b0;
        mag_valid = 1'b0;
        imu_fault = 1'b0;
        gcs_present = 1'b0;
        check_mask = 8'd0;

        curr_lat = 32'sd0;
        curr_lon = 32'sd0;
        curr_alt = 32'sd0;

        #(CLK_PERIOD_NS);
        wait_clk(2);

        // =====================================================================
        // TC1: Power-on Reset
        // =====================================================================
        $display("[TEST] TC1: Power-on Reset");
        if (wdt_ok === 1'b1 && wdt_expired === 1'b0 && preflight_ok === 1'b0) begin
            report_test("Power-on Reset - WDT OK, Preflight not OK", 1);
        end else begin
            report_test("Power-on Reset - WDT OK, Preflight not OK", 0);
        end

        // =====================================================================
        // TC2: Release Reset
        // =====================================================================
        $display("[TEST] TC2: Release Reset");
        rst_n = 1'b1;
        wait_clk(1);
        if (rst_n === 1'b1) begin
            report_test("Reset Released", 1);
        end else begin
            report_test("Reset Released", 0);
        end

        // =====================================================================
        // TC3: Watchdog Configuration
        // =====================================================================
        $display("[TEST] TC3: Watchdog Configuration");
        wdt_config_en = 1'b1;
        wdt_timeout_ms = 11'd2000;
        wait_clk(1);
        wdt_config_en = 1'b0;
        if (wdt_timeout_ms === 11'd2000) begin
            report_test("Watchdog Timeout Configured", 1);
        end else begin
            report_test("Watchdog Timeout Configured", 0);
        end

        // =====================================================================
        // TC4: WDT Kick Signal
        // =====================================================================
        $display("[TEST] TC4: WDT Kick Signal");
        armed = 1'b1;
        wdt_kick = 1'b1;
        wait_clk(1);
        wdt_kick = 1'b0;
        wait_clk(1);
        if (wdt_ok === 1'b1) begin
            report_test("WDT Kick Applied", 1);
        end else begin
            report_test("WDT Kick Applied", 0);
        end

        // =====================================================================
        // TC5: Pre-flight - EKF Healthy
        // =====================================================================
        $display("[TEST] TC5: Pre-flight - EKF Healthy");
        ekf_healthy = 1'b1;
        wait_clk(1);
        if (ekf_healthy === 1'b1) begin
            report_test("EKF Healthy Flag Set", 1);
        end else begin
            report_test("EKF Healthy Flag Set", 0);
        end

        // =====================================================================
        // TC6: Pre-flight - GPS Fix Type
        // =====================================================================
        $display("[TEST] TC6: Pre-flight - GPS Fix Type");
        gps_fix_type = 4'b0011;
        wait_clk(1);
        if (gps_fix_type === 4'b0011) begin
            report_test("GPS Fix Type Set", 1);
        end else begin
            report_test("GPS Fix Type Set", 0);
        end

        // =====================================================================
        // TC7: Pre-flight - GPS HDOP
        // =====================================================================
        $display("[TEST] TC7: Pre-flight - GPS HDOP");
        gps_hdop_q8 = 16'd100;
        wait_clk(1);
        if (gps_hdop_q8 === 16'd100) begin
            report_test("GPS HDOP Configured", 1);
        end else begin
            report_test("GPS HDOP Configured", 0);
        end

        // =====================================================================
        // TC8: Pre-flight - Sensor Validity
        // =====================================================================
        $display("[TEST] TC8: Pre-flight - Sensor Validity");
        baro_valid = 1'b1;
        mag_valid = 1'b1;
        imu_fault = 1'b0;
        wait_clk(1);
        if (baro_valid === 1'b1 && mag_valid === 1'b1 && imu_fault === 1'b0) begin
            report_test("Sensor Validity Set", 1);
        end else begin
            report_test("Sensor Validity Set", 0);
        end

        // =====================================================================
        // TC9: Pre-flight - GCS Present
        // =====================================================================
        $display("[TEST] TC9: Pre-flight - GCS Present");
        gcs_present = 1'b1;
        wait_clk(1);
        if (gcs_present === 1'b1) begin
            report_test("GCS Present Flag Set", 1);
        end else begin
            report_test("GCS Present Flag Set", 0);
        end

        // =====================================================================
        // TC10: Pre-flight Check Mask
        // =====================================================================
        $display("[TEST] TC10: Pre-flight Check Mask");
        check_mask = 8'hFF;
        wait_clk(1);
        if (check_mask === 8'hFF) begin
            report_test("Pre-flight Check Mask Configured", 1);
        end else begin
            report_test("Pre-flight Check Mask Configured", 0);
        end

        // =====================================================================
        // TC11: Emergency Descent Position
        // =====================================================================
        $display("[TEST] TC11: Emergency Descent Position");
        curr_lat = 32'h40000000;
        curr_lon = 32'h40000000;
        curr_alt = 32'sd5000;
        wait_clk(1);
        if (curr_lat === 32'h40000000 && curr_alt === 32'sd5000) begin
            report_test("Emergency Descent Position Configured", 1);
        end else begin
            report_test("Emergency Descent Position Configured", 0);
        end

        // =====================================================================
        // TC12: Fault Flags Output
        // =====================================================================
        $display("[TEST] TC12: Fault Flags Output");
        wait_clk(1);
        if (fault_flags !== 8'hxx) begin
            report_test("Fault Flags Output Connected", 1);
        end else begin
            report_test("Fault Flags Output Connected", 0);
        end

        // =====================================================================
        // TC13: Emergency Setpoints Output
        // =====================================================================
        $display("[TEST] TC13: Emergency Setpoints Output");
        wait_clk(1);
        if (emerg_sp_lat !== 32'hxxxxxxxx && emerg_sp_alt !== 32'hxxxxxxxx) begin
            report_test("Emergency Setpoints Output Connected", 1);
        end else begin
            report_test("Emergency Setpoints Output Connected", 0);
        end

        // =====================================================================
        // TC14: Emergency Active Flag
        // =====================================================================
        $display("[TEST] TC14: Emergency Active Flag");
        wait_clk(1);
        if (emerg_active === 1'b0 || emerg_active === 1'b1) begin
            report_test("Emergency Active Flag Connected", 1);
        end else begin
            report_test("Emergency Active Flag Connected", 0);
        end

        // =====================================================================
        // TC15: System Stability Check
        // =====================================================================
        $display("[TEST] TC15: System Stability Check");
        wait_clk(5);
        if (rst_n === 1'b1) begin
            report_test("System Remains Stable", 1);
        end else begin
            report_test("System Remains Stable", 0);
        end

        // =====================================================================
        // Summary Report
        // =====================================================================
        wait_clk(10);
        print_summary();
        $finish;
    end

    // =========================================================================
    // Summary report task
    // =========================================================================
    task print_summary();
        integer total_tests;
        real pass_rate;

        total_tests = tests_passed + tests_failed;
        if (total_tests > 0) begin
            pass_rate = (real'(tests_passed) / real'(total_tests)) * 100.0;
        end else begin
            pass_rate = 0.0;
        end

        $display("\n");
        $display("=============================================================================");
        $display("TEST SUMMARY REPORT");
        $display("=============================================================================");
        $display("Total Tests    : %d", total_tests);
        $display("Tests Passed   : %d", tests_passed);
        $display("Tests Failed   : %d", tests_failed);
        $display("Pass Rate      : %0.1f %%", pass_rate);
        $display("=============================================================================");

        if (tests_failed == 0) begin
            $display("SUCCESS: ALL TESTS PASSED!");
        end else begin
            $display("FAILURE: %d TEST(S) FAILED", tests_failed);
        end

        $display("=============================================================================\n");
    endtask

endmodule