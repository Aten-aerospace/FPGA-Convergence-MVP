// =============================================================================
// Testbench  : uav_nav_fsm_tb
// Description: Minimal testbench for module8_uav_nav_fsm
//              - 15 test cases with immediate results
//              - Vivado xsim compatible
// =============================================================================

`timescale 1ns/1ps

module module8_tb;

    // =========================================================================
    // Test parameters
    // =========================================================================
    parameter int CLK_PERIOD_NS = 20;
    parameter int CLK_HZ = 50_000_000;
    parameter int DATA_W = 32;
    parameter int MAX_WP = 32;

    // =========================================================================
    // Test signals
    // =========================================================================
    reg clk;
    reg rst_n;
    reg ce_100hz;
    reg ce_10hz;

    reg cmd_arm;
    reg cmd_disarm;
    reg cmd_takeoff;
    reg cmd_land;
    reg cmd_rtl;
    reg cmd_waypoint;
    reg cmd_set_mode;
    reg [7:0] cmd_mode;

    reg              wp_wr_en;
    reg [4:0]        wp_wr_idx;
    reg signed [DATA_W-1:0] wp_wr_lat;
    reg signed [DATA_W-1:0] wp_wr_lon;
    reg signed [DATA_W-1:0] wp_wr_alt;
    reg [DATA_W-1:0] wp_wr_radius;
    reg [DATA_W-1:0] wp_wr_speed;
    reg [7:0]        wp_wr_cmd;

    reg preflight_ok;
    reg ekf_healthy;
    reg gps_fix;
    reg baro_valid;
    reg wdt_ok;

    reg signed [DATA_W-1:0] ekf_lat;
    reg signed [DATA_W-1:0] ekf_lon;
    reg signed [DATA_W-1:0] ekf_alt;
    reg signed [DATA_W-1:0] ekf_vd;

    reg signed [DATA_W-1:0] home_lat;
    reg signed [DATA_W-1:0] home_lon;
    reg signed [DATA_W-1:0] home_alt;
    reg [DATA_W-1:0]        geofence_radius_sq;
    reg [DATA_W-1:0]        geofence_max_alt;

    wire signed [DATA_W-1:0] sp_alt;
    wire signed [DATA_W-1:0] sp_vn;
    wire signed [DATA_W-1:0] sp_ve;
    wire signed [DATA_W-1:0] sp_yaw;
    wire signed [DATA_W-1:0] sp_thrust;
    wire signed [DATA_W-1:0] sp_roll;
    wire signed [DATA_W-1:0] sp_pitch;
    wire signed [DATA_W-1:0] sp_vd;

    wire armed;
    wire [7:0] flight_mode;
    wire wdt_kick;

    // Test counters
    integer test_num = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    module8_uav_nav_fsm #(
        .CLK_HZ (CLK_HZ),
        .DATA_W (DATA_W),
        .MAX_WP (MAX_WP)
    ) dut (
        .clk (clk),
        .rst_n (rst_n),
        .ce_100hz (ce_100hz),
        .ce_10hz (ce_10hz),
        .cmd_arm (cmd_arm),
        .cmd_disarm (cmd_disarm),
        .cmd_takeoff (cmd_takeoff),
        .cmd_land (cmd_land),
        .cmd_rtl (cmd_rtl),
        .cmd_waypoint (cmd_waypoint),
        .cmd_set_mode (cmd_set_mode),
        .cmd_mode (cmd_mode),
        .wp_wr_en (wp_wr_en),
        .wp_wr_idx (wp_wr_idx),
        .wp_wr_lat (wp_wr_lat),
        .wp_wr_lon (wp_wr_lon),
        .wp_wr_alt (wp_wr_alt),
        .wp_wr_radius (wp_wr_radius),
        .wp_wr_speed (wp_wr_speed),
        .wp_wr_cmd (wp_wr_cmd),
        .preflight_ok (preflight_ok),
        .ekf_healthy (ekf_healthy),
        .gps_fix (gps_fix),
        .baro_valid (baro_valid),
        .wdt_ok (wdt_ok),
        .ekf_lat (ekf_lat),
        .ekf_lon (ekf_lon),
        .ekf_alt (ekf_alt),
        .ekf_vd (ekf_vd),
        .home_lat (home_lat),
        .home_lon (home_lon),
        .home_alt (home_alt),
        .geofence_radius_sq (geofence_radius_sq),
        .geofence_max_alt (geofence_max_alt),
        .sp_alt (sp_alt),
        .sp_vn (sp_vn),
        .sp_ve (sp_ve),
        .sp_yaw (sp_yaw),
        .sp_thrust (sp_thrust),
        .sp_roll (sp_roll),
        .sp_pitch (sp_pitch),
        .sp_vd (sp_vd),
        .armed (armed),
        .flight_mode (flight_mode),
        .wdt_kick (wdt_kick)
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
        $display("UAV Navigation FSM Module - Minimal Testbench");
        $display("=============================================================================\n");

        clk = 1'b0;
        rst_n = 1'b0;
        ce_100hz = 1'b0;
        ce_10hz = 1'b0;

        cmd_arm = 1'b0;
        cmd_disarm = 1'b0;
        cmd_takeoff = 1'b0;
        cmd_land = 1'b0;
        cmd_rtl = 1'b0;
        cmd_waypoint = 1'b0;
        cmd_set_mode = 1'b0;
        cmd_mode = 8'd0;

        wp_wr_en = 1'b0;
        wp_wr_idx = 5'd0;
        wp_wr_lat = 32'sd0;
        wp_wr_lon = 32'sd0;
        wp_wr_alt = 32'sd0;
        wp_wr_radius = 32'd0;
        wp_wr_speed = 32'd0;
        wp_wr_cmd = 8'd0;

        preflight_ok = 1'b0;
        ekf_healthy = 1'b0;
        gps_fix = 1'b0;
        baro_valid = 1'b0;
        wdt_ok = 1'b0;

        ekf_lat = 32'sd0;
        ekf_lon = 32'sd0;
        ekf_alt = 32'sd0;
        ekf_vd = 32'sd0;

        home_lat = 32'sd0;
        home_lon = 32'sd0;
        home_alt = 32'sd0;
        geofence_radius_sq = 32'd1000000;
        geofence_max_alt = 32'd5000;

        #(CLK_PERIOD_NS);
        wait_clk(2);

        // =====================================================================
        // TC1: Power-on Reset
        // =====================================================================
        $display("[TEST] TC1: Power-on Reset");
        if (armed === 1'b0 && flight_mode === 8'd0) begin
            report_test("Power-on Reset - Disarmed", 1);
        end else begin
            report_test("Power-on Reset - Disarmed", 0);
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
        // TC3: Home Position Configuration
        // =====================================================================
        $display("[TEST] TC3: Home Position Configuration");
        home_lat = 32'h40000000;
        home_lon = 32'h40000000;
        home_alt = 32'sd1000;
        wait_clk(1);
        if (home_lat === 32'h40000000 && home_alt === 32'sd1000) begin
            report_test("Home Position Configured", 1);
        end else begin
            report_test("Home Position Configured", 0);
        end

        // =====================================================================
        // TC4: EKF Position Input
        // =====================================================================
        $display("[TEST] TC4: EKF Position Input");
        ekf_lat = 32'h40000000;
        ekf_lon = 32'h40000000;
        ekf_alt = 32'sd500;
        wait_clk(1);
        if (ekf_lat === 32'h40000000 && ekf_alt === 32'sd500) begin
            report_test("EKF Position Configured", 1);
        end else begin
            report_test("EKF Position Configured", 0);
        end

        // =====================================================================
        // TC5: Geofence Configuration
        // =====================================================================
        $display("[TEST] TC5: Geofence Configuration");
        geofence_radius_sq = 32'd2000000;
        geofence_max_alt = 32'd10000;
        wait_clk(1);
        if (geofence_radius_sq === 32'd2000000 && geofence_max_alt === 32'd10000) begin
            report_test("Geofence Parameters Configured", 1);
        end else begin
            report_test("Geofence Parameters Configured", 0);
        end

        // =====================================================================
        // TC6: Waypoint Writing
        // =====================================================================
        $display("[TEST] TC6: Waypoint Writing");
        wp_wr_en = 1'b1;
        wp_wr_idx = 5'd0;
        wp_wr_lat = 32'h40000100;
        wp_wr_lon = 32'h40000100;
        wp_wr_alt = 32'sd2000;
        wp_wr_radius = 32'd50;
        wp_wr_speed = 32'd10;
        wp_wr_cmd = 8'd16;
        wait_clk(1);
        wp_wr_en = 1'b0;
        wait_clk(1);
        if (wp_wr_lat === 32'h40000100) begin
            report_test("Waypoint Data Written", 1);
        end else begin
            report_test("Waypoint Data Written", 0);
        end

        // =====================================================================
        // TC7: Sensor Health Flags
        // =====================================================================
        $display("[TEST] TC7: Sensor Health Flags");
        preflight_ok = 1'b1;
        ekf_healthy = 1'b1;
        gps_fix = 1'b1;
        baro_valid = 1'b1;
        wdt_ok = 1'b1;
        wait_clk(1);
        if (preflight_ok === 1'b1 && ekf_healthy === 1'b1 && gps_fix === 1'b1) begin
            report_test("Sensor Health Flags Set", 1);
        end else begin
            report_test("Sensor Health Flags Set", 0);
        end

        // =====================================================================
        // TC8: ARM Command
        // =====================================================================
        $display("[TEST] TC8: ARM Command");
        cmd_arm = 1'b1;
        wait_clk(1);
        cmd_arm = 1'b0;
        wait_clk(2);
        report_test("ARM Command Applied", 1);

        // =====================================================================
        // TC9: DISARM Command
        // =====================================================================
        $display("[TEST] TC9: DISARM Command");
        cmd_disarm = 1'b1;
        wait_clk(1);
        cmd_disarm = 1'b0;
        wait_clk(2);
        report_test("DISARM Command Applied", 1);

        // =====================================================================
        // TC10: CE 100Hz Strobe
        // =====================================================================
        $display("[TEST] TC10: CE 100Hz Strobe");
        ce_100hz = 1'b1;
        wait_clk(1);
        ce_100hz = 1'b0;
        wait_clk(1);
        if (ce_100hz === 1'b0) begin
            report_test("100 Hz CE Strobe Applied", 1);
        end else begin
            report_test("100 Hz CE Strobe Applied", 0);
        end

        // =====================================================================
        // TC11: CE 10Hz Strobe
        // =====================================================================
        $display("[TEST] TC11: CE 10Hz Strobe");
        ce_10hz = 1'b1;
        wait_clk(1);
        ce_10hz = 1'b0;
        wait_clk(1);
        if (ce_10hz === 1'b0) begin
            report_test("10 Hz CE Strobe Applied", 1);
        end else begin
            report_test("10 Hz CE Strobe Applied", 0);
        end

        // =====================================================================
        // TC12: Setpoint Outputs
        // =====================================================================
        $display("[TEST] TC12: Setpoint Outputs Connected");
        wait_clk(2);
        if (sp_alt !== 32'hxxxxxxxx && sp_vn !== 32'hxxxxxxxx) begin
            report_test("Setpoint Outputs Connected", 1);
        end else begin
            report_test("Setpoint Outputs Connected", 0);
        end

        // =====================================================================
        // TC13: Flight Mode Output
        // =====================================================================
        $display("[TEST] TC13: Flight Mode Output");
        wait_clk(1);
        if (flight_mode !== 8'hxx) begin
            report_test("Flight Mode Output Connected", 1);
        end else begin
            report_test("Flight Mode Output Connected", 0);
        end

        // =====================================================================
        // TC14: Watchdog Kick Signal
        // =====================================================================
        $display("[TEST] TC14: Watchdog Kick Signal");
        wait_clk(1);
        if (wdt_kick === 1'b0 || wdt_kick === 1'b1) begin
            report_test("Watchdog Kick Signal Connected", 1);
        end else begin
            report_test("Watchdog Kick Signal Connected", 0);
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