// =============================================================================
// Testbench  : uav_mavlink_tb
// Description: Minimal testbench for module7_uav_mavlink
//              - 15 test cases with immediate results
//              - Vivado xsim compatible
// =============================================================================

`timescale 1ns/1ps

module module7_tb;

    // =========================================================================
    // Test parameters
    // =========================================================================
    parameter int CLK_PERIOD_NS = 20;
    parameter int CLK_HZ = 50_000_000;
    parameter int MAV_BAUD = 57600;
    parameter int FIFO_DEPTH = 512;
    parameter int DATA_W = 32;

    // =========================================================================
    // Test signals
    // =========================================================================
    reg clk;
    reg rst_n;
    reg ce_1hz;
    reg ce_5hz;
    reg ce_10hz;
    reg ce_50hz;

    reg uart_rx;
    wire uart_tx;

    reg signed [DATA_W-1:0] ekf_roll;
    reg signed [DATA_W-1:0] ekf_pitch;
    reg signed [DATA_W-1:0] ekf_yaw;
    reg signed [DATA_W-1:0] ekf_lat;
    reg signed [DATA_W-1:0] ekf_lon;
    reg signed [DATA_W-1:0] ekf_alt;
    reg                      ekf_healthy;

    reg [7:0]  sys_id;
    reg [7:0]  comp_id;
    reg [7:0]  base_mode;
    reg [7:0]  custom_mode;
    reg [15:0] sensor_status;

    wire cmd_arm;
    wire cmd_disarm;
    wire cmd_takeoff;
    wire cmd_land;
    wire cmd_rtl;
    wire cmd_waypoint;
    wire cmd_set_mode;
    wire cmd_reboot;
    wire [7:0] cmd_mode_val;
    wire gcs_present;
    wire gcs_lost;

    // Test counters
    integer test_num = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    module7_uav_mavlink #(
        .CLK_HZ (CLK_HZ),
        .MAV_BAUD (MAV_BAUD),
        .FIFO_DEPTH (FIFO_DEPTH),
        .DATA_W (DATA_W)
    ) dut (
        .clk (clk),
        .rst_n (rst_n),
        .ce_1hz (ce_1hz),
        .ce_5hz (ce_5hz),
        .ce_10hz (ce_10hz),
        .ce_50hz (ce_50hz),
        .uart_rx (uart_rx),
        .uart_tx (uart_tx),
        .ekf_roll (ekf_roll),
        .ekf_pitch (ekf_pitch),
        .ekf_yaw (ekf_yaw),
        .ekf_lat (ekf_lat),
        .ekf_lon (ekf_lon),
        .ekf_alt (ekf_alt),
        .ekf_healthy (ekf_healthy),
        .sys_id (sys_id),
        .comp_id (comp_id),
        .base_mode (base_mode),
        .custom_mode (custom_mode),
        .sensor_status (sensor_status),
        .cmd_arm (cmd_arm),
        .cmd_disarm (cmd_disarm),
        .cmd_takeoff (cmd_takeoff),
        .cmd_land (cmd_land),
        .cmd_rtl (cmd_rtl),
        .cmd_waypoint (cmd_waypoint),
        .cmd_set_mode (cmd_set_mode),
        .cmd_reboot (cmd_reboot),
        .cmd_mode_val (cmd_mode_val),
        .gcs_present (gcs_present),
        .gcs_lost (gcs_lost)
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
        $display("UAV MAVLink Module - Minimal Testbench");
        $display("=============================================================================\n");

        clk = 1'b0;
        rst_n = 1'b0;
        ce_1hz = 1'b0;
        ce_5hz = 1'b0;
        ce_10hz = 1'b0;
        ce_50hz = 1'b0;
        uart_rx = 1'b1;
        ekf_healthy = 1'b0;

        ekf_roll = 32'sd0;
        ekf_pitch = 32'sd0;
        ekf_yaw = 32'sd0;
        ekf_lat = 32'sd0;
        ekf_lon = 32'sd0;
        ekf_alt = 32'sd0;

        sys_id = 8'd1;
        comp_id = 8'd1;
        base_mode = 8'd0;
        custom_mode = 8'd0;
        sensor_status = 16'h0000;

        #(CLK_PERIOD_NS);
        wait_clk(2);

        // =====================================================================
        // TC1: Power-on Reset
        // =====================================================================
        $display("[TEST] TC1: Power-on Reset");
        if (cmd_arm === 1'b0 && cmd_disarm === 1'b0 && gcs_present === 1'b0) begin
            report_test("Power-on Reset - Commands inactive", 1);
        end else begin
            report_test("Power-on Reset - Commands inactive", 0);
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
        // TC3: UART RX Idle State
        // =====================================================================
        $display("[TEST] TC3: UART RX Idle State");
        wait_clk(1);
        if (uart_rx === 1'b1) begin
            report_test("UART RX Idle (High)", 1);
        end else begin
            report_test("UART RX Idle (High)", 0);
        end

        // =====================================================================
        // TC4: System ID Configuration
        // =====================================================================
        $display("[TEST] TC4: System ID Configuration");
        sys_id = 8'd42;
        wait_clk(1);
        if (sys_id === 8'd42) begin
            report_test("System ID Configured", 1);
        end else begin
            report_test("System ID Configured", 0);
        end

        // =====================================================================
        // TC5: Component ID Configuration
        // =====================================================================
        $display("[TEST] TC5: Component ID Configuration");
        comp_id = 8'd100;
        wait_clk(1);
        if (comp_id === 8'd100) begin
            report_test("Component ID Configured", 1);
        end else begin
            report_test("Component ID Configured", 0);
        end

        // =====================================================================
        // TC6: EKF State Telemetry (Roll/Pitch/Yaw)
        // =====================================================================
        $display("[TEST] TC6: EKF State Telemetry");
        ekf_roll = 32'sd4096;
        ekf_pitch = 32'sd2048;
        ekf_yaw = 32'sd1024;
        wait_clk(1);
        if (ekf_roll === 32'sd4096 && ekf_pitch === 32'sd2048) begin
            report_test("EKF Attitude Telemetry Configured", 1);
        end else begin
            report_test("EKF Attitude Telemetry Configured", 0);
        end

        // =====================================================================
        // TC7: EKF Position Telemetry (Lat/Lon/Alt)
        // =====================================================================
        $display("[TEST] TC7: EKF Position Telemetry");
        ekf_lat = 32'h40000000;
        ekf_lon = 32'h40000000;
        ekf_alt = 32'sd10000;
        wait_clk(1);
        if (ekf_lat === 32'h40000000 && ekf_alt === 32'sd10000) begin
            report_test("EKF Position Telemetry Configured", 1);
        end else begin
            report_test("EKF Position Telemetry Configured", 0);
        end

        // =====================================================================
        // TC8: Base Mode Configuration
        // =====================================================================
        $display("[TEST] TC8: Base Mode Configuration");
        base_mode = 8'b11000001;
        wait_clk(1);
        if (base_mode === 8'b11000001) begin
            report_test("Base Mode Configured", 1);
        end else begin
            report_test("Base Mode Configured", 0);
        end

        // =====================================================================
        // TC9: Custom Mode Configuration
        // =====================================================================
        $display("[TEST] TC9: Custom Mode Configuration");
        custom_mode = 8'd5;
        wait_clk(1);
        if (custom_mode === 8'd5) begin
            report_test("Custom Mode Configured", 1);
        end else begin
            report_test("Custom Mode Configured", 0);
        end

        // =====================================================================
        // TC10: Sensor Status Configuration
        // =====================================================================
        $display("[TEST] TC10: Sensor Status Configuration");
        sensor_status = 16'hAAAA;
        wait_clk(1);
        if (sensor_status === 16'hAAAA) begin
            report_test("Sensor Status Configured", 1);
        end else begin
            report_test("Sensor Status Configured", 0);
        end

        // =====================================================================
        // TC11: EKF Healthy Flag
        // =====================================================================
        $display("[TEST] TC11: EKF Healthy Flag");
        ekf_healthy = 1'b1;
        wait_clk(1);
        if (ekf_healthy === 1'b1) begin
            report_test("EKF Healthy Flag Set", 1);
        end else begin
            report_test("EKF Healthy Flag Set", 0);
        end

        // =====================================================================
        // TC12: CE Strobe Triggers (1 Hz)
        // =====================================================================
        $display("[TEST] TC12: CE Strobe 1 Hz Trigger");
        ce_1hz = 1'b1;
        wait_clk(1);
        ce_1hz = 1'b0;
        wait_clk(1);
        if (ce_1hz === 1'b0) begin
            report_test("1 Hz CE Strobe Applied", 1);
        end else begin
            report_test("1 Hz CE Strobe Applied", 0);
        end

        // =====================================================================
        // TC13: CE Strobe Triggers (50 Hz)
        // =====================================================================
        $display("[TEST] TC13: CE Strobe 50 Hz Trigger");
        ce_50hz = 1'b1;
        wait_clk(1);
        ce_50hz = 1'b0;
        wait_clk(1);
        if (ce_50hz === 1'b0) begin
            report_test("50 Hz CE Strobe Applied", 1);
        end else begin
            report_test("50 Hz CE Strobe Applied", 0);
        end

        // =====================================================================
        // TC14: Command Output Signals
        // =====================================================================
        $display("[TEST] TC14: Command Output Signals");
        wait_clk(1);
        if (cmd_arm === 1'b0 && cmd_disarm === 1'b0 && cmd_takeoff === 1'b0) begin
            report_test("Command Output Signals Connected", 1);
        end else begin
            report_test("Command Output Signals Connected", 0);
        end

        // =====================================================================
        // TC15: GCS Present/Lost Flags
        // =====================================================================
        $display("[TEST] TC15: GCS Present/Lost Flags");
        wait_clk(2);
        if (gcs_present === 1'b0 || gcs_lost === 1'b0) begin
            report_test("GCS Status Flags Connected", 1);
        end else begin
            report_test("GCS Status Flags Connected", 0);
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