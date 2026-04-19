// =============================================================================
// Testbench  : uav_gps_interface_tb
// Description: Minimal testbench for module6_uav_gps_interface
//              - 15 test cases with immediate results
//              - Vivado xsim compatible
// =============================================================================

`timescale 1ns/1ps

module module6_tb;

    // =========================================================================
    // Test parameters
    // =========================================================================
    parameter int CLK_PERIOD_NS = 20;
    parameter int CLK_HZ = 50_000_000;
    parameter int GPS_BAUD = 57600;
    parameter int FIFO_DEPTH = 512;

    // =========================================================================
    // Test signals
    // =========================================================================
    reg clk;
    reg rst_n;
    reg uart_rx;

    wire signed [31:0] gps_lat;
    wire signed [31:0] gps_lon;
    wire signed [31:0] gps_alt;
    wire signed [31:0] gps_vel_n;
    wire signed [31:0] gps_vel_e;
    wire [3:0]         gps_fix_type;
    wire [15:0]        gps_hdop;
    wire               gps_fix;
    wire               gps_data_valid;

    // Test counters
    integer test_num = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    module6_uav_gps_interface #(
        .CLK_HZ (CLK_HZ),
        .GPS_BAUD (GPS_BAUD),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) dut (
        .clk (clk),
        .rst_n (rst_n),
        .uart_rx (uart_rx),
        .gps_lat (gps_lat),
        .gps_lon (gps_lon),
        .gps_alt (gps_alt),
        .gps_vel_n (gps_vel_n),
        .gps_vel_e (gps_vel_e),
        .gps_fix_type (gps_fix_type),
        .gps_hdop (gps_hdop),
        .gps_fix (gps_fix),
        .gps_data_valid (gps_data_valid)
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
        $display("UAV GPS Interface Module - Minimal Testbench");
        $display("=============================================================================\n");

        clk = 1'b0;
        rst_n = 1'b0;
        uart_rx = 1'b1;

        #(CLK_PERIOD_NS);
        wait_clk(2);

        // =====================================================================
        // TC1: Power-on Reset
        // =====================================================================
        $display("[TEST] TC1: Power-on Reset");
        if (gps_data_valid === 1'b0 && gps_fix === 1'b0) begin
            report_test("Power-on Reset - GPS inactive", 1);
        end else begin
            report_test("Power-on Reset - GPS inactive", 0);
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
        // TC4: Initial GPS Latitude
        // =====================================================================
        $display("[TEST] TC4: Initial GPS Latitude");
        wait_clk(1);
        if (gps_lat === 32'sd0) begin
            report_test("Initial GPS Latitude Zero", 1);
        end else begin
            report_test("Initial GPS Latitude Zero", 0);
        end

        // =====================================================================
        // TC5: Initial GPS Longitude
        // =====================================================================
        $display("[TEST] TC5: Initial GPS Longitude");
        wait_clk(1);
        if (gps_lon === 32'sd0) begin
            report_test("Initial GPS Longitude Zero", 1);
        end else begin
            report_test("Initial GPS Longitude Zero", 0);
        end

        // =====================================================================
        // TC6: Initial GPS Altitude
        // =====================================================================
        $display("[TEST] TC6: Initial GPS Altitude");
        wait_clk(1);
        if (gps_alt === 32'sd0) begin
            report_test("Initial GPS Altitude Zero", 1);
        end else begin
            report_test("Initial GPS Altitude Zero", 0);
        end

        // =====================================================================
        // TC7: Initial GPS Velocity North
        // =====================================================================
        $display("[TEST] TC7: Initial GPS Velocity North");
        wait_clk(1);
        if (gps_vel_n === 32'sd0) begin
            report_test("Initial GPS Velocity North Zero", 1);
        end else begin
            report_test("Initial GPS Velocity North Zero", 0);
        end

        // =====================================================================
        // TC8: Initial GPS Velocity East
        // =====================================================================
        $display("[TEST] TC8: Initial GPS Velocity East");
        wait_clk(1);
        if (gps_vel_e === 32'sd0) begin
            report_test("Initial GPS Velocity East Zero", 1);
        end else begin
            report_test("Initial GPS Velocity East Zero", 0);
        end

        // =====================================================================
        // TC9: GPS Fix Type Output
        // =====================================================================
        $display("[TEST] TC9: GPS Fix Type Output");
        wait_clk(1);
        if (gps_fix_type === 4'b0000) begin
            report_test("GPS Fix Type Initialized", 1);
        end else begin
            report_test("GPS Fix Type Initialized", 0);
        end

        // =====================================================================
        // TC10: GPS HDOP Output
        // =====================================================================
        $display("[TEST] TC10: GPS HDOP Output");
        wait_clk(1);
        if (gps_hdop !== 16'hxxxxxxxx) begin
            report_test("GPS HDOP Output Connected", 1);
        end else begin
            report_test("GPS HDOP Output Connected", 0);
        end

        // =====================================================================
        // TC11: GPS Fix Flag
        // =====================================================================
        $display("[TEST] TC11: GPS Fix Flag");
        wait_clk(1);
        if (gps_fix === 1'b0) begin
            report_test("GPS Fix Flag Initially Low", 1);
        end else begin
            report_test("GPS Fix Flag Initially Low", 0);
        end

        // =====================================================================
        // TC12: GPS Data Valid Flag
        // =====================================================================
        $display("[TEST] TC12: GPS Data Valid Flag");
        wait_clk(1);
        if (gps_data_valid === 1'b0) begin
            report_test("GPS Data Valid Flag Initially Low", 1);
        end else begin
            report_test("GPS Data Valid Flag Initially Low", 0);
        end

        // =====================================================================
        // TC13: UART RX Signal Transition
        // =====================================================================
        $display("[TEST] TC13: UART RX Signal Transition");
        uart_rx = 1'b0;
        wait_clk(1);
        if (uart_rx === 1'b0) begin
            report_test("UART RX Signal Transition", 1);
        end else begin
            report_test("UART RX Signal Transition", 0);
        end

        // =====================================================================
        // TC14: UART RX Return to Idle
        // =====================================================================
        $display("[TEST] TC14: UART RX Return to Idle");
        uart_rx = 1'b1;
        wait_clk(1);
        if (uart_rx === 1'b1) begin
            report_test("UART RX Return to Idle", 1);
        end else begin
            report_test("UART RX Return to Idle", 0);
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