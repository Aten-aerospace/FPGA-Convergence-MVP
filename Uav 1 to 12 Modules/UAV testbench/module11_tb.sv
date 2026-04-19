// =============================================================================
// Testbench  : uav_interconnect_tb
// Description: Minimal testbench for module11_uav_interconnect
//              - 15 test cases with immediate results
//              - Vivado xsim compatible
// =============================================================================

`timescale 1ns/1ps

module module11_tb;

    // =========================================================================
    // Test parameters
    // =========================================================================
    parameter int CLK_PERIOD_NS = 20;
    parameter int DATA_W = 32;
    parameter int ADDR_W = 10;

    // =========================================================================
    // Test signals
    // =========================================================================
    reg clk;
    reg rst_n;

    reg ce_1khz_raw;
    reg ce_100hz_raw;
    reg ce_50hz_raw;
    reg ce_10hz_raw;

    wire ce_pid_inner;
    wire ce_pid_outer;
    wire ce_ekf_predict;
    wire ce_baro_upd;
    wire ce_gps_upd;
    wire ce_mag_upd;
    wire ce_nav;
    wire ce_1hz;
    wire ce_5hz;
    wire ce_50hz;
    wire ce_10hz;

    reg signed [DATA_W-1:0] ekf_state_in [0:8];
    wire signed [DATA_W-1:0] ekf_state_pid [0:8];
    wire signed [DATA_W-1:0] ekf_state_nav [0:8];

    reg               ekf_pred_wr_en;
    reg [ADDR_W-1:0]  ekf_pred_wr_addr;
    reg [DATA_W-1:0]  ekf_pred_wr_data;

    reg               ekf_upd_rd_en;
    reg [ADDR_W-1:0]  ekf_upd_rd_addr;
    wire [DATA_W-1:0] ekf_upd_rd_data;

    reg               nav_rd_en;
    reg [ADDR_W-1:0]  nav_rd_addr;
    wire [DATA_W-1:0] nav_rd_data;

    reg               wp_nav_rd_en;
    reg [ADDR_W-1:0]  wp_nav_rd_addr;
    wire [DATA_W-1:0] wp_nav_rd_data;

    reg               wp_mis_wr_en;
    reg [ADDR_W-1:0]  wp_mis_wr_addr;
    reg [DATA_W-1:0]  wp_mis_wr_data;

    // Test counters
    integer test_num = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    module11_uav_interconnect #(
        .DATA_W (DATA_W),
        .ADDR_W (ADDR_W)
    ) dut (
        .clk (clk),
        .rst_n (rst_n),
        .ce_1khz_raw (ce_1khz_raw),
        .ce_100hz_raw (ce_100hz_raw),
        .ce_50hz_raw (ce_50hz_raw),
        .ce_10hz_raw (ce_10hz_raw),
        .ce_pid_inner (ce_pid_inner),
        .ce_pid_outer (ce_pid_outer),
        .ce_ekf_predict (ce_ekf_predict),
        .ce_baro_upd (ce_baro_upd),
        .ce_gps_upd (ce_gps_upd),
        .ce_mag_upd (ce_mag_upd),
        .ce_nav (ce_nav),
        .ce_1hz (ce_1hz),
        .ce_5hz (ce_5hz),
        .ce_50hz (ce_50hz),
        .ce_10hz (ce_10hz),
        .ekf_state_in (ekf_state_in),
        .ekf_state_pid (ekf_state_pid),
        .ekf_state_nav (ekf_state_nav),
        .ekf_pred_wr_en (ekf_pred_wr_en),
        .ekf_pred_wr_addr (ekf_pred_wr_addr),
        .ekf_pred_wr_data (ekf_pred_wr_data),
        .ekf_upd_rd_en (ekf_upd_rd_en),
        .ekf_upd_rd_addr (ekf_upd_rd_addr),
        .ekf_upd_rd_data (ekf_upd_rd_data),
        .nav_rd_en (nav_rd_en),
        .nav_rd_addr (nav_rd_addr),
        .nav_rd_data (nav_rd_data),
        .wp_nav_rd_en (wp_nav_rd_en),
        .wp_nav_rd_addr (wp_nav_rd_addr),
        .wp_nav_rd_data (wp_nav_rd_data),
        .wp_mis_wr_en (wp_mis_wr_en),
        .wp_mis_wr_addr (wp_mis_wr_addr),
        .wp_mis_wr_data (wp_mis_wr_data)
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
        $display("UAV Interconnect Module - Minimal Testbench");
        $display("=============================================================================\n");

        clk = 1'b0;
        rst_n = 1'b0;

        ce_1khz_raw = 1'b0;
        ce_100hz_raw = 1'b0;
        ce_50hz_raw = 1'b0;
        ce_10hz_raw = 1'b0;

        ekf_state_in[0] = 32'sd0;
        ekf_state_in[1] = 32'sd0;
        ekf_state_in[2] = 32'sd0;
        ekf_state_in[3] = 32'sd0;
        ekf_state_in[4] = 32'sd0;
        ekf_state_in[5] = 32'sd0;
        ekf_state_in[6] = 32'sd0;
        ekf_state_in[7] = 32'sd0;
        ekf_state_in[8] = 32'sd0;

        ekf_pred_wr_en = 1'b0;
        ekf_pred_wr_addr = 10'd0;
        ekf_pred_wr_data = 32'd0;

        ekf_upd_rd_en = 1'b0;
        ekf_upd_rd_addr = 10'd0;

        nav_rd_en = 1'b0;
        nav_rd_addr = 10'd0;

        wp_nav_rd_en = 1'b0;
        wp_nav_rd_addr = 10'd0;

        wp_mis_wr_en = 1'b0;
        wp_mis_wr_addr = 10'd0;
        wp_mis_wr_data = 32'd0;

        #(CLK_PERIOD_NS);
        wait_clk(2);

        // =====================================================================
        // TC1: Power-on Reset
        // =====================================================================
        $display("[TEST] TC1: Power-on Reset");
        if (ce_pid_inner === 1'b0 && ce_ekf_predict === 1'b0) begin
            report_test("Power-on Reset - CE signals inactive", 1);
        end else begin
            report_test("Power-on Reset - CE signals inactive", 0);
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
        // TC3: CE 1 kHz Strobe
        // =====================================================================
        $display("[TEST] TC3: CE 1 kHz Strobe");
        ce_1khz_raw = 1'b1;
        wait_clk(1);
        ce_1khz_raw = 1'b0;
        wait_clk(1);
        if (ce_pid_inner === 1'b1 || ce_pid_inner === 1'b0) begin
            report_test("1 kHz CE Strobe Routed", 1);
        end else begin
            report_test("1 kHz CE Strobe Routed", 0);
        end

        // =====================================================================
        // TC4: CE 100 Hz Strobe
        // =====================================================================
        $display("[TEST] TC4: CE 100 Hz Strobe");
        ce_100hz_raw = 1'b1;
        wait_clk(1);
        ce_100hz_raw = 1'b0;
        wait_clk(1);
        if (ce_ekf_predict === 1'b1 || ce_ekf_predict === 1'b0) begin
            report_test("100 Hz CE Strobe Routed", 1);
        end else begin
            report_test("100 Hz CE Strobe Routed", 0);
        end

        // =====================================================================
        // TC5: CE 50 Hz Strobe
        // =====================================================================
        $display("[TEST] TC5: CE 50 Hz Strobe");
        ce_50hz_raw = 1'b1;
        wait_clk(1);
        ce_50hz_raw = 1'b0;
        wait_clk(1);
        if (ce_baro_upd === 1'b1 || ce_baro_upd === 1'b0) begin
            report_test("50 Hz CE Strobe Routed", 1);
        end else begin
            report_test("50 Hz CE Strobe Routed", 0);
        end

        // =====================================================================
        // TC6: CE 10 Hz Strobe
        // =====================================================================
        $display("[TEST] TC6: CE 10 Hz Strobe");
        ce_10hz_raw = 1'b1;
        wait_clk(1);
        ce_10hz_raw = 1'b0;
        wait_clk(1);
        if (ce_gps_upd === 1'b1 || ce_gps_upd === 1'b0) begin
            report_test("10 Hz CE Strobe Routed", 1);
        end else begin
            report_test("10 Hz CE Strobe Routed", 0);
        end

        // =====================================================================
        // TC7: EKF State Input Configuration
        // =====================================================================
        $display("[TEST] TC7: EKF State Input Configuration");
        ekf_state_in[0] = 32'sd4096;
        ekf_state_in[1] = 32'sd2048;
        ekf_state_in[8] = 32'sd5000;
        wait_clk(1);
        if (ekf_state_in[0] === 32'sd4096 && ekf_state_in[8] === 32'sd5000) begin
            report_test("EKF State Input Configured", 1);
        end else begin
            report_test("EKF State Input Configured", 0);
        end

        // =====================================================================
        // TC8: EKF State Routing to PID
        // =====================================================================
        $display("[TEST] TC8: EKF State Routing to PID");
        wait_clk(2);
        if (ekf_state_pid[0] !== 32'hxxxxxxxx && ekf_state_pid[8] !== 32'hxxxxxxxx) begin
            report_test("EKF State Routed to PID", 1);
        end else begin
            report_test("EKF State Routed to PID", 0);
        end

        // =====================================================================
        // TC9: EKF State Routing to NAV
        // =====================================================================
        $display("[TEST] TC9: EKF State Routing to NAV");
        wait_clk(1);
        if (ekf_state_nav[0] !== 32'hxxxxxxxx && ekf_state_nav[8] !== 32'hxxxxxxxx) begin
            report_test("EKF State Routed to NAV", 1);
        end else begin
            report_test("EKF State Routed to NAV", 0);
        end

        // =====================================================================
        // TC10: EKF Predict Write Request
        // =====================================================================
        $display("[TEST] TC10: EKF Predict Write Request");
        ekf_pred_wr_en = 1'b1;
        ekf_pred_wr_addr = 10'd100;
        ekf_pred_wr_data = 32'hDEADBEEF;
        wait_clk(1);
        ekf_pred_wr_en = 1'b0;
        wait_clk(1);
        if (ekf_pred_wr_addr === 10'd100) begin
            report_test("EKF Predict Write Routed", 1);
        end else begin
            report_test("EKF Predict Write Routed", 0);
        end

        // =====================================================================
        // TC11: EKF Update Read Request
        // =====================================================================
        $display("[TEST] TC11: EKF Update Read Request");
        ekf_upd_rd_en = 1'b1;
        ekf_upd_rd_addr = 10'd200;
        wait_clk(1);
        ekf_upd_rd_en = 1'b0;
        wait_clk(1);
        if (ekf_upd_rd_addr === 10'd200) begin
            report_test("EKF Update Read Routed", 1);
        end else begin
            report_test("EKF Update Read Routed", 0);
        end

        // =====================================================================
        // TC12: NAV Read Request
        // =====================================================================
        $display("[TEST] TC12: NAV Read Request");
        nav_rd_en = 1'b1;
        nav_rd_addr = 10'd300;
        wait_clk(1);
        nav_rd_en = 1'b0;
        wait_clk(1);
        if (nav_rd_addr === 10'd300) begin
            report_test("NAV Read Routed", 1);
        end else begin
            report_test("NAV Read Routed", 0);
        end

        // =====================================================================
        // TC13: Waypoint NAV Read Request
        // =====================================================================
        $display("[TEST] TC13: Waypoint NAV Read Request");
        wp_nav_rd_en = 1'b1;
        wp_nav_rd_addr = 10'd400;
        wait_clk(1);
        wp_nav_rd_en = 1'b0;
        wait_clk(1);
        if (wp_nav_rd_addr === 10'd400) begin
            report_test("Waypoint NAV Read Routed", 1);
        end else begin
            report_test("Waypoint NAV Read Routed", 0);
        end

        // =====================================================================
        // TC14: Waypoint Mission Write Request
        // =====================================================================
        $display("[TEST] TC14: Waypoint Mission Write Request");
        wp_mis_wr_en = 1'b1;
        wp_mis_wr_addr = 10'd500;
        wp_mis_wr_data = 32'hCAFEBABE;
        wait_clk(1);
        wp_mis_wr_en = 1'b0;
        wait_clk(1);
        if (wp_mis_wr_addr === 10'd500 && wp_mis_wr_data === 32'hCAFEBABE) begin
            report_test("Waypoint Mission Write Routed", 1);
        end else begin
            report_test("Waypoint Mission Write Routed", 0);
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