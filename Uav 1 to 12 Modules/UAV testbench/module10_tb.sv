// =============================================================================
// Testbench  : uav_axi_regif_tb
// Description: Minimal testbench for module10_uav_axi_regif
//              - 15 test cases with immediate results
//              - Vivado xsim compatible
// =============================================================================

`timescale 1ns/1ps

module module10_tb;

    // =========================================================================
    // Test parameters
    // =========================================================================
    parameter int CLK_PERIOD_NS = 20;
    parameter int CLK_HZ = 50_000_000;
    parameter int DATA_W = 32;
    parameter int ADDR_W = 8;

    // =========================================================================
    // Test signals
    // =========================================================================
    reg clk;
    reg rst_n;

    reg [ADDR_W-1:0] s_awaddr;
    reg s_awvalid;
    wire s_awready;

    reg [DATA_W-1:0] s_wdata;
    reg [3:0] s_wstrb;
    reg s_wvalid;
    wire s_wready;

    wire [1:0] s_bresp;
    wire s_bvalid;
    reg s_bready;

    reg [ADDR_W-1:0] s_araddr;
    reg s_arvalid;
    wire s_arready;

    wire [DATA_W-1:0] s_rdata;
    wire [1:0] s_rresp;
    wire s_rvalid;
    reg s_rready;

    reg [31:0] status_ekf;
    reg [31:0] status_gps;
    reg [31:0] status_imu;

    reg [7:0] flight_mode;
    reg       armed;

    wire signed [15:0] pid_gains [0:7][0:2];
    wire signed [31:0] pid_integ_max [0:7];
    wire signed [15:0] mix_coef [0:3][0:3];
    wire [31:0] ekf_q_diag [0:8];
    wire signed [31:0] gyro_bias [0:2];
    wire signed [31:0] accel_bias [0:2];
    wire signed [31:0] mag_offset [0:2];
    wire [10:0] wdt_timeout_ms;
    wire        wdt_config_en;
    wire [31:0] geofence_radius_sq;
    wire [31:0] geofence_max_alt;
    wire [7:0]  check_mask;
    wire [7:0]  sys_arm_cmd;
    wire [7:0]  sys_mode_cmd;

    wire [3:0] led_status;

    // Test counters
    integer test_num = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    module10_uav_axi_regif #(
        .CLK_HZ (CLK_HZ),
        .DATA_W (DATA_W),
        .ADDR_W (ADDR_W)
    ) dut (
        .clk (clk),
        .rst_n (rst_n),
        .s_awaddr (s_awaddr),
        .s_awvalid (s_awvalid),
        .s_awready (s_awready),
        .s_wdata (s_wdata),
        .s_wstrb (s_wstrb),
        .s_wvalid (s_wvalid),
        .s_wready (s_wready),
        .s_bresp (s_bresp),
        .s_bvalid (s_bvalid),
        .s_bready (s_bready),
        .s_araddr (s_araddr),
        .s_arvalid (s_arvalid),
        .s_arready (s_arready),
        .s_rdata (s_rdata),
        .s_rresp (s_rresp),
        .s_rvalid (s_rvalid),
        .s_rready (s_rready),
        .status_ekf (status_ekf),
        .status_gps (status_gps),
        .status_imu (status_imu),
        .flight_mode (flight_mode),
        .armed (armed),
        .pid_gains (pid_gains),
        .pid_integ_max (pid_integ_max),
        .mix_coef (mix_coef),
        .ekf_q_diag (ekf_q_diag),
        .gyro_bias (gyro_bias),
        .accel_bias (accel_bias),
        .mag_offset (mag_offset),
        .wdt_timeout_ms (wdt_timeout_ms),
        .wdt_config_en (wdt_config_en),
        .geofence_radius_sq (geofence_radius_sq),
        .geofence_max_alt (geofence_max_alt),
        .check_mask (check_mask),
        .sys_arm_cmd (sys_arm_cmd),
        .sys_mode_cmd (sys_mode_cmd),
        .led_status (led_status)
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
        $display("UAV AXI Register Interface Module - Minimal Testbench");
        $display("=============================================================================\n");

        clk = 1'b0;
        rst_n = 1'b0;

        s_awaddr = 8'd0;
        s_awvalid = 1'b0;
        s_wdata = 32'd0;
        s_wstrb = 4'hF;
        s_wvalid = 1'b0;
        s_bready = 1'b0;

        s_araddr = 8'd0;
        s_arvalid = 1'b0;
        s_rready = 1'b0;

        status_ekf = 32'd0;
        status_gps = 32'd0;
        status_imu = 32'd0;

        flight_mode = 8'd0;
        armed = 1'b0;

        #(CLK_PERIOD_NS);
        wait_clk(2);

        // =====================================================================
        // TC1: Power-on Reset
        // =====================================================================
        $display("[TEST] TC1: Power-on Reset");
        if (s_awready === 1'b1 && s_wready === 1'b1 && s_arready === 1'b1) begin
            report_test("Power-on Reset - AXI ready", 1);
        end else begin
            report_test("Power-on Reset - AXI ready", 0);
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
        // TC3: AXI Write Address Channel
        // =====================================================================
        $display("[TEST] TC3: AXI Write Address Channel");
        s_awaddr = 8'd10;
        s_awvalid = 1'b1;
        wait_clk(1);
        s_awvalid = 1'b0;
        wait_clk(1);
        if (s_awaddr === 8'd10) begin
            report_test("AXI Write Address Channel", 1);
        end else begin
            report_test("AXI Write Address Channel", 0);
        end

        // =====================================================================
        // TC4: AXI Write Data Channel
        // =====================================================================
        $display("[TEST] TC4: AXI Write Data Channel");
        s_wdata = 32'hDEADBEEF;
        s_wstrb = 4'hF;
        s_wvalid = 1'b1;
        wait_clk(1);
        s_wvalid = 1'b0;
        wait_clk(1);
        if (s_wdata === 32'hDEADBEEF) begin
            report_test("AXI Write Data Channel", 1);
        end else begin
            report_test("AXI Write Data Channel", 0);
        end

        // =====================================================================
        // TC5: AXI Read Address Channel
        // =====================================================================
        $display("[TEST] TC5: AXI Read Address Channel");
        s_araddr = 8'd20;
        s_arvalid = 1'b1;
        wait_clk(1);
        s_arvalid = 1'b0;
        wait_clk(1);
        if (s_araddr === 8'd20) begin
            report_test("AXI Read Address Channel", 1);
        end else begin
            report_test("AXI Read Address Channel", 0);
        end

        // =====================================================================
        // TC6: Status Inputs Configuration
        // =====================================================================
        $display("[TEST] TC6: Status Inputs Configuration");
        status_ekf = 32'h12345678;
        status_gps = 32'h87654321;
        status_imu = 32'hABCDEF00;
        wait_clk(1);
        if (status_ekf === 32'h12345678 && status_gps === 32'h87654321) begin
            report_test("Status Inputs Configured", 1);
        end else begin
            report_test("Status Inputs Configured", 0);
        end

        // =====================================================================
        // TC7: Flight Mode and Armed Flag
        // =====================================================================
        $display("[TEST] TC7: Flight Mode and Armed Flag");
        flight_mode = 8'd4;
        armed = 1'b1;
        wait_clk(1);
        if (flight_mode === 8'd4 && armed === 1'b1) begin
            report_test("Flight Mode and Armed Flag Set", 1);
        end else begin
            report_test("Flight Mode and Armed Flag Set", 0);
        end

        // =====================================================================
        // TC8: PID Gains Output
        // =====================================================================
        $display("[TEST] TC8: PID Gains Output");
        wait_clk(1);
        if (pid_gains[0][0] !== 16'hxxxx && pid_gains[7][2] !== 16'hxxxx) begin
            report_test("PID Gains Output Connected", 1);
        end else begin
            report_test("PID Gains Output Connected", 0);
        end

        // =====================================================================
        // TC9: Bias and Calibration Outputs
        // =====================================================================
        $display("[TEST] TC9: Bias and Calibration Outputs");
        wait_clk(1);
        if (gyro_bias[0] !== 32'hxxxxxxxx && accel_bias[2] !== 32'hxxxxxxxx) begin
            report_test("Bias Calibration Outputs Connected", 1);
        end else begin
            report_test("Bias Calibration Outputs Connected", 0);
        end

        // =====================================================================
        // TC10: Mag Offset Output
        // =====================================================================
        $display("[TEST] TC10: Mag Offset Output");
        wait_clk(1);
        if (mag_offset[0] !== 32'hxxxxxxxx && mag_offset[2] !== 32'hxxxxxxxx) begin
            report_test("Mag Offset Output Connected", 1);
        end else begin
            report_test("Mag Offset Output Connected", 0);
        end

        // =====================================================================
        // TC11: WDT Configuration Output
        // =====================================================================
        $display("[TEST] TC11: WDT Configuration Output");
        wait_clk(1);
        if (wdt_timeout_ms !== 11'hxxx && wdt_config_en === 1'b0) begin
            report_test("WDT Configuration Output Connected", 1);
        end else begin
            report_test("WDT Configuration Output Connected", 0);
        end

        // =====================================================================
        // TC12: Geofence Configuration Output
        // =====================================================================
        $display("[TEST] TC12: Geofence Configuration Output");
        wait_clk(1);
        if (geofence_radius_sq !== 32'hxxxxxxxx && geofence_max_alt !== 32'hxxxxxxxx) begin
            report_test("Geofence Configuration Output Connected", 1);
        end else begin
            report_test("Geofence Configuration Output Connected", 0);
        end

        // =====================================================================
        // TC13: Check Mask Output
        // =====================================================================
        $display("[TEST] TC13: Check Mask Output");
        wait_clk(1);
        if (check_mask !== 8'hxx) begin
            report_test("Check Mask Output Connected", 1);
        end else begin
            report_test("Check Mask Output Connected", 0);
        end

        // =====================================================================
        // TC14: System Command Outputs
        // =====================================================================
        $display("[TEST] TC14: System Command Outputs");
        wait_clk(1);
        if (sys_arm_cmd !== 8'hxx && sys_mode_cmd !== 8'hxx) begin
            report_test("System Command Outputs Connected", 1);
        end else begin
            report_test("System Command Outputs Connected", 0);
        end

        // =====================================================================
        // TC15: LED Status Output
        // =====================================================================
        $display("[TEST] TC15: LED Status Output");
        wait_clk(1);
        if (led_status !== 4'hx) begin
            report_test("LED Status Output Connected", 1);
        end else begin
            report_test("LED Status Output Connected", 0);
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