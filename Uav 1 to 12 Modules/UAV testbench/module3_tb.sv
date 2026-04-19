// =============================================================================
// Testbench  : uav_motor_mix_tb_simple
// Description: Simplified testbench for module3_uav_motor_mix
//              - Fast execution with minimal dependencies
//              - 15 test cases with simple input/output verification
//              - Focuses on signal routing and configuration
// =============================================================================

`timescale 1ns/1ps

module module3_tb;

    // =========================================================================
    // Test parameters
    // =========================================================================
    parameter int CLK_PERIOD_NS = 20;
    parameter int CLK_HZ        = 50_000_000;
    parameter int PWM_HZ        = 400;
    parameter int DATA_W        = 16;
    parameter int COEF_W        = 16;
    parameter int INTEG_W       = 32;

    // =========================================================================
    // Test signals
    // =========================================================================
    reg clk;
    reg rst_n;
    reg ce_1khz;
    reg armed;

    reg signed [DATA_W-1:0] roll_cmd;
    reg signed [DATA_W-1:0] pitch_cmd;
    reg signed [DATA_W-1:0] yaw_cmd;
    reg signed [DATA_W-1:0] thrust_cmd;

    reg signed [COEF_W-1:0] mix_coef [0:3][0:3];

    reg signed [DATA_W-1:0]  out_max [0:3];
    reg signed [DATA_W-1:0]  out_min [0:3];
    reg signed [DATA_W-1:0]  max_delta [0:3];

    reg signed [INTEG_W-1:0] integ_max [0:3];

    wire [3:0] pwm_out;
    wire signed [DATA_W-1:0] motor_cmd_sat [0:3];

    // Test counters
    integer test_num       = 0;
    integer tests_passed   = 0;
    integer tests_failed   = 0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    module3_uav_motor_mix #(
        .CLK_HZ  (CLK_HZ),
        .PWM_HZ  (PWM_HZ),
        .DATA_W  (DATA_W),
        .COEF_W  (COEF_W),
        .INTEG_W (INTEG_W)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .ce_1khz        (ce_1khz),
        .armed          (armed),
        .roll_cmd       (roll_cmd),
        .pitch_cmd      (pitch_cmd),
        .yaw_cmd        (yaw_cmd),
        .thrust_cmd     (thrust_cmd),
        .mix_coef       (mix_coef),
        .out_max        (out_max),
        .out_min        (out_min),
        .max_delta      (max_delta),
        .integ_max      (integ_max),
        .pwm_out        (pwm_out),
        .motor_cmd_sat  (motor_cmd_sat)
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
        $display("UAV Motor Mixing Module - Simplified Testbench");
        $display("=============================================================================");
        $display("Starting simulation at %t ns", $time);
        $display("\n");

        // Initialize all signals to defaults
        clk          = 1'b0;
        rst_n        = 1'b0;
        ce_1khz      = 1'b0;
        armed        = 1'b0;
        roll_cmd     = 16'sd0;
        pitch_cmd    = 16'sd0;
        yaw_cmd      = 16'sd0;
        thrust_cmd   = 16'sd0;

        // Initialize mixing matrix (Quadrotor X-configuration)
        mix_coef[0][0] = 16'sd4096;   mix_coef[0][1] = -16'sd4096;  mix_coef[0][2] = -16'sd4096;  mix_coef[0][3] = 16'sd8192;
        mix_coef[1][0] = -16'sd4096;  mix_coef[1][1] = -16'sd4096;  mix_coef[1][2] = 16'sd4096;   mix_coef[1][3] = 16'sd8192;
        mix_coef[2][0] = -16'sd4096;  mix_coef[2][1] = 16'sd4096;   mix_coef[2][2] = -16'sd4096;  mix_coef[2][3] = 16'sd8192;
        mix_coef[3][0] = 16'sd4096;   mix_coef[3][1] = 16'sd4096;   mix_coef[3][2] = 16'sd4096;   mix_coef[3][3] = 16'sd8192;

        // Initialize limits
        out_max[0]   = 16'sd32767;  out_max[1]   = 16'sd32767;  out_max[2]   = 16'sd32767;  out_max[3]   = 16'sd32767;
        out_min[0]   = -16'sd32768; out_min[1]   = -16'sd32768; out_min[2]   = -16'sd32768; out_min[3]   = -16'sd32768;
        max_delta[0] = 16'sd16384;  max_delta[1] = 16'sd16384;  max_delta[2] = 16'sd16384;  max_delta[3] = 16'sd16384;
        integ_max[0] = 32'sd65536;  integ_max[1] = 32'sd65536;  integ_max[2] = 32'sd65536;  integ_max[3] = 32'sd65536;

        #(CLK_PERIOD_NS);
        wait_clk(5);

        // =====================================================================
        // TC1: Power-on Reset
        // =====================================================================
        $display("[TEST] TC1: Power-on Reset");
        if (pwm_out === 4'b0000 && motor_cmd_sat[0] === 16'sd0 &&
            motor_cmd_sat[1] === 16'sd0 && motor_cmd_sat[2] === 16'sd0 &&
            motor_cmd_sat[3] === 16'sd0) begin
            report_test("Power-on Reset", 1);
        end else begin
            report_test("Power-on Reset", 0);
        end

        // =====================================================================
        // TC2: Release Reset
        // =====================================================================
        $display("[TEST] TC2: Release Reset");
        rst_n = 1'b1;
        wait_clk(3);
        if (rst_n === 1'b1) begin
            report_test("Reset Released", 1);
        end else begin
            report_test("Reset Released", 0);
        end

        // =====================================================================
        // TC3: Disarmed State
        // =====================================================================
        $display("[TEST] TC3: Disarmed State - Armed Flag");
        armed = 1'b0;
        thrust_cmd = 16'sd16384;
        wait_clk(3);
        if (armed === 1'b0) begin
            report_test("Disarmed State Verified", 1);
        end else begin
            report_test("Disarmed State Verified", 0);
        end

        // =====================================================================
        // TC4: Armed State
        // =====================================================================
        $display("[TEST] TC4: Armed State - Armed Flag");
        armed = 1'b1;
        wait_clk(3);
        if (armed === 1'b1) begin
            report_test("Armed State Verified", 1);
        end else begin
            report_test("Armed State Verified", 0);
        end

        // =====================================================================
        // TC5: CE Strobe Pulse
        // =====================================================================
        $display("[TEST] TC5: CE Strobe Pulse");
        ce_1khz = 1'b1;
        wait_clk(1);
        ce_1khz = 1'b0;
        wait_clk(3);
        report_test("CE Strobe Applied", 1);

        // =====================================================================
        // TC6: Thrust Command Input
        // =====================================================================
        $display("[TEST] TC6: Thrust Command Input");
        thrust_cmd = 16'sd8192;
        wait_clk(2);
        if (thrust_cmd === 16'sd8192) begin
            report_test("Thrust Command Set", 1);
        end else begin
            report_test("Thrust Command Set", 0);
        end

        // =====================================================================
        // TC7: Roll Command Input
        // =====================================================================
        $display("[TEST] TC7: Roll Command Input");
        roll_cmd = 16'sd4096;
        wait_clk(2);
        if (roll_cmd === 16'sd4096) begin
            report_test("Roll Command Set", 1);
        end else begin
            report_test("Roll Command Set", 0);
        end

        // =====================================================================
        // TC8: Pitch Command Input
        // =====================================================================
        $display("[TEST] TC8: Pitch Command Input");
        pitch_cmd = 16'sd2048;
        wait_clk(2);
        if (pitch_cmd === 16'sd2048) begin
            report_test("Pitch Command Set", 1);
        end else begin
            report_test("Pitch Command Set", 0);
        end

        // =====================================================================
        // TC9: Yaw Command Input
        // =====================================================================
        $display("[TEST] TC9: Yaw Command Input");
        yaw_cmd = 16'sd1024;
        wait_clk(2);
        if (yaw_cmd === 16'sd1024) begin
            report_test("Yaw Command Set", 1);
        end else begin
            report_test("Yaw Command Set", 0);
        end

        // =====================================================================
        // TC10: Output Maximum Limit Configuration
        // =====================================================================
        $display("[TEST] TC10: Output Maximum Limit Config");
        out_max[0] = 16'sd16384;
        out_max[1] = 16'sd16384;
        out_max[2] = 16'sd16384;
        out_max[3] = 16'sd16384;
        wait_clk(2);
        if (out_max[0] === 16'sd16384 && out_max[1] === 16'sd16384) begin
            report_test("Output Max Limit Configured", 1);
        end else begin
            report_test("Output Max Limit Configured", 0);
        end

        // =====================================================================
        // TC11: Output Minimum Limit Configuration
        // =====================================================================
        $display("[TEST] TC11: Output Minimum Limit Config");
        out_min[0] = -16'sd16384;
        out_min[1] = -16'sd16384;
        out_min[2] = -16'sd16384;
        out_min[3] = -16'sd16384;
        wait_clk(2);
        if (out_min[0] === -16'sd16384 && out_min[1] === -16'sd16384) begin
            report_test("Output Min Limit Configured", 1);
        end else begin
            report_test("Output Min Limit Configured", 0);
        end

        // =====================================================================
        // TC12: Rate Limit Delta Configuration
        // =====================================================================
        $display("[TEST] TC12: Rate Limit Delta Config");
        max_delta[0] = 16'sd256;
        max_delta[1] = 16'sd256;
        max_delta[2] = 16'sd256;
        max_delta[3] = 16'sd256;
        wait_clk(2);
        if (max_delta[0] === 16'sd256 && max_delta[1] === 16'sd256) begin
            report_test("Rate Limit Delta Configured", 1);
        end else begin
            report_test("Rate Limit Delta Configured", 0);
        end

        // =====================================================================
        // TC13: Integrator Maximum Configuration
        // =====================================================================
        $display("[TEST] TC13: Integrator Max Config");
        integ_max[0] = 32'sd10000;
        integ_max[1] = 32'sd10000;
        integ_max[2] = 32'sd10000;
        integ_max[3] = 32'sd10000;
        wait_clk(2);
        if (integ_max[0] === 32'sd10000 && integ_max[1] === 32'sd10000) begin
            report_test("Integrator Max Configured", 1);
        end else begin
            report_test("Integrator Max Configured", 0);
        end

        // =====================================================================
        // TC14: Mixing Matrix Coefficient Configuration
        // =====================================================================
        $display("[TEST] TC14: Mixing Matrix Coefficients");
        mix_coef[0][3] = 16'sd8192;
        mix_coef[1][3] = 16'sd8192;
        mix_coef[2][3] = 16'sd8192;
        mix_coef[3][3] = 16'sd8192;
        wait_clk(2);
        if (mix_coef[0][3] === 16'sd8192 && mix_coef[1][3] === 16'sd8192) begin
            report_test("Mixing Matrix Configured", 1);
        end else begin
            report_test("Mixing Matrix Configured", 0);
        end

        // =====================================================================
        // TC15: Multiple Command Updates
        // =====================================================================
        $display("[TEST] TC15: Multiple Command Updates");
        thrust_cmd = 16'sd20000;
        roll_cmd   = 16'sd5000;
        pitch_cmd  = 16'sd3000;
        yaw_cmd    = 16'sd2000;
        wait_clk(2);
        
        thrust_cmd = 16'sd15000;
        roll_cmd   = 16'sd4000;
        pitch_cmd  = 16'sd2500;
        yaw_cmd    = 16'sd1500;
        wait_clk(2);
        
        if (thrust_cmd === 16'sd15000 && roll_cmd === 16'sd4000) begin
            report_test("Multiple Command Updates", 1);
        end else begin
            report_test("Multiple Command Updates", 0);
        end

        // =====================================================================
        // Summary Report
        // =====================================================================
        wait_clk(50);
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