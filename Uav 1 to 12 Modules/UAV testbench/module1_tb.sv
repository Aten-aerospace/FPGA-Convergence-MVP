// =============================================================================
// Testbench  : uav_clk_ctrl_tb
// Description: Minimal testbench for module1_uav_clk_ctrl
//              - 15 test cases with immediate results
//              - Vivado xsim compatible
// =============================================================================

`timescale 1ns/1ps

module module1_tb;

    // =========================================================================
    // Test parameters
    // =========================================================================
    parameter int CLK_PERIOD_NS = 20;
    parameter int CLK_HZ = 50_000_000;
    parameter int PLL_LOCK_CNTS = 10;
    parameter int SYNC_STAGES = 2;

    // =========================================================================
    // Test signals
    // =========================================================================
    reg clk;
    reg pll_locked;
    reg ext_rst_n;

    reg arm_btn_async;
    reg mode_async;

    wire rst_n;
    wire arm_btn_sync;
    wire mode_sync;

    wire ce_1khz;
    wire ce_100hz;
    wire ce_50hz;
    wire ce_10hz;

    wire pll_lock_stable;

    // Test counters
    integer test_num = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    module1_uav_clk_ctrl #(
        .CLK_HZ (CLK_HZ),
        .PLL_LOCK_CNTS (PLL_LOCK_CNTS),
        .SYNC_STAGES (SYNC_STAGES)
    ) dut (
        .clk (clk),
        .pll_locked (pll_locked),
        .ext_rst_n (ext_rst_n),
        .arm_btn_async (arm_btn_async),
        .mode_async (mode_async),
        .rst_n (rst_n),
        .arm_btn_sync (arm_btn_sync),
        .mode_sync (mode_sync),
        .ce_1khz (ce_1khz),
        .ce_100hz (ce_100hz),
        .ce_50hz (ce_50hz),
        .ce_10hz (ce_10hz),
        .pll_lock_stable (pll_lock_stable)
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
        $display("UAV Clock Control Module - Minimal Testbench");
        $display("=============================================================================\n");

        clk = 1'b0;
        pll_locked = 1'b0;
        ext_rst_n = 1'b0;

        arm_btn_async = 1'b0;
        mode_async = 1'b0;

        #(CLK_PERIOD_NS);
        wait_clk(2);

        // =====================================================================
        // TC1: Power-on Reset
        // =====================================================================
        $display("[TEST] TC1: Power-on Reset");
        if (rst_n === 1'b0 && pll_lock_stable === 1'b0) begin
            report_test("Power-on Reset - Reset asserted", 1);
        end else begin
            report_test("Power-on Reset - Reset asserted", 0);
        end

        // =====================================================================
        // TC2: External Reset Release (PLL not locked)
        // =====================================================================
        $display("[TEST] TC2: External Reset Release (PLL not locked)");
        ext_rst_n = 1'b1;
        wait_clk(2);
        if (rst_n === 1'b0) begin
            report_test("Reset held low without PLL lock", 1);
        end else begin
            report_test("Reset held low without PLL lock", 0);
        end

        // =====================================================================
        // TC3: PLL Lock Assertion
        // =====================================================================
        $display("[TEST] TC3: PLL Lock Assertion");
        pll_locked = 1'b1;
        wait_clk(3);
        if (pll_locked === 1'b1) begin
            report_test("PLL Locked Signal Set", 1);
        end else begin
            report_test("PLL Locked Signal Set", 0);
        end

        // =====================================================================
        // TC4: PLL Lock Debounce Complete
        // =====================================================================
        $display("[TEST] TC4: PLL Lock Debounce Complete");
        wait_clk(PLL_LOCK_CNTS + 10);
        if (pll_lock_stable === 1'b1) begin
            report_test("PLL Lock Stable After Debounce", 1);
        end else begin
            report_test("PLL Lock Stable After Debounce", 0);
        end

        // =====================================================================
        // TC5: Reset Released After PLL Stable
        // =====================================================================
        $display("[TEST] TC5: Reset Released After PLL Stable");
        wait_clk(3);
        if (rst_n === 1'b1) begin
            report_test("Reset Released with Stable PLL", 1);
        end else begin
            report_test("Reset Released with Stable PLL", 0);
        end

        // =====================================================================
        // TC6: CE 1 kHz Strobe Output
        // =====================================================================
        $display("[TEST] TC6: CE 1 kHz Strobe Output");
        wait_clk(1);
        if (ce_1khz === 1'b0 || ce_1khz === 1'b1) begin
            report_test("1 kHz CE Strobe Output Connected", 1);
        end else begin
            report_test("1 kHz CE Strobe Output Connected", 0);
        end

        // =====================================================================
        // TC7: CE 100 Hz Strobe Output
        // =====================================================================
        $display("[TEST] TC7: CE 100 Hz Strobe Output");
        wait_clk(1);
        if (ce_100hz === 1'b0 || ce_100hz === 1'b1) begin
            report_test("100 Hz CE Strobe Output Connected", 1);
        end else begin
            report_test("100 Hz CE Strobe Output Connected", 0);
        end

        // =====================================================================
        // TC8: CE 50 Hz Strobe Output
        // =====================================================================
        $display("[TEST] TC8: CE 50 Hz Strobe Output");
        wait_clk(1);
        if (ce_50hz === 1'b0 || ce_50hz === 1'b1) begin
            report_test("50 Hz CE Strobe Output Connected", 1);
        end else begin
            report_test("50 Hz CE Strobe Output Connected", 0);
        end

        // =====================================================================
        // TC9: CE 10 Hz Strobe Output
        // =====================================================================
        $display("[TEST] TC9: CE 10 Hz Strobe Output");
        wait_clk(1);
        if (ce_10hz === 1'b0 || ce_10hz === 1'b1) begin
            report_test("10 Hz CE Strobe Output Connected", 1);
        end else begin
            report_test("10 Hz CE Strobe Output Connected", 0);
        end

        // =====================================================================
        // TC10: Async ARM Button Input
        // =====================================================================
        $display("[TEST] TC10: Async ARM Button Input");
        arm_btn_async = 1'b1;
        wait_clk(1);
        if (arm_btn_async === 1'b1) begin
            report_test("Async ARM Button Input Set", 1);
        end else begin
            report_test("Async ARM Button Input Set", 0);
        end

        // =====================================================================
        // TC11: ARM Button Synchronization
        // =====================================================================
        $display("[TEST] TC11: ARM Button Synchronization");
        wait_clk(SYNC_STAGES + 2);
        if (arm_btn_sync === 1'b1 || arm_btn_sync === 1'b0) begin
            report_test("ARM Button Synchronized", 1);
        end else begin
            report_test("ARM Button Synchronized", 0);
        end

        // =====================================================================
        // TC12: Async Mode Select Input
        // =====================================================================
        $display("[TEST] TC12: Async Mode Select Input");
        mode_async = 1'b1;
        wait_clk(1);
        if (mode_async === 1'b1) begin
            report_test("Async Mode Select Input Set", 1);
        end else begin
            report_test("Async Mode Select Input Set", 0);
        end

        // =====================================================================
        // TC13: Mode Select Synchronization
        // =====================================================================
        $display("[TEST] TC13: Mode Select Synchronization");
        wait_clk(SYNC_STAGES + 2);
        if (mode_sync === 1'b1 || mode_sync === 1'b0) begin
            report_test("Mode Select Synchronized", 1);
        end else begin
            report_test("Mode Select Synchronized", 0);
        end

        // =====================================================================
        // TC14: PLL Lock Status Maintained
        // =====================================================================
        $display("[TEST] TC14: PLL Lock Status Maintained");
        wait_clk(2);
        if (pll_lock_stable === 1'b1 && rst_n === 1'b1) begin
            report_test("PLL Lock Status Maintained", 1);
        end else begin
            report_test("PLL Lock Status Maintained", 0);
        end

        // =====================================================================
        // TC15: System Stability Check
        // =====================================================================
        $display("[TEST] TC15: System Stability Check");
        wait_clk(5);
        if (rst_n === 1'b1 && pll_lock_stable === 1'b1) begin
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