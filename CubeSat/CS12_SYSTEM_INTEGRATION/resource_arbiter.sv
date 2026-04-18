// =============================================================================
// Module: resource_arbiter
// Subsystem: CS12 - System Integration
// Description: Fixed-priority arbiter for shared FPGA bus resources.
//              Manages access to shared SPI buses (NUM_SPI masters) and
//              shared I2C buses (NUM_I2C masters).
//
//   Priority: lower requester index = higher priority.
//   spi_grant is one-hot: only one SPI master is granted at a time.
//   i2c_grant is one-hot: only one I2C master is granted at a time.
//   Once granted, access is held until the requester de-asserts its request.
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module resource_arbiter #(
    parameter int NUM_SPI = 3,
    parameter int NUM_I2C = 1
)(
    input  logic                  clk,
    input  logic                  rst_n,

    // SPI arbiter
    input  logic [NUM_SPI-1:0]    spi_req,
    output logic [NUM_SPI-1:0]    spi_grant,
    output logic                  spi_busy,

    // I2C arbiter
    input  logic [NUM_I2C-1:0]    i2c_req,
    output logic [NUM_I2C-1:0]    i2c_grant,
    output logic                  i2c_busy
);

    // =========================================================================
    // SPI fixed-priority arbiter
    // =========================================================================
    logic [NUM_SPI-1:0] spi_grant_next;

    always_comb begin
        spi_grant_next = '0;
        // Walk from lowest index (highest priority) upward
        for (int i = NUM_SPI-1; i >= 0; i--) begin
            if (spi_req[i]) spi_grant_next = NUM_SPI'(1 << i);
        end
        // Hold grant while requester is still active
        // (already handled: re-evaluate combinationally each cycle)
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_grant <= '0;
        end else begin
            // If current grantee is still requesting, keep grant.
            // Otherwise re-arbitrate.
            if (|(spi_grant & spi_req)) begin
                spi_grant <= spi_grant; // hold
            end else begin
                spi_grant <= spi_grant_next;
            end
        end
    end

    always_comb spi_busy = |spi_grant;

    // =========================================================================
    // I2C fixed-priority arbiter
    // =========================================================================
    logic [NUM_I2C-1:0] i2c_grant_next;

    always_comb begin
        i2c_grant_next = '0;
        for (int i = NUM_I2C-1; i >= 0; i--) begin
            if (i2c_req[i]) i2c_grant_next = NUM_I2C'(1 << i);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i2c_grant <= '0;
        end else begin
            if (|(i2c_grant & i2c_req)) begin
                i2c_grant <= i2c_grant;
            end else begin
                i2c_grant <= i2c_grant_next;
            end
        end
    end

    always_comb i2c_busy = |i2c_grant;

endmodule
