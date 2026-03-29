`timescale 1ns/1ps

module crc_calc #(
    parameter integer CRC_TYPE = 16  // 16 or 32
)(
    input  logic clk,
    input  logic rst_n,

    input  logic [7:0] data_in,
    input  logic       data_valid,
    input  logic       crc_reset,

    output logic [CRC_TYPE-1:0] crc_out
);

    // ─────────────────────────────────────────────────────────────
    // Local parameters
    // ─────────────────────────────────────────────────────────────
    localparam int WIDTH = CRC_TYPE;

    // Polynomials
    localparam logic [31:0] POLY_16 = 32'h00001021;
    localparam logic [31:0] POLY_32 = 32'h04C11DB7;

    // Initial values
    localparam logic [31:0] INIT_16 = 32'h0000FFFF;
    localparam logic [31:0] INIT_32 = 32'hFFFFFFFF;

    // Select active poly/init
    logic [WIDTH-1:0] poly;
    logic [WIDTH-1:0] init_val;

    always_comb begin
        if (CRC_TYPE == 16) begin
            poly     = POLY_16[WIDTH-1:0];
            init_val = INIT_16[WIDTH-1:0];
        end else begin
            poly     = POLY_32[WIDTH-1:0];
            init_val = INIT_32[WIDTH-1:0];
        end
    end

    // ─────────────────────────────────────────────────────────────
    // CRC register
    // ─────────────────────────────────────────────────────────────
    logic [WIDTH-1:0] crc_reg;
    logic [WIDTH-1:0] crc_next;

    assign crc_out = crc_reg;

    // ─────────────────────────────────────────────────────────────
    // Byte processing (8-bit serial loop)
    // ─────────────────────────────────────────────────────────────
    integer i;
    logic bit_in;
    logic msb;

    always_comb begin
        crc_next = crc_reg;

        if (data_valid) begin
            // Process 8 bits (MSB first)
            for (i = 0; i < 8; i++) begin
                bit_in = data_in[7 - i];
                msb    = crc_next[WIDTH-1];

                crc_next = {crc_next[WIDTH-2:0], bit_in};

                if (msb)
                    crc_next = crc_next ^ poly;
            end
        end
    end

    // ─────────────────────────────────────────────────────────────
    // Sequential update
    // ─────────────────────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_reg <= init_val;
        end else if (crc_reset) begin
            crc_reg <= init_val;
        end else begin
            crc_reg <= crc_next;
        end
    end

endmodule
