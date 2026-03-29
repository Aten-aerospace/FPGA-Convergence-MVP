`timescale 1ns / 1ps

module async_fifo #(
    parameter DATA_W = 32,
    parameter DEPTH  = 16  // must be power of 2
)(
    // Write domain
    input  logic                 wr_clk,
    input  logic                 wr_rst_n,
    input  logic [DATA_W-1:0]    wr_data,
    input  logic                 wr_en,
    output logic                 wr_full,

    // Read domain
    input  logic                 rd_clk,
    input  logic                 rd_rst_n,
    output logic [DATA_W-1:0]    rd_data,
    input  logic                 rd_en,
    output logic                 rd_empty
);

    // ============================================================
    // Derived params
    // ============================================================
    localparam ADDR_W = $clog2(DEPTH);

    // ============================================================
    // Memory (dual-port RAM inference)
    // ============================================================
    logic [DATA_W-1:0] mem [0:DEPTH-1];

    // ============================================================
    // Binary + Gray pointers
    // ============================================================
    logic [ADDR_W:0] wr_ptr_bin, wr_ptr_bin_next;
    logic [ADDR_W:0] rd_ptr_bin, rd_ptr_bin_next;

    logic [ADDR_W:0] wr_ptr_gray, wr_ptr_gray_next;
    logic [ADDR_W:0] rd_ptr_gray, rd_ptr_gray_next;

    // ============================================================
    // Synchronizers (Gray pointers crossing domains)
    // ============================================================
    logic [ADDR_W:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;
    logic [ADDR_W:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2;

    // ============================================================
    // Gray encode function
    // ============================================================
    function automatic [ADDR_W:0] bin2gray(input [ADDR_W:0] b);
        return (b >> 1) ^ b;
    endfunction

    // ============================================================
    // WRITE DOMAIN
    // ============================================================
    always_comb begin
        wr_ptr_bin_next  = wr_ptr_bin + (wr_en & ~wr_full);
        wr_ptr_gray_next = bin2gray(wr_ptr_bin_next);
    end

    // Write pointer
    always_ff @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr_bin  <= '0;
            wr_ptr_gray <= '0;
        end else begin
            wr_ptr_bin  <= wr_ptr_bin_next;
            wr_ptr_gray <= wr_ptr_gray_next;
        end
    end

    // Memory write
    always_ff @(posedge wr_clk) begin
        if (wr_en && !wr_full) begin
            mem[wr_ptr_bin[ADDR_W-1:0]] <= wr_data;
        end
    end

    // Sync read pointer into write domain
    always_ff @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_ptr_gray_sync1 <= '0;
            rd_ptr_gray_sync2 <= '0;
        end else begin
            rd_ptr_gray_sync1 <= rd_ptr_gray;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
        end
    end

    // FULL detection
    assign wr_full =
        (wr_ptr_gray_next == {
            ~rd_ptr_gray_sync2[ADDR_W:ADDR_W-1],
             rd_ptr_gray_sync2[ADDR_W-2:0]
        });

    // ============================================================
    // READ DOMAIN
    // ============================================================
    always_comb begin
        rd_ptr_bin_next  = rd_ptr_bin + (rd_en & ~rd_empty);
        rd_ptr_gray_next = bin2gray(rd_ptr_bin_next);
    end

    // Read pointer
    always_ff @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_bin  <= '0;
            rd_ptr_gray <= '0;
        end else begin
            rd_ptr_bin  <= rd_ptr_bin_next;
            rd_ptr_gray <= rd_ptr_gray_next;
        end
    end

    // Memory read (registered output)
    always_ff @(posedge rd_clk) begin
        if (rd_en && !rd_empty) begin
            rd_data <= mem[rd_ptr_bin[ADDR_W-1:0]];
        end
    end

    // Sync write pointer into read domain
    always_ff @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_ptr_gray_sync1 <= '0;
            wr_ptr_gray_sync2 <= '0;
        end else begin
            wr_ptr_gray_sync1 <= wr_ptr_gray;
            wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
        end
    end

    // EMPTY detection
    assign rd_empty = (rd_ptr_gray == wr_ptr_gray_sync2);

endmodule
