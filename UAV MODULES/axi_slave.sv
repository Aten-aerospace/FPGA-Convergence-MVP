// =============================================================================
// File        : axi_slave.sv
// Module      : axi_slave
// Description : AXI4-Lite slave interface (32-bit data, 256 byte address space).
//               Provides standard AXI4-Lite handshaking for write/read channels.
//               Drives internal register file via simplified single-cycle bus.
// =============================================================================

`timescale 1ns/1ps

module axi_slave #(
    parameter int ADDR_W = 8,   // 8 = 256-byte address space
    parameter int DATA_W = 32
)(
    input  logic clk,
    input  logic rst_n,

    // AXI4-Lite Write Address Channel
    input  logic [ADDR_W-1:0] awaddr,
    input  logic               awvalid,
    output logic               awready,

    // AXI4-Lite Write Data Channel
    input  logic [DATA_W-1:0] wdata,
    input  logic [3:0]         wstrb,
    input  logic               wvalid,
    output logic               wready,

    // AXI4-Lite Write Response Channel
    output logic [1:0] bresp,
    output logic       bvalid,
    input  logic       bready,

    // AXI4-Lite Read Address Channel
    input  logic [ADDR_W-1:0] araddr,
    input  logic               arvalid,
    output logic               arready,

    // AXI4-Lite Read Data Channel
    output logic [DATA_W-1:0] rdata,
    output logic [1:0]         rresp,
    output logic               rvalid,
    input  logic               rready,

    // Internal register bus
    output logic [ADDR_W-1:0] reg_waddr,
    output logic [DATA_W-1:0] reg_wdata,
    output logic               reg_wen,
    output logic [ADDR_W-1:0] reg_raddr,
    input  logic [DATA_W-1:0] reg_rdata,
    output logic               reg_ren
);

    // -------------------------------------------------------------------------
    // Write channel FSM
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] { WR_IDLE, WR_ADDR, WR_DATA, WR_RESP } wr_state_t;
    wr_state_t wr_st;

    logic [ADDR_W-1:0] wr_addr_lat;
    logic [DATA_W-1:0] wr_data_lat;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_st       <= WR_IDLE;
            awready     <= 1'b1;
            wready      <= 1'b1;
            bvalid      <= 1'b0;
            bresp       <= 2'b00; // OKAY
            reg_wen     <= 1'b0;
        end else begin
            reg_wen  <= 1'b0;

            case (wr_st)
                WR_IDLE: begin
                    awready <= 1'b1;
                    wready  <= 1'b1;
                    if (awvalid && wvalid) begin
                        wr_addr_lat <= awaddr;
                        wr_data_lat <= wdata;
                        awready     <= 1'b0;
                        wready      <= 1'b0;
                        wr_st       <= WR_RESP;
                    end else if (awvalid) begin
                        wr_addr_lat <= awaddr;
                        awready     <= 1'b0;
                        wr_st       <= WR_DATA;
                    end
                end
                WR_DATA: begin
                    if (wvalid) begin
                        wr_data_lat <= wdata;
                        wready      <= 1'b0;
                        wr_st       <= WR_RESP;
                    end
                end
                WR_RESP: begin
                    reg_waddr <= wr_addr_lat;
                    reg_wdata <= wr_data_lat;
                    reg_wen   <= 1'b1;
                    bvalid    <= 1'b1;
                    bresp     <= 2'b00;
                    wr_st     <= WR_IDLE;
                    awready   <= 1'b1;
                    wready    <= 1'b1;
                    if (bready) bvalid <= 1'b0;
                end
                default: wr_st <= WR_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Read channel
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arready <= 1'b1;
            rvalid  <= 1'b0;
            rdata   <= '0;
            rresp   <= 2'b00;
            reg_ren <= 1'b0;
        end else begin
            reg_ren <= 1'b0;
            if (arvalid && arready) begin
                reg_raddr <= araddr;
                reg_ren   <= 1'b1;
                arready   <= 1'b0;
            end
            if (reg_ren) begin
                rdata  <= reg_rdata;
                rvalid <= 1'b1;
                rresp  <= 2'b00;
            end
            if (rvalid && rready) begin
                rvalid  <= 1'b0;
                arready <= 1'b1;
            end
        end
    end

endmodule
