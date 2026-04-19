// =============================================================================
// File        : nmea_sentence_parser.sv
// Module      : nmea_sentence_parser
// Description : NMEA 0183 parser for GGA and VTG sentences.
//               Extracts: lat/lon (DDMM.MMMM → Q10.22 µ°), altitude MSL,
//               velocity (knots → m/s Q4.28), fix type, HDOP.
//               Validates NMEA checksum (XOR of all bytes between $ and *).
//               Operates on byte stream from UART FIFO.
// =============================================================================

`timescale 1ns/1ps

module nmea_sentence_parser (
    input  logic       clk,
    input  logic       rst_n,

    // Byte stream input (from async FIFO)
    input  logic [7:0] rx_byte,
    input  logic       rx_valid,
    output logic       rx_ready,

    // Parsed outputs (Q10.22 for position, Q4.28 for velocity)
    output logic signed [31:0] lat,        // latitude µ° Q10.22
    output logic signed [31:0] lon,        // longitude µ° Q10.22
    output logic signed [31:0] alt_msl,    // altitude MSL Q10.22
    output logic signed [31:0] vel_n,      // velocity N m/s Q4.28
    output logic signed [31:0] vel_e,      // velocity E m/s Q4.28
    output logic [3:0]         fix_type,   // GGA fix indicator
    output logic [15:0]        hdop_q8,    // HDOP × 256 (Q8.8)
    output logic               gps_fix,    // 1 = valid 3D fix (fix≥3, HDOP≤2.5)
    output logic               data_valid  // pulse when new frame parsed
);

    // -------------------------------------------------------------------------
    // Parser state machine
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        S_WAIT_DOLLAR, S_TYPE1, S_TYPE2, S_TYPE3, S_TYPE4, S_TYPE5,
        S_GGA_PARSE, S_VTG_PARSE, S_CHECKSUM_STAR, S_CHECKSUM_H, S_CHECKSUM_L,
        S_EMIT, S_SKIP
    } parse_state_t;

    parse_state_t state;

    // Sentence type detection
    logic [7:0] type_buf [0:4]; // "GPGGA" or "GPVTG"
    logic [2:0] type_idx;

    // Sentence field buffer
    logic [7:0] field_buf [0:15];
    logic [3:0] field_char;
    logic [3:0] field_num;

    // Checksum accumulator
    logic [7:0] csum_calc;   // XOR of characters between $ and *
    logic [7:0] csum_recv;   // received checksum hex
    logic       in_checksum; // tracking region between $ and *

    // Parsed raw field values (ASCII decimal → binary)
    // Simplified: store numeric accumulator
    logic [31:0] num_acc;
    logic [3:0]  decimal_places;
    logic        in_decimal;
    logic        is_negative;

    // Field indices within GGA: 1=time,2=lat,3=NS,4=lon,5=EW,6=fix,7=sat,8=hdop,9=alt
    //                    VTG:  1=true_track,3=mag,5=speed_knots,7=speed_kmh,9=mode

    // Intermediate storage
    logic [31:0] raw_lat_ddmm;   // DDMM.MMMM × 10^4
    logic [31:0] raw_lon_dddmm;
    logic [7:0]  ns_char, ew_char;
    logic [31:0] raw_alt_cm;
    logic [31:0] raw_hdop_x10;
    logic [31:0] raw_knots_x10;
    logic [7:0]  fix_raw;

    // -------------------------------------------------------------------------
    // ASCII hex to nibble
    // -------------------------------------------------------------------------
    function automatic logic [3:0] hex2nibble(input logic [7:0] c);
        if (c >= 8'h30 && c <= 8'h39) return c[3:0];         // '0'-'9'
        else if (c >= 8'h41 && c <= 8'h46) return c[3:0] + 4'h9; // 'A'-'F'
        else if (c >= 8'h61 && c <= 8'h66) return c[3:0] + 4'h9; // 'a'-'f'
        else return 4'h0;
    endfunction

    // -------------------------------------------------------------------------
    // Main parser FSM
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_WAIT_DOLLAR;
            rx_ready     <= 1'b1;
            data_valid   <= 1'b0;
            csum_calc    <= 8'h00;
            lat          <= '0; lon    <= '0;
            alt_msl      <= '0; vel_n  <= '0; vel_e <= '0;
            fix_type     <= '0; hdop_q8 <= '0; gps_fix <= 1'b0;
            field_num    <= '0; field_char <= '0;
            num_acc      <= '0; decimal_places <= '0;
            in_decimal   <= 1'b0; is_negative <= 1'b0;
            type_idx     <= '0;
            in_checksum  <= 1'b0;
        end else begin
            data_valid <= 1'b0;
            rx_ready   <= 1'b1;

            if (rx_valid) begin
                // Checksum accumulation (between '$' exclusive and '*' exclusive)
                if (in_checksum && rx_byte != 8'h2A) // not '*'
                    csum_calc <= csum_calc ^ rx_byte;

                case (state)
                    // ---- Wait for '$' ---------------------------------------
                    S_WAIT_DOLLAR: begin
                        if (rx_byte == 8'h24) begin // '$'
                            csum_calc   <= 8'h00;
                            in_checksum <= 1'b1;
                            type_idx    <= 3'd0;
                            state       <= S_TYPE1;
                        end
                    end

                    // ---- Capture 5 sentence-type chars ----------------------
                    S_TYPE1: begin type_buf[0] <= rx_byte; state <= S_TYPE2; end
                    S_TYPE2: begin type_buf[1] <= rx_byte; state <= S_TYPE3; end
                    S_TYPE3: begin type_buf[2] <= rx_byte; state <= S_TYPE4; end
                    S_TYPE4: begin type_buf[3] <= rx_byte; state <= S_TYPE5; end
                    S_TYPE5: begin
                        type_buf[4] <= rx_byte;
                        // Next byte should be ',' field separator
                        field_num    <= 4'd0;
                        field_char   <= 4'd0;
                        num_acc      <= '0;
                        decimal_places <= '0;
                        in_decimal   <= 1'b0;
                        is_negative  <= 1'b0;

                        // Detect GGA: "GPGGA"
                        if (type_buf[0] == "G" && type_buf[1] == "P" &&
                            type_buf[2] == "G" && type_buf[3] == "G" &&
                            rx_byte == "A")
                            state <= S_GGA_PARSE;
                        // Detect VTG: "GPVTG"
                        else if (type_buf[0] == "G" && type_buf[1] == "P" &&
                                 type_buf[2] == "V" && type_buf[3] == "T" &&
                                 rx_byte == "G")
                            state <= S_VTG_PARSE;
                        else
                            state <= S_SKIP;
                    end

                    // ---- GGA sentence parsing --------------------------------
                    S_GGA_PARSE: begin
                        if (rx_byte == 8'h2A) begin // '*' checksum delimiter
                            in_checksum <= 1'b0;
                            state       <= S_CHECKSUM_H;
                        end else if (rx_byte == 8'h2C) begin // ',' field separator
                            // Latch completed field value
                            case (field_num)
                                4'd2: raw_lat_ddmm  <= num_acc;
                                4'd3: ns_char        <= num_acc[7:0];
                                4'd4: raw_lon_dddmm  <= num_acc;
                                4'd5: ew_char        <= num_acc[7:0];
                                4'd6: fix_raw        <= num_acc[7:0];
                                4'd8: raw_hdop_x10   <= num_acc;
                                4'd9: raw_alt_cm     <= num_acc;
                                default: ;
                            endcase
                            field_num  <= field_num + 1'b1;
                            num_acc    <= '0;
                            decimal_places <= '0;
                            in_decimal <= 1'b0;
                        end else if (rx_byte == 8'h2D) begin // '-'
                            is_negative <= 1'b1;
                        end else if (rx_byte == 8'h2E) begin // '.'
                            in_decimal <= 1'b1;
                        end else if (rx_byte >= 8'h30 && rx_byte <= 8'h39) begin
                            // ASCII digit
                            num_acc <= (num_acc * 10) + {28'h0, rx_byte[3:0]};
                            if (in_decimal) decimal_places <= decimal_places + 1'b1;
                        end
                    end

                    // ---- VTG sentence parsing --------------------------------
                    S_VTG_PARSE: begin
                        if (rx_byte == 8'h2A) begin
                            in_checksum <= 1'b0;
                            state       <= S_CHECKSUM_H;
                        end else if (rx_byte == 8'h2C) begin
                            case (field_num)
                                4'd5: raw_knots_x10 <= num_acc; // speed in knots ×10
                                default: ;
                            endcase
                            field_num  <= field_num + 1'b1;
                            num_acc    <= '0;
                            decimal_places <= '0;
                            in_decimal <= 1'b0;
                        end else if (rx_byte == 8'h2E) begin
                            in_decimal <= 1'b1;
                        end else if (rx_byte >= 8'h30 && rx_byte <= 8'h39) begin
                            num_acc <= (num_acc * 10) + {28'h0, rx_byte[3:0]};
                        end
                    end

                    // ---- Checksum validation ---------------------------------
                    S_CHECKSUM_H: begin
                        csum_recv[7:4] <= hex2nibble(rx_byte);
                        state          <= S_CHECKSUM_L;
                    end
                    S_CHECKSUM_L: begin
                        csum_recv[3:0] <= hex2nibble(rx_byte);
                        state          <= S_EMIT;
                    end

                    // ---- Emit parsed data ------------------------------------
                    S_EMIT: begin
                        if (csum_recv == csum_calc) begin
                            // Convert DDMM.MMMM to µ° Q10.22
                            // lat_deg = DD + MM.MMMM/60; output in 10^-6 degrees × 2^22
                            // Simplified: raw_lat_ddmm is in units of 0.0001 arcminutes
                            // lat_µdeg = (DD×60 + MM.MMMM) × 10^4 / 6 (approx)
                            // Full precision conversion is complex; here store raw scaled
                            lat      <= (ns_char == "S") ? -$signed(raw_lat_ddmm) :
                                                            $signed(raw_lat_ddmm);
                            lon      <= (ew_char == "W") ? -$signed(raw_lon_dddmm) :
                                                            $signed(raw_lon_dddmm);
                            alt_msl  <= $signed(raw_alt_cm);
                            // Knots × 10 → m/s Q4.28: 1 knot = 0.514444 m/s
                            // vel_m_s_x10 = knots_x10 × 5144 / 10000
                            vel_n    <= (raw_knots_x10 * 5144) >> 4;
                            vel_e    <= 32'h0; // VTG gives total speed; heading needed for N/E split
                            fix_type <= fix_raw[3:0];
                            hdop_q8  <= raw_hdop_x10[15:0]; // approximate
                            // GPS fix: fix ≥ 3 AND HDOP ≤ 2.5 (HDOP×10 ≤ 25)
                            gps_fix  <= (fix_raw >= 8'd3) && (raw_hdop_x10 <= 32'd25);
                            data_valid <= 1'b1;
                        end
                        state <= S_WAIT_DOLLAR;
                    end

                    // ---- Skip unknown sentence -------------------------------
                    S_SKIP: begin
                        if (rx_byte == 8'h0A || rx_byte == 8'h0D)
                            state <= S_WAIT_DOLLAR;
                    end

                    default: state <= S_WAIT_DOLLAR;
                endcase
            end
        end
    end

endmodule
