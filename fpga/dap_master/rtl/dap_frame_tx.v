/*
 * Assemble and clock out one DAP frame.
 *
 * A frame is, in transmission order and all LSB first:
 *
 *     start bit (1) | CMD (5) | LEN (6) | DATA (LEN) | CRC6 | trailing zero
 *
 * The CRC covers CMD, LEN and DATA - not the start bit, not itself, not the
 * trailing zero.  Bit order is the part that bites: the documented sync wire
 * word 0x09FE1 only comes out if the *first bit on the wire is bit 0* of the
 * integer.  The other convention gives 0x23FC8, so that vector doubles as a
 * statement about byte order and is what tb_dap_frame checks.
 *
 * Two clocks with the line held low precede every frame, not just the first.
 * That was measured, not documented: a frame sent straight after the previous
 * exchange is silently discarded, and that holds for sync itself.  The device
 * needs the idle run to recognise a frame start.
 *
 * dap0 is generated here rather than left to the caller so the frame and its
 * clock cannot drift apart: one bit is presented, then one full clock period.
 * The target latches on the rising edge.
 */

`default_nettype none

module dap_frame_tx #(
    /* Clock divider: dap0 period is 2*(DIV+1) system clocks, so the bit rate
     * is sysclk / (2*(DIV+1)).  Runtime-settable by the register interface. */
    parameter integer DIV_WIDTH = 8
) (
    input  wire                  clk,
    input  wire                  rst,

    input  wire [DIV_WIDTH-1:0]  div,
    input  wire                  start,      /* pulse to begin a frame */
    input  wire [4:0]            cmd,
    /*
     * LEN is the field that goes on the wire; data_bits is how many DATA bits
     * actually follow it.  They are not the same thing and conflating them is
     * a real trap: sync carries LEN 63 with *no* data at all, so a serialiser
     * that treats LEN as the payload width sends 63 bits of nothing and the
     * device ignores the frame.
     */
    input  wire [5:0]            len,
    input  wire [5:0]            data_bits,
    input  wire [62:0]           data,

    output reg                   busy,
    output reg                   done,       /* one-cycle pulse at the end */

    /* To the pads.  dat_oe low hands the line to the target. */
    output reg                   dap0,
    output reg                   dap1,
    output reg                   dat_oe
);
    localparam [2:0] S_IDLE  = 3'd0,
                     S_LEAD  = 3'd1,   /* the two low clocks before the frame */
                     S_START = 3'd2,
                     S_CMD   = 3'd3,
                     S_LEN   = 3'd4,
                     S_DATA  = 3'd5,
                     S_CRC   = 3'd6,
                     S_TRAIL = 3'd7;

    /* Clocks with the line low that precede every frame.  Two is enough: the
     * requirement is one device turnaround cycle plus DAPISC.SISP, which
     * defaults to zero.  The reference probe's eleven are a host-side buffer
     * for its own latency, not a device requirement. */
    localparam integer LEAD_CLOCKS = 2;

    reg [2:0]           state;
    reg [5:0]           index;      /* bit position within the current field */
    reg [DIV_WIDTH-1:0] tick;
    reg                 phase;      /* 0 = first half (dap0 low), 1 = second */

    reg [4:0]  cmd_r;
    reg [5:0]  len_r;
    reg [5:0]  nbits_r;
    reg [62:0] data_r;

    /* The bit currently being presented, and whether the CRC should eat it. */
    reg        cur_bit;
    reg        crc_en;
    reg        crc_rst;
    wire [5:0] crc;

    dap_crc6_gen u_crc (
        .clk    (clk),
        .rst    (crc_rst),
        .en     (crc_en),
        .bit_in (cur_bit),
        .crc    (crc)
    );

    wire tick_done = (tick == div);

    /* What goes on the wire for the current state and index. */
    always @(*) begin
        case (state)
            S_START: cur_bit = 1'b1;
            S_CMD:   cur_bit = cmd_r[index[2:0]];
            S_LEN:   cur_bit = len_r[index[2:0]];
            S_DATA:  cur_bit = data_r[index];
            S_CRC:   cur_bit = crc[index[2:0]];
            default: cur_bit = 1'b0;        /* lead-in and trailing zero */
        endcase
    end

    /* The CRC eats CMD, LEN and DATA only, and only once per bit - on the
     * transition into the second half of the bit period. */
    always @(*) begin
        crc_en = 1'b0;
        if (phase == 1'b0 && tick_done) begin
            case (state)
                S_CMD, S_LEN, S_DATA: crc_en = 1'b1;
                default:              crc_en = 1'b0;
            endcase
        end
    end

    always @(posedge clk) begin
        crc_rst <= 1'b0;
        done    <= 1'b0;

        if (rst) begin
            state  <= S_IDLE;
            busy   <= 1'b0;
            dap0   <= 1'b0;
            dap1   <= 1'b1;      /* parked idle high, probe driving */
            dat_oe <= 1'b1;
            tick   <= {DIV_WIDTH{1'b0}};
            phase  <= 1'b0;
            index  <= 6'd0;
        end else if (state == S_IDLE) begin
            dap0 <= 1'b0;
            if (start) begin
                cmd_r   <= cmd;
                len_r   <= len;
                nbits_r <= data_bits;
                data_r  <= data;
                crc_rst <= 1'b1;
                busy    <= 1'b1;
                dat_oe  <= 1'b1;
                state   <= S_LEAD;
                index   <= 6'd0;
                tick    <= {DIV_WIDTH{1'b0}};
                phase   <= 1'b0;
            end
        end else begin
            /*
             * One bit per two half-periods: present the bit with dap0 low,
             * then raise dap0 for the second half.  The target latches on that
             * rising edge, so the data is already stable when it arrives.
             */
            dap1 <= cur_bit;

            if (!tick_done) begin
                tick <= tick + 1'b1;
            end else begin
                tick <= {DIV_WIDTH{1'b0}};

                if (phase == 1'b0) begin
                    dap0  <= 1'b1;
                    phase <= 1'b1;
                end else begin
                    dap0  <= 1'b0;
                    phase <= 1'b0;

                    /* Advance to the next bit, and the next field when this
                     * one runs out. */
                    case (state)
                        S_LEAD: begin
                            if (index == LEAD_CLOCKS - 1) begin
                                state <= S_START;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_START: begin
                            state <= S_CMD;
                            index <= 6'd0;
                        end
                        S_CMD: begin
                            if (index == 6'd4) begin
                                state <= S_LEN;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_LEN: begin
                            if (index == 6'd5) begin
                                /* Straight to the CRC when nothing follows -
                                 * which is sync's case, LEN 63 and no data. */
                                state <= (nbits_r == 6'd0) ? S_CRC : S_DATA;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_DATA: begin
                            if (index == nbits_r - 1'b1) begin
                                state <= S_CRC;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_CRC: begin
                            if (index == 6'd5) begin
                                state <= S_TRAIL;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_TRAIL: begin
                            state <= S_IDLE;
                            busy  <= 1'b0;
                            done  <= 1'b1;
                        end
                        default: state <= S_IDLE;
                    endcase
                end
            end
        end
    end
endmodule

`default_nettype wire
