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
    /*
     * Clocks with the line low before the frame proper.
     *
     * A register rather than a constant, for the same reason TRAIL and MAXWAIT
     * are: it is the one part of the framing that was found empirically, the
     * notes on the CPU path record eleven as the measured value while the
     * constant here said two, and the next target will have to be swept again.
     */
    input  wire [5:0]            lead,

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

    reg [2:0]           state;
    reg [5:0]           index;      /* bit position within the current field */
    reg [DIV_WIDTH-1:0] tick;
    reg                 phase;      /* 0 = first half (dap0 low), 1 = second */

    reg [4:0]  cmd_r;
    reg [5:0]  len_r;
    reg [5:0]  nbits_r;
    reg [5:0]  lead_r;
    /*
     * The last index of each variable-length field, worked out once when the
     * frame is loaded.
     *
     * Written inline as `index == nbits_r - 1` it is a six-bit subtract feeding
     * a six-bit compare feeding the index update, every bit - a carry chain in
     * the middle of the hot path, and the design's critical path once the
     * register file was pipelined.  The subtraction happens once per frame
     * instead, where it has a whole bit period to settle.
     */
    reg [5:0]  nbits_last;
    reg [5:0]  lead_last;
    reg [62:0] data_r;
    /* The finished CRC, latched when the payload runs out and then shifted out
     * like every other field. */
    reg [5:0]  crc_sr;

    /* The bit currently being presented, and whether the CRC should eat it. */
    reg        cur_bit;
    reg        crc_en;
    reg        crc_rst;
    wire [5:0] crc;

    dap_crc6_gen u_crc (
        .clk    (clk),
        .rst    (crc_rst),
        .en     (crc_en),
        .bit_in (cur_bit), .bit_in2 (1'b0), .wide (1'b0),
        .crc    (crc)
    );

    /* Registered for the same reason as the receiver's: as a comparison it
     * feeds clock enables, and that is what place and route answered by
     * putting the enable on a global net. */
    reg tick_done;

    /*
     * What goes on the wire: always the bottom bit of the field being sent.
     *
     * The fields shift right as they go out rather than being indexed by a
     * counter.  Indexing reads better, but data_r[index] over a 63-bit
     * register is a 63-to-1 multiplexer feeding the CRC generator, and that
     * mux was the design's critical path once the enable chains were fixed -
     * five LUT levels of it.  Shifting costs the same flip-flops and leaves a
     * four-way choice between five single bits.
     */
    always @(*) begin
        case (state)
            S_START: cur_bit = 1'b1;
            S_CMD:   cur_bit = cmd_r[0];
            S_LEN:   cur_bit = len_r[0];
            S_DATA:  cur_bit = data_r[0];
            S_CRC:   cur_bit = crc_sr[0];
            default: cur_bit = 1'b0;        /* lead-in and trailing zero */
        endcase
    end

    /*
     * The bit that becomes current at the end of this one.
     *
     * Needed because DAP1 has to change *on* the falling edge of DAP0, not a
     * clock after it.  Assigning dap1 from cur_bit every cycle leaves only
     * (half period - 1) clocks of setup before the target latches on the
     * rising edge, which is fine at a divider of 1 and is zero at a divider of
     * 0 - so the fastest bit rate the clock generator can produce was
     * unreachable for want of one clock.
     *
     * Within a field this is just the next bit of the shift register, since
     * the fields shift right as they go out; only the field boundaries need
     * saying, and they mirror the state advance below.
     */
    reg next_bit;

    always @(*) begin
        case (state)
            S_LEAD:  next_bit = (index == lead_last) ? 1'b1 : 1'b0;
            S_START: next_bit = cmd_r[0];
            S_CMD:   next_bit = (index == 6'd4) ? len_r[0] : cmd_r[1];
            S_LEN:   next_bit = (index == 6'd5)
                              ? ((nbits_r == 6'd0) ? crc[0] : data_r[0])
                              : len_r[1];
            S_DATA:  next_bit = (index == nbits_last) ? crc[0] : data_r[1];
            S_CRC:   next_bit = (index == 6'd5) ? 1'b0 : crc_sr[1];
            default: next_bit = 1'b0;      /* the trailing zero, then idle */
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

        /* index is loaded at the start of every frame, so it is left out -
         * see the note in the receiver on why this reset list is short. */
        if (rst) begin
            state  <= S_IDLE;
            busy   <= 1'b0;
            dap0   <= 1'b0;
            dap1   <= 1'b1;      /* parked idle high, probe driving */
            dat_oe <= 1'b1;
            tick      <= {DIV_WIDTH{1'b0}};
            tick_done <= (div == {DIV_WIDTH{1'b0}});
            phase     <= 1'b0;
        end else if (state == S_IDLE) begin
            dap0 <= 1'b0;
            if (start) begin
                cmd_r   <= cmd;
                len_r   <= len;
                nbits_r    <= data_bits;
                nbits_last <= data_bits - 1'b1;
                lead_r     <= lead;
                lead_last  <= lead - 1'b1;
                data_r  <= data;
                crc_rst <= 1'b1;
                busy    <= 1'b1;
                dat_oe  <= 1'b1;
                /* Zero lead clocks means straight into the start bit. */
                state   <= (lead == 6'd0) ? S_START : S_LEAD;
                /* The first bit has to be on the wire before the first rising
                 * edge, which is a whole low phase away. */
                dap1    <= (lead == 6'd0) ? 1'b1 : 1'b0;
                index     <= 6'd0;
                tick      <= {DIV_WIDTH{1'b0}};
                tick_done <= (div == {DIV_WIDTH{1'b0}});
                phase     <= 1'b0;
            end
        end else begin
            /*
             * One bit per two half-periods: present the bit with dap0 low,
             * then raise dap0 for the second half.  The target latches on that
             * rising edge, so the data is already stable when it arrives.
             *
             * DAP1 is driven only at the falling edge below, where the field
             * registers shift, so the data and the clock change together and
             * the whole low phase is setup time.
             */
            if (!tick_done) begin
                tick      <= tick + 1'b1;
                tick_done <= (tick + 1'b1 == div);
            end else begin
                tick      <= {DIV_WIDTH{1'b0}};
                tick_done <= (div == {DIV_WIDTH{1'b0}});

                if (phase == 1'b0) begin
                    dap0  <= 1'b1;
                    phase <= 1'b1;
                end else begin
                    dap0  <= 1'b0;
                    phase <= 1'b0;
                    dap1  <= next_bit;   /* changes with the falling edge */

                    /* Advance to the next bit, and the next field when this
                     * one runs out. */
                    case (state)
                        S_LEAD: begin
                            if (index == lead_last) begin
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
                            cmd_r <= {1'b0, cmd_r[4:1]};
                            if (index == 6'd4) begin
                                state <= S_LEN;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_LEN: begin
                            len_r <= {1'b0, len_r[5:1]};
                            if (index == 6'd5) begin
                                /* Straight to the CRC when nothing follows -
                                 * which is sync's case, LEN 63 and no data.
                                 * The generator has already absorbed this
                                 * bit - it does that on the phase-0 edge -
                                 * so `crc` is final and can be latched. */
                                state  <= (nbits_r == 6'd0) ? S_CRC : S_DATA;
                                crc_sr <= crc;
                                index  <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_DATA: begin
                            data_r <= {1'b0, data_r[62:1]};
                            if (index == nbits_last) begin
                                state  <= S_CRC;
                                crc_sr <= crc;
                                index  <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_CRC: begin
                            crc_sr <= {1'b0, crc_sr[5:1]};
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
