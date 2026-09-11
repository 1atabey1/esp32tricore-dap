/*
 * Clock in one DAP reply.
 *
 * A reply is, in order: busy stuffing (the target holding DAP1 low), a start
 * bit, the payload LSB first, and a CRC6.  A bare acknowledge - client_set's,
 * for one - is the start bit and nothing else.
 *
 * Three things here were learned the hard way on silicon rather than read out
 * of a document, and each is why the code looks the way it does.
 *
 * 1. The start-bit search does NOT require a zero before the one.
 *
 *    A rule of "a one is only a start bit if a zero came first" looks safe and
 *    is wrong.  It was satisfied on the bit-bang backend by an artifact - that
 *    path samples before raising the clock, so its first sample is the level
 *    left over from our own frame's trailing zero - and on a backend without
 *    the artifact the same rule skipped the real start bit and locked onto the
 *    one two cells later.  An alternating payload survives that almost intact,
 *    which is how it came back as a plausible-looking value with a dead CRC.
 *
 * 2. What that rule was guarding against is caught properly instead.
 *
 *    A floating line reads as all ones and would otherwise be reported as a
 *    confident 0xFFFFFFFF.  So `idle_high` is raised when the payload and CRC
 *    came back all ones, which is evidence about the wire rather than a guess
 *    made before any data arrived.  The CRC catches it too; this names it.
 *
 * 3. The clocks *after* a reply matter, and the right number is not known.
 *
 *    The device stops answering when a reply is followed by more clocks than
 *    it expects - a too-long read window makes the *next* command go quiet,
 *    which presents as an intermittent failure of an unrelated command.  The
 *    bit-bang path gets it right by accident.  An attempt to reproduce that
 *    deliberately on another backend failed for an acknowledge at every
 *    trailing length tried, and was never settled.  So this does not hardcode
 *    a number: `trail_clocks` is an input, swept on hardware the same way
 *    every other unknown in this project was.
 */

`default_nettype none

module dap_frame_rx #(
    parameter integer DIV_WIDTH = 8
) (
    input  wire                  clk,
    input  wire                  rst,

    input  wire [DIV_WIDTH-1:0]  div,
    input  wire                  start,        /* pulse once the frame is out */

    /* How many payload bits to expect.  Zero means a bare acknowledge: the
     * start bit is the whole reply and there is no CRC to check. */
    input  wire [6:0]            reply_bits,
    /* Clocks to hunt for the start bit before giving up.  The DAPISC reset
     * value allows the target 248, and a generous window costs microseconds. */
    input  wire [15:0]           max_wait,
    /* Clocks to issue after the reply, target still driving.  See note 3. */
    input  wire [7:0]            trail_clocks,
    /*
     * Whether a CRC6 follows the payload.  It does for an ordinary reply, and
     * for the *last* parcel of a block read - but not for the parcels before
     * it, which are start bit plus 32 data bits and nothing else.  Reading six
     * phantom CRC bits after one of those would clock into the next parcel and
     * shift the whole rest of the block.
     */
    input  wire                  expect_crc,

    output reg                   busy,
    output reg                   done,

    output reg  [15:0]           wait_cycles,
    output reg                   timed_out,
    output reg                   idle_high,
    output reg                   crc_ok,
    output reg  [62:0]           payload,
    output reg  [5:0]            crc,

    output reg                   dap0,
    input  wire                  dap1_in,
    output reg                   dat_oe        /* low hands the line over */
);
    localparam [2:0] S_IDLE  = 3'd0,
                     S_HUNT  = 3'd1,
                     S_DATA  = 3'd2,
                     S_CRC   = 3'd3,
                     S_TRAIL = 3'd4,
                     S_END   = 3'd5;

    reg [2:0]           state;
    reg [DIV_WIDTH-1:0] tick;
    reg                 phase;      /* 0 = dap0 low, the half we sample in */
    reg [6:0]           index;
    reg [7:0]           trail_left;
    reg                 all_ones;

    reg [6:0]  nbits_r;
    reg        crc_r;
    reg [7:0]  trail_r;
    reg [15:0] max_wait_r;

    reg        sample;
    reg        crc_en;
    reg        crc_rst;
    wire       residue_ok;

    dap_crc6_check u_check (
        .clk        (clk),
        .rst        (crc_rst),
        .en         (crc_en),
        .bit_in     (sample),
        .residue_ok (residue_ok)
    );

    wire tick_done = (tick == div);
    /* The target updates DAP1 after the falling edge, so it is stable through
     * the low phase; sampling at the end of that phase, just before dap0 goes
     * high again, is the widest margin available. */
    wire sample_now = (phase == 1'b0) && tick_done;

    always @(*) begin
        sample = dap1_in;
        crc_en = 1'b0;
        if (sample_now && (state == S_DATA || state == S_CRC)) begin
            crc_en = 1'b1;
        end
    end

    always @(posedge clk) begin
        crc_rst <= 1'b0;
        done    <= 1'b0;

        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            dap0        <= 1'b0;
            dat_oe      <= 1'b1;
            tick        <= {DIV_WIDTH{1'b0}};
            phase       <= 1'b0;
            timed_out   <= 1'b0;
            idle_high   <= 1'b0;
            crc_ok      <= 1'b0;
            wait_cycles <= 16'd0;
        end else if (state == S_IDLE) begin
            dap0 <= 1'b0;
            if (start) begin
                /*
                 * Hand the line over before the first clock.  Order matters on
                 * the board: the buffer is turned round first and the driver
                 * released after, so the two are never fighting - and it costs
                 * nothing, because the timeout counts clocks and none are
                 * issued here.
                 */
                dat_oe      <= 1'b0;
                nbits_r     <= reply_bits;
                crc_r       <= expect_crc;
                trail_r     <= trail_clocks;
                max_wait_r  <= max_wait;
                trail_left  <= trail_clocks;
                crc_rst     <= 1'b1;
                busy        <= 1'b1;
                state       <= S_HUNT;
                index       <= 7'd0;
                tick        <= {DIV_WIDTH{1'b0}};
                phase       <= 1'b0;
                wait_cycles <= 16'd0;
                timed_out   <= 1'b0;
                idle_high   <= 1'b0;
                crc_ok      <= 1'b0;
                all_ones    <= 1'b1;
                payload     <= 63'd0;
                crc         <= 6'd0;
            end
        end else begin
            if (!tick_done) begin
                tick <= tick + 1'b1;
            end else begin
                tick <= {DIV_WIDTH{1'b0}};

                if (phase == 1'b0) begin
                    /* Sample, then raise the clock for the second half. */
                    case (state)
                        S_HUNT: begin
                            if (dap1_in) begin
                                /* The start bit.  Straight to the payload, or
                                 * to the trailing clocks for an acknowledge. */
                                state <= (nbits_r == 7'd0) ? S_TRAIL : S_DATA;
                                index <= 7'd0;
                                if (nbits_r == 7'd0) begin
                                    crc_ok <= 1'b1;   /* nothing to check */
                                end
                            end else if (wait_cycles + 1'b1 >= max_wait_r) begin
                                timed_out <= 1'b1;
                                state     <= S_TRAIL;
                                trail_left <= 8'd0;
                            end else begin
                                wait_cycles <= wait_cycles + 1'b1;
                            end
                        end
                        S_DATA: begin
                            payload[index[5:0]] <= dap1_in;
                            all_ones            <= all_ones & dap1_in;
                            if (index + 1'b1 == nbits_r) begin
                                state <= crc_r ? S_CRC : S_TRAIL;
                                index <= 7'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_CRC: begin
                            crc[index[2:0]] <= dap1_in;
                            all_ones        <= all_ones & dap1_in;
                            if (index == 7'd5) begin
                                state <= S_TRAIL;
                                index <= 7'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_TRAIL: begin
                            if (trail_left == 8'd0) begin
                                state <= S_END;
                            end else begin
                                trail_left <= trail_left - 1'b1;
                            end
                        end
                        default: state <= S_END;
                    endcase

                    if (state != S_END) begin
                        dap0  <= 1'b1;
                        phase <= 1'b1;
                    end
                end else begin
                    dap0  <= 1'b0;
                    phase <= 1'b0;

                    if (state == S_TRAIL && trail_left == 8'd0) begin
                        state <= S_END;
                    end
                end
            end

            if (state == S_END) begin
                /*
                 * The residue is only meaningful once payload and CRC have both
                 * been fed through; a reply that never got that far reports the
                 * reason it stopped instead.
                 */
                /* Nothing to check means nothing to fail: an acknowledge and a
                 * CRC-less block parcel are both good if they arrived at all. */
                crc_ok    <= (nbits_r == 7'd0 || !crc_r) ? ~timed_out
                                                         : (residue_ok & ~timed_out);
                idle_high <= all_ones & (nbits_r != 7'd0);
                dat_oe    <= 1'b1;        /* take the line back */
                dap0      <= 1'b0;
                busy      <= 1'b0;
                done      <= 1'b1;
                state     <= S_IDLE;
            end
        end
    end
endmodule

`default_nettype wire
