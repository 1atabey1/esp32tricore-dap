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
    /*
     * Diagnostic: clock the reply window and keep every bit, instead of
     * hunting for a start bit first.
     *
     * A frame that draws no reply reports one thing - the hunt ran out - and
     * that single fact cannot tell a line the target is holding low from one
     * nothing is driving, nor show a start bit that arrived at the wrong
     * moment.  This is the fabric's version of dap_probe_set_raw_window(),
     * which is what settled the same question on the CPU path.
     */
    input  wire                  no_hunt,
    /*
     * Wide mode: the reply arrives two bits per clock, even bits on DAP1 and
     * odd ones on DAP2, exactly as the transmitter sends them.
     *
     * Latched at start for the same reason the transmitter latches it.
     */
    input  wire                  wide,

    output reg                   busy,
    output reg                   done,

    output reg  [15:0]           wait_cycles,
    output reg                   timed_out,
    output reg                   idle_high,
    output reg                   crc_ok,
    output wire [62:0]           payload,
    output wire [5:0]            crc,

    /*
     * Did DAP2 carry the start bit at the same instant DAP1 did?
     *
     * The start bit is the one bit the device drives on both lines together, so
     * this is the whole alignment check wide mode gets: if the two capture taps
     * are set right, both lines read 1 at that sample and this comes up.  It is
     * reported rather than enforced - a reply that fails it is still delivered,
     * because the point of the bit is to let the host sweep the taps and see
     * which settings work, and a receiver that refused the frame would hide the
     * very evidence the sweep needs.
     */
    output reg                   start_aligned,

    output reg                   dap0,
    input  wire                  dap1_in,
    input  wire                  dap2_in,
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
    /* The last payload index, worked out once at start for the same reason the
     * transmitter does it: `index + 1 == nbits_r` is an add and a compare in
     * the per-bit path, and one subtraction per reply replaces both. */
    reg [6:0]  nbits_last;
    reg        crc_r;
    reg [15:0] wait_limit;
    /*
     * "The hunt has run out", as a flip-flop.
     *
     * Comparing wait_cycles against wait_limit inline is a 16-bit equality in
     * the clock enable of the counter it is comparing - a loop through four
     * LUT levels and a CEN pin, which is what the design's critical path
     * became once the half-period strobe was registered.  Carried in a flop
     * and updated alongside the counter, the enable is one bit again.
     */
    reg        at_limit;
    reg [7:0]  trail_r;
    /* "There are trailing clocks to issue", as one bit.  The state machine
     * asks three times over whether trail_r is zero, and an eight-bit compare
     * in the next-state logic is what put trail_left's clock enable on the
     * critical path. */
    reg        has_trail;
    /* A registered copy of (state == S_HUNT), for the busy counter's enable. */
    reg        hunting;
    reg [15:0] max_wait_r;
    reg        wide_r;
    /*
     * index counts bit positions, not samples, and steps by two in wide mode.
     *
     * The other way round - counting samples and doubling the index to address
     * the payload - puts a mux and a shift in front of a 63-bit write decoder,
     * and that decoder's enables share LUTs with the sample strobe.  It cost
     * about two and a half megahertz, which wide mode would have handed
     * straight back.  As a plain register the position reaches the decoder with
     * nothing in between.
     */
    /* Six CRC bits are six samples narrow and three wide. */
    wire [6:0] crc_last = wide_r ? 7'd2 : 7'd5;

    /*
     * The two lines are captured into separate registers and interleaved on
     * the way out, rather than written into one register at a computed
     * position.
     *
     * Both of the obvious ways cost the design its clock.  Making the write
     * width conditional on wide_r puts the mode flag into all 63 write
     * enables; addressing one register with `payload[2*index +: 2]` makes
     * yosys build the position as a carry chain and compare it against every
     * bit.  Either way the decoder ends up sharing LUTs with the sample
     * strobe, and the design dropped from 48 MHz to 44.
     *
     * Written this way each line has its own fixed single-bit decoder indexed
     * by a counter that still steps by one - exactly the structure that closed
     * at 48 before wide mode existed - and the whole mode dependence becomes a
     * 2:1 mux per bit at the output, which is a clock away from anything and
     * drives nothing but the top level's reply latch.  The interleave itself
     * is wiring: DAP1 supplies the even bits and DAP2 the odd ones.
     */
    reg [62:0] pay_even;
    reg [31:0] pay_odd;
    reg [5:0]  crc_even;    /* six samples narrow, three wide */
    reg [2:0]  crc_odd;

    genvar g;
    generate
        for (g = 0; g < 32; g = g + 1) begin : interleave
            assign payload[2*g]     = wide_r ? pay_even[g] : pay_even[2*g];
            if (2*g + 1 < 63)
                assign payload[2*g+1] = wide_r ? pay_odd[g] : pay_even[2*g+1];
        end
        for (g = 0; g < 3; g = g + 1) begin : crc_interleave
            assign crc[2*g]   = wide_r ? crc_even[g] : crc_even[2*g];
            assign crc[2*g+1] = wide_r ? crc_odd[g]  : crc_even[2*g+1];
        end
    endgenerate

    reg        sample;
    reg        sample2;
    reg        crc_en;
    reg        crc_rst;
    wire       residue_ok;

    dap_crc6_check u_check (
        .clk        (clk),
        .rst        (crc_rst),
        .en         (crc_en),
        .bit_in     (sample), .bit_in2 (sample2), .wide (wide_r),
        .residue_ok (residue_ok)
    );

    /*
     * The half-period strobe is a flip-flop, not a comparison.
     *
     * As a wire it was `tick == div`, an 8-bit compare feeding the clock
     * enable of the payload, CRC and all_ones registers - and place and route
     * answered that by promoting the enable onto a global buffer, which put a
     * LUT chain plus a global net in series and held the whole design to
     * 31 MHz.  Computed one cycle early into a register, the enable is a flop
     * output and the compare moves off the critical path into that flop's own
     * D input, where it has a full cycle to settle.
     */
    reg tick_done;
    /*
     * Sampled at the END of the high phase - see the long note in the state
     * machine below.  DAP1 arrives two clocks late through the synchroniser,
     * so reading it at the end of the low phase samples the pad ever closer to
     * the falling edge as the divider shrinks; reading it here puts those two
     * clocks on the useful side and removes the divider floor entirely.
     */
    /*
     * Which edge to sample on depends on how much of a bit period there is.
     *
     * DAP1 arrives two clocks late through the synchroniser, so a sample taken
     * at fabric time T reports the pad at T-2.  Taking it at the end of the
     * high phase puts that instant two clocks *into* the bit, which is right
     * for any half period of two clocks or more.
     *
     * At a half period of one clock the whole bit is two clocks, so there is
     * no instant inside it that is two clocks old - the best available is the
     * middle of the *previous* bit, which is the end of the low phase.  The
     * reply then arrives one bit period later than it was sent, and that costs
     * nothing: the start-bit hunt is delayed by exactly the same amount, so
     * the payload still lines up behind it and the receiver simply issues one
     * more clock at the end.  This is the "sample on the opposite edge" the
     * optimisation guide recommends, arrived at from the synchroniser rather
     * than from the round trip.
     */
    reg  sample_phase;
    wire sample_now = (phase == sample_phase) && tick_done;

    reg [6:0] index_next;

    always @(*) begin
        case (state)
            S_DATA:  index_next = (index == nbits_last) ? 7'd0 : index + 7'd1;
            S_CRC:   index_next = (index == crc_last)   ? 7'd0 : index + 7'd1;
            default: index_next = 7'd0;
        endcase
    end

    always @(*) begin
        sample  = dap1_in;
        sample2 = dap2_in;
        crc_en = 1'b0;
        if (sample_now && (state == S_DATA || state == S_CRC)) begin
            crc_en = 1'b1;
        end
    end

    always @(posedge clk) begin
        crc_rst <= 1'b0;
        done    <= 1'b0;
        hunting <= (state == S_HUNT);

        /*
         * Only what has to be right before the first frame.
         *
         * timed_out, idle_high, crc_ok and wait_cycles are all loaded again
         * when a reply starts and are meaningless until one has finished, so
         * resetting them buys nothing and costs fanout - and this reset net
         * reaches nearly every flip-flop in the design, which made it most of
         * the routing on the critical path.  Shortening the net by resetting
         * fewer registers is the fix that worked; a global buffer for it was
         * tried and was worse.
         */
        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            dap0        <= 1'b0;
            dat_oe      <= 1'b1;
            tick        <= {DIV_WIDTH{1'b0}};
            tick_done   <= (div == {DIV_WIDTH{1'b0}});
            phase       <= 1'b0;
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
                /* Samples, not bits: a pair per sample in wide mode, so an
                 * odd payload and the even one above it take the same count -
                 * the extra slot is the pad. */
                nbits_last  <= wide ? ((reply_bits - 1'b1) >> 1)
                                    :  (reply_bits - 1'b1);
                wide_r      <= wide;
                /* No CRC in raw mode: those six clocks are window too. */
                crc_r       <= expect_crc & ~no_hunt;
                trail_r     <= trail_clocks;
                /* At a half period of one clock there is no instant inside
                 * the bit that is two clocks old; the low phase's end is the
                 * middle of the previous one.  See the note above. */
                sample_phase <= (div == {DIV_WIDTH{1'b0}}) ? 1'b0 : 1'b1;
                has_trail   <= (trail_clocks != 8'd0);
                max_wait_r  <= max_wait;
                /* One subtract here, out of the hot path, instead of an add
                 * and a compare on every clocked bit. */
                wait_limit  <= (max_wait == 16'd0) ? 16'd0 : max_wait - 1'b1;
                /* wait_cycles starts at zero, so the limit is already reached
                 * when max_wait allows one clock or none. */
                at_limit    <= (max_wait <= 16'd1);
                trail_left  <= trail_clocks;
                crc_rst     <= 1'b1;
                busy        <= 1'b1;
                /* Straight into the payload: every clock of the window is a
                 * bit worth keeping when the question is what is on the wire
                 * rather than what the reply says. */
                state       <= no_hunt ? S_DATA : S_HUNT;
                index       <= 7'd0;
                tick        <= {DIV_WIDTH{1'b0}};
                tick_done   <= (div == {DIV_WIDTH{1'b0}});
                phase       <= 1'b0;
                wait_cycles <= 16'd0;
                timed_out   <= 1'b0;
                idle_high   <= 1'b0;
                crc_ok      <= 1'b0;
                all_ones    <= 1'b1;
                pay_even    <= 63'd0;
                pay_odd     <= 32'd0;
                crc_even    <= 6'd0;
                crc_odd     <= 3'd0;
                start_aligned <= 1'b0;
            end
        end else begin
            if (!tick_done) begin
                tick      <= tick + 1'b1;
                tick_done <= (tick + 1'b1 == div);
            end else begin
                tick      <= {DIV_WIDTH{1'b0}};
                tick_done <= (div == {DIV_WIDTH{1'b0}});

                /* The clock, generated whichever edge the bit is taken on. */
                if (phase == 1'b0) begin
                    if (state != S_END) begin
                        dap0  <= 1'b1;
                        phase <= 1'b1;
                    end
                end else begin
                    dap0  <= 1'b0;
                    phase <= 1'b0;
                end

                /* The bit, on whichever edge sample_phase names. */
                if (phase == sample_phase) begin
                    case (state)
                        S_HUNT: begin
                            if (dap1_in) begin
                                /* The start bit.  Straight to the payload, or
                                 * to the trailing clocks for an acknowledge. */
                                state <= (nbits_r == 7'd0)
                                         ? (has_trail ? S_TRAIL : S_END)
                                         : S_DATA;
                                /* The one instant both lines carry the same
                                 * bit, so the one place the taps can be
                                 * checked against each other. */
                                start_aligned <= dap2_in;
                                if (nbits_r == 7'd0) begin
                                    crc_ok <= 1'b1;   /* nothing to check */
                                end
                            /*
                             * Equality against a limit worked out once at
                             * start, not "counter + 1 >= limit".  The latter is
                             * a 16-bit add feeding a 16-bit compare in one
                             * cycle, and it was the critical path that held the
                             * whole design to 35 MHz.
                             */
                            end else if (at_limit) begin
                                timed_out  <= 1'b1;
                                state      <= S_END;
                                trail_left <= 8'd0;
                            end
                            /* The counter itself is updated outside this case
                             * - see the note below. */
                        end
                        S_DATA: begin
                            /*
                             * Two bits every sample, in both modes.
                             *
                             * Narrow mode writes DAP2's bit one position ahead
                             * and then overwrites it on the next sample, so the
                             * only one that survives is at position nbits -
                             * outside the payload the caller reads.  Making the
                             * width conditional instead puts wide_r into all 63
                             * write enables, which is what the mode flag must
                             * stay out of.
                             *
                             * An odd wide payload picks up one pad bit above
                             * it, in the same slot the transmitter sends a zero
                             * into and the caller likewise does not read.
                             */
                            pay_even[index[5:0]] <= dap1_in;
                            pay_odd[index[4:0]]  <= dap2_in;
                            all_ones <= all_ones & dap1_in & (dap2_in | ~wide_r);
                            if (index == nbits_last) begin
                                state <= crc_r ? S_CRC
                                       : (has_trail ? S_TRAIL : S_END);
                            end
                        end
                        S_CRC: begin
                            crc_even[index[2:0]] <= dap1_in;
                            crc_odd[index[1:0]]  <= dap2_in;
                            all_ones <= all_ones & dap1_in & (dap2_in | ~wide_r);
                            if (index == crc_last) begin
                                state <= has_trail ? S_TRAIL : S_END;
                            end
                        end
                        /*
                         * Exactly `trail` clocks, no more.  Entering this
                         * state at all costs one, so a zero count skips it
                         * rather than passing through - which the old form did
                         * not, and it is the difference between issuing the
                         * number of trailing clocks the device was asked for
                         * and one more than that.
                         */
                        S_TRAIL: begin
                            if (trail_left == 8'd1) begin
                                state <= S_END;
                            end else begin
                                trail_left <= trail_left - 1'b1;
                            end
                        end
                        default: state <= S_END;
                    endcase

                    /*
                     * The busy-cycle counter, gated on a registered copy of
                     * "we are hunting" rather than on the state itself.
                     *
                     * Inside the case above, its clock enable carries the
                     * three-bit state decode on top of the two conditions it
                     * actually needs, and that decode is what the critical
                     * path kept coming back to.  The copy is one cycle stale,
                     * which cannot matter here: the state only ever changes at
                     * one of these sample instants, and the next one is at
                     * least four clocks away even at the lowest divider.
                     */
                    /*
                     * The bit index, updated on every sample with nothing but
                     * the half-period strobe in its enable.
                     *
                     * Which field is being counted, and whether it has run
                     * out, decides the *value* - and in the data path that has
                     * a whole clock to settle.  Inside the case it decided the
                     * clock enable instead, and that state decode was the
                     * critical path.
                     */
                    index <= index_next;

                    if (hunting && !dap1_in && !at_limit) begin
                        wait_cycles <= wait_cycles + 1'b1;
                        at_limit    <= (wait_cycles + 1'b1 == wait_limit);
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
                /*
                 * Only a window that was actually sampled can be all ones.  A
                 * hunt that ran out never reaches S_DATA, so all_ones still
                 * holds its initial 1 and idle_high came up on every timeout -
                 * naming the wire as undriven when the truth was that it never
                 * went high at all, which is the opposite fault.
                 */
                idle_high <= all_ones & (nbits_r != 7'd0) & ~timed_out;
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
