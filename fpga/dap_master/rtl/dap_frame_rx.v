/*
 * Clock in one DAP reply: busy stuffing (target holds DAP1 low), start bit,
 * payload LSB first, CRC6.  A bare acknowledge is the start bit alone.
 *
 * The start-bit search takes the first one; it does not require a zero first.
 * An all-ones window raises idle_high (floating line).  Clocks after the reply
 * matter to the device, so their count is the trail_clocks input.
 */

`default_nettype none

module dap_frame_rx #(
    parameter integer DIV_WIDTH = 8
) (
    input  wire                  clk,
    input  wire                  rst,

    input  wire [DIV_WIDTH-1:0]  div,
    input  wire                  start,        /* pulse once the frame is out */

    /* Payload bits to expect; zero means a bare acknowledge, no CRC. */
    input  wire [6:0]            reply_bits,
    /* Clocks to hunt for the start bit before giving up. */
    input  wire [15:0]           max_wait,
    /* Clocks to issue after the reply, target still driving. */
    input  wire [7:0]            trail_clocks,
    /* A CRC6 follows the payload.  Not for block-read parcels before the last,
     * which are start bit plus 32 data bits only. */
    input  wire                  expect_crc,
    /* Diagnostic: clock the reply window and keep every bit, no start-bit hunt. */
    input  wire                  no_hunt,
    /* Wide mode: two bits per clock, even on DAP1 and odd on DAP2.  Latched at
     * start. */
    input  wire                  wide,
    /* One sample every fabric clock, `div` ignored; dap0 is then "a clock
     * this cycle", as in dap_frame_tx.  Latched at start. */
    input  wire                  fast,
    /*
     * Fast mode: a DAP0 clock's bit reaches dap1_in lag+1 cycles after it.
     * Only those cycles are samples, so the line held between clocks (the
     * next bit, already presented) is never read twice; and the clocks still
     * in flight when the start bit is seen are counted against the reply,
     * so the target gets exactly its clocks.
     */
    input  wire [2:0]            lag,

    /*
     * Block read, streamed: parcels counts this one and those after it.
     * While another follows and `room` says the FIFO takes two more words,
     * a parcel does not end the reply: DAP0 keeps running, its clocks
     * becoming the next parcel's busy bits, and the hunt starts again at the
     * next sample.  pdone then marks the payload complete for this cycle
     * only.  A parcel seen without room ends the reply as before (done), and
     * the sequencer restarts the receiver once the FIFO has drained.
     */
    input  wire                  stream,
    input  wire [8:0]            parcels,
    input  wire                  room,
    output reg                   pdone,

    output reg                   busy,
    output reg                   done,

    output reg  [15:0]           wait_cycles,
    output reg                   timed_out,
    output reg                   idle_high,
    output reg                   crc_ok,
    output wire [62:0]           payload,
    output wire [5:0]            crc,

    /* DAP2 also read 1 at the start bit, the one bit driven on both lines.
     * Reported, not enforced, so the host can sweep the capture taps. */
    output reg                   start_aligned,

    /* To the pad, which registers it on the way out (a clock later). */
    output reg                   dap0,
    input  wire                  dap1_in,
    /* The same sample, a separate register, for the decisions (timing). */
    input  wire                  dap1_ctl,
    input  wire                  dap2_in,
    output reg                   dat_oe        /* low hands the line over */
);
    localparam [2:0] S_IDLE  = 3'd0,
                     S_HUNT  = 3'd1,
                     S_DATA  = 3'd2,
                     S_CRC   = 3'd3,
                     S_TRAIL = 3'd4,
                     S_END   = 3'd5;

    /* One-hot: the state tests sit on the sample enables (timing). */
    (* fsm_encoding = "one-hot" *)
    reg [2:0]           state;
    /* Half-period counter, counting div down to one: tick_done then sits
     * behind a constant compare, not an adder and a compare with div. */
    reg [DIV_WIDTH-1:0] tick;
    localparam [DIV_WIDTH-1:0] TICK_ONE = 1;
    reg                 phase;      /* 0 = dap0 low, the half we sample in */
    reg [7:0]           trail_left;
    reg                 all_ones;

    reg [6:0]  nbits_r;
    /* Last payload sample, computed once at start (timing). */
    reg [6:0]  nbits_last;
    reg        crc_r;
    /* Busy samples still allowed, counted down beside wait_cycles so the
     * limit is a constant compare (timing). */
    reg [15:0] wait_left;
    /* Hunt limit reached, registered to keep the counter's enable one bit. */
    reg        at_limit;
    reg [7:0]  trail_r;
    /* trail_r != 0, registered (timing). */
    reg        has_trail;
    /* A registered copy of (state == S_HUNT), for the busy counter's enable. */
    reg        hunting;
    /*
     * What start loads from max_wait, computed ahead (timing): MAXWAIT is
     * written in an SPI transfer before the one that starts the frame.
     */
    reg [15:0] wait_left0 = 16'd0;
    reg        wait_none  = 1'b1;
    always @(posedge clk) begin
        wait_left0 <= (max_wait == 16'd0) ? 16'd0 : max_wait - 1'b1;
        wait_none  <= (max_wait <= 16'd1);
    end
    reg        wide_r;
    reg        fast_r;
    /* A half period of one clock; always so in fast mode. */
    wire       div_zero = fast_r | (div == {DIV_WIDTH{1'b0}});
    /* Fields count sample positions, two bits a step in wide mode: six CRC
     * bits are six samples narrow and three wide. */
    wire [6:0] crc_last = wide_r ? 7'd2 : 7'd5;

    /*
     * Each line is captured into its own register and interleaved at the
     * output (DAP1 even bits, DAP2 odd), keeping the mode flag out of the
     * write decoders (timing).
     */
    reg [62:0] pay_even;
    reg [31:0] pay_odd;
    reg [5:0]  crc_even;    /* six samples narrow, three wide */
    reg [2:0]  crc_odd;

    /*
     * Capture is a shift register per field with a fixed entry point: a
     * sample enters at the field's last position (the one-hot *_at mask)
     * and everything shifts down a place, so after the field's last sample
     * its first sits at bit 0 and the bits above stay clear.  Every bit of
     * a field shares one enable and one clear.  (Writing bit k by an index
     * gave each flop its own enable; an iCE40 tile's eight flops share one,
     * so those flops were scattered a tile apart and routing dominated.)
     */
    reg [62:0] pay_at = 63'd0;
    reg [5:0]  crc_at = 6'd0;
    integer    k;

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

    /* Half-period strobe, registered one cycle early (timing). */
    reg tick_done;
    /*
     * Sample edge.  DAP1 arrives two clocks late through the synchroniser, so
     * sample at the end of the high phase.  At a half period of one clock,
     * sample at the end of the low phase instead: the reply then lands one bit
     * later, and the hunt absorbs it.  Fast mode samples every clock.
     */
    reg  sample_phase;
    /*
     * Fast mode's sample strobe.  clk_hist: the clocks of the last cycles,
     * newest in bit 0.  `leaving` is the clock that lands next cycle: phase,
     * otherwise unused in fast mode, is loaded with it (inverted, against a
     * sample_phase of 0), so phase == sample_phase marks a cycle whose
     * dap1_in carries the bit of the clock lag+1 cycles back, and the strobe
     * stays the plain one on the payload write enables (timing).
     */
    reg  [7:0] clk_hist;
    /* lag is at least one (dap_top adds the pads' clock) and changes only
     * between replies, so its tap index is registered (timing). */
    reg  [2:0] lag_tap = 3'd0;
    always @(posedge clk) lag_tap <= lag - 3'd1;
    wire       leaving    = clk_hist[lag_tap];
    wire sample_at  = (phase == sample_phase);
    wire sample_now = sample_at && tick_done;
    /*
     * The strobe the bit is actually taken on, a register in both modes: it
     * gates every payload write enable.  The pads register DAP0 and DAP1 on
     * their way out, a clock after this engine's dap0, so normal mode takes
     * the bit a clock after its edge (sample_now, a clock late): the line
     * is then sampled where it was, relative to the pins.  Samples are at
     * least two clocks apart, so the state it updates is still seen by the
     * next phase decision.  Fast mode takes it where a clock lands, from
     * `leaving` a clock early (phase is then ~leaving); its LAG includes
     * the pads' clock.
     */
    reg  sample_take;

    /*
     * at_last: this sample is the current field's last, registered so no
     * field-end compare sits on the sample paths (timing).  left counts
     * down the field's samples after this one, so at_last is a compare
     * against a constant.  nb_zero: a one-sample payload, for a payload
     * that starts already at its end.
     */
    reg       at_last;
    reg       nb_zero;
    reg [6:0] left;
    wire [6:0] nbits_last_in = wide ? ((reply_bits - 1'b1) >> 1) : (reply_bits - 1'b1);

    /* The entry points, fixed for a reply and decoded off its registers;
     * the first sample is at least two clocks after start. */
    always @(posedge clk) begin
        for (k = 0; k < 63; k = k + 1) pay_at[k] <= (nbits_last == k);
        for (k = 0; k < 6; k = k + 1)  crc_at[k] <= (crc_last == k);
    end

    (* keep *) wire take_data = sample_take && state == S_DATA;
    (* keep *) wire take_crc  = sample_take && state == S_CRC;
    wire            cap_clear = state == S_IDLE && start;

    always @(posedge clk) begin
        if (cap_clear) begin
            pay_even <= 63'd0;
            pay_odd  <= 32'd0;
            crc_even <= 6'd0;
            crc_odd  <= 3'd0;
        end else begin
            /* Two bits every sample, in both modes.  Narrow mode's DAP2
             * bits land in pay_odd, unread. */
            if (take_data) begin
                for (k = 0; k < 63; k = k + 1)
                    pay_even[k] <= pay_at[k] ? dap1_in
                                 : (k < 62) ? pay_even[k + 1] : 1'b0;
                for (k = 0; k < 32; k = k + 1)
                    pay_odd[k]  <= pay_at[k] ? dap2_in
                                 : (k < 31) ? pay_odd[k + 1] : 1'b0;
            end
            if (take_crc) begin
                for (k = 0; k < 6; k = k + 1)
                    crc_even[k] <= crc_at[k] ? dap1_in
                                 : (k < 5) ? crc_even[k + 1] : 1'b0;
                for (k = 0; k < 3; k = k + 1)
                    crc_odd[k]  <= crc_at[k] ? dap2_in
                                 : (k < 2) ? crc_odd[k + 1] : 1'b0;
            end
        end
    end

    always @(*) begin
        sample  = dap1_in;
        sample2 = dap2_in;
        crc_en = 1'b0;
        if (sample_take && (state == S_DATA || state == S_CRC)) begin
            crc_en = 1'b1;
        end
    end

    /*
     * Fast mode's clock budget.  after_r: clocks after the start bit's, for
     * payload, CRC and trailing clocks, registered (the first sample comes
     * at least two cycles after start).  The clocks run unbroken from the
     * first until the budget stops them, so when the start bit is seen the
     * lag+1 clocks in flight (this cycle's included) are already part of
     * them; the rest, pulse_init, are counted down in pulses_left.
     * Assumes lag >= 1, which the pad and tap registers always add.
     */
    reg  [8:0] after_r;
    reg  [8:0] pulse_init;
    reg  [8:0] pulses_left;
    reg        counting;
    /*
     * The budget's decisions, registered (timing): pi_go is pulse_init > 1,
     * or a streamed parcel (no budget); go_r is whether to clock next cycle.
     * Everything up to pulse_init is fixed for the whole reply, so it is
     * pipelined, one operation a stage: after_r, then the two differences,
     * then pulse_init and pi_go.  The
     * first sample is lag + 1 clocks after the first clock, and lag counts
     * the pads' clock, so they are ready in time.
     */
    reg        pi_go, go_r;
    wire [8:0] after_all = ((nbits_r == 7'd0) ? 9'd0 : {2'd0, nbits_last} + 9'd1)
                         + (crc_r ? {2'd0, crc_last} + 9'd1 : 9'd0)
                         + (has_trail ? {1'b0, trail_r} : 9'd0);
    /* lag only changes between replies; its sums are registered. */
    reg  [8:0] in_flight  = 9'd1;
    reg  [8:0] in_flight1 = 9'd2;
    /* after_r - in_flight and after_r - (in_flight + 1), sign in bit 9 */
    reg  [9:0] left0, left1;
    wire       start_seen = sample_take && (state == S_HUNT) && dap1_ctl;

    /*
     * Streaming.  cnt_r: parcels from the current one on.  The decision at
     * a start bit reads only cont_ok, registered two stages back from
     * cnt_r and room (timing: start_seen is on the critical path); the
     * bookkeeping follows a clock later (ss_cont), at least a payload
     * before anything reads it again.  cont_r: the current parcel streams
     * into the next.
     */
    reg        stream_r;
    reg  [8:0] cnt_r;
    reg        more_r, last_nx, cont_ok, cont_r, ss_cont;
    /* What the start bit loads into the budget, cont_ok already folded in,
     * so that mux keeps the inputs it had without streaming (timing). */
    reg        nc_r;
    always @(posedge clk) begin
        more_r  <= stream_r && (cnt_r > 9'd1);
        last_nx <= (cnt_r == 9'd2);
        cont_ok <= more_r && room;
        nc_r    <= !(more_r && room);
        ss_cont <= start_seen && cont_ok;
    end

    /*
     * DAP0's next value.  Normal mode: high in the second half of the
     * period.  Fast mode: high for every cycle that takes a sample.
     */
    reg dap0_d;
    always @(*) begin
        if (rst || state == S_END)
            dap0_d = 1'b0;
        else if (state == S_IDLE)
            dap0_d = start & fast;
        else if (!tick_done)
            dap0_d = dap0;
        else if (!fast_r)
            dap0_d = ~phase;
        /* Fast mode, from a register (timing): the clock after the start
         * bit's sample always goes out, the budget's from the one after.
         * The budget ends every reply that finds its start bit; S_END stops
         * the rest (a timeout, raw mode) a clock later. */
        else
            dap0_d = go_r;
    end


    always @(posedge clk) begin
        crc_rst <= 1'b0;
        done    <= 1'b0;
        pdone   <= 1'b0;
        hunting <= (state == S_HUNT);
        dap0    <= dap0_d;
        sample_take <= !rst && state != S_IDLE && (fast_r ? leaving : sample_now);

        in_flight  <= {6'd0, lag} + 9'd1;
        in_flight1 <= {6'd0, lag} + 9'd2;
        after_r    <= after_all;
        left0      <= {1'b0, after_r} - {1'b0, in_flight};
        left1      <= {1'b0, after_r} - {1'b0, in_flight1};
        pulse_init <= left0[9] ? 9'd0 : left0[8:0];
        pi_go      <= (!left1[9] && (left1 != 10'd0)) || (more_r && room);
        /* A new reply starts with nothing in flight. */
        clk_hist <= (state == S_IDLE) ? 8'd0 : {clk_hist[6:0], dap0};
        if (state == S_IDLE) begin
            counting    <= 1'b0;
            pulses_left <= 9'd0;
            go_r        <= 1'b1;
        end else if (fast_r && start_seen) begin
            /* A streamed parcel has no budget: the clocks just go on.
             * cont_ok enters as data, not into the enable (timing). */
            counting    <= nc_r;
            pulses_left <= pulse_init;
            go_r        <= pi_go;
        end else begin
            /*
             * Counted down without an enable (timing): once go_r has
             * dropped it stays down, so the count may run past zero and
             * wrap, unread.  A clock now leaves pulses_left - 1 for after
             * this one.
             */
            pulses_left <= pulses_left - 1'b1;
            if (counting) begin
                go_r <= go_r && (pulses_left > 9'd2);
            end
        end

        /* Reset only what must be right before the first frame; the rest is
         * loaded at start (keeps reset fanout down). */
        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            fast_r      <= 1'b0;
            dat_oe      <= 1'b1;
            tick        <= div;
            tick_done   <= (div == {DIV_WIDTH{1'b0}});
            phase       <= 1'b0;
        end else if (state == S_IDLE) begin
            /*
             * The reply's parameters, taken every idle clock rather than on
             * start: the inputs are set on start's own edge, nothing reads
             * these while idle, and start stays out of their enables (timing).
             */
            nbits_r     <= reply_bits;
            /* Samples, not bits: wide mode takes a pair per sample; an odd
             * payload gets one pad bit. */
            nbits_last  <= nbits_last_in;
            /* One sample: nbits_last_in == 0, without its subtract
             * (timing).  Raw mode starts in the payload. */
            nb_zero     <= (reply_bits == 7'd1) || (wide && reply_bits == 7'd2);
            at_last     <= (reply_bits == 7'd1) || (wide && reply_bits == 7'd2);
            wide_r      <= wide;
            fast_r      <= fast;
            /* No CRC in raw mode: those six clocks are window too. */
            crc_r       <= expect_crc & ~no_hunt;
            trail_r     <= trail_clocks;
            /* Half period of one clock: sample at the low phase's end.
             * Fast mode: wherever a clock lands (phase is ~leaving). */
            sample_phase <= (fast || div == {DIV_WIDTH{1'b0}}) ? 1'b0 : 1'b1;
            has_trail   <= (trail_clocks != 8'd0);
            wait_left   <= wait_left0;
            /* wait_cycles starts at zero, so the limit is already reached
             * when max_wait allows one clock or none. */
            at_limit    <= wait_none;
            trail_left  <= trail_clocks;
            left        <= nbits_last_in;
            tick        <= div;
            tick_done   <= fast | (div == {DIV_WIDTH{1'b0}});
            phase       <= fast;          /* fast: nothing has landed */
            all_ones    <= 1'b1;
            stream_r    <= stream & ~no_hunt;
            cnt_r       <= parcels;
            cont_r      <= 1'b0;
            /* What the last reply reported stays until the next starts. */
            if (start) begin
                /* Hand the line over before the first clock. */
                dat_oe      <= 1'b0;
                crc_rst     <= 1'b1;
                busy        <= 1'b1;
                /* Raw mode goes straight into the payload. */
                state       <= no_hunt ? S_DATA : S_HUNT;
                wait_cycles <= 16'd0;
                timed_out   <= 1'b0;
                idle_high   <= 1'b0;
                crc_ok      <= 1'b0;
                start_aligned <= 1'b0;
            end
        end else begin
            /*
             * A parcel that streams on: the next one is counted, and whether
             * it is the block's last (CRC, trailing clocks, a budget) is set
             * now, well before its start bit, so the budget's pipeline above
             * has settled by then.
             */
            if (ss_cont) begin
                cont_r <= 1'b1;
                cnt_r  <= cnt_r - 1'b1;
                crc_r  <= last_nx;
            end else if (pdone) begin
                cont_r <= 1'b0;
            end

            if (!tick_done) begin
                tick      <= tick - 1'b1;
                tick_done <= (tick == TICK_ONE);
            end else begin
                tick      <= div;
                tick_done <= div_zero;

                /* The clock (dap0_d), generated whichever edge the bit is
                 * taken on.  Fast mode: the landing strobe instead. */
                if (fast_r) begin
                    phase <= ~leaving;
                end else if (phase == 1'b0) begin
                    if (state != S_END) begin
                        phase <= 1'b1;
                    end
                end else begin
                    phase <= 1'b0;
                end
            end

            /* The bit, on whichever edge sample_phase names (sample_take). */
            if (sample_take) begin
                case (state)
                    S_HUNT: begin
                        if (dap1_ctl) begin
                            /* The start bit.  Straight to the payload, or
                             * to the trailing clocks for an acknowledge. */
                            state <= (nbits_r == 7'd0)
                                     ? (has_trail ? S_TRAIL : S_END)
                                     : S_DATA;
                            start_aligned <= dap2_in;
                            if (nbits_r == 7'd0) begin
                                crc_ok <= 1'b1;   /* nothing to check */
                            end
                        end else if (at_limit) begin
                            timed_out  <= 1'b1;
                            state      <= S_END;
                        end
                        /* wait_cycles is updated after this case. */
                    end
                    S_DATA: begin
                        /* The bits themselves: take_data, above. */
                        all_ones <= all_ones & dap1_in & (dap2_in | ~wide_r);
                        if (at_last && cont_r) begin
                            /* Streamed: hunt for the next parcel's start
                             * bit from the very next sample.  Its capture
                             * needs no clear: a full field shifts every old
                             * bit out. */
                            state       <= S_HUNT;
                            pdone       <= 1'b1;
                            crc_rst     <= 1'b1;
                        end else if (at_last) begin
                            state <= crc_r ? S_CRC
                                   : (has_trail ? S_TRAIL : S_END);
                        end
                    end
                    S_CRC: begin
                        /* The bits themselves: take_crc, above. */
                        all_ones <= all_ones & dap1_in & (dap2_in | ~wide_r);
                        if (at_last) begin
                            state <= has_trail ? S_TRAIL : S_END;
                        end
                    end
                    /* Exactly trail_clocks clocks; a zero count skips
                     * this state. */
                    S_TRAIL: begin
                        if (trail_left == 8'd1) begin
                            state <= S_END;
                        end else begin
                            trail_left <= trail_left - 1'b1;
                        end
                    end
                    default: state <= S_END;
                endcase

                /* Updated outside the case to keep the state decode out
                 * of their enables (timing).  hunting is one cycle stale,
                 * which is harmless: state only changes at a sample. */
                case (state)
                    S_DATA: begin
                        /* After the payload's last: the CRC's first. */
                        left    <= at_last ? crc_last : left - 7'd1;
                        at_last <= !at_last && (left == 7'd1);
                    end
                    S_CRC: begin
                        left    <= left - 7'd1;
                        at_last <= !at_last && (left == 7'd1);
                    end
                    default: begin                /* the payload's first */
                        left    <= nbits_last;
                        at_last <= nb_zero;
                    end
                endcase

                /* Fast mode samples back to back, so the stale flag
                 * can count the first payload sample too: WAIT may read
                 * one high there. */
                if (hunting && !dap1_ctl && !at_limit) begin
                    wait_cycles <= wait_cycles + 1'b1;
                    wait_left   <= wait_left - 1'b1;
                    at_limit    <= (wait_left == 16'd1);
                end
            end

            /*
             * A streamed parcel's hunt gets its own timeout, as the device's
             * does.  Re-armed a clock after its start bit, during the
             * payload, where nothing counts (off the sample path: timing).
             * WAIT then reports the last parcel's busy clocks.
             */
            if (ss_cont) begin
                wait_cycles <= 16'd0;
                wait_left   <= wait_left0;
                at_limit    <= wait_none;
            end

            if (state == S_END) begin
                /* An acknowledge or a CRC-less parcel is good if it arrived. */
                crc_ok    <= (nbits_r == 7'd0 || !crc_r) ? ~timed_out
                                                         : (residue_ok & ~timed_out);
                /* Only a sampled window can be all ones; not on a timeout. */
                idle_high <= all_ones & (nbits_r != 7'd0) & ~timed_out;
                dat_oe    <= 1'b1;        /* take the line back */
                busy      <= 1'b0;
                done      <= 1'b1;
                state     <= S_IDLE;
            end
        end
    end
endmodule

`default_nettype wire
