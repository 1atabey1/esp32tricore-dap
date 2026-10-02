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

    output reg                   dap0,
    /* What dap0 holds next cycle, and the same for normal mode only (a few
     * registers deep), for the pad's two output registers. */
    output reg                   dap0_d,
    output reg                   dap0_next,
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
    /* Last payload index, computed once at start (timing). */
    reg [6:0]  nbits_last;
    reg        crc_r;
    reg [15:0] wait_limit;
    /* Hunt limit reached, registered to keep the counter's enable one bit. */
    reg        at_limit;
    reg [7:0]  trail_r;
    /* trail_r != 0, registered (timing). */
    reg        has_trail;
    /* A registered copy of (state == S_HUNT), for the busy counter's enable. */
    reg        hunting;
    reg [15:0] max_wait_r;
    reg        wide_r;
    reg        fast_r;
    /* A half period of one clock; always so in fast mode. */
    wire       div_zero = fast_r | (div == {DIV_WIDTH{1'b0}});
    /* index counts sample positions, so it covers two bits per step in wide mode. */
    /* Six CRC bits are six samples narrow and three wide. */
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
    wire       leaving    = (lag == 3'd0) ? dap0 : clk_hist[lag - 3'd1];
    wire sample_at  = (phase == sample_phase);
    wire sample_now = sample_at && tick_done;

    reg [6:0] index_next;

    /*
     * at_last: index is the current field's last sample, registered so no
     * field-end compare sits on the sample paths (timing).  nb_zero: a
     * one-sample payload, for a payload that starts already at its end.
     */
    reg       at_last;
    reg       nb_zero;
    wire [6:0] nbits_last_in = wide ? ((reply_bits - 1'b1) >> 1) : (reply_bits - 1'b1);

    always @(*) begin
        case (state)
            S_DATA,
            S_CRC:   index_next = at_last ? 7'd0 : index + 7'd1;
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
     * The budget's decisions, registered (timing): pi_gt1 is pulse_init > 1;
     * go_r is whether to clock next cycle.
     */
    reg        pi_gt1, go_r;
    wire [8:0] after_all = ((nbits_r == 7'd0) ? 9'd0 : {2'd0, nbits_last} + 9'd1)
                         + (crc_r ? {2'd0, crc_last} + 9'd1 : 9'd0)
                         + (has_trail ? {1'b0, trail_r} : 9'd0);
    wire [8:0] in_flight  = {6'd0, lag} + 9'd1;
    wire       start_seen = sample_now && (state == S_HUNT) && dap1_in;

    /*
     * DAP0's next value, computed here so the pad can register it.  Normal
     * mode: high in the second half of the period.  Fast mode: high for every
     * cycle that takes a sample.
     */
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

    /* The same for normal mode only, for the pad's first-half register (fast
     * mode holds that half low): a few registers deep (timing). */
    always @(*) begin
        if (rst || state == S_IDLE || state == S_END)
            dap0_next = 1'b0;
        else if (!tick_done)
            dap0_next = dap0;
        else
            dap0_next = ~phase;
    end

    always @(posedge clk) begin
        crc_rst <= 1'b0;
        done    <= 1'b0;
        hunting <= (state == S_HUNT);
        dap0    <= dap0_d;

        after_r    <= after_all;
        pulse_init <= (after_r > in_flight) ? after_r - in_flight : 9'd0;
        pi_gt1     <= (after_r > in_flight + 9'd1);
        /* A new reply starts with nothing in flight. */
        clk_hist <= (state == S_IDLE) ? 8'd0 : {clk_hist[6:0], dap0};
        if (state == S_IDLE) begin
            counting    <= 1'b0;
            pulses_left <= 9'd0;
            go_r        <= 1'b1;
        end else if (fast_r && start_seen) begin
            counting    <= 1'b1;
            pulses_left <= pulse_init;
            go_r        <= pi_gt1;
        end else begin
            if (pulses_left != 9'd0) begin
                pulses_left <= pulses_left - 1'b1;
            end
            /* A clock now leaves pulses_left - 1 for after this one. */
            if (counting) begin
                go_r <= (pulses_left > 9'd2);
            end
        end

        /* Reset only what must be right before the first frame; the rest is
         * loaded at start (keeps reset fanout down). */
        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            fast_r      <= 1'b0;
            dat_oe      <= 1'b1;
            tick        <= {DIV_WIDTH{1'b0}};
            tick_done   <= (div == {DIV_WIDTH{1'b0}});
            phase       <= 1'b0;
        end else if (state == S_IDLE) begin
            if (start) begin
                /* Hand the line over before the first clock. */
                dat_oe      <= 1'b0;
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
                max_wait_r  <= max_wait;
                wait_limit  <= (max_wait == 16'd0) ? 16'd0 : max_wait - 1'b1;
                /* wait_cycles starts at zero, so the limit is already reached
                 * when max_wait allows one clock or none. */
                at_limit    <= (max_wait <= 16'd1);
                trail_left  <= trail_clocks;
                crc_rst     <= 1'b1;
                busy        <= 1'b1;
                /* Raw mode goes straight into the payload. */
                state       <= no_hunt ? S_DATA : S_HUNT;
                index       <= 7'd0;
                tick        <= {DIV_WIDTH{1'b0}};
                tick_done   <= fast | (div == {DIV_WIDTH{1'b0}});
                phase       <= fast;          /* fast: nothing has landed */
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
                tick_done <= div_zero;

                /* The clock (dap0_next), generated whichever edge the bit is
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

                /* The bit, on whichever edge sample_phase names. */
                if (sample_at) begin
                    case (state)
                        S_HUNT: begin
                            if (dap1_in) begin
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
                                trail_left <= 8'd0;
                            end
                            /* wait_cycles is updated after this case. */
                        end
                        S_DATA: begin
                            /* Two bits every sample, in both modes.  Narrow
                             * mode's DAP2 bit is overwritten next sample; the
                             * last lands above the payload, unread. */
                            pay_even[index[5:0]] <= dap1_in;
                            pay_odd[index[4:0]]  <= dap2_in;
                            all_ones <= all_ones & dap1_in & (dap2_in | ~wide_r);
                            if (at_last) begin
                                state <= crc_r ? S_CRC
                                       : (has_trail ? S_TRAIL : S_END);
                            end
                        end
                        S_CRC: begin
                            crc_even[index[2:0]] <= dap1_in;
                            crc_odd[index[1:0]]  <= dap2_in;
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
                    index <= index_next;
                    case (state)
                        S_DATA:  at_last <= at_last ? 1'b0 : (index + 7'd1 == nbits_last);
                        S_CRC:   at_last <= at_last ? 1'b0 : (index + 7'd1 == crc_last);
                        default: at_last <= nb_zero;      /* the payload's first */
                    endcase

                    /* Fast mode samples back to back, so the stale flag
                     * can count the first payload sample too: WAIT may read
                     * one high there. */
                    if (hunting && !dap1_in && !at_limit) begin
                        wait_cycles <= wait_cycles + 1'b1;
                        at_limit    <= (wait_cycles + 1'b1 == wait_limit);
                    end
                end
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
