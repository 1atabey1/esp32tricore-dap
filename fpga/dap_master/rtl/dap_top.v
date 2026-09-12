/*
 * DAP master top level: register map, sequencer, reply FIFO.
 *
 * The reason this exists in fabric at all is the block read.  On the ESP32 the
 * cost of a 1 kB block is 256 parcels, each one a start-bit hunt and 32 clocks
 * driven by software; that path was measured at 453 kB/s and the limit was the
 * per-bit loop, not the wire.  Here the host writes a command, sets a parcel
 * count, and reads the answer back as one DMA burst - the fabric issues every
 * parcel itself and the host is not in the loop at all.
 *
 * Register map.  Byte wide, because the SPI transport is byte wide, and
 * little-endian for multi-byte fields to match the host.
 *
 *   0x00 STATUS   ro  0 busy, 1 done, 2 timed_out, 3 idle_high, 4 crc_ok,
 *                     5 fifo_empty, 6 fifo_full, 7 overrun
 *   0x01 CTRL     wo  0 start frame, 1 start block, 2 abort, 3 clear fifo
 *   0x02 DIV      rw  bit period = 2*(DIV+1) fabric clocks
 *   0x03 CMD      rw  the 5-bit command
 *   0x04 LEN      rw  the LEN field as it goes on the wire
 *   0x05 DBITS    rw  how many DATA bits follow it - not the same thing as LEN
 *   0x06 RBITS    rw  payload bits to expect back; 0 is a bare acknowledge
 *   0x07 TRAIL    rw  clocks to issue after a reply
 *   0x08 MAXWAIT  rw  16-bit, low byte first
 *   0x0A PARCELS  rw  block read: parcels minus one, so 0xFF is 256
 *   0x0B FLAGS    rw  0 TRST asserted, 1 raw reply window (no start-bit hunt),
 *                     2 wide mode (DAP1 even bits, DAP2 odd),
 *                     3 raw frame (DATA is the whole frame, DBITS its length),
 *                     5 receive wide without driving DAP2
 *   0x0C LEAD     rw  idle clocks before each frame
 *   0x0D LEVEL    ro  16-bit bytes waiting in the FIFO, low byte first
 *   0x0F SKEW     rw  capture tap: [1:0] DAP1, [3:2] DAP2, in fabric clocks
 *   0x10 DATA     rw  64-bit frame payload, low byte first
 *   0x20 REPLY    ro  32 bits of the last reply, low byte first
 *   0x24 RCRC     ro  the six CRC bits that followed
 *   0x25 WAIT     ro  16-bit busy cycle count, low byte first
 *   0x27 LINES    ro  0 DAP2 carried the start bit too (wide mode alignment),
 *                     1/2 DAP2 seen low/high while we were transmitting,
 *                     3/4 DAP2 seen low/high while the target had the lines,
 *                     5/6 DAP1 seen low/high while the target had the lines
 *   0x40 FIFO     ro  pops one byte; does not auto-increment
 *   0x41 SCAN     ro  16-bit, low byte first: the level on every other BANK0
 *                     pad, for finding which one a target signal reaches
 *
 * Everything the device turned out to be fussy about is a register rather than
 * a constant - DIV, TRAIL, MAXWAIT - because every one of those was settled on
 * hardware by sweeping it, and the next person will have to sweep them again
 * against a different target.
 */

`default_nettype none

module dap_top #(
    parameter integer FIFO_DEPTH = 1088    /* 1 kB block plus headroom */
) (
    input  wire clk,
    input  wire rst,

    /* ESP32 link */
    input  wire spi_sck,
    input  wire spi_si,
    output wire spi_so,
    input  wire spi_ss,

    /* Target */
    output wire dap0,
    inout  wire dap1,
    output reg  trst,
    /* Bidirectional in wide mode and an input otherwise: a line the design is
     * not using is better left undriven than held at a level. */
    inout  wire dap2,

    /* Every other BANK0 pad, as inputs - see the note in the board top. */
    input  wire [15:0] scan
);
    /* ------------------------------------------------------------------ */
    /* Registers                                                           */
    /* ------------------------------------------------------------------ */

    reg [7:0]  r_div    = 8'd11;     /* ~2 MHz from a 48 MHz fabric clock */
    reg [4:0]  r_cmd;
    reg [5:0]  r_len;
    reg [5:0]  r_dbits;
    reg [6:0]  r_rbits;
    reg [7:0]  r_trail  = 8'd1;
    reg [5:0]  r_lead   = 6'd2;
    reg [15:0] r_maxwait = 16'd256;
    reg [7:0]  r_parcels;
    reg [63:0] r_data;
    reg        r_no_hunt = 1'b0;  /* FLAGS bit 1: keep the whole reply window */
    reg        r_wide    = 1'b0;  /* FLAGS bit 2: two bits per DAP0 clock */
    /*
     * FLAGS bit 3: DATA is the finished frame, not its payload.
     *
     * The wide framing rule is not documented anywhere this project can reach,
     * and the reconstruction it was built from - per-field padding, covered by
     * the CRC - is rejected by the device.  Rebuilding the bitstream for each
     * guess is a twenty-minute loop; assembling candidate frames on the host is
     * a one-second loop.  So the fabric stops having an opinion about framing
     * and the host holds it instead.
     */
    reg        r_raw     = 1'b0;
    /*
     * FLAGS bit 5: sample both lines, drive only DAP1.
     *
     * Looking at what DAP2 is doing must not mean driving it.  While the
     * target has that pin configured as a push-pull output - which it does, by
     * default, because in two-pin mode DAP2 is an ordinary port pin the
     * application owns - a wide transmission from this end puts two drivers on
     * one net through a 22 ohm series resistor.  That is about 150 mA, and
     * sixty-four frames of it browned the board out into a boot loop.
     *
     * With this bit the receiver deinterleaves both lines and the transmitter
     * stays narrow, so DAP2 is only ever read.  It is what the wiring probe
     * uses, and it is safe against a target that has not yet handed the pin
     * over.
     */
    reg        r_rx_wide = 1'b0;
    /*
     * Per-line capture tap, in fabric clocks back from the sample instant.
     *
     * Wide mode needs this and narrow mode does not: the silicon does not
     * guarantee DAP1 and DAP2 leave the pads together, and at a divider of 0 a
     * bit period is two fabric clocks, so a skew of one clock between the lines
     * is half a bit.  A tap per line lets the host sweep the two independently
     * and keep whichever pair reads sync's 0xAAAAAAAA back correctly - which is
     * a measurement, not a guess about the board.
     *
     * Zero on both is the narrow-mode behaviour exactly, so this costs nothing
     * until it is used.
     */
    reg [1:0]  r_skew1   = 2'd0;
    reg [1:0]  r_skew2   = 2'd0;

    reg [31:0] s_reply;
    reg [5:0]  s_crc;
    reg [15:0] s_wait;
    reg        s_done, s_timed_out, s_idle_high, s_crc_ok, s_overrun;
    /* Wide mode only: did the reply's start bit appear on DAP2 as well. */
    reg        s_aligned;
    /* The scan pads, through two stages like every other asynchronous input.
     * They feed nothing but a register read, so the second stage is where they
     * stop. */
    reg [15:0] scan_meta, scan_sync;

    /*
     * Sticky witnesses: what levels each data line was seen at, split by who
     * was driving at the time.
     *
     * "Wide mode does not answer" has three causes that a reply value cannot
     * tell apart, and these separate them with four flip-flops:
     *
     *   nothing set while transmitting -> our own driver is not reaching the
     *       pad, or the input path from it is dead.  The transmitter drives
     *       real data onto DAP2 during a wide command, so both levels must
     *       appear there; if they do not, the problem is below the protocol.
     *   both set while transmitting, nothing while receiving -> the pad and
     *       the net are fine and the target is simply not driving DAP2, which
     *       points at the device's mode rather than at this end.
     *   both set in both phases -> the line is alive in both directions and
     *       the fault is in the framing or the capture phase.
     *
     * DAP1's receive-phase witnesses are there for contrast: a reply window
     * that reads all zeros means something holds the line down, and knowing
     * whether DAP1 moved at all separates a stuffing target from a dead link.
     *
     * Cleared when a frame starts, so each pair describes one exchange.
     */
    reg        s_tx2_lo, s_tx2_hi;
    reg        s_rx2_lo, s_rx2_hi;
    reg        s_rx1_lo, s_rx1_hi;
    /*
     * "The sequencer is running", as a flip-flop rather than a compare.
     *
     * STATUS bit 0 was `q != Q_IDLE` written inline, which puts a three-bit
     * compare on the input of the read mux - and that mux is the design's
     * critical path now that the frame engines are not.  One cycle stale is
     * harmless: the host reads this over SPI, which is slower than the fabric
     * by more than an order of magnitude.
     */
    reg        s_busy;

    /* ------------------------------------------------------------------ */
    /* SPI                                                                 */
    /* ------------------------------------------------------------------ */

    wire [6:0] reg_addr;
    wire [7:0] reg_wdata;
    wire       reg_we, reg_re, reg_consume;
    reg        reg_re_d, reg_re_d2, reg_re_d3;
    reg  [7:0] reg_rdata;
    /* First pipeline stage of the read: each address group's byte, plus which
     * group the address is in.  The control group's sixteen bytes are the one
     * mux too deep to build in a single cycle, so it arrives as two halves and
     * is chosen from in the second stage. */
    reg  [7:0] q_ctrl_lo, q_ctrl_hi;
    reg  [7:0] q_ctrl, q_dat, q_rep, q_fifo;
    reg  [7:0] q_dat_d, q_rep_d, q_fifo_d;
    reg  [2:0] q_grp, q_grp_d;
    reg        q_port, q_port_d;
    reg        q_hi;

    spi_slave u_spi (
        .clk (clk), .rst (rst),
        .spi_sck (spi_sck), .spi_si (spi_si), .spi_so (spi_so), .spi_ss (spi_ss),
        .reg_addr (reg_addr), .reg_wdata (reg_wdata),
        .reg_we (reg_we), .reg_re (reg_re), .reg_consume (reg_consume),
        .reg_rdata (reg_rdata),
        .selected ()
    );

    /* ------------------------------------------------------------------ */
    /* Reply FIFO                                                          */
    /* ------------------------------------------------------------------ */

    reg [7:0]  fifo_mem [0:FIFO_DEPTH-1];
    reg [11:0] fifo_wr, fifo_rd;
    reg [11:0] fifo_count;
    /*
     * Registered rather than compared combinationally.  A 12-bit magnitude
     * compare against the depth sits in the middle of the sequencer's decisions
     * and is the sort of thing that quietly sets the whole design's clock
     * ceiling; a flip-flop updated alongside the counter costs nothing.
     */
    reg        fifo_empty = 1'b1;
    reg        fifo_full  = 1'b0;

    reg        fifo_push;
    reg [7:0]  fifo_din;
    reg        fifo_clear;

    /*
     * Popped the moment the byte is shifted out, combinationally rather than a
     * cycle later.
     *
     * The read of the *next* byte is pipelined and latches two cycles after
     * the fetch, so a pop that took an extra cycle to register would leave the
     * old head in place when that latch happened and send the same byte twice.
     * The address does not auto-increment at the port, so reg_addr still reads
     * 0x40 here.
     */
    /*
     * The port-address test is registered.  Comparing reg_addr here put a
     * seven-bit compare in front of the FIFO's read address, and the address
     * does not move during a drain anyway - it is the one register that does
     * not auto-increment, so a cycle-old answer is the same answer.
     */
    reg        at_port;
    always @(posedge clk) begin
        at_port <= (reg_addr == 7'h40);
    end

    wire       fifo_pop = reg_consume && at_port;

    /*
     * The byte at the read pointer, kept in a register.
     *
     * The FIFO port read used to be a memory lookup inside the register
     * decode - so a host read of 0x40 was a RAM access, the empty-flag mux and
     * the whole address decoder in one clock, and that became the critical
     * path once the enable chains and the payload mux were dealt with.  Read
     * ahead every cycle instead and the decode sees a flip-flop.
     *
     * Re-read unconditionally rather than only when the pointer moves: a push
     * into an empty FIFO changes the byte under a stationary read pointer, and
     * an update conditioned on the pointer would miss it.
     */
    /*
     * The incremented read pointer is carried in a register rather than
     * computed here.  As an inline `fifo_rd + 1` it is a twelve-bit carry
     * chain sitting in front of the FIFO's read address, and that chain became
     * the critical path; maintained alongside the pointer it costs twelve
     * flip-flops and leaves a plain two-way mux.
     */
    reg  [11:0] fifo_rd_p1;
    wire [11:0] fifo_rd_next = (fifo_pop && !fifo_empty) ? fifo_rd_p1 : fifo_rd;
    reg  [7:0]  fifo_head;

    always @(posedge clk) begin
        fifo_head <= fifo_mem[fifo_rd_next];
    end

    always @(posedge clk) begin
        if (rst || fifo_clear) begin
            fifo_wr    <= 12'd0;
            fifo_rd    <= 12'd0;
            fifo_rd_p1 <= 12'd1;
            fifo_count <= 12'd0;
            fifo_empty <= 1'b1;
            fifo_full  <= 1'b0;
        end else begin
            if (fifo_push && !fifo_full) begin
                fifo_mem[fifo_wr] <= fifo_din;
                fifo_wr <= (fifo_wr == FIFO_DEPTH-1) ? 12'd0 : fifo_wr + 1'b1;
            end
            if (fifo_pop && !fifo_empty) begin
                fifo_rd    <= fifo_rd_p1;
                fifo_rd_p1 <= (fifo_rd_p1 == FIFO_DEPTH-1) ? 12'd0
                                                           : fifo_rd_p1 + 1'b1;
            end
            case ({fifo_push && !fifo_full, fifo_pop && !fifo_empty})
                2'b10: begin
                    fifo_count <= fifo_count + 1'b1;
                    fifo_empty <= 1'b0;
                    fifo_full  <= (fifo_count + 1'b1 == FIFO_DEPTH[11:0]);
                end
                2'b01: begin
                    fifo_count <= fifo_count - 1'b1;
                    fifo_full  <= 1'b0;
                    fifo_empty <= (fifo_count == 12'd1);
                end
                default: ;
            endcase
        end
    end

    /* ------------------------------------------------------------------ */
    /* Frame transmit and receive                                          */
    /* ------------------------------------------------------------------ */

    reg         tx_start, rx_start;
    wire        tx_busy, tx_done, tx_dap0, tx_dap1, tx_oe, tx_dap2, tx_oe2;
    wire        rx_aligned;
    wire        rx_busy, rx_done, rx_timed_out, rx_idle_high, rx_crc_ok;
    wire [15:0] rx_wait;
    wire [62:0] rx_payload;
    wire [5:0]  rx_crc;
    wire        rx_dap0, rx_oe;

    reg [6:0]   rx_bits;
    reg         rx_expect_crc;

    /* dap1 is bidirectional: driven while either engine claims it, released to
     * the target otherwise.  One of the two is always idle, so the mux is safe
     * rather than a race. */
    wire dap1_in;
    wire drive    = tx_busy ? tx_oe : rx_oe;
    wire dap1_out = tx_dap1;

    assign dap1 = drive ? dap1_out : 1'bz;

    /*
     * Synchronise the target's data before looking at it.
     *
     * DAP1 is driven by the target against its own clock domain - our DAP0
     * paces it, but the pad still changes asynchronously to this fabric clock -
     * so sampling the pad combinationally is a metastability hazard as well as
     * a long route: place and route put that path across the whole chip and it
     * set the design's clock ceiling on its own.
     *
     * The cost is two fabric clocks of delay on the sample point, about 42 ns
     * at 48 MHz, against a bit period of 500 ns at the default divider.  That
     * is inside the window where the target holds the bit, so it moves where we
     * look rather than what we see.
     */
    /*
     * Two stages of synchroniser and two more of adjustable delay.
     *
     * The first two are the metastability guard and are not optional.  The
     * extra taps are the per-line capture calibration: r_skew names how many
     * further clocks back the bit is taken from, so 0 is exactly what narrow
     * mode always did and 1..3 walk the sample later into the bit.  Delaying
     * the *sample* rather than advancing the clock is what keeps this out of
     * the timing path - it is a shift register and a four-way mux on one bit,
     * nowhere near the frame engines.
     */
    reg [3:0] dap1_sync, dap2_sync;
    reg       dap1_tap,  dap2_tap;
    always @(posedge clk) begin
        dap1_sync <= {dap1_sync[2:0], dap1};
        dap2_sync <= {dap2_sync[2:0], dap2};
        /*
         * The tap choice is registered, not wired into the sample.
         *
         * As a plain mux on dap1_in it sat in front of the receiver's
         * start-bit hunt, and that four-way choice on one bit became the
         * design's critical path - about two megahertz of it.  Here it feeds
         * nothing but a flip-flop's D input and has a whole clock to settle.
         *
         * Counting from dap1_sync[0] rather than [1] is what keeps tap 0
         * identical to the two-flop synchroniser this replaced: one stage in
         * the shift register plus this register is the same two clocks of
         * delay narrow mode was tuned against.  Taps 1 to 3 walk the sample
         * one clock later each.
         */
        scan_meta <= scan;
        scan_sync <= scan_meta;
        dap1_tap  <= dap1_sync[r_skew1];
        dap2_tap  <= dap2_sync[r_skew2];
    end
    assign dap1_in = dap1_tap;
    wire   dap2_in = dap2_tap;

    assign dap0    = tx_busy ? tx_dap0 : rx_dap0;
    /* DAP2 is an output only while a wide frame is being sent; the reply comes
     * back on it and every other moment leaves it to the target. */
    assign dap2    = (tx_busy && tx_oe2) ? tx_dap2 : 1'bz;

    dap_frame_tx #(.DIV_WIDTH(8)) u_tx (
        .clk (clk), .rst (rst), .div (r_div),
        .start (tx_start), .cmd (r_cmd), .len (r_len),
        .data_bits (r_dbits), .data (r_data[62:0]), .lead (r_lead),
        .wide (r_wide), .raw (r_raw),
        .busy (tx_busy), .done (tx_done),
        .dap0 (tx_dap0), .dap1 (tx_dap1), .dat_oe (tx_oe),
        .dap2 (tx_dap2), .dat2_oe (tx_oe2)
    );

    dap_frame_rx #(.DIV_WIDTH(8)) u_rx (
        .clk (clk), .rst (rst), .div (r_div),
        .start (rx_start), .reply_bits (rx_bits),
        .max_wait (r_maxwait), .trail_clocks (r_trail),
        .expect_crc (rx_expect_crc),
        .no_hunt (r_no_hunt),
        .wide (r_wide | r_rx_wide),
        .start_aligned (rx_aligned),
        .busy (rx_busy), .done (rx_done),
        .wait_cycles (rx_wait), .timed_out (rx_timed_out),
        .idle_high (rx_idle_high), .crc_ok (rx_crc_ok),
        .payload (rx_payload), .crc (rx_crc),
        .dap0 (rx_dap0), .dap1_in (dap1_in), .dap2_in (dap2_in), .dat_oe (rx_oe)
    );

    /* ------------------------------------------------------------------ */
    /* Sequencer                                                           */
    /* ------------------------------------------------------------------ */

    localparam [2:0] Q_IDLE    = 3'd0,
                     Q_TX      = 3'd1,
                     Q_RX      = 3'd2,
                     Q_PARCEL  = 3'd3,
                     Q_STORE   = 3'd4,
                     Q_DONE    = 3'd5;

    reg [2:0] q;
    reg       is_block;
    reg [8:0] parcels_left;
    reg [1:0] store_byte;
    reg [31:0] store_word;

    reg start_frame_req, start_block_req;

    always @(posedge clk) begin
        tx_start   <= 1'b0;
        rx_start   <= 1'b0;
        fifo_push  <= 1'b0;
        s_busy     <= (q != Q_IDLE);

        /*
         * Sampled here rather than in the frame engines: this is one flop per
         * witness with the pad's already-synchronised level on its D input,
         * and it stays out of the engines' enable chains entirely.
         */
        if (tx_start) begin
            s_tx2_lo <= 1'b0;  s_tx2_hi <= 1'b0;
            s_rx2_lo <= 1'b0;  s_rx2_hi <= 1'b0;
            s_rx1_lo <= 1'b0;  s_rx1_hi <= 1'b0;
        end else begin
            if (tx_busy) begin
                s_tx2_lo <= s_tx2_lo | ~dap2_in;
                s_tx2_hi <= s_tx2_hi |  dap2_in;
            end
            if (rx_busy) begin
                s_rx2_lo <= s_rx2_lo | ~dap2_in;
                s_rx2_hi <= s_rx2_hi |  dap2_in;
                s_rx1_lo <= s_rx1_lo | ~dap1_in;
                s_rx1_hi <= s_rx1_hi |  dap1_in;
            end
        end

        if (rst) begin
            q         <= Q_IDLE;
            s_done    <= 1'b0;
            s_overrun <= 1'b0;
        end else begin
            case (q)
                Q_IDLE: begin
                    if (start_frame_req || start_block_req) begin
                        is_block      <= start_block_req;
                        parcels_left  <= {1'b0, r_parcels} + 9'd1;
                        /*
                         * Only s_done is cleared here.  The three reply flags
                         * moved to Q_TX: clearing them from the start request
                         * put that request in the clock enable of every one of
                         * them, and the chain from start_frame_req through the
                         * state decode to those enables was the design's
                         * critical path.  They are set by the receive and read
                         * after it, so clearing them as the receive begins is
                         * the same guarantee one state later.
                         */
                        s_done        <= 1'b0;
                        tx_start      <= 1'b1;
                        q             <= Q_TX;
                    end
                end

                Q_TX: begin
                    if (tx_done) begin
                        s_timed_out   <= 1'b0;
                        s_idle_high   <= 1'b0;
                        s_crc_ok      <= 1'b0;
                        s_aligned     <= 1'b0;
                        /*
                         * A block read answers with parcels rather than one
                         * reply: 32 bits each, and a CRC only on the last.
                         */
                        rx_bits       <= is_block ? 7'd32 : r_rbits;
                        rx_expect_crc <= is_block ? (parcels_left == 9'd1) : 1'b1;
                        rx_start      <= 1'b1;
                        q             <= is_block ? Q_PARCEL : Q_RX;
                    end
                end

                Q_RX: begin
                    if (rx_done) begin
                        s_reply     <= rx_payload[31:0];
                        s_crc       <= rx_crc;
                        s_wait      <= rx_wait;
                        s_timed_out <= rx_timed_out;
                        s_idle_high <= rx_idle_high;
                        s_crc_ok    <= rx_crc_ok;
                        s_aligned   <= rx_aligned;
                        q           <= Q_DONE;
                    end
                end

                Q_PARCEL: begin
                    if (rx_done) begin
                        store_word  <= rx_payload[31:0];
                        store_byte  <= 2'd0;
                        s_wait      <= rx_wait;
                        s_crc       <= rx_crc;
                        /*
                         * A parcel that timed out ends the block there.  Going
                         * on would push whatever the wire happened to read into
                         * the FIFO behind good data, and the host has no way to
                         * tell the two apart once they are bytes in a buffer.
                         */
                        if (rx_timed_out) begin
                            s_timed_out <= 1'b1;
                            q           <= Q_DONE;
                        end else begin
                            s_crc_ok <= rx_crc_ok;
                            q        <= Q_STORE;
                        end
                    end
                end

                Q_STORE: begin
                    if (fifo_full) begin
                        /* Nobody is draining. Say so rather than lose bytes
                         * quietly - a short block the host knows about beats a
                         * full one with a hole in it. */
                        s_overrun <= 1'b1;
                        q         <= Q_DONE;
                    end else begin
                        fifo_push <= 1'b1;
                        fifo_din  <= store_word[7:0];
                        store_word <= {8'd0, store_word[31:8]};
                        if (store_byte == 2'd3) begin
                            parcels_left <= parcels_left - 1'b1;
                            if (parcels_left == 9'd1) begin
                                q <= Q_DONE;
                            end else begin
                                rx_expect_crc <= (parcels_left == 9'd2);
                                rx_start      <= 1'b1;
                                q             <= Q_PARCEL;
                            end
                        end else begin
                            store_byte <= store_byte + 1'b1;
                        end
                    end
                end

                Q_DONE: begin
                    s_done <= 1'b1;
                    q      <= Q_IDLE;
                end

                default: q <= Q_IDLE;
            endcase
        end
    end

    /* ------------------------------------------------------------------ */
    /* Register access                                                     */
    /* ------------------------------------------------------------------ */

    always @(posedge clk) begin
        start_frame_req <= 1'b0;
        start_block_req <= 1'b0;
        /*
         * Defaulted here, with the only code that sets it.
         *
         * It used to default to zero in the sequencer and be set here, which
         * is two always blocks driving one register - and yosys resolves that
         * by picking one, so CTRL bit 3 was tying fifo_clear to a constant and
         * clearing the FIFO did nothing at all.  Silent, because a block read
         * drains the FIFO completely anyway, so nothing had yet depended on
         * the clear actually happening.
         */
        fifo_clear      <= 1'b0;

        if (rst) begin
            trst      <= 1'b1;         /* released; asserting it resets the target */
            r_no_hunt <= 1'b0;         /* ordinary hunting is the operating mode */
            r_wide    <= 1'b0;         /* narrow until the host has run DAPISC */
            r_raw     <= 1'b0;
            r_rx_wide <= 1'b0;
            r_skew1   <= 2'd0;
            r_skew2   <= 2'd0;
        end else begin
            if (reg_we) begin
                case (reg_addr)
                    7'h01: begin
                        start_frame_req <= reg_wdata[0];
                        start_block_req <= reg_wdata[1];
                        if (reg_wdata[3]) fifo_clear <= 1'b1;
                    end
                    7'h02: r_div     <= reg_wdata;
                    7'h03: r_cmd     <= reg_wdata[4:0];
                    7'h04: r_len     <= reg_wdata[5:0];
                    7'h05: r_dbits   <= reg_wdata[5:0];
                    7'h06: r_rbits   <= reg_wdata[6:0];
                    7'h07: r_trail   <= reg_wdata;
                    7'h08: r_maxwait[7:0]  <= reg_wdata;
                    7'h09: r_maxwait[15:8] <= reg_wdata;
                    7'h0A: r_parcels <= reg_wdata;
                    7'h0C: r_lead    <= reg_wdata[5:0];
                    7'h0B: begin
                        trst      <= ~reg_wdata[0];      /* 1 = assert = drive low */
                        r_no_hunt <=  reg_wdata[1];
                        r_wide    <=  reg_wdata[2];
                        r_raw     <=  reg_wdata[3];
                        r_rx_wide <=  reg_wdata[5];
                    end
                    7'h0F: begin
                        r_skew1 <= reg_wdata[1:0];
                        r_skew2 <= reg_wdata[3:2];
                    end
                    7'h10: r_data[7:0]   <= reg_wdata;
                    7'h11: r_data[15:8]  <= reg_wdata;
                    7'h12: r_data[23:16] <= reg_wdata;
                    7'h13: r_data[31:24] <= reg_wdata;
                    7'h14: r_data[39:32] <= reg_wdata;
                    7'h15: r_data[47:40] <= reg_wdata;
                    7'h16: r_data[55:48] <= reg_wdata;
                    7'h17: r_data[63:56] <= reg_wdata;
                    default: ;
                endcase
            end

            /*
             * Three cycles to answer a fetch, and the decode split in two so
             * those cycles are worth having.
             *
             * Adding a pipeline register after the mux would change nothing:
             * the path from reg_addr through a five-level mux is still one
             * clock as far as place and route is concerned, and there is no
             * multicycle constraint to tell it otherwise.  The mux itself has
             * to be cut, so the group values are registered first (a short mux
             * on the low address bits) and the choice between them second.
             *
             * Four cycles now, not three.  Three was already needed because
             * the FIFO's head register updates a cycle after the pop, so the
             * group stage needs one more to catch it - otherwise a drain sends
             * each byte twice.  The fourth is the control group's sixteen-way
             * byte mux, which is four LUT levels of routing in one clock and
             * became the design's critical path once the frame engines were
             * dealt with and the DAP2 pad tightened the placement.  Split into
             * two eight-way muxes and a choice between them a cycle later, it
             * is half that.
             *
             * The budget is unchanged: the dummy byte a read sends is eight
             * SPI clocks, sixteen fabric clocks at the fastest link rate this
             * board runs, so four is still a quarter of it.
             */
            q_ctrl_lo <= rd_ctrl_lo;
            q_ctrl_hi <= rd_ctrl_hi;
            q_hi      <= reg_addr[3];
            q_dat    <= rd_dat;
            q_rep    <= rd_rep;
            q_fifo   <= rd_fifo;
            q_grp    <= reg_addr[6:4];
            q_port   <= (reg_addr[6:4] == 3'h4);

            /* Second stage: pick the control half, and carry everything else
             * along so the four groups still arrive together. */
            q_ctrl   <= q_hi ? q_ctrl_hi : q_ctrl_lo;
            q_dat_d  <= q_dat;
            q_rep_d  <= q_rep;
            q_fifo_d <= q_fifo;
            q_grp_d  <= q_grp;
            q_port_d <= q_port;

            reg_re_d  <= reg_re;
            reg_re_d2 <= reg_re_d;
            reg_re_d3 <= reg_re_d2;
            if (reg_re_d3) begin
                reg_rdata <= (q_grp_d == 3'h0) ? q_ctrl  :
                             (q_grp_d == 3'h1) ? q_dat_d :
                             (q_grp_d == 3'h2) ? q_rep_d :
                             q_port_d          ? q_fifo_d : 8'h00;
            end
        end
    end

    /*
     * The read mux, grouped by address range rather than written as one case
     * over all seven address bits.
     *
     * Yosys builds the flat version into a six-level LUT chain, and with
     * everything else on the critical path dealt with that decode was the
     * design's limit at about 43 MHz - short of the 48 the oscillator can
     * give.  Split into three small muxes on the low bits and one choice
     * between them on the high bits, it is three levels for the same result.
     */
    reg [7:0] rd_ctrl_lo, rd_ctrl_hi, rd_dat, rd_rep;

    always @(*) begin
        case (reg_addr[2:0])
            3'h0: rd_ctrl_lo = {s_overrun, fifo_full, fifo_empty, s_crc_ok,
                                s_idle_high, s_timed_out, s_done, s_busy};
            3'h2: rd_ctrl_lo = r_div;
            3'h3: rd_ctrl_lo = {3'd0, r_cmd};
            3'h4: rd_ctrl_lo = {2'd0, r_len};
            3'h5: rd_ctrl_lo = {2'd0, r_dbits};
            3'h6: rd_ctrl_lo = {1'b0, r_rbits};
            default: rd_ctrl_lo = r_trail;             /* 0x07 */
        endcase

        case (reg_addr[2:0])
            3'h0: rd_ctrl_hi = r_maxwait[7:0];         /* 0x08 */
            3'h1: rd_ctrl_hi = r_maxwait[15:8];
            3'h2: rd_ctrl_hi = r_parcels;
            3'h3: rd_ctrl_hi = {2'd0, r_rx_wide, 1'b0,
                                r_raw, r_wide, r_no_hunt, ~trst};
            3'h4: rd_ctrl_hi = {2'd0, r_lead};
            3'h7: rd_ctrl_hi = {4'd0, r_skew2, r_skew1};
            /*
             * How many bytes are waiting.  This is what lets the host drain
             * while the block is still arriving instead of after it: the wire
             * takes about 2 ms for a 1 kB block and the drain about 1.6 ms,
             * and run one after the other that is most of the cost of a block.
             */
            3'h5: rd_ctrl_hi = fifo_count[7:0];
            default: rd_ctrl_hi = {4'd0, fifo_count[11:8]};   /* 0x0E */
        endcase

        /*
         * DATA reads back.  Without this the host cannot tell a payload that
         * failed to reach the fabric from one the device simply ignored: the
         * frame is well formed either way and the only symptom is silence.
         * Every other writable register was already readable and this one
         * being write-only was an oversight - it cost a session.
         */
        case (reg_addr[2:0])
            3'd0: rd_dat = r_data[7:0];
            3'd1: rd_dat = r_data[15:8];
            3'd2: rd_dat = r_data[23:16];
            3'd3: rd_dat = r_data[31:24];
            3'd4: rd_dat = r_data[39:32];
            3'd5: rd_dat = r_data[47:40];
            3'd6: rd_dat = r_data[55:48];
            default: rd_dat = r_data[63:56];
        endcase

        case (reg_addr[2:0])
            3'd0: rd_rep = s_reply[7:0];
            3'd1: rd_rep = s_reply[15:8];
            3'd2: rd_rep = s_reply[23:16];
            3'd3: rd_rep = s_reply[31:24];
            3'd4: rd_rep = {2'd0, s_crc};
            3'd5: rd_rep = s_wait[7:0];
            3'd6: rd_rep = s_wait[15:8];
            /*
             * 0x27 LINES, in the slot the reply group already spends on a
             * default - so reading it costs no extra mux level.
             *
             * Where these are read from is not cosmetic: they come from the
             * receiver's end of the chip, and decoding them in the control
             * group or in LEVEL's high byte was measured at 46.7 and 48.5 MHz
             * respectively, against 50.4 here - and at 48.5 the block-read
             * drain, which polls LEVEL thousands of times per block, started
             * losing bytes.
             */
            default: rd_rep = {1'b0, s_rx1_hi, s_rx1_lo, s_rx2_hi, s_rx2_lo,
                               s_tx2_hi, s_tx2_lo, s_aligned};
        endcase
    end

    /*
     * The FIFO port, and the scan word beside it.
     *
     * This group is the right home for something read rarely: its mux is two
     * entries wide, where the control group's is sixteen and putting anything
     * extra there has cost this design several megahertz more than once.
     */
    reg [7:0] rd_fifo;

    always @(*) begin
        case (reg_addr[1:0])
            2'd1:    rd_fifo = scan_sync[7:0];
            2'd2:    rd_fifo = scan_sync[15:8];
            default: rd_fifo = fifo_empty ? 8'h00 : fifo_head;
        endcase
    end
endmodule

`default_nettype wire
