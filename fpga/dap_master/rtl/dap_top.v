/*
 * DAP master top level: register map, sequencer, reply and write FIFOs.
 *
 * The host writes a command and a parcel count; the fabric issues every parcel
 * of a block read or write itself, and the data moves as one SPI burst.
 *
 * Register map.  Byte wide, multi-byte fields little-endian.
 *
 *   0x00 STATUS   ro  0 busy, 1 done, 2 timed_out, 3 idle_high, 4 crc_ok,
 *                     5 fifo_empty, 6 fifo_full, 7 overrun (aborted on a full FIFO)
 *   0x01 CTRL     wo  0 start frame, 1 start block read (queued if busy),
 *                     2 abort a block stalled on its FIFO (and drop a queued start),
 *                     3 clear reply fifo, 4 start block write,
 *                     5 clear write fifo
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
 *                     5 receive wide without driving DAP2,
 *                     6 fast: one bit per fabric clock (48 MHz), DIV ignored
 *   0x0C LEAD     rw  idle clocks before each frame
 *   0x0D LEVEL    ro  12-bit bytes waiting in the FIFO, low byte first;
 *                     0x0E bit 4: a block start is queued (chaining)
 *   0x0F SKEW     rw  capture tap: [1:0] DAP1, [3:2] DAP2, in fabric clocks;
 *                     4 DAP1, 5 DAP2: sample half a clock later (falling edge),
 *                     [7:6] fast mode LAG: a DAP0 clock's bit is sampled
 *                     LAG + DAP1 tap + 1 fabric clocks after it
 *   0x10 DATA     rw  64-bit frame payload, low byte first
 *   0x18 GO       wo  CTRL again, after DATA: the frame registers and their
 *                     start in one auto-incrementing burst.  Its clear bit
 *                     is safe there for the same reason as CTRL's: it is the
 *                     last byte of the transfer.
 *   0x20 REPLY    ro  32 bits of the last reply, low byte first
 *   0x24 RCRC     ro  the six CRC bits that followed
 *   0x25 WAIT     ro  16-bit busy cycle count, low byte first
 *   0x27 ALIGN    ro  0 DAP2 carried the start bit too (wide mode alignment)
 *   0x40 FIFO     ro  pops one byte; does not auto-increment
 *   0x50 FIFO2    ro  the reply FIFO over two lines, with a level prefix
 *                     (spi_slave.v); does not auto-increment
 *   0x51 TESTPUSH wo  pushes one byte into the reply FIFO (sequencer idle),
 *                     for the host's self-test of 0x40/0x50
 *   0x43 WLEVEL   ro  16-bit bytes waiting in the write FIFO, low byte first
 *   0x48 WFIFO    wo  pushes one byte; does not auto-increment
 */

`default_nettype none

module dap_top #(
    /* Reply FIFO: 2**FIFO_AW bytes, one maximum block read at 10. */
    parameter integer FIFO_AW     = 10,
    /* Half a block: Q_WFETCH stalls on empty, so the host refills mid-block. */
    parameter integer WFIFO_DEPTH = 512
) (
    input  wire clk,
    input  wire rst,

    /* ESP32 link */
    input  wire spi_sck,
    inout  wire spi_si,     /* the second data line for 0x50 reads */
    output wire spi_so,
    input  wire spi_ss,

    /* Target */
    output wire dap0,
    inout  wire dap1,
    output reg  trst,
    /* Driven only while sending a wide frame; an input otherwise. */
    inout  wire dap2
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
    /* FLAGS bit 3: DATA is the finished frame; the host does the framing. */
    reg        r_raw     = 1'b0;
    /*
     * FLAGS bit 5: sample both lines, drive only DAP1.  Safe while the target
     * still drives DAP2 as a port pin; driving it then would short two outputs.
     */
    reg        r_rx_wide = 1'b0;
    /*
     * Per-line capture tap, in fabric clocks, for wide mode where DAP1 and DAP2
     * may be skewed.  Zero on both is the narrow-mode behaviour.
     */
    reg [1:0]  r_skew1   = 2'd0;
    reg [1:0]  r_skew2   = 2'd0;
    /* SKEW bits 4 and 5: take that line from the pad's falling-edge register. */
    reg        r_edge1   = 1'b0;
    reg        r_edge2   = 1'b0;
    /* SKEW [7:6]: the pad-to-sample latency beyond the tap, fast mode only. */
    reg [1:0]  r_lag     = 2'd0;
    /*
     * The receiver's whole latency, registered off the register file; the
     * + 1 is the output pad registers' clock.
     */
    reg [2:0]  rx_lag    = 3'd1;
    always @(posedge clk) rx_lag <= {1'b0, r_lag} + {1'b0, r_skew1} + 3'd1;
    /*
     * FLAGS bit 6: a bit every fabric clock.  The pad then makes DAP0 from
     * both clock edges: low for the first half of each cycle, high for the
     * second, the data changing at the cycle start.
     */
    reg        r_fast    = 1'b0;

    reg [31:0] s_reply;
    reg [5:0]  s_crc;
    reg [15:0] s_wait;
    reg        s_done, s_timed_out, s_idle_high, s_crc_ok, s_overrun;
    /* Wide mode only: did the reply's start bit appear on DAP2 as well. */
    reg        s_aligned;

    /* Registered busy flag, kept off the read mux's critical path. */
    reg        s_busy;

    /* ------------------------------------------------------------------ */
    /* SPI                                                                 */
    /* ------------------------------------------------------------------ */

    wire [6:0] reg_addr;
    wire [7:0] reg_wdata;
    wire       reg_we, reg_re;
    reg        reg_re_d, reg_re_d2, reg_re_d3;
    reg  [7:0] reg_rdata;
    /* Read pipeline: per-group bytes, the control group split in two halves. */
    reg  [7:0] q_ctrl_lo, q_ctrl_hi;
    reg  [7:0] q_ctrl, q_dat, q_rep, q_fifo;
    reg  [7:0] q_dat_d, q_rep_d, q_fifo_d;
    reg  [2:0] q_grp, q_grp_d;
    reg        q_port, q_port_d;
    reg        q_hi;

    /* The reply FIFO's read side, on SCK. */
    wire [7:0]       fifo_head;
    wire             fifo_rd_empty, fifo_pop, fifo_ahead;
    wire [FIFO_AW:0] fifo_rd_level;

    spi_slave #(.AW(FIFO_AW)) u_spi (
        .clk (clk), .rst (rst),
        .spi_sck (spi_sck), .spi_si (spi_si), .spi_so (spi_so), .spi_ss (spi_ss),
        .reg_addr (reg_addr), .reg_wdata (reg_wdata),
        .reg_we (reg_we), .reg_re (reg_re),
        .reg_rdata (reg_rdata),
        .fifo_head (fifo_head), .fifo_empty (fifo_rd_empty),
        .fifo_level (fifo_rd_level), .fifo_pop (fifo_pop), .fifo_ahead (fifo_ahead),
        .status ({s_busy, s_timed_out, s_overrun, r_block_pend}),
        .selected ()
    );

    /*
     * Register writes, a clock after the SPI bus.  The bus comes from the
     * SPI slave's corner of the die; registered here, every write decode
     * starts from flops beside the registers it feeds (timing).  A clock
     * is nothing next to a byte on the wire.
     */
    reg        wr_we = 1'b0;
    reg  [6:0] wr_addr;
    reg  [7:0] wr_data;
    always @(posedge clk) begin
        wr_we   <= reg_we && !rst;
        wr_addr <= reg_addr;
        wr_data <= reg_wdata;
    end
    /* CTRL, or GO (0x18): the same register at the end of the frame burst. */
    wire ctrl_we = wr_we && (wr_addr == 7'h01 || wr_addr == 7'h18);

    /*
     * The read side's copy of the address, for the same reason.  It costs
     * the read pipeline nothing: its second stage is ready a clock before
     * reg_re_d3 takes it, and a FIFO pop's new head reaches the first
     * stage on that same clock either way.
     */
    reg  [6:0] rd_addr;
    always @(posedge clk) rd_addr <= reg_addr;

    /* ------------------------------------------------------------------ */
    /* Reply FIFO                                                          */
    /* ------------------------------------------------------------------ */

    localparam integer FIFO_DEPTH = 1 << FIFO_AW;

    /* The writer's view: conservative, a pop shows up a few clocks late. */
    wire [FIFO_AW:0] fifo_cnt;
    wire [11:0]      fifo_count = fifo_cnt;
    wire             fifo_empty, fifo_full;
    /* Space for a whole parcel with a cycle of slack (the sequencer's gate). */
    reg              fifo_room  = 1'b1;

    reg        fifo_push;
    reg [7:0]  fifo_din;
    reg        fifo_clear;

    reply_fifo #(.AW(FIFO_AW)) u_fifo (
        .clk (clk), .rst (rst), .clear (fifo_clear),
        .push (fifo_push), .din (fifo_din),
        .count (fifo_cnt), .empty (fifo_empty), .full (fifo_full),
        .sck (spi_sck), .pop (fifo_pop), .ahead (fifo_ahead), .head (fifo_head),
        .rd_empty (fifo_rd_empty), .rd_level (fifo_rd_level)
    );

    /*
     * Room to stream a parcel on, decided at its start bit: its own word, the
     * next one, the one the pusher may still hold, and the count's lag.
     */
    reg              rx_room = 1'b1;

    always @(posedge clk) begin
        fifo_room <= (fifo_cnt < FIFO_DEPTH - 8);
        rx_room   <= (fifo_cnt < FIFO_DEPTH - 20);
    end

    /* ------------------------------------------------------------------ */
    /* Write FIFO                                                          */
    /* ------------------------------------------------------------------ */

    /* Words for a block write, streamed out by the sequencer as parcels. */
    reg [7:0]  wfifo_mem [0:WFIFO_DEPTH-1];
    reg [11:0] wfifo_wr, wfifo_rd;
    reg [11:0] wfifo_count;
    reg        wfifo_empty = 1'b1;
    reg        wfifo_full  = 1'b0;

    reg        wfifo_clear;
    reg [7:0]  wfifo_head;

    wire       wfifo_push = wr_we && (wr_addr == 7'h48) && !wfifo_full;

    /*
     * Pop combinationally from the state.  wfifo_head trails wfifo_rd by one
     * cycle, so a byte may be taken at most every other cycle.
     */
    wire       wfetch_go = (q == Q_WFETCH) && !wfetch_wait && !wfifo_empty;
    wire       wfifo_pop = wfetch_go;

    always @(posedge clk) begin
        wfifo_head <= wfifo_mem[wfifo_rd];

        if (rst || wfifo_clear) begin
            wfifo_wr    <= 12'd0;
            wfifo_rd    <= 12'd0;
            wfifo_count <= 12'd0;
            wfifo_empty <= 1'b1;
            wfifo_full  <= 1'b0;
        end else begin
            case ({wfifo_push, wfifo_pop})
                2'b10: begin
                    wfifo_mem[wfifo_wr] <= wr_data;
                    wfifo_wr    <= (wfifo_wr == WFIFO_DEPTH - 1) ? 12'd0
                                                                 : wfifo_wr + 1'b1;
                    wfifo_count <= wfifo_count + 1'b1;
                    wfifo_empty <= 1'b0;
                    wfifo_full  <= (wfifo_count + 1'b1 == WFIFO_DEPTH);
                end
                2'b01: begin
                    wfifo_rd    <= (wfifo_rd == WFIFO_DEPTH - 1) ? 12'd0
                                                                 : wfifo_rd + 1'b1;
                    wfifo_count <= wfifo_count - 1'b1;
                    wfifo_empty <= (wfifo_count == 12'd1);
                    wfifo_full  <= 1'b0;
                end
                2'b11: begin
                    wfifo_mem[wfifo_wr] <= wr_data;
                    wfifo_wr    <= (wfifo_wr == WFIFO_DEPTH - 1) ? 12'd0
                                                                 : wfifo_wr + 1'b1;
                    wfifo_rd    <= (wfifo_rd == WFIFO_DEPTH - 1) ? 12'd0
                                                                 : wfifo_rd + 1'b1;
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
    /* Block-read streaming (dap_frame_rx); assigned by the sequencer. */
    wire        rx_pdone, rx_stream;
    wire [8:0]  rx_parcels;
    wire [15:0] rx_wait;
    wire [62:0] rx_payload;
    wire [5:0]  rx_crc;
    wire        rx_dap0, rx_oe;

    reg [6:0]   rx_bits;
    reg         rx_expect_crc;

    /* dap1 is driven while either engine claims it; only one is ever busy. */
    wire dap1_in;
    /*
     * Between the parcels of a block the device keeps driving DAP1, and with
     * flow control that gap can last; the receiver's idle "take the line
     * back" must not fight it there.
     */
    wire block_gap = is_block && (q == Q_PARCEL || q == Q_STORE);
    wire drive    = tx_busy ? tx_oe : (rx_oe && !block_gap);
    wire dap1_out = tx_dap1;

    /* DAP2 is driven only while a wide frame is being sent. */
    wire drive2 = tx_busy && tx_oe2;

    /*
     * The pads.  Every DAP output and output enable leaves from a register
     * in its I/O cell, so DAP0, DAP1 and DAP2 share one clock-to-pad delay
     * and their skew is the bank's (0.5 ns), not the placer's.  (Driven
     * from fabric, DAP1's enable reached its pad 13.9 ns after the clock,
     * later than fast mode's DAP0 edge at half a cycle, and moved with
     * every seed.)  The pins therefore lag the engines by one clock, all
     * of them alike; the receiver takes its bit a clock later to match
     * (sample_late, and LAG + 1 in fast mode), so the line is sampled
     * where it always was relative to the pins.
     *
     * DAP1 and DAP2 sample in the I/O cell on both clock edges.  Every pad
     * takes both clocks: DAP0 and DAP1 share an I/O tile, whose two cells
     * must agree on them.  CLOCK_ENABLE is left open, as Lattice's
     * technology library advises: tied to 1 it costs a LUT, and nextpnr
     * then lost about 4 MHz across the whole fabric.
     */
    wire dap1_p, dap1_n, dap2_p, dap2_n;

    /* PIN_OUTPUT_REGISTERED_ENABLE_REGISTERED, PIN_INPUT_DDR */
    SB_IO #(.PIN_TYPE(6'b1101_00)) u_dap1_io (
        .PACKAGE_PIN   (dap1),
        .INPUT_CLK     (clk),
        .OUTPUT_CLK    (clk),
        .OUTPUT_ENABLE (drive),
        .D_OUT_0       (dap1_out),
        .D_IN_0        (dap1_p),
        .D_IN_1        (dap1_n)
    );

    SB_IO #(.PIN_TYPE(6'b1101_00)) u_dap2_io (
        .PACKAGE_PIN   (dap2),
        .INPUT_CLK     (clk),
        .OUTPUT_CLK    (clk),
        .OUTPUT_ENABLE (drive2),
        .D_OUT_0       (tx_dap2),
        .D_IN_0        (dap2_p),
        .D_IN_1        (dap2_n)
    );

    /*
     * DAP0 from the I/O cell's DDR output register: D_OUT_0 holds the first
     * half of each cycle, D_OUT_1 the second, both a clock behind the
     * engines' dap0 like the other pins.  Normal modes put dap0 in both
     * halves.  Fast mode holds the first half low and raises the second for
     * every cycle with a bit, so DAP0 rises mid-cycle with half a clock of
     * setup and of hold.  Only one engine is ever busy and an idle one holds
     * dap0 low.  The second half comes straight from a register: the
     * falling-edge register leaves only half a clock (timing).
     */
    wire dap0_eng = tx_dap0 | rx_dap0;
    reg  dap0_hi  = 1'b0;
    always @(posedge clk) dap0_hi <= dap0_eng;

    SB_IO #(.PIN_TYPE(6'b0100_00)) u_dap0_io (
        .PACKAGE_PIN   (dap0),
        .INPUT_CLK     (clk),
        .OUTPUT_CLK    (clk),
        .D_OUT_0       (dap0_eng & ~r_fast),
        .D_OUT_1       (dap0_hi)
    );

    /*
     * Synchronisers plus per-line capture tap.  Stage 0 is the pad register,
     * the rising edge's or (SKEW bit 4/5) the falling edge's, half a clock
     * later.  Tap 0 is two clocks of delay (narrow-mode timing); taps 1..3
     * sample one clock later each.  The tap mux is registered to keep it off
     * the start-bit hunt path.
     */
    wire      dap1_s0 = r_edge1 ? dap1_n : dap1_p;
    wire      dap2_s0 = r_edge2 ? dap2_n : dap2_p;
    reg [3:1] dap1_sync, dap2_sync;
    wire [3:0] dap1_taps = {dap1_sync, dap1_s0};
    wire [3:0] dap2_taps = {dap2_sync, dap2_s0};
    /*
     * The falling-edge register has half a clock to reach the tap, so it
     * goes through one LUT only: the select is decoded once (one-hot,
     * registered) and everything else is ORed together ahead of it.
     */
    reg       fall1 = 1'b0, fall2 = 1'b0;     /* tap 0, falling edge */
    reg       rise1 = 1'b1, rise2 = 1'b1;     /* tap 0, rising edge */
    always @(posedge clk) begin
        fall1 <= (r_skew1 == 2'd0) &&  r_edge1;
        rise1 <= (r_skew1 == 2'd0) && !r_edge1;
        fall2 <= (r_skew2 == 2'd0) &&  r_edge2;
        rise2 <= (r_skew2 == 2'd0) && !r_edge2;
    end
    (* keep *) wire dap1_rest = (r_skew1 == 2'd0) ? (rise1 & dap1_p)
                                                  : dap1_taps[r_skew1];
    (* keep *) wire dap2_rest = (r_skew2 == 2'd0) ? (rise2 & dap2_p)
                                                  : dap2_taps[r_skew2];
    reg       dap1_tap,  dap2_tap;
    /*
     * A copy of DAP1's sample for the receiver's decisions (start bit, busy
     * count, clock budget), so they do not share the net that fans out to
     * every payload and CRC bit (timing).  Kept: yosys would merge it.
     */
    (* keep *) reg dap1_ctl;
    always @(posedge clk) begin
        dap1_sync <= dap1_taps[2:0];
        dap2_sync <= dap2_taps[2:0];
        dap1_tap  <= dap1_rest | (fall1 & dap1_n);
        dap1_ctl  <= dap1_rest | (fall1 & dap1_n);
        dap2_tap  <= dap2_rest | (fall2 & dap2_n);
    end
    assign dap1_in = dap1_tap;
    wire   dap2_in = dap2_tap;

    /*
     * A block-write parcel is a start bit and 32 data bits, sent through the
     * raw-frame path.  The sequencer assembles it straight into DATA, so a block
     * write leaves DATA, DBITS and FLAGS holding the last parcel.
     */
    wire        tx_parcel = (q == Q_WPARCEL);
    wire [5:0]  tx_dbits  = tx_parcel ? 6'd33 : r_dbits;

    dap_frame_tx #(.DIV_WIDTH(8)) u_tx (
        .clk (clk), .rst (rst), .div (r_div),
        .start (tx_start), .cmd (r_cmd), .len (r_len),
        .data_bits (tx_dbits), .data (r_data[62:0]), .lead (r_lead),
        .wide (r_wide), .raw (r_raw | tx_parcel), .fast (r_fast),
        .busy (tx_busy), .done (tx_done),
        .dap0 (tx_dap0),
        .dap1 (tx_dap1), .dat_oe (tx_oe),
        .dap2 (tx_dap2), .dat2_oe (tx_oe2)
    );

    /* Registered: the OR otherwise lands on the receiver's start path. */
    reg rx_wide_any = 1'b0;
    always @(posedge clk) rx_wide_any <= r_wide | r_rx_wide;

    dap_frame_rx #(.DIV_WIDTH(8)) u_rx (
        .clk (clk), .rst (rst), .div (r_div),
        .start (rx_start), .reply_bits (rx_bits),
        .max_wait (r_maxwait), .trail_clocks (r_trail),
        .expect_crc (rx_expect_crc),
        .no_hunt (r_no_hunt),
        .wide (rx_wide_any), .fast (r_fast), .lag (rx_lag),
        .stream (rx_stream), .parcels (rx_parcels),
        .room (rx_room), .pdone (rx_pdone),
        .start_aligned (rx_aligned),
        .busy (rx_busy), .done (rx_done),
        .wait_cycles (rx_wait), .timed_out (rx_timed_out),
        .idle_high (rx_idle_high), .crc_ok (rx_crc_ok),
        .payload (rx_payload), .crc (rx_crc),
        .dap0 (rx_dap0),
        .dap1_in (dap1_in), .dap1_ctl (dap1_ctl), .dap2_in (dap2_in), .dat_oe (rx_oe)
    );

    /* ------------------------------------------------------------------ */
    /* Sequencer                                                           */
    /* ------------------------------------------------------------------ */

    localparam [3:0] Q_IDLE    = 4'd0,
                     Q_TX      = 4'd1,
                     Q_RX      = 4'd2,
                     Q_PARCEL  = 4'd3,
                     Q_STORE   = 4'd4,
                     Q_DONE    = 4'd5,
                     /* Block write: command ack, then parcel + ack per word. */
                     Q_WACK    = 4'd6,
                     Q_WFETCH  = 4'd7,
                     Q_WPARCEL = 4'd8;

    /* One-hot: the state decode sits on the sequencer's critical paths. */
    (* fsm_encoding = "one-hot" *) reg [3:0] q;
    reg       is_block;
    reg       is_bwrite;
    reg [8:0] parcels_left;
    /*
     * parcels_left == 1 and == 2, kept beside the counter so no compare sits
     * on the state changes that read them (timing).
     */
    reg       pl1, pl2;
    reg [31:0] store_word;

    assign rx_stream  = is_block && !is_bwrite;
    assign rx_parcels = parcels_left;

    /*
     * The pusher: a received parcel goes into the reply FIFO a byte a clock
     * while the sequencer is already receiving the next one, so the DAP no
     * longer waits four clocks per parcel, and the store is out of the
     * sequencer's enables (timing).  TESTPUSH bytes go in here too, idle only.
     */
    reg        store_go = 1'b0;
    reg        pushing  = 1'b0;
    reg [1:0]  push_left;
    reg [31:0] push_word;

    always @(posedge clk) begin
        fifo_push <= 1'b0;
        if (rst) begin
            pushing <= 1'b0;
        end else if (store_go) begin
            push_word <= store_word;
            push_left <= 2'd3;
            pushing   <= 1'b1;
        end else if (pushing) begin
            fifo_push <= 1'b1;
            fifo_din  <= push_word[7:0];
            push_word <= {8'd0, push_word[31:8]};
            push_left <= push_left - 1'b1;
            if (push_left == 2'd0)
                pushing <= 1'b0;
        end else if (q == Q_IDLE && wr_we && wr_addr == 7'h51) begin
            /* TESTPUSH (0x51): a byte straight into the reply FIFO, so the
             * host can prove its read paths with no target. */
            fifo_push <= 1'b1;
            fifo_din  <= wr_data;
        end
    end

    /* Bytes of the current block-write word taken from the write FIFO. */
    reg [1:0]  wbyte;
    reg        wfetch_wait;
    /* Registered "last parcel" flag, updated alongside the counter. */
    reg        wlast;

    reg start_frame_req, start_bwrite_req;
    /* CTRL bit 2: leave a block stalled on a full reply or empty write FIFO. */
    reg abort_req = 1'b0;
    /* CTRL bit 1 written: queue a block read (decoded a clock early). */
    reg pend_set  = 1'b0;

    /*
     * A block-read start written while the sequencer is busy is held and
     * taken the moment it goes idle, so the host can queue the next block
     * (frame registers loaded after this one began) and blocks run back to
     * back with the FIFO drained continuously.  A failed block drops it.
     */
    reg  r_block_pend = 1'b0;
    /* The host never mixes a queued block with another start. */
    wire block_take   = (q == Q_IDLE) && r_block_pend;
    wire block_cancel = (q == Q_DONE) && (s_timed_out || s_overrun);

    /*
     * STATUS timed_out, kept out of the sequencer's case (timing): cleared
     * as each frame's command goes out, set by a reply that timed out (one
     * arrives only in Q_WACK, Q_RX or Q_PARCEL) or by an abort that ends a
     * stalled block.  A reply that did not time out leaves it clear.
     */
    wire to_clear = (q == Q_TX) && tx_done;
    wire to_abort = abort_req &&
                    (q == Q_WFETCH || (q == Q_STORE && !fifo_room));
    always @(posedge clk) begin
        if (to_clear)
            s_timed_out <= 1'b0;
        else if ((rx_done && rx_timed_out) || to_abort)
            s_timed_out <= 1'b1;
    end

    always @(posedge clk) begin
        tx_start    <= 1'b0;
        rx_start    <= 1'b0;
        store_go    <= 1'b0;
        wfetch_wait <= 1'b0;
        /* Busy through the idle cycle between chained blocks, and until the
         * last parcel is in the FIFO. */
        s_busy      <= (q != Q_IDLE) || r_block_pend || store_go || pushing;

        if (rst) begin
            q         <= Q_IDLE;
            s_done    <= 1'b0;
            s_overrun <= 1'b0;
            is_bwrite <= 1'b0;
        end else begin
            case (q)
                Q_IDLE: begin
                    if (start_frame_req || r_block_pend || start_bwrite_req) begin
                        is_block      <= r_block_pend;
                        is_bwrite     <= start_bwrite_req;
                        parcels_left  <= {1'b0, r_parcels} + 9'd1;
                        pl1           <= (r_parcels == 8'd0);
                        pl2           <= (r_parcels == 8'd1);
                        /* At least one parcel always follows the command. */
                        wlast         <= 1'b0;
                        /* The reply flags are cleared in Q_TX. */
                        s_done        <= 1'b0;
                        s_overrun     <= 1'b0;
                        tx_start      <= 1'b1;
                        q             <= Q_TX;
                    end
                end

                Q_TX: begin
                    if (tx_done) begin
                        s_idle_high   <= 1'b0;
                        s_crc_ok      <= 1'b0;
                        s_aligned     <= 1'b0;
                        /*
                         * Plain frame: r_rbits and a CRC.  Block read: 32-bit
                         * parcels, CRC on the last only.  Block write: a bare
                         * start bit per command and per parcel.
                         */
                        rx_bits       <= is_bwrite ? 7'd0
                                       : is_block  ? 7'd32 : r_rbits;
                        rx_expect_crc <= is_bwrite ? 1'b0
                                       : is_block  ? pl1
                                                   : 1'b1;
                        rx_start      <= 1'b1;
                        q             <= is_bwrite ? Q_WACK
                                       : is_block  ? Q_PARCEL : Q_RX;
                    end
                end

                /*
                 * Acknowledge for the command and every parcel.  A timeout ends
                 * the block: the device has stopped listening.
                 */
                Q_WACK: begin
                    if (rx_done) begin
                        /* Written on every ack; the last one leaves the result. */
                        s_wait      <= rx_wait;
                        s_crc_ok    <= ~rx_timed_out;

                        if (rx_timed_out || wlast) begin
                            q <= Q_DONE;
                        end else begin
                            wbyte <= 2'd0;
                            q     <= Q_WFETCH;
                        end
                    end
                end

                /*
                 * Four FIFO bytes make one parcel, one byte every other cycle.
                 * An empty FIFO stalls rather than fails; the device's MAXWAIT
                 * catches a host that has stopped.
                 */
                Q_WFETCH: begin
                    if (abort_req) begin
                        q           <= Q_DONE;
                    end else if (wfetch_go) begin
                        wfetch_wait <= 1'b1;
                        if (wbyte == 2'd3) begin
                            tx_start <= 1'b1;
                            q        <= Q_WPARCEL;
                        end else begin
                            wbyte <= wbyte + 1'b1;
                        end
                    end
                end

                Q_WPARCEL: begin
                    if (tx_done) begin
                        parcels_left  <= parcels_left - 1'b1;
                        pl1           <= pl2;
                        pl2           <= (parcels_left == 9'd3);
                        wlast         <= pl1;
                        rx_bits       <= 7'd0;
                        rx_expect_crc <= 1'b0;
                        rx_start      <= 1'b1;
                        q             <= Q_WACK;
                    end
                end

                Q_RX: begin
                    if (rx_done) begin
                        s_reply     <= rx_payload[31:0];
                        s_crc       <= rx_crc;
                        s_wait      <= rx_wait;
                        s_idle_high <= rx_idle_high;
                        s_crc_ok    <= rx_crc_ok;
                        s_aligned   <= rx_aligned;
                        q           <= Q_DONE;
                    end
                end

                Q_PARCEL: begin
                    /*
                     * A streamed parcel: the receiver is already hunting for
                     * the next one, and it checked the FIFO's room before it
                     * let this one run on, so the word goes straight to the
                     * pusher.
                     */
                    if (rx_pdone) begin
                        store_word   <= rx_payload[31:0];
                        store_go     <= 1'b1;
                        parcels_left <= parcels_left - 1'b1;
                        pl1          <= pl2;
                        pl2          <= (parcels_left == 9'd3);
                    end
                    if (rx_done) begin
                        store_word  <= rx_payload[31:0];
                        s_wait      <= rx_wait;
                        s_crc       <= rx_crc;
                        /* A timed-out parcel ends the block; no junk in the FIFO. */
                        if (rx_timed_out) begin
                            q           <= Q_DONE;
                        end else begin
                            s_crc_ok <= rx_crc_ok;
                            q        <= Q_STORE;
                        end
                    end
                end

                /*
                 * Flow control: a word is stored only with room for it (the
                 * level is a few clocks old and the pusher may still hold the
                 * last word, hence the margin of eight).  Waiting here stops
                 * DAP0 between parcels, which pauses the device; the host
                 * catches up and the block resumes.  An abort gives up.
                 */
                Q_STORE: begin
                    if (!fifo_room) begin
                        if (abort_req) begin
                            s_overrun   <= 1'b1;
                            q           <= Q_DONE;
                        end
                    end else begin
                        store_go     <= 1'b1;
                        parcels_left <= parcels_left - 1'b1;
                        pl1          <= pl2;
                        pl2          <= (parcels_left == 9'd3);
                        if (pl1) begin
                            q <= Q_DONE;
                        end else begin
                            rx_expect_crc <= pl2;
                            rx_start      <= 1'b1;
                            q             <= Q_PARCEL;
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
        start_bwrite_req <= 1'b0;
        /* Driven only from this block; two drivers get resolved to one by yosys. */
        fifo_clear      <= 1'b0;
        wfifo_clear     <= 1'b0;

        abort_req <= !rst && ctrl_we && wr_data[2];

        if (rst || block_take || block_cancel || abort_req) begin
            r_block_pend <= 1'b0;
        end
        pend_set  <= !rst && ctrl_we && wr_data[1];
        if (pend_set) begin
            r_block_pend <= 1'b1;
        end

        if (rst) begin
            trst      <= 1'b1;         /* released; asserting it resets the target */
            r_no_hunt <= 1'b0;         /* ordinary hunting is the operating mode */
            r_wide    <= 1'b0;         /* narrow until the host has run DAPISC */
            r_raw     <= 1'b0;
            r_rx_wide <= 1'b0;
            r_skew1   <= 2'd0;
            r_skew2   <= 2'd0;
            r_edge1   <= 1'b0;
            r_edge2   <= 1'b0;
            r_lag     <= 2'd0;
            r_fast    <= 1'b0;
        end else begin
            /*
             * Block-write parcel assembly, here because this block owns DATA.
             * Bit 0 is the start bit, bits 32:1 the word.
             */
            if (wfetch_go) begin
                r_data <= {31'd0, wfifo_head, r_data[32:9], 1'b1};
            end

            if (wr_we) begin
                case (wr_addr)
                    7'h01, 7'h18: begin
                        start_frame_req <= wr_data[0];
                        start_bwrite_req <= wr_data[4];
                        if (wr_data[3]) fifo_clear <= 1'b1;
                        if (wr_data[5]) wfifo_clear <= 1'b1;
                    end
                    7'h02: r_div     <= wr_data;
                    7'h03: r_cmd     <= wr_data[4:0];
                    7'h04: r_len     <= wr_data[5:0];
                    7'h05: r_dbits   <= wr_data[5:0];
                    7'h06: r_rbits   <= wr_data[6:0];
                    7'h07: r_trail   <= wr_data;
                    7'h08: r_maxwait[7:0]  <= wr_data;
                    7'h09: r_maxwait[15:8] <= wr_data;
                    7'h0A: r_parcels <= wr_data;
                    7'h0C: r_lead    <= wr_data[5:0];
                    7'h0B: begin
                        trst      <= ~wr_data[0];      /* 1 = assert = drive low */
                        r_no_hunt <=  wr_data[1];
                        r_wide    <=  wr_data[2];
                        r_raw     <=  wr_data[3];
                        r_rx_wide <=  wr_data[5];
                        r_fast    <=  wr_data[6];
                    end
                    7'h0F: begin
                        r_skew1 <= wr_data[1:0];
                        r_skew2 <= wr_data[3:2];
                        r_edge1 <= wr_data[4];
                        r_edge2 <= wr_data[5];
                        r_lag   <= wr_data[7:6];
                    end
                    7'h10: r_data[7:0]   <= wr_data;
                    7'h11: r_data[15:8]  <= wr_data;
                    7'h12: r_data[23:16] <= wr_data;
                    7'h13: r_data[31:24] <= wr_data;
                    7'h14: r_data[39:32] <= wr_data;
                    7'h15: r_data[47:40] <= wr_data;
                    7'h16: r_data[55:48] <= wr_data;
                    7'h17: r_data[63:56] <= wr_data;
                    default: ;
                endcase
            end

            /*
             * Four-cycle read pipeline, well inside the SPI dummy byte:
             * stage 1 registers each group's byte (control group as two halves),
             * stage 2 picks the control half, stage 3 picks the group.
             */
            q_ctrl_lo <= rd_ctrl_lo;
            q_ctrl_hi <= rd_ctrl_hi;
            q_hi      <= rd_addr[3];
            q_dat    <= rd_dat;
            q_rep    <= rd_rep;
            q_fifo   <= rd_fifo;
            q_grp    <= rd_addr[6:4];
            q_port   <= (rd_addr[6:4] == 3'h4);

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
     * Read mux, grouped by address range to keep the LUT depth short.  The
     * byte within a group is chosen by a one-hot select registered off the
     * bus address alongside rd_addr, so each 8:1 is an AND-OR, two LUT levels
     * (timing).
     */
    reg [7:0] rd_sel = 8'd1;
    always @(posedge clk) rd_sel <= 8'd1 << reg_addr[2:0];

    function [7:0] pick8(input [7:0] sel, input [63:0] v);   /* v: byte 7 .. byte 0 */
        integer i;
        begin
            pick8 = 8'd0;
            for (i = 0; i < 8; i = i + 1)
                pick8 = pick8 | ({8{sel[i]}} & v[8*i +: 8]);
        end
    endfunction

    wire [7:0] rd_ctrl_lo = pick8(rd_sel, {
        r_trail,                                        /* 0x07 */
        {1'b0, r_rbits},
        {2'd0, r_dbits},
        {2'd0, r_len},
        {3'd0, r_cmd},
        r_div,
        r_trail,                                        /* 0x01, CTRL: write only */
        {s_overrun, fifo_full, fifo_empty, s_crc_ok,
         s_idle_high, s_timed_out, s_done, s_busy}});   /* 0x00 */

    wire [7:0] rd_ctrl_hi = pick8(rd_sel, {
        {r_lag, r_edge2, r_edge1, r_skew2, r_skew1},    /* 0x0F */
        /* 0x0E: LEVEL high nibble, and a block start still queued. */
        {3'd0, r_block_pend, fifo_count[11:8]},
        /* LEVEL: reply FIFO fill, so the host can drain during a block. */
        fifo_count[7:0],
        {2'd0, r_lead},
        {1'b0, r_fast, r_rx_wide, 1'b0, r_raw, r_wide, r_no_hunt, ~trst},
        r_parcels,
        r_maxwait[15:8],
        r_maxwait[7:0]});                               /* 0x08 */

    wire [7:0] rd_dat = pick8(rd_sel, r_data);

    wire [7:0] rd_rep = pick8(rd_sel, {
        /* 0x27 ALIGN.  Decoded here, not in the control group, for timing. */
        {7'd0, s_aligned},
        s_wait[15:8],
        s_wait[7:0],
        {2'd0, s_crc},
        s_reply});                                      /* 0x20 .. 0x23 */

    /* FIFO port group: reply FIFO data and write FIFO level. */
    reg [7:0] rd_fifo;

    always @(*) begin
        case (rd_addr[3:0])
            /* WLEVEL: bytes waiting in the write FIFO (the host knows the depth). */
            4'h3:    rd_fifo = wfifo_count[7:0];
            4'h4:    rd_fifo = {4'd0, wfifo_count[11:8]};
            /* 0x40/0x50 data comes from the FIFO's SCK side directly. */
            default: rd_fifo = 8'h00;
        endcase
    end
endmodule

`default_nettype wire
