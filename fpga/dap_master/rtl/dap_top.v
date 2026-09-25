/*
 * DAP master top level: register map, sequencer, reply and write FIFOs.
 *
 * The host writes a command and a parcel count; the fabric issues every parcel
 * of a block read or write itself, and the data moves as one SPI burst.
 *
 * Register map.  Byte wide, multi-byte fields little-endian.
 *
 *   0x00 STATUS   ro  0 busy, 1 done, 2 timed_out, 3 idle_high, 4 crc_ok,
 *                     5 fifo_empty, 6 fifo_full, 7 overrun
 *   0x01 CTRL     wo  0 start frame, 1 start block read, 2 abort,
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
 *                     5 receive wide without driving DAP2
 *   0x0C LEAD     rw  idle clocks before each frame
 *   0x0D LEVEL    ro  16-bit bytes waiting in the FIFO, low byte first
 *   0x0F SKEW     rw  capture tap: [1:0] DAP1, [3:2] DAP2, in fabric clocks
 *   0x10 DATA     rw  64-bit frame payload, low byte first
 *   0x20 REPLY    ro  32 bits of the last reply, low byte first
 *   0x24 RCRC     ro  the six CRC bits that followed
 *   0x25 WAIT     ro  16-bit busy cycle count, low byte first
 *   0x27 ALIGN    ro  0 DAP2 carried the start bit too (wide mode alignment)
 *   0x40 FIFO     ro  pops one byte; does not auto-increment
 *   0x43 WLEVEL   ro  16-bit bytes waiting in the write FIFO, low byte first
 *   0x48 WFIFO    wo  pushes one byte; does not auto-increment
 */

`default_nettype none

module dap_top #(
    parameter integer FIFO_DEPTH  = 1088,  /* 1 kB block plus headroom */
    /* Half a block: Q_WFETCH stalls on empty, so the host refills mid-block. */
    parameter integer WFIFO_DEPTH = 512
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
    wire       reg_we, reg_re, reg_consume;
    reg        reg_re_d, reg_re_d2, reg_re_d3;
    reg  [7:0] reg_rdata;
    /* Read pipeline: per-group bytes, the control group split in two halves. */
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
    /* Registered flags, updated alongside the counter. */
    reg        fifo_empty = 1'b1;
    reg        fifo_full  = 1'b0;

    reg        fifo_push;
    reg [7:0]  fifo_din;
    reg        fifo_clear;

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

    wire       wfifo_push = reg_we && (reg_addr == 7'h48) && !wfifo_full;

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
                    wfifo_mem[wfifo_wr] <= reg_wdata;
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
                    wfifo_mem[wfifo_wr] <= reg_wdata;
                    wfifo_wr    <= (wfifo_wr == WFIFO_DEPTH - 1) ? 12'd0
                                                                 : wfifo_wr + 1'b1;
                    wfifo_rd    <= (wfifo_rd == WFIFO_DEPTH - 1) ? 12'd0
                                                                 : wfifo_rd + 1'b1;
                end
                default: ;
            endcase
        end
    end

    /*
     * Reply FIFO pops as the byte is shifted out.  The port test is registered;
     * 0x40 does not auto-increment, so a cycle-old answer is the same answer.
     */
    reg        at_port;
    always @(posedge clk) begin
        at_port <= (reg_addr == 7'h40);
    end

    wire       fifo_pop = reg_consume && at_port;

    /*
     * The byte at the read pointer, re-read every cycle so a push into an empty
     * FIFO is seen.  fifo_rd_p1 keeps the increment out of the read address path.
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

    /* dap1 is driven while either engine claims it; only one is ever busy. */
    wire dap1_in;
    wire drive    = tx_busy ? tx_oe : rx_oe;
    wire dap1_out = tx_dap1;

    assign dap1 = drive ? dap1_out : 1'bz;

    /*
     * Input synchronisers plus per-line capture tap.  Tap 0 is two clocks of
     * delay (narrow-mode timing); taps 1..3 sample one clock later each.  The
     * tap mux is registered to keep it off the start-bit hunt path.
     */
    reg [3:0] dap1_sync, dap2_sync;
    reg       dap1_tap,  dap2_tap;
    always @(posedge clk) begin
        dap1_sync <= {dap1_sync[2:0], dap1};
        dap2_sync <= {dap2_sync[2:0], dap2};
        dap1_tap  <= dap1_sync[r_skew1];
        dap2_tap  <= dap2_sync[r_skew2];
    end
    assign dap1_in = dap1_tap;
    wire   dap2_in = dap2_tap;

    assign dap0    = tx_busy ? tx_dap0 : rx_dap0;
    /* DAP2 is driven only while a wide frame is being sent. */
    assign dap2    = (tx_busy && tx_oe2) ? tx_dap2 : 1'bz;

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
        .wide (r_wide), .raw (r_raw | tx_parcel),
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

    reg [3:0] q;
    reg       is_block;
    reg       is_bwrite;
    reg [8:0] parcels_left;
    reg [1:0] store_byte;
    reg [31:0] store_word;

    /* Bytes of the current block-write word taken from the write FIFO. */
    reg [1:0]  wbyte;
    reg        wfetch_wait;
    /* Registered "last parcel" flag, updated alongside the counter. */
    reg        wlast;

    reg start_frame_req, start_block_req, start_bwrite_req;

    always @(posedge clk) begin
        tx_start    <= 1'b0;
        rx_start    <= 1'b0;
        fifo_push   <= 1'b0;
        wfetch_wait <= 1'b0;
        s_busy      <= (q != Q_IDLE);

        if (rst) begin
            q         <= Q_IDLE;
            s_done    <= 1'b0;
            s_overrun <= 1'b0;
            is_bwrite <= 1'b0;
        end else begin
            case (q)
                Q_IDLE: begin
                    if (start_frame_req || start_block_req || start_bwrite_req) begin
                        is_block      <= start_block_req;
                        is_bwrite     <= start_bwrite_req;
                        parcels_left  <= {1'b0, r_parcels} + 9'd1;
                        /* At least one parcel always follows the command. */
                        wlast         <= 1'b0;
                        /* The reply flags are cleared in Q_TX. */
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
                         * Plain frame: r_rbits and a CRC.  Block read: 32-bit
                         * parcels, CRC on the last only.  Block write: a bare
                         * start bit per command and per parcel.
                         */
                        rx_bits       <= is_bwrite ? 7'd0
                                       : is_block  ? 7'd32 : r_rbits;
                        rx_expect_crc <= is_bwrite ? 1'b0
                                       : is_block  ? (parcels_left == 9'd1)
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
                        s_timed_out <= rx_timed_out;
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
                    if (wfetch_go) begin
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
                        wlast         <= (parcels_left == 9'd1);
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
                        /* A timed-out parcel ends the block; no junk in the FIFO. */
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
                        /* Nobody is draining: stop and flag overrun. */
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
        start_bwrite_req <= 1'b0;
        /* Driven only from this block; two drivers get resolved to one by yosys. */
        fifo_clear      <= 1'b0;
        wfifo_clear     <= 1'b0;

        if (rst) begin
            trst      <= 1'b1;         /* released; asserting it resets the target */
            r_no_hunt <= 1'b0;         /* ordinary hunting is the operating mode */
            r_wide    <= 1'b0;         /* narrow until the host has run DAPISC */
            r_raw     <= 1'b0;
            r_rx_wide <= 1'b0;
            r_skew1   <= 2'd0;
            r_skew2   <= 2'd0;
        end else begin
            /*
             * Block-write parcel assembly, here because this block owns DATA.
             * Bit 0 is the start bit, bits 32:1 the word.
             */
            if (wfetch_go) begin
                r_data <= {31'd0, wfifo_head, r_data[32:9], 1'b1};
            end

            if (reg_we) begin
                case (reg_addr)
                    7'h01: begin
                        start_frame_req <= reg_wdata[0];
                        start_block_req <= reg_wdata[1];
                        start_bwrite_req <= reg_wdata[4];
                        if (reg_wdata[3]) fifo_clear <= 1'b1;
                        if (reg_wdata[5]) wfifo_clear <= 1'b1;
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
             * Four-cycle read pipeline, well inside the SPI dummy byte:
             * stage 1 registers each group's byte (control group as two halves),
             * stage 2 picks the control half, stage 3 picks the group.
             */
            q_ctrl_lo <= rd_ctrl_lo;
            q_ctrl_hi <= rd_ctrl_hi;
            q_hi      <= reg_addr[3];
            q_dat    <= rd_dat;
            q_rep    <= rd_rep;
            q_fifo   <= rd_fifo;
            q_grp    <= reg_addr[6:4];
            q_port   <= (reg_addr[6:4] == 3'h4);

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

    /* Read mux, grouped by address range to keep the LUT depth short. */
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
            /* LEVEL: reply FIFO fill, so the host can drain during a block. */
            3'h5: rd_ctrl_hi = fifo_count[7:0];
            default: rd_ctrl_hi = {4'd0, fifo_count[11:8]};   /* 0x0E */
        endcase

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
            /* 0x27 ALIGN.  Decoded here, not in the control group, for timing. */
            default: rd_rep = {7'd0, s_aligned};
        endcase
    end

    /* FIFO port group: reply FIFO data and write FIFO level. */
    reg [7:0] rd_fifo;

    always @(*) begin
        case (reg_addr[3:0])
            /* WLEVEL: bytes waiting in the write FIFO (the host knows the depth). */
            4'h3:    rd_fifo = wfifo_count[7:0];
            4'h4:    rd_fifo = {4'd0, wfifo_count[11:8]};
            default: rd_fifo = fifo_empty ? 8'h00 : fifo_head;
        endcase
    end
endmodule

`default_nettype wire
