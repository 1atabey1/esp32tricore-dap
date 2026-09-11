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
 *   0x0B FLAGS    rw  0 TRST asserted
 *   0x10 DATA     rw  64-bit frame payload, low byte first
 *   0x20 REPLY    ro  32 bits of the last reply, low byte first
 *   0x24 RCRC     ro  the six CRC bits that followed
 *   0x25 WAIT     ro  16-bit busy cycle count, low byte first
 *   0x40 FIFO     ro  pops one byte; does not auto-increment
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
    output wire dap2
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
    reg [15:0] r_maxwait = 16'd256;
    reg [7:0]  r_parcels;
    reg [63:0] r_data;

    reg [31:0] s_reply;
    reg [5:0]  s_crc;
    reg [15:0] s_wait;
    reg        s_done, s_timed_out, s_idle_high, s_crc_ok, s_overrun;

    /* ------------------------------------------------------------------ */
    /* SPI                                                                 */
    /* ------------------------------------------------------------------ */

    wire [6:0] reg_addr;
    wire [7:0] reg_wdata;
    wire       reg_we, reg_re;
    reg  [7:0] reg_rdata;

    spi_slave u_spi (
        .clk (clk), .rst (rst),
        .spi_sck (spi_sck), .spi_si (spi_si), .spi_so (spi_so), .spi_ss (spi_ss),
        .reg_addr (reg_addr), .reg_wdata (reg_wdata),
        .reg_we (reg_we), .reg_re (reg_re), .reg_rdata (reg_rdata),
        .selected ()
    );

    /* ------------------------------------------------------------------ */
    /* Reply FIFO                                                          */
    /* ------------------------------------------------------------------ */

    reg [7:0]  fifo_mem [0:FIFO_DEPTH-1];
    reg [11:0] fifo_wr, fifo_rd;
    reg [11:0] fifo_count;
    wire       fifo_empty = (fifo_count == 12'd0);
    wire       fifo_full  = (fifo_count >= FIFO_DEPTH[11:0]);

    reg        fifo_push;
    reg [7:0]  fifo_din;
    reg        fifo_pop;
    reg        fifo_clear;

    always @(posedge clk) begin
        if (rst || fifo_clear) begin
            fifo_wr    <= 12'd0;
            fifo_rd    <= 12'd0;
            fifo_count <= 12'd0;
        end else begin
            if (fifo_push && !fifo_full) begin
                fifo_mem[fifo_wr] <= fifo_din;
                fifo_wr <= (fifo_wr == FIFO_DEPTH-1) ? 12'd0 : fifo_wr + 1'b1;
            end
            if (fifo_pop && !fifo_empty) begin
                fifo_rd <= (fifo_rd == FIFO_DEPTH-1) ? 12'd0 : fifo_rd + 1'b1;
            end
            case ({fifo_push && !fifo_full, fifo_pop && !fifo_empty})
                2'b10:   fifo_count <= fifo_count + 1'b1;
                2'b01:   fifo_count <= fifo_count - 1'b1;
                default: fifo_count <= fifo_count;
            endcase
        end
    end

    /* ------------------------------------------------------------------ */
    /* Frame transmit and receive                                          */
    /* ------------------------------------------------------------------ */

    reg         tx_start, rx_start;
    wire        tx_busy, tx_done, tx_dap0, tx_dap1, tx_oe;
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

    assign dap1    = drive ? dap1_out : 1'bz;
    assign dap1_in = dap1;
    assign dap0    = tx_busy ? tx_dap0 : rx_dap0;
    assign dap2    = 1'bz;              /* not driven by this design */

    dap_frame_tx #(.DIV_WIDTH(8)) u_tx (
        .clk (clk), .rst (rst), .div (r_div),
        .start (tx_start), .cmd (r_cmd), .len (r_len),
        .data_bits (r_dbits), .data (r_data[62:0]),
        .busy (tx_busy), .done (tx_done),
        .dap0 (tx_dap0), .dap1 (tx_dap1), .dat_oe (tx_oe)
    );

    dap_frame_rx #(.DIV_WIDTH(8)) u_rx (
        .clk (clk), .rst (rst), .div (r_div),
        .start (rx_start), .reply_bits (rx_bits),
        .max_wait (r_maxwait), .trail_clocks (r_trail),
        .expect_crc (rx_expect_crc),
        .busy (rx_busy), .done (rx_done),
        .wait_cycles (rx_wait), .timed_out (rx_timed_out),
        .idle_high (rx_idle_high), .crc_ok (rx_crc_ok),
        .payload (rx_payload), .crc (rx_crc),
        .dap0 (rx_dap0), .dap1_in (dap1_in), .dat_oe (rx_oe)
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
        fifo_clear <= 1'b0;

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
                        s_done        <= 1'b0;
                        s_timed_out   <= 1'b0;
                        s_idle_high   <= 1'b0;
                        s_crc_ok      <= 1'b0;
                        tx_start      <= 1'b1;
                        q             <= Q_TX;
                    end
                end

                Q_TX: begin
                    if (tx_done) begin
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
        fifo_pop        <= 1'b0;

        if (rst) begin
            trst <= 1'b1;              /* released; asserting it resets the target */
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
                    7'h0B: trst      <= ~reg_wdata[0];   /* 1 = assert = drive low */
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

            if (reg_re) begin
                case (reg_addr)
                    7'h00: reg_rdata <= {s_overrun, fifo_full, fifo_empty,
                                         s_crc_ok, s_idle_high, s_timed_out,
                                         s_done, (q != Q_IDLE)};
                    7'h02: reg_rdata <= r_div;
                    7'h03: reg_rdata <= {3'd0, r_cmd};
                    7'h04: reg_rdata <= {2'd0, r_len};
                    7'h05: reg_rdata <= {2'd0, r_dbits};
                    7'h06: reg_rdata <= {1'b0, r_rbits};
                    7'h07: reg_rdata <= r_trail;
                    7'h08: reg_rdata <= r_maxwait[7:0];
                    7'h09: reg_rdata <= r_maxwait[15:8];
                    7'h0A: reg_rdata <= r_parcels;
                    7'h20: reg_rdata <= s_reply[7:0];
                    7'h21: reg_rdata <= s_reply[15:8];
                    7'h22: reg_rdata <= s_reply[23:16];
                    7'h23: reg_rdata <= s_reply[31:24];
                    7'h24: reg_rdata <= {2'd0, s_crc};
                    7'h25: reg_rdata <= s_wait[7:0];
                    7'h26: reg_rdata <= s_wait[15:8];
                    7'h40: begin
                        /* The port address: one byte out of the FIFO, and the
                         * address does not move, so a burst drains it. */
                        reg_rdata <= fifo_empty ? 8'h00 : fifo_mem[fifo_rd];
                        fifo_pop  <= ~fifo_empty;
                    end
                    default: reg_rdata <= 8'h00;
                endcase
            end
        end
    end
endmodule

`default_nettype wire
