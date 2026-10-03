/*
 * SPI slave and register bus for the ESP32.  SPI mode 0, MSB first:
 *
 *     byte 0        bit 7 = write, bits 6:0 = register address
 *     write         bytes 1..n   data, address auto-incrementing
 *     read          byte 1 is a dummy, bytes 2..n+1 are the data
 *
 * The dummy byte gives the register file a full byte time for its first fetch.
 * Addresses at or above AUTOINC_STOP do not increment, so a burst read of a
 * FIFO port drains it.
 *
 * Two ports read the reply FIFO, whose read side runs on SCK (reply_fifo), so
 * a burst costs no fabric clocks per byte:
 *
 *   0x40  one line, as any read: dummy, then FIFO bytes; an empty FIFO reads
 *         as zeros and does not pop.
 *   0x50  two lines (the host's half-duplex dual read): header on SI, eight
 *         dummy clocks with neither side driving SI, then two bits a clock,
 *         SO the odd bit and SI the even one (bits 7,6 first).  The data
 *         opens with a two-byte prefix taken as the header ends,
 *             byte 0  level[7:0]
 *             byte 1  {busy, timed_out, overrun, queued, 1'b0, level[10:8]}
 *         (queued: a block start is held, LEVEL's high-byte bit 4)
 *         and then exactly `level` FIFO bytes; anything clocked after them
 *         reads as zeros and pops nothing, so no byte is ever lost.  A byte
 *         pops after its last clock, so a host that clocks a part of one more
 *         (a dummy clock beyond eight, to sample a slow line later) loses
 *         nothing either.
 *
 * SCK clocks the shift registers directly.  Register-level events cross to
 * the fabric clock by toggle handshake; the data beside each toggle is stable
 * for eight SCK periods.  SO and SI leave from registers in their I/O cells,
 * on SCK's falling edge.
 */

`default_nettype none

module spi_slave #(
    parameter integer AW = 10                   /* reply FIFO address width */
) (
    input  wire        clk,
    input  wire        rst,

    /* Pads.  The ESP32 is the master; SI turns round for 0x50 reads. */
    input  wire        spi_sck,
    inout  wire        spi_si,
    output wire        spi_so,
    input  wire        spi_ss,      /* active low */

    /* Register bus.  we and re are single-cycle pulses in the fabric domain. */
    output reg  [6:0]  reg_addr,
    output reg  [7:0]  reg_wdata,
    output reg         reg_we,
    output reg         reg_re,
    input  wire [7:0]  reg_rdata,

    /* Reply FIFO read side, on SCK (reply_fifo). */
    input  wire [7:0]  fifo_head,
    input  wire        fifo_empty,
    input  wire [AW:0] fifo_level,
    output wire        fifo_pop,
    output wire        fifo_ahead,      /* head: the byte after the pointer's */
    /* {busy, timed_out, overrun, queued}, fabric clock: synchronised here. */
    input  wire [3:0]  status,

    /* High while a transaction is in progress, for registers that care. */
    output wire        selected
);
    /* Registers at or above this address do not auto-increment. */
    localparam [6:0] AUTOINC_STOP = 7'h40;
    localparam [6:0] PORT_FIFO    = 7'h40;
    localparam [6:0] PORT_FIFO2   = 7'h50;

    assign selected = ~spi_ss;

    /* ------------------------------------------------------------------ */
    /* Pads                                                                */
    /* ------------------------------------------------------------------ */

    wire       si_in;
    reg  [7:0] shift_out = 8'd0;
    reg        dual = 1'b0;                     /* 0x50 data phase: SI is ours */

    assign spi_so = shift_out[7];

    /*
     * PIN_OUTPUT_TRISTATE, PIN_INPUT.  SI is driven only while `dual`, which
     * the select clears the moment it rises: the host has it back before it
     * can drive it again.
     */
    SB_IO #(.PIN_TYPE(6'b1010_01)) u_si_io (
        .PACKAGE_PIN   (spi_si),
        .OUTPUT_ENABLE (dual),
        .D_OUT_0       (shift_out[6]),
        .D_IN_0        (si_in)
    );

    /* ------------------------------------------------------------------ */
    /* SCK domain                                                          */
    /* ------------------------------------------------------------------ */
    /* Reset asynchronously by the select, since SCK is idle between transfers. */

    reg [7:0] shift_in = 8'd0;
    reg [2:0] bit_count = 3'd0;
    reg       have_header = 1'b0;
    reg       is_write = 1'b0;
    reg       in_dummy = 1'b0;      /* a read's dummy byte is on the wire */
    reg       port1 = 1'b0;         /* a read of 0x40 */
    reg       port2 = 1'b0;         /* a read of 0x50 */

    reg [6:0] hdr_addr = 7'd0;
    reg [7:0] wr_byte = 8'd0;

    /* Toggles, one per byte-level event, for the fabric to notice. */
    reg       hdr_tog = 1'b0;
    reg       wr_tog  = 1'b0;
    reg       rd_tog  = 1'b0;

    /* The byte the fabric wants sent next. */
    reg [7:0] tx_byte = 8'd0;

    /* 0x50: clocks into the dual data phase, bytes sent, prefix and budget. */
    reg [1:0]  dclk = 2'd0;
    reg [1:0]  dbyte = 2'd0;        /* 0, 1: the prefix; 2: FIFO bytes */
    reg [AW:0] budget = 0;
    reg [AW:0] snap_level = 0;
    reg [3:0]  snap_status = 4'd0;
    reg [3:0]  st_s1 = 4'd0, st_s2 = 4'd0;

    /*
     * The next byte is prepared on the rising edge before the falling edge
     * that loads it, so the falling edge has two LUTs to do (timing): `load`
     * (this falling edge starts a byte), `use_tx` (it is a register byte,
     * taken from tx_byte there and then, which keeps the register path's
     * whole crossing budget), and `prep` (any other byte: the dummy, the
     * prefix, a FIFO byte).  sent_valid: prep holds a real FIFO byte, so the
     * 0x40 pop is decided on what was sent, not on the FIFO a clock later.
     */
    reg        load = 1'b0;
    reg        use_tx = 1'b0;
    reg  [7:0] prep = 8'd0;
    /*
     * And what the rising edge after that load does, decided with it, so no
     * falling-edge flop reaches a rising-edge enable in half a clock: `take`,
     * a register byte is being sent (step the fabric on); `pop`, a FIFO byte
     * is (0x40: the FIFO had one when it was prepared; 0x50: within budget).
     */
    reg        take = 1'b0;
    reg        pop  = 1'b0;
    /* 0x50: the byte on the wire is a FIFO byte (it pops after its last
     * clock, and the FIFO's head is meanwhile the one after it). */
    reg        cur_fifo = 1'b0;

    wire [7:0] shift_in_full = {shift_in[6:0], si_in};

    /* The last clock of a dual-phase byte. */
    wire dual_last = dual && dclk == 2'd3;
    /* The dummy's last clock, on a 0x50 read: SI is ours from the next edge. */
    wire dual_go   = bit_count == 3'd7 && in_dummy && port2;
    wire dual_next = dual || dual_go;

    assign fifo_pop   = pop;
    assign fifo_ahead = cur_fifo;

    /* What the byte after this one is, as of this byte's last clock. */
    wire [1:0] dbyte_next  = (dbyte == 2'd2) ? 2'd2 : dbyte + 1'b1;
    wire       budget_left = pop ? (budget != 1) : (budget != 0);

    wire [7:0]  fifo_byte = fifo_empty ? 8'h00 : fifo_head;
    /* The level as the prefix carries it, eleven bits (AW is at most 10). */
    wire [10:0] lvl11     = snap_level;

    always @(posedge spi_sck) begin
        st_s1 <= status;
        st_s2 <= st_s1;
    end

    always @(posedge spi_sck or posedge spi_ss) begin
        if (spi_ss) begin
            bit_count     <= 3'd0;
            have_header   <= 1'b0;
            is_write      <= 1'b0;
            port1         <= 1'b0;
            port2         <= 1'b0;
            in_dummy      <= 1'b0;
            dual          <= 1'b0;
            dclk          <= 2'd0;
            dbyte         <= 2'd0;
            cur_fifo      <= 1'b0;
            load          <= 1'b0;
            use_tx        <= 1'b0;
            take          <= 1'b0;
            pop           <= 1'b0;
        end else begin
            shift_in  <= shift_in_full;
            bit_count <= bit_count + 1'b1;

            /* First bit of an outgoing register byte taken: consumed. */
            if (take) begin
                rd_tog <= ~rd_tog;
            end

            if (bit_count == 3'd7) begin
                /* A read's header is followed by the dummy, then data. */
                in_dummy <= !have_header && !shift_in_full[7];
                if (!have_header) begin
                    have_header <= 1'b1;
                    is_write    <= shift_in_full[7];
                    hdr_addr    <= shift_in_full[6:0];
                    port1       <= !shift_in_full[7] && shift_in_full[6:0] == PORT_FIFO;
                    port2       <= !shift_in_full[7] && shift_in_full[6:0] == PORT_FIFO2;
                    hdr_tog     <= ~hdr_tog;
                    /* The header has clocked the FIFO's view up to date. */
                    snap_level  <= fifo_level;
                    budget      <= fifo_level;
                    snap_status <= st_s2;
                end else if (is_write) begin
                    wr_byte <= shift_in_full;
                    wr_tog  <= ~wr_tog;
                end
            end
            if (dual_go) begin
                dual <= 1'b1;
            end

            if (dual) begin
                dclk <= dclk + 1'b1;
            end
            /* A byte's last clock: it is out (and popped if a FIFO byte). */
            if (dual_last) begin
                dbyte    <= dbyte_next;
                cur_fifo <= dbyte_next == 2'd2 && budget_left;
            end
            if (pop) begin
                budget <= budget - 1'b1;
            end

            /* The byte the next falling edge loads, if it loads one. */
            load   <= dual_next ? (!dual || dclk == 2'd3) : (bit_count == 3'd7);
            use_tx <= bit_count == 3'd7 && have_header && !is_write && !port1 && !dual_next;
            if (!have_header) begin
                prep <= 8'h00;                          /* the dummy */
            end else if (!dual) begin
                /* Entering the dual phase: the prefix's first byte. */
                prep <= dual_next ? lvl11[7:0] : fifo_byte;
            end else begin
                /* The byte after this one (prepared on this one's last
                 * clock; the head is already the next FIFO byte). */
                case (dbyte_next)
                    2'd1:    prep <= {snap_status, 1'b0, lvl11[10:8]};
                    default: prep <= budget_left ? fifo_head : 8'h00;
                endcase
            end
            /* The header's own byte boundary loads the dummy: no take. */
            take <= bit_count == 3'd7 && have_header && !is_write && !port1 && !dual_next;
            pop  <= dual ? (dclk == 2'd2 && cur_fifo)
                         : (bit_count == 3'd7 && have_header && port1 && !fifo_empty);
        end
    end

    /*
     * Mode 0: present on the falling edge.  Load the prepared byte at a byte
     * boundary (the dummy, while a read's first fetch is in flight, is a
     * prepared zero), shift otherwise; two bits a clock in the dual phase.
     */
    always @(negedge spi_sck or posedge spi_ss) begin
        if (spi_ss) begin
            shift_out <= 8'd0;
        end else if (load) begin
            shift_out <= use_tx ? tx_byte : prep;
        end else if (dual) begin
            shift_out <= {shift_out[5:0], 2'b00};
        end else begin
            shift_out <= {shift_out[6:0], 1'b0};
        end
    end

    /* ------------------------------------------------------------------ */
    /* Fabric domain                                                       */
    /* ------------------------------------------------------------------ */
    /* Toggle synchronisers and edge detectors. */
    reg [2:0] hdr_sync = 3'd0, wr_sync = 3'd0, rd_sync = 3'd0;

    always @(posedge clk) begin
        hdr_sync <= {hdr_sync[1:0], hdr_tog};
        wr_sync  <= {wr_sync[1:0],  wr_tog};
        rd_sync  <= {rd_sync[1:0],  rd_tog};
    end

    wire hdr_event = hdr_sync[2] ^ hdr_sync[1];
    wire wr_event  = wr_sync[2]  ^ wr_sync[1];
    wire rd_event  = rd_sync[2]  ^ rd_sync[1];

    /* The address counter lives here; only the header address crosses. */
    reg pending_inc = 1'b0;

    always @(posedge clk) begin
        reg_we      <= 1'b0;
        reg_re      <= 1'b0;

        if (rst) begin
            reg_addr    <= 7'd0;
            pending_inc <= 1'b0;
        end else begin
            if (hdr_event) begin
                reg_addr <= hdr_addr;
                /* A read's first fetch starts while the dummy is on the wire. */
                reg_re   <= ~is_write;
            end

            /* Increment the cycle after reg_we, so the write sees its own address. */
            if (pending_inc) begin
                pending_inc <= 1'b0;
                if (reg_addr < AUTOINC_STOP) begin
                    reg_addr <= reg_addr + 1'b1;
                end
            end

            if (wr_event) begin
                reg_wdata   <= wr_byte;
                reg_we      <= 1'b1;
                pending_inc <= 1'b1;
            end

            if (rd_event) begin
                /* Byte went out: step on, fetch the next one. */
                if (reg_addr < AUTOINC_STOP) begin
                    reg_addr <= reg_addr + 1'b1;
                end
                reg_re <= 1'b1;
            end
        end
    end

    /* Latest register-file answer, picked up at the next byte boundary. */
    always @(posedge clk) begin
        if (rst) begin
            tx_byte <= 8'd0;
        end else begin
            tx_byte <= reg_rdata;
        end
    end
endmodule

`default_nettype wire
