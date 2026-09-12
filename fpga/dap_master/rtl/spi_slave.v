/*
 * SPI slave and register bus, for the ESP32 to drive the DAP master.
 *
 * Transaction shape, SPI mode 0, MSB first - which is what the ESP32 master
 * does by default, so nothing unusual is asked of the host:
 *
 *     byte 0        bit 7 = write, bits 6:0 = register address
 *     write         bytes 1..n   data, address auto-incrementing
 *     read          byte 1 is a dummy, bytes 2..n+1 are the data
 *
 * The dummy byte on reads is what lets this run fast.  Without it the register
 * file has half an SCK period to answer the first fetch, so the whole address
 * decode has to fit in one clock - and that decode was the critical path
 * holding the design below the oscillator's 48 MHz.  With it, every fetch has
 * a full byte to complete and the cost is one byte per transaction: a
 * thousandth of a 1 kB block read.
 *
 * Auto-increment is what makes this DMA-friendly, and that is the point of the
 * whole exercise: a block read becomes one long SPI transaction the ESP32 can
 * DMA straight into a buffer, rather than a round trip per parcel.  Addresses
 * at or above AUTOINC_STOP do not increment, so a burst read of the reply FIFO
 * drains it instead of walking off into unmapped registers.
 *
 * SCK IS A CLOCK HERE, NOT A SIGNAL TO WATCH.
 *
 * This used to synchronise SCK into the fabric clock and hunt for its edges,
 * which is simple and costs the whole point of the link: seeing every edge
 * needs sysclk >= 4x SCK, so a 48 MHz fabric capped the host at 12 MHz, and
 * past that it did not degrade gracefully - at 16 MHz the register probe wrote
 * 0x5A and read 0x2D, which is 0x5A shifted right by one, a dropped edge.
 *
 * That cap was the system's bottleneck by a wide margin.  With the DAP wire at
 * 24 MHz a 1 kB block spent 0.35 ms on the wire and 0.72 ms being shifted out
 * over SPI, so the wire could be doubled again - wide mode included - for
 * almost nothing.  Clocking the shift register from SCK removes the ratio
 * rule entirely; what is left is ordinary I/O timing.
 *
 * Crossing back to the fabric is by toggle handshake, and it is comfortable
 * rather than clever: every event this raises - a header, a data byte written,
 * a byte shifted out - happens once per *byte*, so the fabric has eight SCK
 * periods to notice one and read the data beside it, while the synchronisers
 * cost two or three fabric clocks.  The multi-bit fields are stable for that
 * whole window by construction.
 */

`default_nettype none

module spi_slave (
    input  wire        clk,
    input  wire        rst,

    /* Pads.  The ESP32 is the master, so all but so are inputs. */
    input  wire        spi_sck,
    input  wire        spi_si,
    output wire        spi_so,
    input  wire        spi_ss,      /* active low */

    /* Register bus.  we and re are single-cycle pulses in the fabric domain. */
    output reg  [6:0]  reg_addr,
    output reg  [7:0]  reg_wdata,
    output reg         reg_we,
    output reg         reg_re,
    /*
     * Pulses once the master has committed to clocking the byte in reg_rdata -
     * on its first rising edge, not when it was loaded.
     *
     * Every byte is loaded on the falling edge that precedes its first bit,
     * which means the falling edge after the *last* bit of a burst loads one
     * more byte that the master then never clocks, because it raises the
     * select instead.  Popping a port register at load time therefore throws
     * that byte away: invisible when a block is drained in one transaction,
     * and exactly the three bytes missing from a 1 kB block drained in four.
     */
    output reg         reg_consume,
    input  wire [7:0]  reg_rdata,

    /* High while a transaction is in progress, for registers that care. */
    output wire        selected
);
    /*
     * Registers at or above this address do not auto-increment.  One address
     * that behaves like a port rather than a location is all the FIFO needs,
     * and keeping the rule this simple means the host can reason about it.
     */
    localparam [6:0] AUTOINC_STOP = 7'h40;

    assign selected = ~spi_ss;

    /* ------------------------------------------------------------------ */
    /* SCK domain                                                          */
    /* ------------------------------------------------------------------ */
    /*
     * Reset by the select rather than by the fabric reset: there is no SCK
     * while the slave is idle, so a synchronous reset here would never be
     * applied.  The select is guaranteed quiet around SCK edges - the master
     * asserts it well before the first one - which is what makes it usable
     * this way.
     */

    reg [7:0] shift_in = 8'd0;
    reg [2:0] bit_count = 3'd0;
    reg       have_header = 1'b0;
    reg       is_write = 1'b0;
    reg       first_data = 1'b1;    /* the dummy byte a read sends first */
    reg       sending_dummy = 1'b0;

    reg [6:0] hdr_addr = 7'd0;
    reg [7:0] wr_byte = 8'd0;

    /* Toggles, one per byte-level event, for the fabric to notice. */
    reg       hdr_tog = 1'b0;
    reg       wr_tog  = 1'b0;
    reg       rd_tog  = 1'b0;

    /* The byte the fabric wants sent next, and the one being shifted out. */
    reg [7:0] tx_byte = 8'd0;
    reg [7:0] shift_out = 8'd0;

    wire [7:0] shift_in_full = {shift_in[6:0], spi_si};

    always @(posedge spi_sck or posedge spi_ss) begin
        if (spi_ss) begin
            bit_count     <= 3'd0;
            have_header   <= 1'b0;
            is_write      <= 1'b0;
        end else begin
            shift_in  <= shift_in_full;
            bit_count <= bit_count + 1'b1;

            /*
             * The first bit of an outgoing byte has just been taken, so that
             * byte is really on its way: account for it here rather than when
             * it was loaded, which is what stops a burst's final,
             * never-clocked load from eating a byte.
             */
            if (bit_count == 3'd0 && have_header && !is_write && !sending_dummy) begin
                rd_tog <= ~rd_tog;
            end

            if (bit_count == 3'd7) begin
                if (!have_header) begin
                    have_header <= 1'b1;
                    is_write    <= shift_in_full[7];
                    hdr_addr    <= shift_in_full[6:0];
                    hdr_tog     <= ~hdr_tog;
                end else if (is_write) begin
                    wr_byte <= shift_in_full;
                    wr_tog  <= ~wr_tog;
                end
            end
        end
    end

    /*
     * Mode 0: the master samples on the rising edge, so the slave presents on
     * the falling one.  At a byte boundary that means loading whatever the
     * fabric has put in tx_byte; in between it is a plain shift.
     */
    /* first_data and sending_dummy belong to this block alone.  They were
     * cleared here and reset in the posedge block, which is two drivers on one
     * register: X in simulation and whichever the synthesiser picks on
     * silicon. */
    always @(negedge spi_sck or posedge spi_ss) begin
        if (spi_ss) begin
            shift_out     <= 8'd0;
            first_data    <= 1'b1;
            sending_dummy <= 1'b0;
        end else if (bit_count == 3'd0) begin
            if (have_header && !is_write && first_data) begin
                /* The dummy.  The fetch for the real first byte is still in
                 * flight; sending zeros here is what buys it the time. */
                shift_out     <= 8'h00;
                first_data    <= 1'b0;
                sending_dummy <= 1'b1;
            end else begin
                shift_out     <= tx_byte;
                sending_dummy <= 1'b0;
            end
        end else begin
            shift_out <= {shift_out[6:0], 1'b0};
        end
    end

    assign spi_so = shift_out[7];

    /* ------------------------------------------------------------------ */
    /* Fabric domain                                                       */
    /* ------------------------------------------------------------------ */
    /*
     * Three toggles, synchronised and edge-detected.  The data beside each one
     * - hdr_addr, wr_byte - was written on the same SCK edge that flipped it
     * and does not change again for eight more SCK periods, so by the time the
     * edge is seen two or three fabric clocks later it is long settled.
     */
    reg [2:0] hdr_sync = 3'd0, wr_sync = 3'd0, rd_sync = 3'd0;

    always @(posedge clk) begin
        hdr_sync <= {hdr_sync[1:0], hdr_tog};
        wr_sync  <= {wr_sync[1:0],  wr_tog};
        rd_sync  <= {rd_sync[1:0],  rd_tog};
    end

    wire hdr_event = hdr_sync[2] ^ hdr_sync[1];
    wire wr_event  = wr_sync[2]  ^ wr_sync[1];
    wire rd_event  = rd_sync[2]  ^ rd_sync[1];

    /*
     * The address lives here, not in the SCK domain.
     *
     * Only the header's address crosses; every step after it is this side's
     * own counter.  That keeps a multi-bit value that changes per byte out of
     * the crossing entirely - the alternative is exporting a counter that
     * moves while the fabric is decoding it.
     */
    reg pending_inc = 1'b0;

    always @(posedge clk) begin
        reg_we      <= 1'b0;
        reg_re      <= 1'b0;
        reg_consume <= 1'b0;

        if (rst) begin
            reg_addr    <= 7'd0;
            pending_inc <= 1'b0;
        end else begin
            if (hdr_event) begin
                reg_addr <= hdr_addr;
                /*
                 * A read's first fetch starts now, while the dummy byte is on
                 * the wire.  is_write is stable in the SCK domain by this
                 * point for the same reason hdr_addr is.
                 */
                reg_re   <= ~is_write;
            end

            /*
             * The address must not move in the same cycle as the pulse that
             * uses it.  Incrementing alongside reg_we hands the consumer the
             * *next* address with this byte's data, so a burst write lands one
             * register late and the first one is never written at all - which
             * looks like an addressing fault in whatever is behind the bus
             * rather than a timing one here.
             */
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
                /* The byte just went out: account for it, step on, and ask for
                 * the next one.  Seven and a half bit times to answer. */
                reg_consume <= 1'b1;
                if (reg_addr < AUTOINC_STOP) begin
                    reg_addr <= reg_addr + 1'b1;
                end
                reg_re <= 1'b1;
            end
        end
    end

    /* Whatever the register file last answered, waiting for the SCK domain to
     * pick it up at the next byte boundary. */
    always @(posedge clk) begin
        if (rst) begin
            tx_byte <= 8'd0;
        end else begin
            tx_byte <= reg_rdata;
        end
    end
endmodule

`default_nettype wire
