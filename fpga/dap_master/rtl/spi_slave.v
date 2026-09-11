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
 * file has half an SCK period to answer the first fetch - two fabric clocks at
 * the fastest legal SCK - so the whole address decode has to fit in one clock,
 * and that decode was the critical path holding the design below the
 * oscillator's 48 MHz.  With it, every fetch including the first has a full
 * byte to complete, the decode is pipelined, and the cost is one byte per
 * transaction: a thousandth of a 1 kB block read.
 *
 * Auto-increment is what makes this DMA-friendly, and that is the point of the
 * whole exercise: a block read becomes one long SPI transaction the ESP32 can
 * DMA straight into a buffer, rather than a round trip per parcel.  Addresses
 * with AUTOINC_STOP set do not increment, so a burst read of the reply FIFO
 * drains it instead of walking off into unmapped registers.
 *
 * Clock domains.  SCK comes from the ESP32 and is asynchronous to the fabric
 * clock, so it is synchronised and edge-detected rather than used as a clock.
 * That imposes a real constraint: the fabric must sample fast enough to see
 * every SCK edge, so
 *
 *     SCK <= sysclk / 4
 *
 * At the 48 MHz internal oscillator that is 12 MHz, comfortably above what the
 * link needs - a DAP bit costs several fabric clocks anyway, so the SPI side is
 * not the bottleneck.  Going faster than that does not degrade gracefully: it
 * drops bits silently, so it is a constraint to respect rather than to test.
 */

`default_nettype none

module spi_slave (
    input  wire        clk,
    input  wire        rst,

    /* Pads.  The ESP32 is the master, so all but so are inputs. */
    input  wire        spi_sck,
    input  wire        spi_si,
    output reg         spi_so,
    input  wire        spi_ss,      /* active low */

    /* Register bus.  we and re are single-cycle pulses. */
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
     * A byte whose first rising edge arrived is a byte that was really sent.
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

    /* Two-flop synchronisers, then edge detection on the synchronised copy. */
    reg [2:0] sck_sync;
    reg [2:0] ss_sync;
    reg [1:0] si_sync;

    always @(posedge clk) begin
        sck_sync <= {sck_sync[1:0], spi_sck};
        ss_sync  <= {ss_sync[1:0],  spi_ss};
        si_sync  <= {si_sync[0],    spi_si};
    end

    wire sck_rise = (sck_sync[2:1] == 2'b01);
    wire sck_fall = (sck_sync[2:1] == 2'b10);
    wire ss_active = ~ss_sync[1];
    wire ss_start  = (ss_sync[2:1] == 2'b10);   /* falling edge: transaction begins */

    assign selected = ss_active;

    reg [7:0] shift_in;
    reg [7:0] shift_out;
    reg [2:0] bit_count;
    reg       have_header;
    reg       is_write;
    /* The dummy byte a read sends before its first data byte, so the register
     * file has a whole byte to answer rather than half an SCK period. */
    reg       first_data;
    /* Whether the byte currently going out is that dummy.  A dummy must not
     * pop a port register - it is not data the master asked for. */
    reg       sending_dummy;

    /*
     * The address must not move in the same cycle as the pulse that uses it.
     * Incrementing alongside reg_we hands the consumer the *next* address with
     * this byte's data, so a burst write lands one register late and the first
     * one is never written at all - which looks like an addressing fault in
     * whatever is behind the bus rather than a timing one here.
     */
    reg       pending_inc;

    always @(posedge clk) begin
        reg_we      <= 1'b0;
        reg_re      <= 1'b0;
        reg_consume <= 1'b0;

        /* Applied a cycle after the pulse, so the pulse carried the address
         * the byte actually belongs to. */
        if (pending_inc) begin
            pending_inc <= 1'b0;
            if (reg_addr < AUTOINC_STOP) begin
                reg_addr <= reg_addr + 1'b1;
            end
        end

        if (rst) begin
            bit_count     <= 3'd0;
            have_header   <= 1'b0;
            is_write      <= 1'b0;
            first_data    <= 1'b1;
            sending_dummy <= 1'b0;
            spi_so        <= 1'b0;
            shift_out     <= 8'd0;
        end else if (ss_start) begin
            /* A fresh transaction: the next byte is the header. */
            bit_count   <= 3'd0;
            have_header <= 1'b0;
            is_write      <= 1'b0;
            first_data    <= 1'b1;
            sending_dummy <= 1'b0;
            shift_out     <= 8'd0;
            spi_so        <= 1'b0;
        end else if (!ss_active) begin
            bit_count     <= 3'd0;
            have_header   <= 1'b0;
            first_data    <= 1'b1;
            sending_dummy <= 1'b0;
            spi_so        <= 1'b0;
        end else begin
            if (sck_rise) begin
                /* Mode 0: the master presents on the falling edge, so the bit
                 * is stable here. */
                shift_in  <= {shift_in[6:0], si_sync[1]};
                bit_count <= bit_count + 1'b1;

                /*
                 * The first bit of an outgoing byte has just been taken, so
                 * that byte is really on its way: account for it and ask for
                 * the next one.  Doing both here rather than at the load is
                 * what stops a burst's final, never-clocked load from eating a
                 * byte - and it gives the fetch seven and a half bit times to
                 * complete instead of half of one.
                 */
                if (bit_count == 3'd0 && have_header && !is_write &&
                    !sending_dummy) begin
                    reg_consume <= 1'b1;
                    if (reg_addr < AUTOINC_STOP) begin
                        reg_addr <= reg_addr + 1'b1;
                    end
                    reg_re <= 1'b1;
                end

                if (bit_count == 3'd7) begin
                    if (!have_header) begin
                        have_header <= 1'b1;
                        is_write    <= shift_in[6];      /* bit 7 of the byte */
                        reg_addr    <= {shift_in[5:0], si_sync[1]};
                        /*
                         * A read needs its first byte ready before the master
                         * clocks it, which is the very next falling edge - so
                         * the fetch is requested here, on the edge that
                         * completes the header.
                         */
                        reg_re      <= ~shift_in[6];
                    end else begin
                        if (is_write) begin
                            reg_wdata <= {shift_in[6:0], si_sync[1]};
                            reg_we    <= 1'b1;
                            /* Walk on next cycle, so this pulse carried the
                             * address this byte belongs to. */
                            pending_inc <= 1'b1;
                        end
                        /*
                         * A read issues nothing here any more.  Its fetches
                         * are driven from the falling edge that starts each
                         * outgoing byte, which is what gives them a whole byte
                         * to complete instead of half an SCK period.
                         */
                    end
                end
            end

            if (sck_fall) begin
                /*
                 * Present the next output bit.  At the end of a byte the
                 * register file has had a whole half period to answer the
                 * fetch, so this reloads from it rather than shifting.
                 */
                if (bit_count == 3'd0 && have_header && !is_write) begin
                    if (first_data) begin
                        /*
                         * The dummy.  The fetch for the real first byte was
                         * issued when the header completed and is still in
                         * flight; sending zeros here is what buys it the time.
                         */
                        spi_so        <= 1'b0;
                        shift_out     <= 8'h00;
                        first_data    <= 1'b0;
                        sending_dummy <= 1'b1;
                    end else begin
                        spi_so        <= reg_rdata[7];
                        shift_out     <= {reg_rdata[6:0], 1'b0};
                        sending_dummy <= 1'b0;
                    end
                end else if (bit_count == 3'd0) begin
                    spi_so    <= reg_rdata[7];
                    shift_out <= {reg_rdata[6:0], 1'b0};
                end else begin
                    spi_so    <= shift_out[7];
                    shift_out <= {shift_out[6:0], 1'b0};
                end
            end
        end
    end
endmodule

`default_nettype wire
