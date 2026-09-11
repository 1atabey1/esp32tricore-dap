/*
 * SPI slave and register bus, for the ESP32 to drive the DAP master.
 *
 * Transaction shape, SPI mode 0, MSB first - which is what the ESP32 master
 * does by default, so nothing unusual is asked of the host:
 *
 *     byte 0        bit 7 = write, bits 6:0 = register address
 *     bytes 1..n    data, address auto-incrementing
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

    /*
     * The address must not move in the same cycle as the pulse that uses it.
     * Incrementing alongside reg_we hands the consumer the *next* address with
     * this byte's data, so a burst write lands one register late and the first
     * one is never written at all - which looks like an addressing fault in
     * whatever is behind the bus rather than a timing one here.
     */
    reg       pending_inc;

    always @(posedge clk) begin
        reg_we <= 1'b0;
        reg_re <= 1'b0;

        /* Applied a cycle after the pulse, so the pulse carried the address
         * the byte actually belongs to. */
        if (pending_inc) begin
            pending_inc <= 1'b0;
            if (reg_addr < AUTOINC_STOP) begin
                reg_addr <= reg_addr + 1'b1;
            end
        end

        if (rst) begin
            bit_count   <= 3'd0;
            have_header <= 1'b0;
            is_write    <= 1'b0;
            spi_so      <= 1'b0;
            shift_out   <= 8'd0;
        end else if (ss_start) begin
            /* A fresh transaction: the next byte is the header. */
            bit_count   <= 3'd0;
            have_header <= 1'b0;
            is_write    <= 1'b0;
            shift_out   <= 8'd0;
            spi_so      <= 1'b0;
        end else if (!ss_active) begin
            bit_count   <= 3'd0;
            have_header <= 1'b0;
            spi_so      <= 1'b0;
        end else begin
            if (sck_rise) begin
                /* Mode 0: the master presents on the falling edge, so the bit
                 * is stable here. */
                shift_in  <= {shift_in[6:0], si_sync[1]};
                bit_count <= bit_count + 1'b1;

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
                        end else begin
                            /*
                             * A read has already moved on: the fetch for byte
                             * n+1 has to be issued now, so the address steps
                             * first and the pulse goes with it.
                             */
                            if (reg_addr < AUTOINC_STOP) begin
                                reg_addr <= reg_addr + 1'b1;
                            end
                            reg_re <= 1'b1;
                        end
                    end
                end
            end

            if (sck_fall) begin
                /*
                 * Present the next output bit.  At the end of a byte the
                 * register file has had a whole half period to answer the
                 * fetch, so this reloads from it rather than shifting.
                 */
                if (bit_count == 3'd0) begin
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
