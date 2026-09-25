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
 * SCK clocks the shift registers directly.  Byte-level events cross to the
 * fabric clock by toggle handshake; the data beside each toggle is stable for
 * eight SCK periods.
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
     * Pulses on the first SCK edge of the byte in reg_rdata, not when it was
     * loaded: the load after a burst's last byte is never clocked out.
     */
    output reg         reg_consume,
    input  wire [7:0]  reg_rdata,

    /* High while a transaction is in progress, for registers that care. */
    output wire        selected
);
    /* Registers at or above this address do not auto-increment. */
    localparam [6:0] AUTOINC_STOP = 7'h40;

    assign selected = ~spi_ss;

    /* ------------------------------------------------------------------ */
    /* SCK domain                                                          */
    /* ------------------------------------------------------------------ */
    /* Reset asynchronously by the select, since SCK is idle between transfers. */

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

            /* First bit of an outgoing byte taken: that byte is consumed. */
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
     * Mode 0: present on the falling edge.  Load tx_byte at a byte boundary,
     * shift otherwise.  first_data and sending_dummy are driven only here.
     */
    always @(negedge spi_sck or posedge spi_ss) begin
        if (spi_ss) begin
            shift_out     <= 8'd0;
            first_data    <= 1'b1;
            sending_dummy <= 1'b0;
        end else if (bit_count == 3'd0) begin
            if (have_header && !is_write && first_data) begin
                /* The dummy, while the first fetch is in flight. */
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
        reg_consume <= 1'b0;

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
                /* Byte went out: consume, step on, fetch the next one. */
                reg_consume <= 1'b1;
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
