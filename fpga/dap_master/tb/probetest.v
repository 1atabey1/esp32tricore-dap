/*
 * Which stage of the slave actually fires?
 *
 * Known good: bitstream loads, oscillator runs, fabric drives SPI_SO, the
 * select and rising clock edges arrive, and the host's read path returns 0xFF
 * from a pin held high.  The slave still shifts out nothing.
 *
 * So latch the stages one at a time and put the chosen one on SPI_SO, which the
 * host can read as a GPIO after a transaction.  PROBE picks which:
 *
 *   0  a falling clock edge was seen        - the edge detector works both ways
 *   1  eight rising edges were counted      - the bit counter advances
 *   2  the header byte completed            - have_header was set
 *   3  the header decoded to address 0x07   - the address shifted in correctly
 *
 * The first of these that reads back 0 is where it stops.
 */

`default_nettype none

module probetest (
    input  wire spi_sck,
    input  wire spi_si,
    input  wire spi_ss,
    output wire spi_so,
    output wire trst
);
    wire clk;
    SB_HFOSC #(.CLKHF_DIV("0b01")) u_osc (
        .CLKHFPU (1'b1), .CLKHFEN (1'b1), .CLKHF (clk)
    );

    reg [2:0] sck_sync;
    reg [2:0] ss_sync;
    reg [1:0] si_sync;
    always @(posedge clk) begin
        sck_sync <= {sck_sync[1:0], spi_sck};
        ss_sync  <= {ss_sync[1:0],  spi_ss};
        si_sync  <= {si_sync[0],    spi_si};
    end

    wire sck_rise  = (sck_sync[2:1] == 2'b01);
    wire sck_fall  = (sck_sync[2:1] == 2'b10);
    wire ss_active = ~ss_sync[1];
    wire ss_start  = (ss_sync[2:1] == 2'b10);

    reg [7:0] shift_in;
    reg [2:0] bit_count;
    reg       have_header;
    reg [6:0] addr;

    reg seen_fall   = 1'b0;
    reg seen_eight  = 1'b0;
    reg seen_header = 1'b0;
    reg seen_addr   = 1'b0;

    always @(posedge clk) begin
        if (ss_start) begin
            bit_count   <= 3'd0;
            have_header <= 1'b0;
        end else if (ss_active) begin
            if (sck_fall) begin
                seen_fall <= 1'b1;
            end
            if (sck_rise) begin
                shift_in  <= {shift_in[6:0], si_sync[1]};
                bit_count <= bit_count + 1'b1;
                if (bit_count == 3'd7) begin
                    seen_eight <= 1'b1;
                    if (!have_header) begin
                        have_header <= 1'b1;
                        seen_header <= 1'b1;
                        addr        <= {shift_in[5:0], si_sync[1]};
                        if ({shift_in[5:0], si_sync[1]} == 7'h07) begin
                            seen_addr <= 1'b1;
                        end
                    end
                end
            end
        end
    end

    assign spi_so = (`PROBE == 0) ? seen_fall   :
                    (`PROBE == 1) ? seen_eight  :
                    (`PROBE == 2) ? seen_header : seen_addr;
    assign trst = 1'b1;
endmodule

`default_nettype wire
