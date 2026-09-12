/*
 * Does the internal oscillator actually run?
 *
 * The earlier pin test proved the fabric can drive SPI_SO as user IO, but it
 * drove it from a constant - so it said nothing about whether anything is
 * clocked.  If SB_HFOSC is not running, every sequential element in the DAP
 * design is frozen, the SPI slave never shifts, and the symptom is exactly what
 * is being seen: a bitstream that loads and a register file that never answers.
 *
 * This drives SPI_SO from a counter bit slow enough to sample over HTTP.  At
 * 24 MHz bit 21 toggles about every 90 ms, so a handful of reads a second apart
 * must see both levels if the clock is alive, and will see a stuck level if it
 * is not.
 */

`default_nettype none

module osctest (
    output wire spi_so,
    output wire trst
);
    wire clk;
    SB_HFOSC #(
        .CLKHF_DIV("0b01")      /* the same 24 MHz the DAP design asks for */
    ) u_osc (
        .CLKHFPU (1'b1),
        .CLKHFEN (1'b1),
        .CLKHF   (clk)
    );

    reg [21:0] count = 22'd0;
    always @(posedge clk) begin
        count <= count + 1'b1;
    end

    assign spi_so = count[21];
    assign trst   = 1'b1;       /* target reset left released */
endmodule

`default_nettype wire
