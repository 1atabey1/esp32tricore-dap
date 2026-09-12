/*
 * Which ESP32 pin reaches which FPGA pad?
 *
 * MISO was confirmed empirically - a bitstream holding SPI_SO at a known level
 * showed up on GPIO39.  SCK and MOSI never were: their mapping came from a
 * schematic reading that was explicitly uncertain at the time, and the fact
 * that configuration works only proves the *config* logic sees them, which is
 * hardwired rather than routed through the fabric.
 *
 * This echoes one chosen input straight back on SPI_SO.  Driving that ESP32 pin
 * by hand and reading MISO then says whether the assumption holds, with no
 * clock, no protocol and no peripheral involved.
 */

`default_nettype none

module passthru (
    input  wire spi_sck,
    input  wire spi_si,
    output wire spi_so,
    output wire trst
);
    assign spi_so = `ECHO ? spi_si : spi_sck;
    assign trst   = 1'b1;
endmodule

`default_nettype wire
