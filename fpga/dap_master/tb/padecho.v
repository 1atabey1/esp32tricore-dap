/*
 * Echo one package pin onto SPI_SO, to map the link pin by pin.
 *
 * The assumption that the ESP32's SCK, MOSI and CS land on the FPGA's
 * identically named config pins turned out to be wrong: GPIO38, the clock,
 * arrives on package pin 15, which the PCF had labelled SPI_SI.  Configuration
 * still works because the config logic is hardwired to those pads whatever a
 * bitstream calls them - so it could never have caught this.
 *
 * PAD picks which package pin is echoed; the build supplies a matching PCF.
 * Driving one ESP32 pin and reading MISO then identifies that pad exactly.
 */

`default_nettype none

module padecho (
    input  wire pad_in,
    output wire spi_so,
    output wire trst
);
    assign spi_so = pad_in;
    assign trst   = 1'b1;
endmodule

`default_nettype wire
