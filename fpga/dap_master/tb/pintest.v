/*
 * Can the fabric drive the configuration SPI pins as user IO?
 *
 * The DAP image loads - CDONE comes up - but its register file does not answer,
 * and the pin mapping is not the suspect: configuration itself proves the
 * ESP32's SCK, MOSI and CS reach the FPGA's dedicated config pins, which are by
 * definition the package numbers the PCF uses.
 *
 * What is not proven is that those pads can be driven by the *design* once
 * configuration is over.  This drives SPI_SO - package 14, the ESP32's MISO on
 * GPIO39 - to a level chosen at build time, so reading that GPIO from the host
 * answers the question with one bit.
 *
 * Nothing else is driven.  trst is held released so the target is left alone.
 */

`default_nettype none

module pintest (
    output wire spi_so,
    output wire trst
);
    wire clk;
    SB_HFOSC #(
        .CLKHF_DIV("0b01")
    ) u_osc (
        .CLKHFPU (1'b1),
        .CLKHFEN (1'b1),
        .CLKHF   (clk)
    );

    reg [23:0] count = 24'd0;
    always @(posedge clk) begin
        count <= count + 1'b1;
    end

    /* SO_LEVEL is set on the yosys command line, so the same source builds
     * both halves of the experiment. */
    assign spi_so = `SO_LEVEL | (count[23] & 1'b0);
    assign trst   = 1'b1;
endmodule

`default_nettype wire
