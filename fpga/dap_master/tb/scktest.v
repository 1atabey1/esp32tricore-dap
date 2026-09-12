/*
 * Does SCK - and does the select - actually reach the fabric?
 *
 * Established so far: the bitstream loads, the oscillator runs, and the fabric
 * can drive SPI_SO as user IO.  What is not established is whether the ESP32's
 * SPI peripheral is clocking the pads at all after the bitstream loader took
 * them back as plain GPIO and this code re-pointed them.
 *
 * So latch it.  SPI_SO reads back 0 until the fabric has seen both a falling
 * select and a clock edge, and 1 forever after.  One read before a transaction
 * and one after turns "the register file does not answer" into either "nothing
 * arrives" or "it arrives and my slave is wrong", which are different bugs.
 */

`default_nettype none

module scktest (
    input  wire spi_sck,
    input  wire spi_ss,
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

    /* Synchronise both, the same way the real slave does, so this tests the
     * path the real slave actually uses rather than a different one. */
    reg [2:0] sck_sync;
    reg [2:0] ss_sync;
    always @(posedge clk) begin
        sck_sync <= {sck_sync[1:0], spi_sck};
        ss_sync  <= {ss_sync[1:0],  spi_ss};
    end

    reg seen_sck = 1'b0;
    reg seen_ss  = 1'b0;

    always @(posedge clk) begin
        if (sck_sync[2:1] == 2'b01) seen_sck <= 1'b1;   /* a rising clock edge */
        if (ss_sync[2:1]  == 2'b10) seen_ss  <= 1'b1;   /* the select falling */
    end

    /* Both, so a 1 means the whole selection-and-clock story arrived. */
    assign spi_so = seen_sck & seen_ss;
    assign trst   = 1'b1;
endmodule

`default_nettype wire
