/*
 * The smallest bitstream that still proves something.
 *
 * The DAP image was refused - CDONE never came up - and the file itself is
 * structurally fine: it round-trips through iceunpack as a valid .device 5k
 * image and its tail matches the stock one's.  So the question is whether this
 * toolchain can produce anything this loader accepts, or whether something in
 * the design specifically is at fault.
 *
 * This drives one pin, uses the internal oscillator, and claims none of the
 * SPI configuration pins - which is the leading suspect, since the DAP image
 * reuses all four of them as user IO.
 *
 * trst is driven high deliberately: that is the released state, the one the
 * target wants.  Driving it low would hold the application under test in reset
 * for as long as this image is loaded.
 */

`default_nettype none

module loadtest (
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

    /* A counter so the oscillator and some fabric survive optimisation. */
    reg [23:0] count = 24'd0;
    always @(posedge clk) begin
        count <= count + 1'b1;
    end

    /* Released, and kept that way.  The counter is referenced so it stays. */
    assign trst = 1'b1 | count[23];
endmodule

`default_nettype wire
