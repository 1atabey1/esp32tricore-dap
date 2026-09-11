/*
 * Board top for the DAP-only bitstream.
 *
 * Everything above this is testable without a board; this is the part that can
 * only be right or wrong on hardware, so it is kept as small as possible and
 * contains no protocol logic at all.
 *
 * Two things it has to provide that a testbench supplied for free:
 *
 *   - A clock.  The internal oscillator is used rather than the PLL the stock
 *     bitstream runs, because nothing in this design needs a particular
 *     frequency - the DAP bit rate is divided down from it and is a register -
 *     and depending on an external clock source whose routing we recovered by
 *     reverse engineering would be one more thing that can be wrong on the
 *     first power-up.
 *
 *   - A reset.  iCE40 fabric comes out of configuration with flip-flops at
 *     their initial values, so a reset is not strictly required, but holding
 *     one for a while after config means the SPI slave cannot latch a partial
 *     transaction from whatever the ESP32 was doing while the bitstream loaded.
 */

`default_nettype none

module dap_board_top (
    /* Target, through the 22R networks on Port C. */
    output wire dap0,      /* PC02, the clock this master generates */
    inout  wire dap1,      /* PC03, bidirectional data */
    output wire trst,      /* PC04, target reset on this bench */

    /* ESP32, on the FPGA configuration SPI reused as user IO. */
    input  wire spi_sck,
    input  wire spi_si,
    output wire spi_so,
    input  wire spi_ss
);
    /*
     * 24 MHz, not the 48 the oscillator can give.
     *
     * Place and route closes at about 36 MHz, and running a design above its
     * static timing because it happens to work on the bench is how a probe
     * becomes intermittent later.  24 MHz leaves half again in margin.
     *
     * What it costs: the SPI slave synchronises an asynchronous SCK and so
     * needs sysclk >= 4x SCK, which caps the host link at 6 MHz - about
     * 750 kB/s of readback.  Still well above the 453 kB/s the host-driven path
     * manages, and the block read wins on round trips rather than on raw link
     * rate anyway.  Getting to 48 MHz means registering the pad inputs and
     * outputs in SB_IO; the longest paths are pad-to-fabric routes, not logic.
     *
     * CLKHFPU and CLKHFEN are the power-up and enable strobes, tied on.
     */
    wire clk;
    SB_HFOSC #(
        .CLKHF_DIV("0b01")
    ) u_osc (
        .CLKHFPU (1'b1),
        .CLKHFEN (1'b1),
        .CLKHF   (clk)
    );

    /* Hold reset for a while after configuration, then release it and stay. */
    reg [7:0] rst_count = 8'd0;
    reg       rst       = 1'b1;

    always @(posedge clk) begin
        if (rst_count != 8'hFF) begin
            rst_count <= rst_count + 1'b1;
            rst       <= 1'b1;
        end else begin
            rst <= 1'b0;
        end
    end

    /*
     * dap2 is deliberately absent.  The connector has it and the stock image
     * drives it, but this design has no use for it, and an unconstrained iCE40
     * pad stays an input - which is the right thing to do to a target line
     * nothing intends to drive.
     */
    dap_top #(
        .FIFO_DEPTH (1024)          /* exactly one maximum block read */
    ) u_dap (
        .clk     (clk),
        .rst     (rst),
        .spi_sck (spi_sck),
        .spi_si  (spi_si),
        .spi_so  (spi_so),
        .spi_ss  (spi_ss),
        .dap0    (dap0),
        .dap1    (dap1),
        .trst    (trst),
        .dap2    ()
    );
endmodule

`default_nettype wire
