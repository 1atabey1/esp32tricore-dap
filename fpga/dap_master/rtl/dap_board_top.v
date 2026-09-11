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
    inout  wire dap2,      /* PC01, the odd bits in wide mode */
    output wire trst,      /* PC04, target reset on this bench */

    /* ESP32, on the FPGA configuration SPI reused as user IO. */
    input  wire spi_sck,
    input  wire spi_si,
    output wire spi_so,
    input  wire spi_ss
);
    /*
     * 48 MHz, the fastest the oscillator offers.
     *
     * This was 24 for a long time, because place and route closed at about 36
     * and running a design above its static timing because it happens to work
     * on the bench is how a probe becomes intermittent later.  Everything in
     * this design that is measured in bytes per second is a fraction of this
     * number twice over - the DAP clock is sysclk/(2*(div+1)) and the host
     * link is capped at sysclk/4 by the SPI slave's SCK synchroniser - so the
     * doubling is worth more than anything else available.
     *
     * Getting here took the register-read mux split in two behind a dummy
     * byte, the half-period strobes and terminal counts registered, the
     * transmitter shifting instead of indexing, and the reset trimmed to the
     * registers that actually need it.  It closes just under 50 MHz on a
     * pinned placer seed; see the Makefile.
     *
     * CLKHFPU and CLKHFEN are the power-up and enable strobes, tied on.
     */
    wire clk;
    SB_HFOSC #(
        .CLKHF_DIV("0b00")
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
     * Not on a global buffer, though it reaches almost every flip-flop here
     * and its fanout is most of the critical path's routing.  Tried: putting
     * it on one costs the flops their dedicated reset input, so the reset has
     * to be folded back into each one's logic instead, and the design lost
     * about 8 MHz.  Keeping the net short is done by resetting fewer registers
     * rather than by driving the net harder - see the receiver and the
     * transmitter, where anything reloaded at the start of a frame is left out
     * of the reset entirely.
     */

    /*
     * dap2 is here now, and it is bidirectional for the same reason dap1 is:
     * wide mode sends the odd bits of a frame on it and the device answers on
     * it.  Outside a wide frame it is tristated, so a target that is not in
     * wide mode still sees a line nothing is driving.
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
