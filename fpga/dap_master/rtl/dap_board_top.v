/*
 * Board top for the DAP-only bitstream: internal oscillator, power-on reset,
 * pin mapping.  No protocol logic.  The reset keeps the SPI slave from latching
 * a partial transaction left over from configuration.
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
     * 48 MHz internal oscillator (CLKHF_DIV 0b00).  Timing closes just under
     * 50 MHz on a pinned placer seed; see the Makefile.
     */
    wire clk;
    SB_HFOSC #(
        .CLKHF_DIV("0b00")
    ) u_osc (
        .CLKHFPU (1'b1),
        .CLKHFEN (1'b1),
        .CLKHF   (clk)
    );

    /* Hold reset for 255 clocks after configuration, then release it. */
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
        .dap2    (dap2)
    );
endmodule

`default_nettype wire
