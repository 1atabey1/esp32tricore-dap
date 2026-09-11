/*
 * The real SPI slave, with the register file replaced by a constant.
 *
 * Established: the bitstream loads, the oscillator runs, the fabric drives
 * SPI_SO, and both the select and the clock reach it.  A register read still
 * returns 0x00 - and 0x00 is not the reset value of the register that was read,
 * so the answer is not coming from the register file at all.
 *
 * This narrows it to one question.  If the host reads 0xA5 here, the slave
 * shifts out correctly and the fault is in the register decode or in dap_top.
 * If it still reads 0x00, the fault is in the slave itself and everything above
 * it is innocent.
 */

`default_nettype none

module slavetest (
    input  wire spi_sck,
    input  wire spi_si,
    output wire spi_so,
    input  wire spi_ss,
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

    /* Held in reset briefly after configuration, as the board top does. */
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

    spi_slave u_spi (
        .clk (clk), .rst (rst),
        .spi_sck (spi_sck), .spi_si (spi_si), .spi_so (spi_so), .spi_ss (spi_ss),
        .reg_addr (), .reg_wdata (), .reg_we (), .reg_re (),
        .reg_rdata (8'hA5),        /* every address answers the same */
        .selected ()
    );

    assign trst = 1'b1;
endmodule

`default_nettype wire
