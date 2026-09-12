/*
 * How many clock edges actually arrive while the select is low?
 *
 * Everything else checks out: the host's transaction succeeds, the pads are
 * routed to the SPI peripheral, the fabric is clocked and can drive SPI_SO,
 * and the select does fall.  Yet the slave never counts eight rising edges
 * inside one selection, which is the only way it can decode a header.
 *
 * "Never eight" is not a number, so count them.  SPI_SO reads back 1 once at
 * least THRESHOLD rising edges have been seen within a single selection - the
 * counter restarts on every falling select, so this measures one transaction
 * rather than a lifetime total.
 */

`default_nettype none

module freecount (
    input  wire spi_sck,
    input  wire spi_ss,
    output wire spi_so,
    output wire trst
);
    wire clk;
    SB_HFOSC #(.CLKHF_DIV("0b01")) u_osc (
        .CLKHFPU (1'b1), .CLKHFEN (1'b1), .CLKHF (clk)
    );

    reg [2:0] sck_sync;
    reg [2:0] ss_sync;
    always @(posedge clk) begin
        sck_sync <= {sck_sync[1:0], spi_sck};
        ss_sync  <= {ss_sync[1:0],  spi_ss};
    end

    wire sck_rise  = (sck_sync[2:1] == 2'b01);
    wire ss_active = ~ss_sync[1];
    wire ss_start  = (ss_sync[2:1] == 2'b10);

    reg [15:0] rises   = 16'd0;
    reg        reached = 1'b0;

    always @(posedge clk) begin
        if (sck_rise) begin
            rises <= rises + 1'b1;
            if (rises + 1'b1 >= `THRESHOLD) begin
                reached <= 1'b1;     /* sticky, so one read after is enough */
            end
        end
    end

    assign spi_so = reached;
    assign trst   = 1'b1;
endmodule

`default_nettype wire
