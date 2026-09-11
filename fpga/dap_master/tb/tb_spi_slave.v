/*
 * The SPI slave against a driven master, with a small register file behind it.
 *
 * What is covered:
 *
 *   write burst     a header plus three bytes lands in three consecutive
 *                   registers, so auto-increment works
 *   read burst      the same three come back in one transaction
 *   port address    an address at or above AUTOINC_STOP does not advance, so
 *                   a burst read of it drains a FIFO rather than walking off
 *                   into whatever is next
 *   back to back    a second transaction after deselect starts clean
 *
 * The last one matters more than it looks: a slave that does not reset its bit
 * counter on select produces a stream that is right once and wrong afterwards,
 * which is the sort of fault that looks like noise on a scope.
 */

`timescale 1ns / 1ps
`default_nettype none

module tb_spi_slave;

    reg clk = 1'b0;
    always #5 clk = ~clk;          /* 100 MHz fabric */

    reg rst = 1'b1;

    reg  sck = 1'b0;
    reg  si  = 1'b0;
    reg  ss  = 1'b1;
    wire so;

    wire [6:0] reg_addr;
    wire [7:0] reg_wdata;
    wire       reg_we, reg_re;
    reg  [7:0] reg_rdata;

    spi_slave dut (
        .clk (clk), .rst (rst),
        .spi_sck (sck), .spi_si (si), .spi_so (so), .spi_ss (ss),
        .reg_addr (reg_addr), .reg_wdata (reg_wdata),
        .reg_we (reg_we), .reg_re (reg_re), .reg_rdata (reg_rdata),
        .selected ()
    );

    /* A register file, plus a counter behind the non-incrementing port so a
     * burst read of it returns a different byte each time - which is how a
     * FIFO behaves and what the port address exists for. */
    reg [7:0] regs [0:63];
    reg [7:0] port_next = 8'hA0;

    always @(posedge clk) begin
        if (reg_we && reg_addr < 7'h40) begin
            regs[reg_addr[5:0]] <= reg_wdata;
        end
        if (reg_re) begin
            if (reg_addr >= 7'h40) begin
                reg_rdata <= port_next;
                port_next <= port_next + 1'b1;
            end else begin
                reg_rdata <= regs[reg_addr[5:0]];
            end
        end
    end

    /* ---- an SPI master, mode 0, MSB first ---------------------------- */
    localparam integer HALF = 200;     /* 2.5 MHz: well inside sysclk/4 */

    task spi_byte;
        input  [7:0] tx;
        output [7:0] rx;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                si = tx[i];
                #HALF;
                sck = 1'b1;            /* the slave samples here */
                rx[i] = so;
                #HALF;
                sck = 1'b0;            /* and presents the next bit here */
            end
        end
    endtask

    task spi_select;   begin ss = 1'b0; #HALF; end endtask
    task spi_deselect; begin #HALF; ss = 1'b1; #(HALF*4); end endtask

    integer errors = 0;
    reg [7:0] got;

    task check;
        input [199:0] name;
        input [7:0]   a;
        input [7:0]   b;
        begin
            if (a === b) $display("  ok   %0s = 0x%02h", name, a);
            else begin
                $display("  FAIL %0s: got 0x%02h want 0x%02h", name, a, b);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_spi_slave.vcd");
        $dumpvars(0, tb_spi_slave);

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);

        /* ---- write three bytes from address 0x05 ---- */
        $display("write burst to 0x05");
        spi_select;
        spi_byte(8'h85, got);          /* write, addr 0x05 */
        spi_byte(8'h11, got);
        spi_byte(8'h22, got);
        spi_byte(8'h33, got);
        spi_deselect;

        check("reg 0x05", regs[5], 8'h11);
        check("reg 0x06", regs[6], 8'h22);
        check("reg 0x07", regs[7], 8'h33);

        /* ---- read them back in one transaction ---- */
        $display("read burst from 0x05");
        spi_select;
        spi_byte(8'h05, got);          /* read, addr 0x05 */
        spi_byte(8'h00, got); check("read 0x05", got, 8'h11);
        spi_byte(8'h00, got); check("read 0x06", got, 8'h22);
        spi_byte(8'h00, got); check("read 0x07", got, 8'h33);
        spi_deselect;

        /* ---- the non-incrementing port ---- */
        $display("burst read of the port address 0x40");
        spi_select;
        spi_byte(8'h40, got);          /* read, addr 0x40 */
        spi_byte(8'h00, got); check("port byte 1", got, 8'hA0);
        spi_byte(8'h00, got); check("port byte 2", got, 8'hA1);
        spi_byte(8'h00, got); check("port byte 3", got, 8'hA2);
        spi_deselect;
        check("address did not walk", reg_addr, 8'h40);

        /* ---- a second transaction must start clean ---- */
        $display("back to back transactions");
        spi_select;
        spi_byte(8'h8A, got);          /* write, addr 0x0A */
        spi_byte(8'h5A, got);
        spi_deselect;
        check("reg 0x0A", regs[10], 8'h5A);

        spi_select;
        spi_byte(8'h0A, got);
        spi_byte(8'h00, got); check("reread 0x0A", got, 8'h5A);
        spi_deselect;

        $display("");
        if (errors == 0) $display("PASSED (0 failures)");
        else             $display("FAILED (%0d failure%s)", errors, (errors == 1) ? "" : "s");
        $finish;
    end

    initial begin
        #5000000;
        $display("FAILED (timeout)");
        $finish;
    end
endmodule

`default_nettype wire
