/*
 * The whole master, driven over SPI the way the ESP32 will drive it, against a
 * fake target.
 *
 * The block read is the case that matters.  It is the reason for putting any
 * of this in fabric: on the host each parcel costs a start-bit hunt and 32
 * software-driven clocks, and the win here is that the host writes a command,
 * says how many parcels, and reads the whole answer back as one burst.  So the
 * test drives it exactly that way and checks the bytes come out in order.
 */

`timescale 1ns / 1ps
`default_nettype none

module tb_dap_top;

    reg clk = 1'b0;
    always #5 clk = ~clk;          /* 100 MHz fabric */

    reg rst = 1'b1;

    reg  sck = 1'b0, si = 1'b0, ss = 1'b1;
    wire so;

    wire dap0, dap2, trst;
    wire dap1;

    /* The fake target drives dap1 only when the master has let go. */
    reg  target_drive = 1'b0;
    reg  target_bit   = 1'b1;
    assign dap1 = target_drive ? target_bit : 1'bz;

    dap_top #(.FIFO_DEPTH(64)) dut (
        .clk (clk), .rst (rst),
        .spi_sck (sck), .spi_si (si), .spi_so (so), .spi_ss (ss),
        .dap0 (dap0), .dap1 (dap1), .trst (trst), .dap2 (dap2)
    );

    /* ---- SPI master, mode 0, MSB first ---- */
    localparam integer HALF = 60;

    task spi_byte;
        input  [7:0] tx;
        output [7:0] rx;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                si = tx[i];
                #HALF; sck = 1'b1; rx[i] = so;
                #HALF; sck = 1'b0;
            end
        end
    endtask

    reg [7:0] scratch;

    task wr;
        input [6:0] addr;
        input [7:0] val;
        begin
            ss = 1'b0; #HALF;
            spi_byte({1'b1, addr}, scratch);
            spi_byte(val, scratch);
            #HALF; ss = 1'b1; #(HALF*4);
        end
    endtask

    task rd;
        input  [6:0] addr;
        output [7:0] val;
        begin
            ss = 1'b0; #HALF;
            spi_byte({1'b0, addr}, scratch);
            spi_byte(8'h00, val);
            #HALF; ss = 1'b1; #(HALF*4);
        end
    endtask

    /* ---- the fake target ---- */
    task drive_bit;
        input value;
        begin
            target_bit = value;
            @(posedge dap0);
            @(negedge dap0);
        end
    endtask

    function [5:0] crc_of;
        input [63:0] value;
        input integer nbits;
        integer i;
        reg [5:0] st;
        reg fb;
        begin
            st = 6'h20;
            for (i = 0; i < nbits; i = i + 1) begin
                fb = st[0] ^ value[i];
                st = {1'b0, st[5:1]};
                if (fb) st = st ^ 6'h30;
            end
            crc_of = st;
        end
    endfunction

    /* One parcel: a start bit and 32 data bits, CRC only when asked. */
    task send_parcel;
        input [31:0] value;
        input        with_crc;
        integer i;
        reg [5:0] c;
        begin
            c = crc_of({32'd0, value}, 32);
            drive_bit(1'b0);              /* one busy cycle */
            drive_bit(1'b1);              /* start bit */
            for (i = 0; i < 32; i = i + 1) drive_bit(value[i]);
            if (with_crc)
                for (i = 0; i < 6; i = i + 1) drive_bit(c[i]);
            target_bit = 1'b0;
        end
    endtask

    integer errors = 0;

    task check;
        input [199:0] name;
        input [31:0]  a;
        input [31:0]  b;
        begin
            if (a === b) $display("  ok   %0s = 0x%0h", name, a);
            else begin
                $display("  FAIL %0s: got 0x%0h want 0x%0h", name, a, b);
                errors = errors + 1;
            end
        end
    endtask

    reg [7:0] b0, b1, b2, b3;
    integer   p;

    initial begin
        $dumpfile("tb_dap_top.vcd");
        $dumpvars(0, tb_dap_top);

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);

        /*
         * DIV 5, not 1.  The receiver sees DAP1 through a two-flop
         * synchroniser, so its sample is two fabric clocks stale; the half
         * period has to be longer than that or the sample lands in the
         * neighbouring bit.  DIV 1 gives a two-clock half period and is not a
         * legal operating point - it produced a testbench that passed against
         * an unsynchronised design and broke the moment the synchroniser
         * arrived, which is the test being wrong rather than the design.
         */
        wr(7'h02, 8'd5);       /* DIV: 12 fabric clocks per bit */
        wr(7'h07, 8'd1);       /* TRAIL */
        wr(7'h08, 8'd64);      /* MAXWAIT low */
        wr(7'h09, 8'd0);

        /* ---- a single frame: sync, expecting 32 bits back ---- */
        $display("single frame: sync, 32-bit reply");
        wr(7'h03, 8'h10);      /* CMD  = sync */
        wr(7'h04, 8'd63);      /* LEN  = 63 */
        wr(7'h05, 8'd0);       /* DBITS = 0, sync carries no data */
        wr(7'h06, 8'd32);      /* RBITS */

        fork
            wr(7'h01, 8'h01);  /* CTRL: start frame */
            begin
                /* The target answers once the master lets go of the line. */
                wait (dut.u_rx.dat_oe == 1'b0);
                target_drive = 1'b1;
                send_parcel(32'hAAAAAAAA, 1'b1);
                target_drive = 1'b0;
            end
        join

        /* Wait for done. */
        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("status crc_ok", scratch[4], 1'b1);
        check("status not timed out", scratch[2], 1'b0);

        rd(7'h20, b0); rd(7'h21, b1); rd(7'h22, b2); rd(7'h23, b3);
        check("reply", {b3, b2, b1, b0}, 32'hAAAAAAAA);

        /* ---- a block read of three parcels ---- */
        $display("block read, 3 parcels, one SPI burst to collect");
        wr(7'h01, 8'h08);      /* CTRL: clear the fifo */
        wr(7'h0A, 8'd2);       /* PARCELS = 3 - 1 */
        wr(7'h03, 8'h0A);      /* CMD = client_blockread */
        wr(7'h04, 8'd40);
        wr(7'h05, 8'd40);
        wr(7'h10, 8'h0C);      /* some plausible payload */

        fork
            wr(7'h01, 8'h02);  /* CTRL: start block */
            begin
                for (p = 0; p < 3; p = p + 1) begin
                    wait (dut.u_rx.dat_oe == 1'b0);
                    target_drive = 1'b1;
                    send_parcel(32'h11223344 + p, (p == 2));
                    target_drive = 1'b0;
                    @(posedge clk);
                end
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("block not timed out", scratch[2], 1'b0);
        check("block did not overrun", scratch[7], 1'b0);
        check("fifo has data", scratch[5], 1'b0);

        /* Drain as the host would: one transaction, address 0x40 held. */
        $display("draining the fifo in one burst");
        ss = 1'b0; #HALF;
        spi_byte(8'h40, scratch);
        for (p = 0; p < 3; p = p + 1) begin
            spi_byte(8'h00, b0);
            spi_byte(8'h00, b1);
            spi_byte(8'h00, b2);
            spi_byte(8'h00, b3);
            check("parcel", {b3, b2, b1, b0}, 32'h11223344 + p);
        end
        #HALF; ss = 1'b1; #(HALF*4);

        rd(7'h00, scratch);
        check("fifo now empty", scratch[5], 1'b1);

        $display("");
        if (errors == 0) $display("PASSED (0 failures)");
        else             $display("FAILED (%0d failure%s)", errors, (errors == 1) ? "" : "s");
        $finish;
    end

    initial begin
        #20000000;
        $display("FAILED (timeout)");
        $finish;
    end
endmodule

`default_nettype wire
