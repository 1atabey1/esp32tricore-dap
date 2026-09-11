/*
 * Check the RTL against the vectors the host test binary already pins down.
 *
 * These are the same numbers components/dap_probe/test/test_dap_frame.c
 * asserts, which is the point: the C and the Verilog are independent
 * implementations of the same wire format, so agreeing on the documented
 * vectors means something.  Disagreeing means one of them is wrong before a
 * pin has moved.
 *
 *   sync  = CMD 0x10, LEN 63, no data  -> CRC 9,  wire word 0x09FE1, 19 bits
 *   LEN 0 frame                        -> CRC 25
 *
 * The wire word is the frame with the *first transmitted bit in bit 0*.  The
 * other convention gives 0x23FC8, so this vector tests bit order as much as
 * it tests the checksum.
 */

`timescale 1ns / 1ps
`default_nettype none

module tb_dap_frame;

    reg clk = 1'b0;
    always #5 clk = ~clk;          /* 100 MHz */

    reg         rst   = 1'b1;
    reg         start = 1'b0;
    reg  [4:0]  cmd   = 5'h10;
    reg  [5:0]  len   = 6'd63;
    reg  [5:0]  dbits = 6'd0;
    reg  [62:0] data  = 63'd0;

    wire busy, done, dap0, dap1, dat_oe;

    dap_frame_tx #(.DIV_WIDTH(8)) dut (
        .clk (clk), .rst (rst),
        .div (8'd1),               /* fast, so the test runs in little time */
        .start (start), .cmd (cmd), .len (len), .data_bits (dbits), .data (data),
        .lead (6'd2),
        .busy (busy), .done (done),
        .dap0 (dap0), .dap1 (dap1), .dat_oe (dat_oe)
    );

    /*
     * Sample dap1 on every rising edge of dap0, which is what the target does.
     * The lead-in clocks are part of the frame as the device sees it, so they
     * land in the capture too and the check skips them explicitly rather than
     * quietly trimming - a lead-in that went missing is a bug worth failing on.
     */
    reg [95:0] captured;
    integer    nbits;
    reg        dap0_d;

    always @(posedge clk) begin
        dap0_d <= dap0;
        if (!rst && dap0 && !dap0_d) begin
            captured[nbits] <= dap1;
            nbits           <= nbits + 1;
        end
    end

    integer errors = 0;

    task check_eq;
        input [127:0] name;
        input [63:0]  got;
        input [63:0]  want;
        begin
            if (got !== want) begin
                $display("  FAIL %0s: got 0x%0h, want 0x%0h", name, got, want);
                errors = errors + 1;
            end else begin
                $display("  ok   %0s = 0x%0h", name, got);
            end
        end
    endtask

    /* The frame proper, with the lead-in clocks removed. */
    localparam integer LEAD = 2;

    initial begin
        $dumpfile("tb_dap_frame.vcd");
        $dumpvars(0, tb_dap_frame);

        nbits    = 0;
        captured = 96'd0;
        dap0_d   = 1'b0;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        $display("sync: CMD 0x10, LEN 63, no data");
        cmd   = 5'h10;
        len   = 6'd63;
        dbits = 6'd0;          /* sync carries LEN 63 and no data */
        data  = 63'd0;
        @(posedge clk) start = 1'b1;
        @(posedge clk) start = 1'b0;
        wait (done);
        @(posedge clk);

        check_eq("lead-in clocks", nbits - 19, LEAD);
        /* The 19 frame bits, first transmitted bit in bit 0. */
        check_eq("wire word", (captured >> LEAD) & 96'h7FFFF, 64'h09FE1);

        /* Bit 18 is the trailing zero; bits 12..17 are the CRC. */
        check_eq("crc", (captured >> (LEAD + 12)) & 96'h3F, 64'd9);

        $display("LEN 0 frame: CMD 0x10, LEN 0");
        nbits    = 0;
        captured = 96'd0;
        len      = 6'd0;
        dbits    = 6'd0;
        @(posedge clk) start = 1'b1;
        @(posedge clk) start = 1'b0;
        wait (done);
        @(posedge clk);

        /* start + 5 + 6 + 0 + 6 + 1 = 19 bits here too. */
        check_eq("len0 crc", (captured >> (LEAD + 12)) & 96'h3F, 64'd25);

        /*
         * Frames that carry a DATA field.
         *
         * The gap that mattered: everything above has no payload, and on
         * hardware exactly those worked while every frame with data drew no
         * reply at all.  The expected words come from the C builder, which is
         * itself checked against the documented vectors - so this compares two
         * implementations rather than the RTL against itself.
         */
        $display("client_set(1): CMD 0x1C, LEN 3, 3 data bits");
        nbits    = 0;
        captured = 96'd0;
        cmd      = 5'h1C;
        len      = 6'd3;
        dbits    = 6'd3;
        data     = 63'd1;
        @(posedge clk) start = 1'b1;
        @(posedge clk) start = 1'b0;
        wait (done);
        @(posedge clk);
        check_eq("frame length", nbits - LEAD, 22);
        check_eq("client_set word", (captured >> LEAD) & 96'h3FFFFF, 64'h1B10F9);

        $display("client_read CLIENT_ID: CMD 0x1A, LEN 7, 7 data bits");
        nbits    = 0;
        captured = 96'd0;
        cmd      = 5'h1A;
        len      = 6'd7;
        dbits    = 6'd7;
        data     = 63'h4F;
        @(posedge clk) start = 1'b1;
        @(posedge clk) start = 1'b0;
        wait (done);
        @(posedge clk);
        check_eq("frame length", nbits - LEAD, 26);
        check_eq("client_read word", (captured >> LEAD) & 96'h3FFFFFF, 64'h1ECF1F5);

        $display("client_write IOCONF: CMD 0x08, LEN 16, 16 data bits");
        nbits    = 0;
        captured = 96'd0;
        cmd      = 5'h08;
        len      = 6'd16;
        dbits    = 6'd16;
        data     = 63'h810;
        @(posedge clk) start = 1'b1;
        @(posedge clk) start = 1'b0;
        wait (done);
        @(posedge clk);
        check_eq("frame length", nbits - LEAD, 35);
        check_eq("client_write word", (captured >> LEAD) & 96'h7FFFFFFFF, 64'h240810411);

        $display("");
        if (errors == 0) begin
            $display("PASSED (0 failures)");
        end else begin
            $display("FAILED (%0d failure%s)", errors, (errors == 1) ? "" : "s");
        end
        $finish;
    end

    /* A frame that never completes should not hang the run forever. */
    initial begin
        #500000;
        $display("FAILED (timeout)");
        $finish;
    end
endmodule

`default_nettype wire
