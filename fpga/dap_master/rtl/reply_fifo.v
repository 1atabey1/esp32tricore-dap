/*
 * Reply FIFO with its read side in the SPI clock domain.
 *
 * The sequencer pushes bytes on the fabric clock; the SPI slave pops them on
 * SCK, so a burst read costs no fabric clocks per byte (a byte handed across
 * by toggle and back took about eight, which capped SCK below the fabric
 * clock).  The memory is block RAM with separate read and write clocks; the
 * pointers cross as Gray code, one more bit than the address so full and
 * empty differ.
 *
 * SCK runs only during transfers.  The read side therefore sees pushes made
 * while it was idle once the next transfer's header has clocked them through
 * (eight edges; the synchroniser and level need four), and the write side
 * sees pops on the fabric clock, which always runs.  Both views are
 * conservative: the writer may think the FIFO fuller than it is, the reader
 * emptier.
 *
 * clear is a fabric-clock pulse.  It resets the read pointer asynchronously,
 * which is safe only while SCK is quiet: the host clears with CTRL alone in
 * its transfer, and the pulse comes a few fabric clocks after that transfer's
 * last edge.
 */

`default_nettype none

module reply_fifo #(
    parameter integer AW = 10                   /* depth 2**AW bytes */
) (
    /* Fabric side */
    input  wire          clk,
    input  wire          rst,
    input  wire          clear,
    input  wire          push,
    input  wire [7:0]    din,
    output reg  [AW:0]   count,                 /* bytes held, as the writer sees it */
    output reg           empty,
    output reg           full,

    /* SPI side, all on SCK's rising edge */
    input  wire          sck,
    input  wire          pop,                   /* take head (ignored when empty) */
    /* head is the byte after the read pointer's (the next one, while the
     * pointer's own is on the wire) */
    input  wire          ahead,
    output reg  [7:0]    head,                  /* the byte at the read pointer */
    output wire          rd_empty,
    output reg  [AW:0]   rd_level               /* bytes held, as the reader sees it */
);
    localparam integer DEPTH = 1 << AW;

    reg [7:0] mem [0:DEPTH-1];

    function [AW:0] bin2gray(input [AW:0] b);
        bin2gray = b ^ (b >> 1);
    endfunction

    function [AW:0] gray2bin(input [AW:0] g);
        integer i;
        begin
            gray2bin[AW] = g[AW];
            for (i = AW - 1; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i + 1] ^ g[i];
        end
    endfunction

    /* ------------------------------------------------------------------ */
    /* Write side                                                          */
    /* ------------------------------------------------------------------ */

    reg [AW:0] wr_bin  = 0;
    reg [AW:0] wr_gray = 0;
    /* The read pointer, synchronised, and converted a stage later (timing). */
    reg [AW:0] rg_s1 = 0, rg_s2 = 0;
    reg [AW:0] rd_bin_w = 0;

    wire do_push = push && !full;
    /* At pointer width: the pointers wrap, and an integer compare would
     * widen the subtraction past them. */
    wire [AW:0] held = wr_bin - rd_bin_w;

    always @(posedge clk) begin
        if (do_push)
            mem[wr_bin[AW-1:0]] <= din;
    end

    always @(posedge clk) begin
        if (rst || clear) begin
            wr_bin   <= 0;
            wr_gray  <= 0;
            rg_s1    <= 0;
            rg_s2    <= 0;
            rd_bin_w <= 0;
            count    <= 0;
            empty    <= 1'b1;
            full     <= 1'b0;
        end else begin
            rg_s1    <= rd_gray;
            rg_s2    <= rg_s1;
            rd_bin_w <= gray2bin(rg_s2);
            if (do_push) begin
                wr_bin  <= wr_bin + 1'b1;
                wr_gray <= bin2gray(wr_bin + 1'b1);
            end
            /* From the pointers as they were last cycle: a clock late, and
             * the push of this cycle not yet in it (conservative for room,
             * the sequencer's gate keeps a margin). */
            count <= held;
            empty <= (wr_bin == rd_bin_w);
            full  <= held[AW];          /* DEPTH bytes held */
        end
    end

    /* ------------------------------------------------------------------ */
    /* Read side (SCK)                                                     */
    /* ------------------------------------------------------------------ */

    reg [AW:0] rd_bin  = 0;
    reg [AW:0] rd_p1   = 1;                     /* rd_bin + 1, kept (no adder on RADDR) */
    reg [AW:0] rd_gray = 0;
    reg [AW:0] wg_s1 = 0, wg_s2 = 0;
    reg [AW:0] wr_bin_r = 0;

    assign rd_empty = (rd_bin == wr_bin_r);

    /*
     * The pointer and its Gray copy reset asynchronously on clear (see
     * above); the synchroniser and level follow within four edges.
     */
    always @(posedge sck or posedge clear) begin
        if (clear) begin
            rd_bin  <= 0;
            rd_p1   <= 1;
            rd_gray <= 0;
        end else if (pop && !rd_empty) begin
            rd_bin  <= rd_p1;
            rd_p1   <= rd_p1 + 1'b1;
            rd_gray <= bin2gray(rd_p1);
        end
    end

    always @(posedge sck) begin
        wg_s1    <= wr_gray;
        wg_s2    <= wg_s1;
        wr_bin_r <= gray2bin(wg_s2);
        rd_level <= wr_bin_r - rd_bin;
        /* Registered read: the byte at the pointer (or the one after it)
         * one edge after it moves. */
        head     <= mem[ahead ? rd_p1[AW-1:0] : rd_bin[AW-1:0]];
    end
endmodule

`default_nettype wire
