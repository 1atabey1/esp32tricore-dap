/*
 * DAP CRC6, both directions.  Ported bit for bit from
 * components/dap_probe/dap_frame.c and checked against the same vectors.
 *
 * Generator: right-shifting Galois LFSR.  Checker: independent Fibonacci LFSR
 * whose residue is zero for a good frame.  One bit per `en`, LSB first; in wide
 * mode a pair per clock, bit_in (even) then bit_in2 (odd).
 */

`default_nettype none

module dap_crc6_gen (
    input  wire       clk,
    input  wire       rst,      /* synchronous, reloads the seed */
    input  wire       en,
    input  wire       bit_in,
    input  wire       bit_in2,  /* the odd bit of the pair, wide mode only */
    input  wire       wide,
    output wire [5:0] crc
);
    localparam [5:0] GALOIS_POLY = 6'h30;
    localparam [5:0] GALOIS_SEED = 6'h20;

    reg [5:0] state;

    /* Shift right, and fold the polynomial back in on a set feedback. */
    function [5:0] step;
        input [5:0] s;
        input       b;
        begin
            step = (s[0] ^ b) ? ({1'b0, s[5:1]} ^ GALOIS_POLY)
                              :  {1'b0, s[5:1]};
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state <= GALOIS_SEED;
        end else if (en) begin
            /* bit_in is the earlier bit of the pair, so it goes first. */
            state <= wide ? step(step(state, bit_in), bit_in2)
                          : step(state, bit_in);
        end
    end

    assign crc = state;
endmodule


module dap_crc6_check (
    input  wire       clk,
    input  wire       rst,
    input  wire       en,
    input  wire       bit_in,
    input  wire       bit_in2,  /* the odd bit of the pair, wide mode only */
    input  wire       wide,
    output wire       residue_ok   /* valid once the whole frame has been fed */
);
    localparam [5:0] FIBO_POLY = 6'h03;
    localparam [5:0] FIBO_SEED = 6'h3F;

    reg [5:0] state;

    /* Taps at bits 0 and 1 (FIBO_POLY) XOR the input, shifted in at the top. */
    function [5:0] step;
        input [5:0] s;
        input       b;
        begin
            step = {b ^ s[0] ^ s[1], s[5:1]};
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state <= FIBO_SEED;
        end else if (en) begin
            state <= wide ? step(step(state, bit_in), bit_in2)
                          : step(state, bit_in);
        end
    end

    assign residue_ok = (state == 6'b0);
endmodule

`default_nettype wire
