/*
 * DAP CRC6, both directions.
 *
 * Ported bit for bit from components/dap_probe/dap_frame.c, which is checked
 * against the documented vectors by a host test binary - so these two agree by
 * construction rather than by hope, and the same vectors verify this module.
 *
 * The two topologies are deliberately different.  The generator is a
 * right-shifting Galois LFSR; the checker is a Fibonacci LFSR whose residue
 * lands on zero for a good frame.  A checker built from the generator's own
 * structure would only ever prove self-consistency, which is exactly the
 * mistake this protocol punishes: a wrong-but-consistent CRC produces a device
 * that answers sync and ignores everything after it.
 *
 * Both are fed one bit per clock with `en` high, LSB of the frame first, which
 * is transmission order.
 */

`default_nettype none

module dap_crc6_gen (
    input  wire       clk,
    input  wire       rst,      /* synchronous, reloads the seed */
    input  wire       en,
    input  wire       bit_in,
    output wire [5:0] crc
);
    localparam [5:0] GALOIS_POLY = 6'h30;
    localparam [5:0] GALOIS_SEED = 6'h20;

    reg [5:0] state;

    wire feedback = state[0] ^ bit_in;

    always @(posedge clk) begin
        if (rst) begin
            state <= GALOIS_SEED;
        end else if (en) begin
            /* Shift right, and fold the polynomial back in on a set feedback. */
            state <= feedback ? ({1'b0, state[5:1]} ^ GALOIS_POLY)
                              :  {1'b0, state[5:1]};
        end
    end

    assign crc = state;
endmodule


module dap_crc6_check (
    input  wire       clk,
    input  wire       rst,
    input  wire       en,
    input  wire       bit_in,
    output wire       residue_ok   /* valid once the whole frame has been fed */
);
    localparam [5:0] FIBO_POLY = 6'h03;
    localparam [5:0] FIBO_SEED = 6'h3F;

    reg [5:0] state;

    /* The taps named by FIBO_POLY, XORed with the incoming bit and shifted in
     * at the top.  FIBO_POLY is 6'h03, so the taps are bits 0 and 1. */
    wire feedback = bit_in ^ state[0] ^ state[1];

    always @(posedge clk) begin
        if (rst) begin
            state <= FIBO_SEED;
        end else if (en) begin
            state <= {feedback, state[5:1]};
        end
    end

    assign residue_ok = (state == 6'b0);
endmodule

`default_nettype wire
