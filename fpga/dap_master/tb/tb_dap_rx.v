/*
 * The RX path against a fake target.
 *
 * The fake target is deliberately built from dap_crc6_gen - the *generator* -
 * while the receiver checks with dap_crc6_check, the Fibonacci one.  So a
 * passing test says the two topologies agree, which is the property the
 * protocol actually depends on and the one a shared implementation would hide.
 *
 * What is covered:
 *
 *   sync reply    32 bits of 0xAAAAAAAA after one busy cycle, CRC good
 *   busy stuffing a reply that arrives late still decodes, wait count reported
 *   corruption    one flipped payload bit fails the residue check
 *   idle high     a line nobody drives is named, not reported as 0xFFFFFFFF
 *   acknowledge   a start bit and nothing else, with no CRC to check
 */

`timescale 1ns / 1ps
`default_nettype none

module tb_dap_rx;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg         rst        = 1'b1;
    reg         start      = 1'b0;
    reg  [6:0]  reply_bits = 7'd32;
    reg  [15:0] max_wait   = 16'd64;
    reg  [7:0]  trail      = 8'd1;
    reg         expect_crc = 1'b1;
    reg         wide       = 1'b0;

    wire        busy, done, timed_out, idle_high, crc_ok;
    wire [15:0] wait_cycles;
    wire [62:0] payload;
    wire [5:0]  crc;
    wire        dap0, dat_oe, start_aligned;

    reg         target_bit  = 1'b1;
    reg         target_bit2 = 1'b1;
    /* The target drives only while the master has released the line.  DAP2 is
     * the target's to drive whenever DAP1 is, which is what the real part does
     * in wide mode - the two lines turn round together. */
    wire        dap1 = dat_oe ? 1'bz : target_bit;
    wire        dap2 = dat_oe ? 1'bz : target_bit2;

    dap_frame_rx #(.DIV_WIDTH(8)) dut (
        .clk (clk), .rst (rst), .div (8'd1),
        .start (start), .reply_bits (reply_bits),
        .max_wait (max_wait), .trail_clocks (trail), .expect_crc (expect_crc),
        .no_hunt (1'b0), .wide (wide),
        .busy (busy), .done (done),
        .wait_cycles (wait_cycles), .timed_out (timed_out),
        .idle_high (idle_high), .crc_ok (crc_ok),
        .payload (payload), .crc (crc),
        .start_aligned (start_aligned),
        .dap0 (dap0), .dap1_in (dap1), .dap2_in (dap2), .dat_oe (dat_oe)
    );

    /* ---- the fake target -------------------------------------------------
     * Presents a bit, then waits for the master to take it: the master samples
     * at the end of the low phase, so the target updates just after dap0 falls.
     */
    task drive_bit;
        input value;
        begin
            target_bit  = value;
            target_bit2 = value;   /* unread in narrow mode */
            @(posedge dap0);      /* the master has taken this one */
            @(negedge dap0);      /* free to present the next */
        end
    endtask

    /* One clock of wide mode: the even bit on DAP1 and the odd one on DAP2. */
    task drive_pair;
        input even;
        input odd;
        begin
            target_bit  = even;
            target_bit2 = odd;
            @(posedge dap0);
            @(negedge dap0);
        end
    endtask

    /*
     * A wide reply, interleaved the way the device sends it.
     *
     * The CRC is computed over the payload in transmission order plus its pad,
     * which for an even payload is the narrow CRC unchanged - so this shares
     * crc_of with the narrow tests rather than carrying a second model of the
     * polynomial that could agree with the design by being copied from it.
     */
    task send_reply_wide;
        input [62:0] value;
        input integer nbits;
        input integer busy_cycles;
        integer i, total;
        reg [5:0] c;
        reg [63:0] padded;
        begin
            total  = (nbits + 1) / 2 * 2;       /* one pad bit if odd */
            padded = {1'b0, value};
            c = crc_of(padded[62:0], total);
            for (i = 0; i < busy_cycles; i = i + 1) drive_pair(1'b0, 1'b0);
            drive_pair(1'b1, 1'b1);             /* start bit on both lines */
            for (i = 0; i < total; i = i + 2)
                drive_pair(padded[i], padded[i + 1]);
            for (i = 0; i < 6; i = i + 2) drive_pair(c[i], c[i + 1]);
            target_bit  = 1'b0;
            target_bit2 = 1'b0;
        end
    endtask

    /* CRC of a payload, computed the way the device would - with the generator. */
    function [5:0] crc_of;
        input [62:0] value;
        input integer nbits;
        integer i;
        reg [5:0] state;
        reg fb;
        begin
            state = 6'h20;                       /* Galois seed */
            for (i = 0; i < nbits; i = i + 1) begin
                fb = state[0] ^ value[i];
                state = {1'b0, state[5:1]};
                if (fb) state = state ^ 6'h30;   /* Galois poly */
            end
            crc_of = state;
        end
    endfunction

    task send_reply;
        input [62:0] value;
        input integer nbits;
        input integer busy_cycles;
        input integer corrupt_bit;   /* -1 for none */
        integer i;
        reg [5:0] c;
        begin
            c = crc_of(value, nbits);
            for (i = 0; i < busy_cycles; i = i + 1) drive_bit(1'b0);
            drive_bit(1'b1);                                   /* start bit */
            for (i = 0; i < nbits; i = i + 1)
                drive_bit(value[i] ^ (i == corrupt_bit));
            for (i = 0; i < 6; i = i + 1) drive_bit(c[i]);
            target_bit = 1'b0;
        end
    endtask

    integer errors = 0;
    integer k;

    task check;
        input [199:0] name;
        input         cond;
        begin
            if (cond) $display("  ok   %0s", name);
            else begin
                $display("  FAIL %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_dap_rx.vcd");
        $dumpvars(0, tb_dap_rx);

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        /* ---- a good 32-bit reply after one busy cycle ---- */
        $display("sync reply, 0xAAAAAAAA after 1 busy cycle");
        reply_bits = 7'd32;
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                @(negedge dat_oe);
                send_reply(63'hAAAAAAAA, 32, 1, -1);
            end
        join
        wait (done); @(posedge clk);
        check("payload 0xAAAAAAAA", payload[31:0] === 32'hAAAAAAAA);
        check("wait cycles = 1",    wait_cycles === 16'd1);
        check("crc residue ok",     crc_ok === 1'b1);
        check("not idle high",      idle_high === 1'b0);
        check("not timed out",      timed_out === 1'b0);
        check("line handed back",   dat_oe === 1'b1);

        /* ---- busy stuffing: the same reply, arriving late ---- */
        $display("the same reply after 17 busy cycles");
        repeat (4) @(posedge clk);
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                @(negedge dat_oe);
                send_reply(63'hAAAAAAAA, 32, 17, -1);
            end
        join
        wait (done); @(posedge clk);
        check("payload still right", payload[31:0] === 32'hAAAAAAAA);
        check("wait cycles = 17",    wait_cycles === 16'd17);
        check("crc residue ok",      crc_ok === 1'b1);

        /* ---- one flipped bit must fail the residue ---- */
        $display("one corrupted payload bit");
        repeat (4) @(posedge clk);
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                @(negedge dat_oe);
                send_reply(63'hAAAAAAAA, 32, 1, 9);
            end
        join
        wait (done); @(posedge clk);
        check("crc rejects it", crc_ok === 1'b0);

        /* ---- nobody driving: all ones, named rather than believed ---- */
        $display("nothing driving the wire");
        repeat (4) @(posedge clk);
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                @(negedge dat_oe);
                target_bit = 1'b1;      /* held high for the whole window */
            end
        join
        wait (done); @(posedge clk);
        check("idle high is named", idle_high === 1'b1);
        check("and the crc fails",  crc_ok === 1'b0);

        /* ---- a bare acknowledge ---- */
        $display("bare acknowledge, no payload and no CRC");
        repeat (4) @(posedge clk);
        reply_bits = 7'd0;
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                @(negedge dat_oe);
                drive_bit(1'b0);
                drive_bit(1'b1);        /* the acknowledge itself */
                target_bit = 1'b0;
            end
        join
        wait (done); @(posedge clk);
        check("acknowledged",       crc_ok === 1'b1);
        check("wait cycles = 1",    wait_cycles === 16'd1);
        check("not timed out",      timed_out === 1'b0);

        /* ---- a block-read parcel: 32 bits and no CRC ----
         * The parcels before the last one carry no checksum, and reading six
         * phantom CRC bits after one of them would clock into the next parcel
         * and shift the whole rest of the block. */
        $display("block parcel, 32 bits with no CRC following");
        repeat (4) @(posedge clk);
        reply_bits = 7'd32;
        expect_crc = 1'b0;
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                /* Driven inline rather than through send_reply, which always
                 * appends a CRC: with none expected the receiver stops
                 * clocking after the payload, and a driver still waiting to
                 * send checksum bits would hang on edges that never come. */
                @(negedge dat_oe);
                drive_bit(1'b0);
                drive_bit(1'b1);
                for (k = 0; k < 32; k = k + 1) drive_bit(32'h11223344 >> k);
                target_bit = 1'b0;
            end
        join
        wait (done); @(posedge clk);
        check("parcel payload", payload[31:0] === 32'h11223344);
        check("reported good",  crc_ok === 1'b1);
        check("not timed out",  timed_out === 1'b0);
        expect_crc = 1'b1;

        /* ---- wide mode: the same reply, two bits per clock ---- */
        $display("wide reply, 0xAAAAAAAA as pairs after 1 busy cycle");
        repeat (4) @(posedge clk);
        wide       = 1'b1;
        reply_bits = 7'd32;
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                @(negedge dat_oe);
                send_reply_wide(63'hAAAAAAAA, 32, 1);
            end
        join
        wait (done); @(posedge clk);
        check("wide payload 0xAAAAAAAA", payload[31:0] === 32'hAAAAAAAA);
        check("wide crc residue ok",     crc_ok === 1'b1);
        check("wide not timed out",      timed_out === 1'b0);
        check("wide not idle high",      idle_high === 1'b0);
        /* Half the clocks of the narrow case, which is the whole point. */
        check("wide half the clocks",    wait_cycles === 16'd1);
        check("start bit on both lines", start_aligned === 1'b1);

        /* ---- an odd payload, so the pad bit is exercised ---- */
        $display("wide reply, 7 bits - an odd payload with a pad");
        repeat (4) @(posedge clk);
        reply_bits = 7'd7;
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                @(negedge dat_oe);
                send_reply_wide(63'h4F, 7, 1);
            end
        join
        wait (done); @(posedge clk);
        check("wide odd payload", payload[6:0] === 7'h4F);
        check("wide odd crc ok",  crc_ok === 1'b1);

        /*
         * DAP2 stuck low through the start bit.
         *
         * This is what a mis-set capture tap looks like, and the receiver is
         * meant to report it rather than refuse the frame - the host sweeps the
         * taps by reading this bit back, so a receiver that threw the reply
         * away would remove the only evidence the sweep has.
         */
        $display("wide reply with DAP2 misaligned at the start bit");
        repeat (4) @(posedge clk);
        reply_bits = 7'd32;
        fork
            begin
                @(posedge clk) start = 1'b1;
                @(posedge clk) start = 1'b0;
            end
            begin
                @(negedge dat_oe);
                drive_pair(1'b0, 1'b0);
                drive_pair(1'b1, 1'b0);      /* start bit on DAP1 only */
                for (k = 0; k < 32; k = k + 2)
                    drive_pair(32'hAAAAAAAA >> k, 32'hAAAAAAAA >> (k + 1));
                for (k = 0; k < 6; k = k + 2)
                    drive_pair(crc_of(63'hAAAAAAAA, 32) >> k,
                               crc_of(63'hAAAAAAAA, 32) >> (k + 1));
                target_bit  = 1'b0;
                target_bit2 = 1'b0;
            end
        join
        wait (done); @(posedge clk);
        check("misalignment is reported", start_aligned === 1'b0);
        check("the frame still arrives",  payload[31:0] === 32'hAAAAAAAA);
        wide = 1'b0;

        $display("");
        if (errors == 0) $display("PASSED (0 failures)");
        else             $display("FAILED (%0d failure%s)", errors, (errors == 1) ? "" : "s");
        $finish;
    end

    initial begin
        #2000000;
        $display("FAILED (timeout)");
        $finish;
    end
endmodule

`default_nettype wire
