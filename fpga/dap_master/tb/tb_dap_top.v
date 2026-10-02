/*
 * The whole master, driven over SPI the way the ESP32 drives it, against a
 * fake target: single frames, a payload frame, block read and block write.
 */

`timescale 1ns / 1ps
`default_nettype none

/* Fast mode's LAG for this bench's round trip, and the target's busy bits
 * before each block-read parcel; overridable to sweep them. */
`ifndef FAST_LAG
`define FAST_LAG 1
`endif
`ifndef FAST_BUSY
`define FAST_BUSY 1
`endif

module tb_dap_top;

    /* 50 MHz, near the 48 MHz hardware clock. */
    reg clk = 1'b0;
    always #10 clk = ~clk;

    /* Reply round trip (driver, PCB, TC38x pad, input): 6-12 ns, 8 modelled. */
    localparam TARGET_DELAY = 8;

    reg rst = 1'b1;

    reg  sck = 1'b0, si = 1'b0, ss = 1'b1;
    wire so;

    wire dap0, dap2, trst;
    wire dap1;

    /* The fake target drives dap1 only when the master has let go. */
    reg  target_drive = 1'b0;
    reg  target_bit   = 1'b1;
    wire dap1_driven = target_drive ? target_bit : 1'bz;
    assign #TARGET_DELAY dap1 = dap1_driven;

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

    /* A read sends a dummy byte after the header while the register file
     * fetches. */
    task rd;
        input  [6:0] addr;
        output [7:0] val;
        begin
            ss = 1'b0; #HALF;
            spi_byte({1'b0, addr}, scratch);
            spi_byte(8'h00, scratch);
            spi_byte(8'h00, val);
            #HALF; ss = 1'b1; #(HALF*4);
        end
    endtask

    /* Multi-byte bursts, as the host writes DATA and CMD..RBITS. */
    reg [7:0] burst [0:15];
    integer   bi;

    task wr_burst;
        input [6:0]     addr;
        input integer   n;
        begin
            ss = 1'b0; #HALF;
            spi_byte({1'b1, addr}, scratch);
            for (bi = 0; bi < n; bi = bi + 1) spi_byte(burst[bi], scratch);
            #HALF; ss = 1'b1; #(HALF*4);
        end
    endtask

    task rd_burst;
        input [6:0]     addr;
        input integer   n;
        begin
            ss = 1'b0; #HALF;
            spi_byte({1'b0, addr}, scratch);
            spi_byte(8'h00, scratch);          /* the dummy */
            for (bi = 0; bi < n; bi = bi + 1) spi_byte(8'h00, burst[bi]);
            #HALF; ss = 1'b1; #(HALF*4);
        end
    endtask

    /* What went out on the wire, sampled at DAP0 rising, transmitter only.
     * On the edge itself, not the fabric clock: fast mode raises DAP0 at
     * mid-cycle. */
    reg [95:0] sent, sent2;
    integer    sent_bits;

    always @(posedge dap0) begin
        if (!rst && dut.u_tx.busy) begin
            sent[sent_bits]  <= dap1;
            sent2[sent_bits] <= dap2;
            sent_bits        <= sent_bits + 1;
        end
    end

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

    /* Busy bits before each parcel's start bit. */
    integer busy_bits = 1;

    /* One parcel: a start bit and 32 data bits, CRC only when asked. */
    task send_parcel;
        input [31:0] value;
        input        with_crc;
        integer i;
        reg [5:0] c;
        begin
            c = crc_of({32'd0, value}, 32);
            for (i = 0; i < busy_bits; i = i + 1)
                drive_bit(1'b0);          /* busy cycles */
            drive_bit(1'b1);              /* start bit */
            for (i = 0; i < 32; i = i + 1) drive_bit(value[i]);
            if (with_crc)
                for (i = 0; i < 6; i = i + 1) drive_bit(c[i]);
            target_bit = 1'b0;
        end
    endtask

    /* A bare acknowledge: one busy cycle, then the start bit.  The device
     * answers a client_blockwrite frame and each parcel with one. */
    task send_ack;
        begin
            drive_bit(1'b0);
            drive_bit(1'b1);
            target_bit = 1'b0;
        end
    endtask

    /* Every parcel the transmitter sent (start bit + 32 data bits), captured
     * off the wire; the start bit is dropped and the word kept. */
    reg [31:0] parcel_seen [0:7];
    integer    parcels_seen = 0;
    reg [5:0]  parcel_bit   = 6'd0;
    reg [31:0] parcel_acc;
    reg        in_parcel    = 1'b0;

    always @(posedge dap0 or posedge rst) begin
        if (rst) begin
            parcels_seen <= 0;
            parcel_bit   <= 6'd0;
            in_parcel    <= 1'b0;
        /* Parcels only; the command frame's start bit must not count. */
        end else if (dut.tx_parcel && capture_parcels) begin
            if (!in_parcel) begin
                if (dap1) begin          /* the start bit */
                    in_parcel  <= 1'b1;
                    parcel_bit <= 6'd0;
                end
            end else begin
                parcel_acc <= {dap1, parcel_acc[31:1]};
                if (parcel_bit == 6'd31) begin
                    parcel_seen[parcels_seen[2:0]] <= {dap1, parcel_acc[31:1]};
                    parcels_seen <= parcels_seen + 1;
                    in_parcel    <= 1'b0;
                end else begin
                    parcel_bit <= parcel_bit + 1'b1;
                end
            end
        end
    end

    /*
     * A whole frame as raw mode takes it: start bit, CMD, LEN, ndata bits of
     * DATA, CRC6 over those three, trailing zero; bit 0 first.
     */
    function [63:0] frame_of;
        input [4:0]   cmd;
        input [5:0]   len;
        input [63:0]  data;
        input integer ndata;
        reg   [63:0]  body;
        reg   [5:0]   c;
        begin
            body = {53'd0, len, cmd} | (data << 11);
            c = crc_of(body, 11 + ndata);
            frame_of = 64'd1 | (body << 1) | ({58'd0, c} << (12 + ndata));
        end
    endfunction

    /* Two line words as one wide raw frame: DAP1 even bits, DAP2 odd. */
    function [63:0] interleave;
        input [10:0] even;
        input [10:0] odd;
        integer i;
        begin
            interleave = 64'd0;
            for (i = 0; i < 11; i = i + 1) begin
                interleave[2*i]   = even[i];
                interleave[2*i+1] = odd[i];
            end
        end
    endfunction

    /* DATA and DBITS for a raw frame. */
    task load_raw;
        input [63:0]  f;
        input integer n;
        integer k;
        begin
            for (k = 0; k < 8; k = k + 1) burst[k] = f[8*k +: 8];
            wr_burst(7'h10, 8);
            wr(7'h05, n);
        end
    endtask

    reg capture_parcels = 1'b0;

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
    reg       stall_seen;
    reg [31:0] fc_words [0:19];
    integer   fc_bad, fc_got, fc_n, fc_i;
    reg [7:0] c0, c1, c2, c3;          /* the drain thread's own bytes */

    initial begin
        $dumpfile("tb_dap_top.vcd");
        $dumpvars(0, tb_dap_top);

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);

        /* DIV 5 for most of the run; DIV 0 and 1 are tested at the end. */
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

        /*
         * ---- a payload frame through the register file: client_set(1) ----
         *
         * LEN 3, three data bits, as dap_probe_client_set sends.  Expected
         * wire word 0x1B10F9 over 22 bits, from the C frame builder.
         */
        $display("payload frame through the register file: client_set(1)");

        burst[0] = 8'h01; burst[1] = 8'h00; burst[2] = 8'h00; burst[3] = 8'h00;
        burst[4] = 8'h00; burst[5] = 8'h00; burst[6] = 8'h00; burst[7] = 8'h00;
        wr_burst(7'h10, 8);                    /* DATA = 1 */

        burst[0] = 8'h1C;                      /* CMD   */
        burst[1] = 8'd3;                       /* LEN   */
        burst[2] = 8'd3;                       /* DBITS */
        burst[3] = 8'd0;                       /* RBITS: a bare acknowledge */
        wr_burst(7'h03, 4);

        /* Did the bytes land where they were addressed? */
        rd_burst(7'h10, 8);
        check("DATA reads back", {burst[3], burst[2], burst[1], burst[0]}, 32'd1);
        rd_burst(7'h03, 4);
        check("CMD",   burst[0], 32'h1C);
        check("LEN",   burst[1], 32'd3);
        check("DBITS", burst[2], 32'd3);
        check("RBITS", burst[3], 32'd0);

        sent_bits = 0;
        sent      = 96'd0;

        fork
            wr(7'h01, 8'h01);                  /* CTRL: start frame */
            begin
                wait (dut.u_rx.dat_oe == 1'b0);
                target_drive = 1'b1;
                drive_bit(1'b0);
                drive_bit(1'b1);               /* the acknowledge */
                target_bit   = 1'b0;
                target_drive = 1'b0;
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("acknowledge not timed out", scratch[2], 1'b0);

        /* Two lead-in clocks precede the frame proper. */
        check("frame length", sent_bits - 2, 22);
        check("client_set word", (sent >> 2) & 96'h3FFFFF, 32'h1B10F9);

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

        /* Drain in two bursts: a read fetches one byte ahead, and the unused
         * fetch at a burst's end must not pop the FIFO. */
        $display("draining the fifo in two bursts");
        ss = 1'b0; #HALF;
        spi_byte(8'h40, scratch);
        spi_byte(8'h00, scratch);              /* the dummy */
        spi_byte(8'h00, b0);
        spi_byte(8'h00, b1);
        spi_byte(8'h00, b2);
        spi_byte(8'h00, b3);
        check("parcel", {b3, b2, b1, b0}, 32'h11223344);
        #HALF; ss = 1'b1; #(HALF*4);

        ss = 1'b0; #HALF;
        spi_byte(8'h40, scratch);
        spi_byte(8'h00, scratch);              /* the dummy */
        for (p = 1; p < 3; p = p + 1) begin
            spi_byte(8'h00, b0);
            spi_byte(8'h00, b1);
            spi_byte(8'h00, b2);
            spi_byte(8'h00, b3);
            check("parcel", {b3, b2, b1, b0}, 32'h11223344 + p);
        end
        #HALF; ss = 1'b1; #(HALF*4);

        rd(7'h00, scratch);
        check("fifo now empty", scratch[5], 1'b1);

        /*
         * ---- two chained block reads ----
         *
         * The second start is written while the first block runs; the fabric
         * holds it and starts the next block as soon as the first is done.
         */
        $display("chained block reads: second start queued while busy");
        wr(7'h01, 8'h08);      /* CTRL: clear the fifo */
        wr(7'h0A, 8'd1);       /* PARCELS = 2 - 1 */

        fork
            begin
                wr(7'h01, 8'h02);                  /* start block 1 */
                wait (dut.q == 4'd3);              /* receiving its parcels */
                wr(7'h01, 8'h02);                  /* queue block 2 */
                rd(7'h0E, scratch);
                check("start queued", scratch[4], 1'b1);
            end
            begin
                for (p = 0; p < 4; p = p + 1) begin
                    wait (dut.u_rx.dat_oe == 1'b0);
                    target_drive = 1'b1;
                    send_parcel(32'hA0B0C000 + p, (p == 1) || (p == 3));
                    target_drive = 1'b0;
                    @(posedge clk);
                    /* Between the blocks the master sends the next frame. */
                    if (p == 1) wait (dut.u_rx.dat_oe == 1'b1);
                end
            end
        join

        scratch = 8'h01;
        while (scratch[0]) rd(7'h00, scratch);  /* busy until both are done */
        check("chain done", scratch[1], 1'b1);
        check("chain not timed out", scratch[2], 1'b0);
        rd(7'h0D, b0);
        check("chain left 16 bytes", {24'd0, b0}, 32'd16);
        rd(7'h0E, scratch);
        check("nothing left queued", scratch[4], 1'b0);

        ss = 1'b0; #HALF;
        spi_byte(8'h40, scratch);
        spi_byte(8'h00, scratch);              /* the dummy */
        for (p = 0; p < 4; p = p + 1) begin
            spi_byte(8'h00, b0);
            spi_byte(8'h00, b1);
            spi_byte(8'h00, b2);
            spi_byte(8'h00, b3);
            check("chained parcel", {b3, b2, b1, b0}, 32'hA0B0C000 + p);
        end
        #HALF; ss = 1'b1; #(HALF*4);

        /*
         * ---- flow control ----
         *
         * 20 parcels into a 64-byte FIFO with nobody draining: the block must
         * stall between parcels (DAP0 quiet, DAP1 released) and finish intact
         * once the host drains.
         */
        $display("flow control: a block larger than the FIFO");
        wr(7'h01, 8'h08);      /* CTRL: clear the fifo */
        wr(7'h0A, 8'd19);      /* PARCELS = 20 - 1 */
        stall_seen = 1'b0;

        fork
            wr(7'h01, 8'h02);
            begin
                for (p = 0; p < 20; p = p + 1) begin
                    wait (dut.u_rx.dat_oe == 1'b0);
                    target_drive = 1'b1;
                    send_parcel(32'hC0DE0000 + p, (p == 19));
                    /* The device keeps the line through the gap. */
                    @(posedge clk);
                end
                target_drive = 1'b0;
            end
            begin
                /* Let it fill and stall before draining anything. */
                wait (dut.q == 4'd4 && !dut.fifo_room);
                repeat (200) @(posedge clk);
                stall_seen = (dut.q == 4'd4) && !dut.dap0;
                check("stall releases DAP1", {31'd0, dut.drive}, 32'd0);
                /* Drain as the host does: only what LEVEL reports. */
                fc_got = 0;
                while (fc_got < 20) begin
                    rd(7'h0D, c0);
                    fc_n = c0 / 4;
                    if (fc_n > 0) begin
                        ss = 1'b0; #HALF;
                        spi_byte(8'h40, scratch);
                        spi_byte(8'h00, scratch);
                        for (fc_i = 0; fc_i < fc_n; fc_i = fc_i + 1) begin
                            spi_byte(8'h00, c0);
                            spi_byte(8'h00, c1);
                            spi_byte(8'h00, c2);
                            spi_byte(8'h00, c3);
                            fc_words[fc_got + fc_i] = {c3, c2, c1, c0};
                        end
                        #HALF; ss = 1'b1; #(HALF*4);
                        fc_got = fc_got + fc_n;
                    end
                end
            end
        join

        scratch = 8'h01;
        while (scratch[0]) rd(7'h00, scratch);
        check("flow-controlled block stalled", {31'd0, stall_seen}, 32'd1);
        check("flow-controlled block not timed out", scratch[2], 1'b0);
        check("flow-controlled block no overrun", scratch[7], 1'b0);
        fc_bad = 0;
        for (p = 0; p < 20; p = p + 1) if (fc_words[p] !== 32'hC0DE0000 + p) fc_bad = fc_bad + 1;
        check("flow-controlled words intact", fc_bad, 32'd0);

        /*
         * ---- the fastest divider ----
         *
         * DIV 0: one-clock half period, one bit every two fabric clocks, with
         * the round trip modelled.  Tested here, not in tb_dap_rx, because
         * only this bench includes the DAP1 synchroniser.
         */
        $display("the fastest divider: sync at DIV 0");
        wr(7'h02, 8'd0);
        wr(7'h03, 8'h10);
        wr(7'h04, 8'd63);
        wr(7'h05, 8'd0);
        wr(7'h06, 8'd32);

        sent_bits = 0;
        sent      = 96'd0;

        fork
            wr(7'h01, 8'h01);
            begin
                wait (dut.u_rx.dat_oe == 1'b0);
                target_drive = 1'b1;
                send_parcel(32'hAAAAAAAA, 1'b1);
                target_drive = 1'b0;
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("DIV 0 crc_ok", scratch[4], 1'b1);
        check("DIV 0 not timed out", scratch[2], 1'b0);
        rd(7'h20, b0); rd(7'h21, b1); rd(7'h22, b2); rd(7'h23, b3);
        check("DIV 0 reply", {b3, b2, b1, b0}, 32'hAAAAAAAA);
        /* The sent frame must still be right at this rate. */
        check("DIV 0 sync word", (sent >> 2) & 96'h7FFFF, 32'h09FE1);

        /* DIV 1: a two-clock half period, as long as the DAP1 synchroniser. */
        $display("the lowest divider: sync at DIV 1");
        wr(7'h02, 8'd1);
        wr(7'h03, 8'h10);      /* CMD  = sync */
        wr(7'h04, 8'd63);      /* LEN  = 63 */
        wr(7'h05, 8'd0);       /* DBITS */
        wr(7'h06, 8'd32);      /* RBITS */

        fork
            wr(7'h01, 8'h01);
            begin
                wait (dut.u_rx.dat_oe == 1'b0);
                target_drive = 1'b1;
                send_parcel(32'hAAAAAAAA, 1'b1);
                target_drive = 1'b0;
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("DIV 1 crc_ok", scratch[4], 1'b1);
        check("DIV 1 not timed out", scratch[2], 1'b0);
        rd(7'h20, b0); rd(7'h21, b1); rd(7'h22, b2); rd(7'h23, b3);
        check("DIV 1 reply", {b3, b2, b1, b0}, 32'hAAAAAAAA);

        /* ---- a block write of three words ---- */
        $display("block write, 3 words pushed through the write FIFO");
        wr(7'h01, 8'h20);                      /* CTRL: clear the write fifo */

        /* The words go in first; the fabric streams them once started. */
        burst[0] = 8'h44; burst[1] = 8'h33; burst[2] = 8'h22; burst[3] = 8'h11;
        burst[4] = 8'h88; burst[5] = 8'h77; burst[6] = 8'h66; burst[7] = 8'h55;
        burst[8] = 8'hEF; burst[9] = 8'hBE; burst[10] = 8'hAD; burst[11] = 8'hDE;
        wr_burst(7'h48, 12);

        /* The level should be exactly what was pushed. */
        rd(7'h43, scratch);
        check("write fifo took 12 bytes", {24'd0, scratch}, 32'd12);

        /* dap_phy_fpga_block_write()'s order: FIFO, DATA, CMD..RBITS burst,
         * PARCELS, start. */
        burst[0] = 8'h00; burst[1] = 8'h00; burst[2] = 8'h00; burst[3] = 8'h40;
        burst[4] = 8'h00; burst[5] = 8'h00; burst[6] = 8'h00; burst[7] = 8'h00;
        wr_burst(7'h10, 8);                    /* DATA, the command payload */

        burst[0] = 8'h09;                      /* CMD: client_blockwrite */
        burst[1] = 8'd40;                      /* LEN */
        burst[2] = 8'd40;                      /* DBITS */
        burst[3] = 8'd0;                       /* RBITS: a bare acknowledge */
        wr_burst(7'h03, 4);
        wr(7'h0A, 8'd2);                       /* PARCELS: three words */
        capture_parcels = 1'b1;
        parcels_seen = 0;

        fork
            wr(7'h01, 8'h10);                  /* CTRL: start block write */
            begin
                /* The command frame's acknowledge, then one per parcel. */
                for (p = 0; p < 4; p = p + 1) begin
                    wait (dut.u_rx.dat_oe == 1'b0);
                    target_drive = 1'b1;
                    send_ack;
                    target_drive = 1'b0;
                    /* Wait for the line to come back, or the same window is
                     * answered twice. */
                    wait (dut.u_rx.dat_oe == 1'b1);
                end
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        capture_parcels = 1'b0;

        check("three parcels went out", parcels_seen, 32'd3);
        check("parcel 0", parcel_seen[0], 32'h11223344);
        check("parcel 1", parcel_seen[1], 32'h55667788);
        check("parcel 2", parcel_seen[2], 32'hDEADBEEF);
        check("block write not timed out", {31'd0, scratch[2]}, 32'd0);

        /*
         * ---- fast mode: a bit every fabric clock ----
         *
         * DAP0 comes from the pad's DDR register, high in the second half of
         * each cycle.  Fast mode sends raw frames only, so the frames are
         * built here (frame_of, checked against the known wire words): sync
         * with a reply, a payload frame, a block read and a block write.
         */
        $display("fast mode: sync, a bit per fabric clock");
        check("frame_of(sync)", frame_of(5'h10, 6'd63, 64'd0, 0), 32'h09FE1);
        check("frame_of(client_set(1))", frame_of(5'h1C, 6'd3, 64'd1, 3), 32'h1B10F9);
        wr(7'h0B, 8'h48);                      /* FLAGS: fast, raw frame */
        rd(7'h0B, scratch);
        check("FLAGS reads back fast", {24'd0, scratch}, 32'h48);
        /* This bench's round trip: a clock's bit is sampled two cycles on. */
        wr(7'h0F, `FAST_LAG << 6);
        rd(7'h0F, scratch);
        check("SKEW reads back LAG", {24'd0, scratch}, `FAST_LAG << 6);
        load_raw(frame_of(5'h10, 6'd63, 64'd0, 0), 19);
        wr(7'h06, 8'd32);

        sent_bits = 0;
        sent      = 96'd0;

        fork
            wr(7'h01, 8'h01);
            begin
                wait (dut.u_rx.dat_oe == 1'b0);
                target_drive = 1'b1;
                send_parcel(32'h5A5AC33C, 1'b1);
                target_drive = 1'b0;
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("fast crc_ok", scratch[4], 1'b1);
        check("fast not timed out", scratch[2], 1'b0);
        rd(7'h20, b0); rd(7'h21, b1); rd(7'h22, b2); rd(7'h23, b3);
        check("fast reply", {b3, b2, b1, b0}, 32'h5A5AC33C);
        check("fast sync clocks", sent_bits - 2, 19);
        check("fast sync word", (sent >> 2) & 96'h7FFFF, 32'h09FE1);

        $display("fast mode: client_set(1)");
        load_raw(frame_of(5'h1C, 6'd3, 64'd1, 3), 22);
        wr(7'h06, 8'd0);

        sent_bits = 0;
        sent      = 96'd0;

        fork
            wr(7'h01, 8'h01);
            begin
                wait (dut.u_rx.dat_oe == 1'b0);
                target_drive = 1'b1;
                send_ack;
                target_drive = 1'b0;
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("fast acknowledge not timed out", scratch[2], 1'b0);
        check("fast frame length", sent_bits - 2, 22);
        check("fast client_set word", (sent >> 2) & 96'h3FFFFF, 32'h1B10F9);

        /*
         * Sampling trails DAP0 by two clocks, two bits here.  With LAG right
         * the target still gets exactly each parcel's clocks, so one busy bit
         * before the next start bit is enough.
         */
        $display("fast mode: block read, 3 parcels");
        busy_bits = `FAST_BUSY;
        wr(7'h01, 8'h08);
        wr(7'h0A, 8'd2);
        load_raw(frame_of(5'h0A, 6'd40, 64'h0C, 40), 59);

        fork
            wr(7'h01, 8'h02);
            begin
                for (p = 0; p < 3; p = p + 1) begin
                    wait (dut.u_rx.dat_oe == 1'b0);
                    target_drive = 1'b1;
                    send_parcel(32'h0BADF00D + 32'h01010101 * p, (p == 2));
                    target_drive = 1'b0;
                    @(posedge clk);
                end
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("fast block not timed out", scratch[2], 1'b0);
        check("fast block crc_ok", scratch[4], 1'b1);
        ss = 1'b0; #HALF;
        spi_byte(8'h40, scratch);
        spi_byte(8'h00, scratch);              /* the dummy */
        for (p = 0; p < 3; p = p + 1) begin
            spi_byte(8'h00, b0);
            spi_byte(8'h00, b1);
            spi_byte(8'h00, b2);
            spi_byte(8'h00, b3);
            check("fast parcel", {b3, b2, b1, b0}, 32'h0BADF00D + 32'h01010101 * p);
        end
        #HALF; ss = 1'b1; #(HALF*4);
        busy_bits = 1;

        $display("fast mode: block write, 3 words");
        wr(7'h01, 8'h20);
        burst[0] = 8'h01; burst[1] = 8'h02; burst[2] = 8'h03; burst[3] = 8'h04;
        burst[4] = 8'hA5; burst[5] = 8'h5A; burst[6] = 8'hFF; burst[7] = 8'h00;
        burst[8] = 8'h0D; burst[9] = 8'hF0; burst[10] = 8'hAD; burst[11] = 8'h0B;
        wr_burst(7'h48, 12);
        load_raw(frame_of(5'h09, 6'd40, 64'h40000000, 40), 59);
        wr(7'h06, 8'd0);
        wr(7'h0A, 8'd2);
        capture_parcels = 1'b1;
        parcels_seen = 0;

        fork
            wr(7'h01, 8'h10);
            begin
                for (p = 0; p < 4; p = p + 1) begin
                    wait (dut.u_rx.dat_oe == 1'b0);
                    target_drive = 1'b1;
                    send_ack;
                    target_drive = 1'b0;
                    wait (dut.u_rx.dat_oe == 1'b1);
                end
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        capture_parcels = 1'b0;
        check("fast: three parcels went out", parcels_seen, 32'd3);
        check("fast parcel 0", parcel_seen[0], 32'h04030201);
        check("fast parcel 1", parcel_seen[1], 32'h00FF5AA5);
        check("fast parcel 2", parcel_seen[2], 32'h0BADF00D);
        check("fast block write not timed out", {31'd0, scratch[2]}, 32'd0);

        /* Wide and fast: two bits a clock, sync's line words as tb_dap_frame
         * has them, interleaved into one raw frame.  Only the transmit side
         * is checked; DAP2 floats here. */
        $display("fast and wide: sync goes out as pairs");
        wr(7'h0B, 8'h4C);                      /* FLAGS: fast, raw, wide */
        load_raw(interleave(11'h179, 11'h371), 22);
        wr(7'h06, 8'd0);

        sent_bits = 0;
        sent      = 96'd0;
        sent2     = 96'd0;

        fork
            wr(7'h01, 8'h01);
            begin
                wait (dut.u_rx.dat_oe == 1'b0);
                target_drive = 1'b1;
                send_ack;
                target_drive = 1'b0;
            end
        join

        scratch = 8'h00;
        while (!scratch[1]) rd(7'h00, scratch);
        check("fast wide sync clocks", sent_bits - 2, 11);
        check("fast wide sync dap1", (sent  >> 2) & 96'h7FF, 32'h179);
        check("fast wide sync dap2", (sent2 >> 2) & 96'h7FF, 32'h371);

        wr(7'h0B, 8'h00);
        wr(7'h0F, 8'h00);

        $display("");
        if (errors == 0) $display("PASSED (0 failures)");
        else             $display("FAILED (%0d failure%s)", errors, (errors == 1) ? "" : "s");
        $finish;
    end

    initial begin
        #20000000;
        $display("  at timeout: q %0d, fifo %0d, room %0d, parcels_left %0d, rx state %0d, oe %0d",
                 dut.q, dut.fifo_count, dut.fifo_room, dut.parcels_left, dut.u_rx.state,
                 dut.u_rx.dat_oe);
        $display("FAILED (timeout)");
        $finish;
    end
endmodule

`default_nettype wire
