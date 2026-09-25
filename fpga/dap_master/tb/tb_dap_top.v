/*
 * The whole master, driven over SPI the way the ESP32 drives it, against a
 * fake target: single frames, a payload frame, block read and block write.
 */

`timescale 1ns / 1ps
`default_nettype none

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

    /* What went out on the wire, sampled at DAP0 rising, transmitter only. */
    reg [95:0] sent;
    integer    sent_bits;
    reg        dap0_d;

    always @(posedge clk) begin
        dap0_d <= dap0;
        if (!rst && dap0 && !dap0_d && dut.u_tx.busy) begin
            sent[sent_bits] <= dap1;
            sent_bits       <= sent_bits + 1;
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
    integer    parcels_seen;
    reg [5:0]  parcel_bit;
    reg [31:0] parcel_acc;
    reg        in_parcel;

    always @(posedge clk) begin
        if (rst) begin
            parcels_seen <= 0;
            parcel_bit   <= 6'd0;
            in_parcel    <= 1'b0;
        /* Parcels only; the command frame's start bit must not count. */
        end else if (dap0 && !dap0_d && dut.tx_parcel && capture_parcels) begin
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
