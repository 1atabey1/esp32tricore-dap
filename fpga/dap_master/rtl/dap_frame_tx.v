/*
 * Assemble and clock out one DAP frame, all fields LSB first:
 *
 *     lead (0s) | start bit (1) | CMD (5) | LEN (6) | DATA (LEN) | CRC6 | trailing zero
 *
 * The CRC covers CMD, LEN and DATA.  First bit on the wire is bit 0: sync is
 * 0x09FE1 (tb_dap_frame checks it).  The device needs the low lead-in before
 * every frame, sync included.  dap0 is generated here; the target latches on
 * the rising edge.
 */

`default_nettype none

module dap_frame_tx #(
    /* Clock divider: dap0 period is 2*(DIV+1) system clocks, so the bit rate
     * is sysclk / (2*(DIV+1)).  Runtime-settable by the register interface. */
    parameter integer DIV_WIDTH = 8
) (
    input  wire                  clk,
    input  wire                  rst,

    input  wire [DIV_WIDTH-1:0]  div,
    input  wire                  start,      /* pulse to begin a frame */
    input  wire [4:0]            cmd,
    /* LEN is the wire field; data_bits is how many DATA bits follow.  Sync has
     * LEN 63 and no data. */
    input  wire [5:0]            len,
    input  wire [5:0]            data_bits,
    input  wire [62:0]           data,
    /* Clocks with the line low before the frame proper. */
    input  wire [5:0]            lead,
    /*
     * Wide mode: two bits per DAP0 clock, even bits on DAP1 and odd on DAP2.
     * Latched at start.  Each field's clock count halves, and a field with an
     * odd bit count gets one zero pad bit.
     */
    input  wire                  wide,
    /*
     * Raw frame: `data` holds the whole frame (start bit through trailing
     * zero, bit 0 first, `data_bits` long) and is shifted out as given, with
     * no assembly or CRC.
     */
    input  wire                  raw,
    /*
     * One bit every fabric clock, `div` ignored.  dap0 then means "a clock
     * this cycle": the pad raises DAP0 for the second half of the cycle, with
     * DAP1 changing at the cycle start.  Raw frames only: the CRC is taken
     * half a period before the bit's field ends, and there is no half period
     * here (keeping it out of the field logic is timing).  Latched at start.
     */
    input  wire                  fast,

    output reg                   busy,
    output reg                   done,       /* one-cycle pulse at the end */

    /* To the pads.  dat_oe low hands the line to the target. */
    output reg                   dap0,
    /* What dap0 holds next cycle, and the same for normal mode only (a few
     * registers deep), for the pad's two output registers. */
    output reg                   dap0_d,
    output reg                   dap0_next,
    output reg                   dap1,
    output reg                   dat_oe,
    /* DAP2 carries the odd bits, driven only in wide mode. */
    output reg                   dap2,
    /* Held for the whole frame, not decoded from the state: toggling it at
     * field boundaries takes the board down. */
    output reg                   dat2_oe
);
    localparam [3:0] S_IDLE  = 4'd0,
                     S_LEAD  = 4'd1,   /* low clocks before the frame */
                     S_START = 4'd2,
                     S_CMD   = 4'd3,
                     S_LEN   = 4'd4,
                     S_DATA  = 4'd5,
                     S_CRC   = 4'd6,
                     S_TRAIL = 4'd7,
                     S_RAW   = 4'd8;   /* the host supplied the whole frame */

    reg [3:0]           state;
    reg [5:0]           index;      /* bit position within the current field */
    reg [DIV_WIDTH-1:0] tick;
    reg                 phase;      /* 0 = first half (dap0 low), 1 = second */

    reg [4:0]  cmd_r;
    reg [5:0]  len_r;
    reg [5:0]  nbits_r;
    reg [5:0]  lead_r;
    /* Last index of each variable-length field, computed at load (timing). */
    reg [5:0]  nbits_last;
    reg [5:0]  lead_last;
    reg [62:0] data_r;
    reg        wide_r;
    reg        raw_r;
    reg        fast_r;
    /* A half period of one clock; always so in fast mode. */
    wire       div_zero = fast_r | (div == {DIV_WIDTH{1'b0}});

    /* Field lengths in clock periods: wide mode halves them (CMD plus pad is 3). */
    wire [5:0] cmd_last = wide_r ? 6'd2 : 6'd4;
    wire [5:0] f6_last  = wide_r ? 6'd2 : 6'd5;   /* LEN and CRC, both six */

    /*
     * at_last: index is the current field's last position, registered so
     * no field-end compare sits on the advance paths (timing).  It is
     * computed at each advance from the field the index stays in, or, at a
     * field boundary, from the field that follows.  nb_zero: a one-clock
     * payload, for a field that starts already at its end.
     */
    reg        at_last;
    reg        nb_zero;
    reg  [5:0] last_cur;
    always @(*) begin
        case (state)
            S_LEAD:         last_cur = lead_last;
            S_CMD:          last_cur = cmd_last;
            S_LEN, S_CRC:   last_cur = f6_last;
            default:        last_cur = nbits_last;  /* S_DATA, S_RAW */
        endcase
    end
    wire [5:0] nbits_last_in = wide ? ((data_bits - 1'b1) >> 1) : (data_bits - 1'b1);
    /* The finished CRC, latched when the payload runs out and then shifted out
     * like every other field. */
    reg [5:0]  crc_sr;

    /* The bit currently being presented, and whether the CRC should eat it.
     * In wide mode that is a pair: cur_bit is the even bit, on DAP1. */
    reg        cur_bit;
    reg        cur_bit2;
    reg        crc_en;
    reg        crc_rst;
    wire [5:0] crc;

    dap_crc6_gen u_crc (
        .clk    (clk),
        .rst    (crc_rst),
        .en     (crc_en),
        .bit_in (cur_bit), .bit_in2 (cur_bit2), .wide (wide_r),
        .crc    (crc)
    );

    /* Half-period strobe, registered (timing). */
    reg tick_done;

    /* The bottom bit of the current field; fields shift right as they go out
     * (a 63:1 index mux does not meet timing). */
    always @(*) begin
        case (state)
            /* The start bit goes out on both lines; wide mode aligns on it. */
            S_START: begin cur_bit = 1'b1;      cur_bit2 = 1'b1;      end
            S_CMD:   begin cur_bit = cmd_r[0];  cur_bit2 = cmd_r[1];  end
            S_LEN:   begin cur_bit = len_r[0];  cur_bit2 = len_r[1];  end
            S_DATA:  begin cur_bit = data_r[0]; cur_bit2 = data_r[1]; end
            S_CRC:   begin cur_bit = crc_sr[0]; cur_bit2 = crc_sr[1]; end
            /* No CRC for a raw frame; these are unused. */
            S_RAW:   begin cur_bit = data_r[0]; cur_bit2 = data_r[1]; end
            /* lead-in and trailing zero */
            default: begin cur_bit = 1'b0;      cur_bit2 = 1'b0;      end
        endcase
    end

    /*
     * The bit that becomes current at the next falling edge, so DAP1 changes
     * with DAP0 falling and the whole low phase is setup (needed at div 0).
     * Field boundaries mirror the state advance below.
     */
    reg next_bit;
    reg next_bit2;

    /* Next slot within a field: one position on narrow, two on wide. */
    wire cmd_n0  = wide_r ? cmd_r[2]  : cmd_r[1];
    wire cmd_n1  = cmd_r[3];
    wire len_n0  = wide_r ? len_r[2]  : len_r[1];
    wire len_n1  = len_r[3];
    wire data_n0 = wide_r ? data_r[2] : data_r[1];
    wire data_n1 = data_r[3];
    wire crc_n0  = wide_r ? crc_sr[2] : crc_sr[1];
    wire crc_n1  = crc_sr[3];

    always @(*) begin
        case (state)
            S_LEAD: begin
                /* After the lead-in: the start bit, or bit 0/1 of a raw frame. */
                next_bit  = at_last
                          ? (raw_r ? data_r[0] : 1'b1) : 1'b0;
                next_bit2 = at_last
                          ? (raw_r ? data_r[1] : 1'b1) : 1'b0;
            end
            S_START: begin
                next_bit  = cmd_r[0];
                next_bit2 = cmd_r[1];
            end
            S_CMD: begin
                next_bit  = at_last ? len_r[0] : cmd_n0;
                next_bit2 = at_last ? len_r[1] : cmd_n1;
            end
            S_LEN: begin
                next_bit  = at_last
                          ? ((nbits_r == 6'd0) ? crc[0] : data_r[0])
                          : len_n0;
                next_bit2 = at_last
                          ? ((nbits_r == 6'd0) ? crc[1] : data_r[1])
                          : len_n1;
            end
            S_DATA: begin
                next_bit  = at_last ? crc[0] : data_n0;
                next_bit2 = at_last ? crc[1] : data_n1;
            end
            S_RAW: begin
                /* The host supplied the trailing zero; then the idle line. */
                next_bit  = at_last ? 1'b0 : data_n0;
                next_bit2 = at_last ? 1'b0 : data_n1;
            end
            S_CRC: begin
                next_bit  = at_last ? 1'b0 : crc_n0;
                next_bit2 = at_last ? 1'b0 : crc_n1;
            end
            default: begin                 /* the trailing zero, then idle */
                next_bit  = 1'b0;
                next_bit2 = 1'b0;
            end
        endcase
    end

    /* The CRC eats CMD, LEN and DATA only, and only once per bit - on the
     * transition into the second half of the bit period. */
    always @(*) begin
        crc_en = 1'b0;
        if (phase == 1'b0 && tick_done) begin
            case (state)
                S_CMD, S_LEN, S_DATA: crc_en = 1'b1;
                default:              crc_en = 1'b0;   /* S_RAW included */
            endcase
        end
    end

    /* The last bit of the frame is on the wire: the next advance ends it. */
    wire ending = (state == S_TRAIL) || (state == S_RAW && at_last);

    /*
     * DAP0's next value.  Normal mode: low in the first half of the period,
     * high in the second.  Fast mode: high for every cycle with a bit on the
     * wire.
     */
    always @(*) begin
        if (rst)
            dap0_d = 1'b0;
        else if (state == S_IDLE)
            dap0_d = start & fast;
        else if (!tick_done)
            dap0_d = dap0;
        else if (phase == 1'b0)
            dap0_d = 1'b1;
        else
            dap0_d = fast_r & ~ending;
    end

    /* The same for normal mode only, for the pad's first-half register (fast
     * mode holds that half low): a few registers deep (timing). */
    always @(*) begin
        if (rst || state == S_IDLE)
            dap0_next = 1'b0;
        else if (!tick_done)
            dap0_next = dap0;
        else
            dap0_next = ~phase;
    end

    always @(posedge clk) begin
        crc_rst <= 1'b0;
        done    <= 1'b0;
        dap0    <= dap0_d;

        /* index is loaded at the start of every frame, so it is not reset. */
        if (rst) begin
            state   <= S_IDLE;
            busy    <= 1'b0;
            fast_r  <= 1'b0;
            dap1    <= 1'b1;      /* parked idle high, probe driving */
            dat_oe  <= 1'b1;
            dap2    <= 1'b1;
            dat2_oe <= 1'b0;      /* released until a wide frame claims it */
            wide_r  <= 1'b0;
            tick      <= {DIV_WIDTH{1'b0}};
            tick_done <= (div == {DIV_WIDTH{1'b0}});
            phase     <= 1'b0;
        end else if (state == S_IDLE) begin
            if (start) begin
                cmd_r   <= cmd;
                len_r   <= len;
                nbits_r    <= data_bits;
                /* One clock per pair in wide mode; an odd payload gets a
                 * zero pad bit from data_r. */
                nbits_last <= nbits_last_in;
                /* One clock: nbits_last_in == 0, without its subtract. */
                nb_zero    <= (data_bits == 6'd1) || (wide && data_bits == 6'd2);
                lead_r     <= lead;
                lead_last  <= lead - 1'b1;
                data_r  <= data;
                crc_rst <= 1'b1;
                busy    <= 1'b1;
                dat_oe  <= 1'b1;
                wide_r  <= wide;
                raw_r   <= raw;
                fast_r  <= fast;
                dat2_oe <= wide;
                /* Zero lead clocks means straight into the frame. */
                state   <= (lead == 6'd0) ? (raw ? S_RAW : S_START) : S_LEAD;
                at_last  <= (lead != 6'd0) ? (lead == 6'd1)
                          : (raw && ((data_bits == 6'd1) || (wide && data_bits == 6'd2)));
                /* The first bit has to be on the wire before the first rising
                 * edge, which is a whole low phase away. */
                dap1    <= (lead == 6'd0) ? (raw ? data[0] : 1'b1) : 1'b0;
                dap2    <= (lead == 6'd0) ? (raw ? data[1] : 1'b1) : 1'b0;
                index     <= 6'd0;
                tick      <= {DIV_WIDTH{1'b0}};
                tick_done <= fast | (div == {DIV_WIDTH{1'b0}});
                /* Fast mode stays in the second half: an advance each clock,
                 * without fast_r on the enables (timing). */
                phase     <= fast;
            end
        end else begin
            /* One bit per period: present it with dap0 low, then raise dap0;
             * DAP1 changes only at the falling edge.  Fast mode skips the
             * first half: a bit per clock, the pad making the edges. */
            if (!tick_done) begin
                tick      <= tick + 1'b1;
                tick_done <= (tick + 1'b1 == div);
            end else begin
                tick      <= {DIV_WIDTH{1'b0}};
                tick_done <= div_zero;

                if (phase == 1'b0) begin
                    phase <= 1'b1;
                end else begin
                    phase <= fast_r;
                    dap1  <= next_bit;   /* changes with the falling edge */
                    dap2  <= next_bit2;

                    /* Advance to the next bit, and the next field when this
                     * one runs out. */
                    if (at_last || state == S_START) begin
                        case (state)
                            S_LEAD:  at_last <= raw_r && nb_zero;
                            S_LEN:   at_last <= (nbits_r != 6'd0) && nb_zero;
                            default: at_last <= 1'b0;
                        endcase
                    end else begin
                        at_last <= (index + 1'b1 == last_cur);
                    end

                    case (state)
                        S_LEAD: begin
                            if (at_last) begin
                                state <= raw_r ? S_RAW : S_START;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_START: begin
                            state <= S_CMD;
                            index <= 6'd0;
                        end
                        S_CMD: begin
                            cmd_r <= wide_r ? {2'b0, cmd_r[4:2]}
                                            : {1'b0, cmd_r[4:1]};
                            if (at_last) begin
                                state <= S_LEN;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_LEN: begin
                            len_r <= wide_r ? {2'b0, len_r[5:2]}
                                            : {1'b0, len_r[5:1]};
                            if (at_last) begin
                                /* No data (sync): straight to the CRC.  The
                                 * generator took this bit on phase 0, so
                                 * `crc` is final. */
                                state  <= (nbits_r == 6'd0) ? S_CRC : S_DATA;
                                crc_sr <= crc;
                                index  <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_DATA: begin
                            data_r <= wide_r ? {2'b0, data_r[62:2]}
                                             : {1'b0, data_r[62:1]};
                            if (at_last) begin
                                state  <= S_CRC;
                                crc_sr <= crc;
                                index  <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_CRC: begin
                            crc_sr <= wide_r ? {2'b0, crc_sr[5:2]}
                                             : {1'b0, crc_sr[5:1]};
                            if (at_last) begin
                                state <= S_TRAIL;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_RAW: begin
                            data_r <= wide_r ? {2'b0, data_r[62:2]}
                                             : {1'b0, data_r[62:1]};
                            if (at_last) begin
                                state <= S_IDLE;
                                busy  <= 1'b0;
                                done  <= 1'b1;
                                index <= 6'd0;
                            end else begin
                                index <= index + 1'b1;
                            end
                        end
                        S_TRAIL: begin
                            state <= S_IDLE;
                            busy  <= 1'b0;
                            done  <= 1'b1;
                        end
                        default: state <= S_IDLE;
                    endcase
                end
            end
        end
    end
endmodule

`default_nettype wire
