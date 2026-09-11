#include "gdb_rsp.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>

#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"

static const char *TAG = "GDB_RSP";

/* A 1 kB payload covers `g` (35 registers, 280 hex digits) with room to spare,
 * and bounds what one malformed packet can cost. */
#define RSP_BUF_BYTES       1200
#define RSP_MAX_MEM_BYTES   512

/* How long to wait for a core to stop when GDB asks it to. */
#define RSP_HALT_TIMEOUT_MS 1000
#define RSP_STEP_TIMEOUT_MS 500

/* The Cerberus trigger line used to drive break-in.  Any of 1..7 works; this
 * one is simply not otherwise spoken for on this bench. */
#define RSP_HALT_LINE       1

/* Signals GDB understands.  TRAP is what a breakpoint or a completed step
 * reports; INT is a user interrupt. */
#define SIGTRAP 5
#define SIGINT  2

typedef struct {
    int      sock;
    bool     no_ack;          /* QStartNoAckMode agreed */
    bool     running;         /* the target was resumed and we are watching it */
    int      cur_core;        /* the core `g`, `m` and friends address */
    uint32_t last_signal;
} rsp_session_t;

static TaskHandle_t s_task;
static volatile bool s_should_run;
static volatile bool s_connected;
static int  s_listen_sock = -1;
static uint16_t s_port = GDB_RSP_DEFAULT_PORT;

static char s_in[RSP_BUF_BYTES];
static char s_out[RSP_BUF_BYTES];

/* ------------------------------------------------------------------------ */
/* Target description                                                        */
/* ------------------------------------------------------------------------ */
/*
 * Served over qXfer:features:read, which is what makes the `g` packet layout
 * unambiguous: GDB numbers registers in the order they appear here, so the
 * client cannot disagree with us about where PC is.  The order matches
 * tricore.h: D0..D15, A0..A15, PCXI, PSW, PC.
 *
 * A4 is the small-data base and A10/A11 are the stack pointer and return
 * address in the TriCore ABI, so they are typed and named accordingly - that is
 * what lets GDB unwind and show a stack at all.
 */
static const char k_target_xml[] =
    "<?xml version=\"1.0\"?>"
    "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">"
    "<target version=\"1.0\">"
    "<architecture>tricore</architecture>"
    "<feature name=\"org.gnu.gdb.tricore.core\">"
    "<reg name=\"d0\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d1\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d2\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d3\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d4\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d5\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d6\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d7\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d8\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d9\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d10\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d11\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d12\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d13\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d14\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d15\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"a0\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a1\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a2\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a3\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a4\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a5\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a6\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a7\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a8\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a9\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a10\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a11\" bitsize=\"32\" type=\"code_ptr\"/>"
    "<reg name=\"a12\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a13\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a14\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a15\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"pcxi\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"psw\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>"
    "</feature>"
    "</target>";

/* ------------------------------------------------------------------------ */
/* Hex helpers                                                               */
/* ------------------------------------------------------------------------ */

static const char k_hex[] = "0123456789abcdef";

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    return -1;
}

static size_t hex_encode(char *dst, const uint8_t *src, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        dst[2 * i]     = k_hex[src[i] >> 4];
        dst[2 * i + 1] = k_hex[src[i] & 0xF];
    }
    return 2 * len;
}

static size_t hex_decode(uint8_t *dst, const char *src, size_t max)
{
    size_t n = 0;
    while (n < max) {
        const int hi = hex_value(src[2 * n]);
        const int lo = hex_value(src[2 * n + 1]);
        if (hi < 0 || lo < 0) {
            break;
        }
        dst[n++] = (uint8_t)((hi << 4) | lo);
    }
    return n;
}

/* A 32-bit value as GDB wants it in a `g` packet: little-endian byte order,
 * each byte as two hex digits. */
static size_t hex_u32_le(char *dst, uint32_t value)
{
    const uint8_t bytes[4] = {
        (uint8_t)value, (uint8_t)(value >> 8),
        (uint8_t)(value >> 16), (uint8_t)(value >> 24),
    };
    return hex_encode(dst, bytes, 4);
}

static uint32_t parse_hex(const char **p)
{
    uint32_t value = 0;
    int digit;
    while ((digit = hex_value(**p)) >= 0) {
        value = (value << 4) | (uint32_t)digit;
        (*p)++;
    }
    return value;
}

/* ------------------------------------------------------------------------ */
/* Packet transport                                                          */
/* ------------------------------------------------------------------------ */

static bool sock_send_all(int sock, const char *data, size_t len)
{
    while (len) {
        const int n = send(sock, data, len, 0);
        if (n <= 0) {
            return false;
        }
        data += n;
        len  -= (size_t)n;
    }
    return true;
}

static bool send_packet(rsp_session_t *s, const char *payload, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + (uint8_t)payload[i]);
    }

    char header = '$';
    char trailer[3] = { '#', k_hex[sum >> 4], k_hex[sum & 0xF] };

    if (!sock_send_all(s->sock, &header, 1) ||
        !sock_send_all(s->sock, payload, len) ||
        !sock_send_all(s->sock, trailer, sizeof(trailer))) {
        return false;
    }
    return true;
}

static bool send_str(rsp_session_t *s, const char *payload)
{
    return send_packet(s, payload, strlen(payload));
}

static bool send_ok(rsp_session_t *s)    { return send_str(s, "OK"); }
static bool send_empty(rsp_session_t *s) { return send_str(s, ""); }

static bool send_error(rsp_session_t *s, int code)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "E%02X", code & 0xFF);
    return send_str(s, buf);
}

/*
 * Read one packet.  Returns its length, -1 on a dead socket, or -2 when the
 * client sent an interrupt (0x03) rather than a packet - which is how Ctrl-C
 * arrives and is the only byte that means anything outside a packet.
 */
static int read_packet(rsp_session_t *s, char *buf, size_t max, uint32_t timeout_ms)
{
    struct timeval tv = {
        .tv_sec  = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    setsockopt(s->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    enum { WAIT_START, IN_BODY, CSUM_HI, CSUM_LO } state = WAIT_START;
    size_t  len = 0;
    uint8_t sum = 0;
    uint8_t want = 0;

    for (;;) {
        char c;
        const int n = recv(s->sock, &c, 1, 0);
        if (n == 0) {
            return -1;                          /* the client closed */
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return -3;                      /* nothing waiting */
            }
            return -1;
        }

        switch (state) {
        case WAIT_START:
            if (c == '$') {
                state = IN_BODY;
                len = 0;
                sum = 0;
            } else if (c == 0x03) {
                return -2;                      /* interrupt */
            }
            /* '+' and '-' outside a packet are acks; nothing to do with them. */
            break;

        case IN_BODY:
            if (c == '#') {
                state = CSUM_HI;
            } else if (len + 1 < max) {
                buf[len++] = c;
                sum = (uint8_t)(sum + (uint8_t)c);
            } else {
                state = WAIT_START;             /* oversized: drop it */
            }
            break;

        case CSUM_HI:
            want = (uint8_t)(hex_value(c) << 4);
            state = CSUM_LO;
            break;

        case CSUM_LO:
            want = (uint8_t)(want | hex_value(c));
            buf[len] = '\0';
            if (!s->no_ack) {
                const char ack = (want == sum) ? '+' : '-';
                sock_send_all(s->sock, &ack, 1);
            }
            if (want == sum) {
                return (int)len;
            }
            state = WAIT_START;                 /* bad checksum: wait for a resend */
            break;
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Stop reporting                                                            */
/* ------------------------------------------------------------------------ */

/*
 * Why the core stopped, in the form GDB wants.  The trigger accumulator says
 * which comparator tripped, and the slot says whether that was a breakpoint or
 * a watchpoint - reading it is what turns "stopped" into something the user can
 * act on.  It clears on read, so it is read exactly once per stop.
 */
static bool send_stop_reply(rsp_session_t *s, int core, int signal)
{
    char     reply[128];
    size_t   n = 0;
    uint32_t acc = 0;

    n += (size_t)snprintf(reply + n, sizeof(reply) - n, "T%02x", signal & 0xFF);

    if (tricore_trigger_acc(core, &acc) == ESP_OK && acc != 0) {
        for (int slot = 0; slot < TRICORE_NUM_TRIGGERS; slot++) {
            if (!(acc & (1u << slot))) {
                continue;
            }
            const tricore_bp_t *bp = tricore_bp_of_slot(core, slot);
            if (bp == NULL) {
                continue;
            }
            if (bp->kind == TRICORE_BP_WATCH) {
                const char *kind = bp->on_read
                                 ? (bp->on_write ? "awatch" : "rwatch")
                                 : "watch";
                n += (size_t)snprintf(reply + n, sizeof(reply) - n,
                                      "%s:%" PRIx32 ";", kind, bp->addr);
            } else if (bp->kind == TRICORE_BP_USER) {
                n += (size_t)snprintf(reply + n, sizeof(reply) - n, "hwbreak:;");
            }
            break;
        }
    }

    n += (size_t)snprintf(reply + n, sizeof(reply) - n, "thread:%x;", core + 1);
    s->last_signal = (uint32_t)signal;
    return send_packet(s, reply, n);
}

/* ------------------------------------------------------------------------ */
/* Packet handlers                                                           */
/* ------------------------------------------------------------------------ */

static int thread_to_core(const char *p)
{
    /* Thread IDs are the core index plus one; 0 and -1 mean "any". */
    const long id = strtol(p, NULL, 16);
    if (id <= 0) {
        return tricore_core_index(0);
    }
    return (int)id - 1;
}

static void handle_query(rsp_session_t *s, const char *packet)
{
    if (strncmp(packet, "qSupported", 10) == 0) {
        char reply[160];
        snprintf(reply, sizeof(reply),
                 "PacketSize=%X;qXfer:features:read+;hwbreak+;swbreak-;"
                 "QStartNoAckMode+;vContSupported+;multiprocess-",
                 RSP_BUF_BYTES - 32);
        send_str(s, reply);
        return;
    }

    if (strncmp(packet, "qXfer:features:read:", 20) == 0) {
        const char *p = strchr(packet + 20, ':');
        if (p == NULL) {
            send_error(s, 22);
            return;
        }
        p++;
        const uint32_t offset = parse_hex(&p);
        if (*p == ',') { p++; }
        uint32_t length = parse_hex(&p);

        const size_t total = sizeof(k_target_xml) - 1;
        if (offset >= total) {
            send_str(s, "l");
            return;
        }
        if (length > RSP_BUF_BYTES - 32) {
            length = RSP_BUF_BYTES - 32;
        }
        size_t take = total - offset;
        if (take > length) {
            take = length;
        }
        s_out[0] = (offset + take < total) ? 'm' : 'l';
        memcpy(s_out + 1, k_target_xml + offset, take);
        send_packet(s, s_out, take + 1);
        return;
    }

    if (strcmp(packet, "QStartNoAckMode") == 0) {
        send_ok(s);
        s->no_ack = true;           /* only after the OK is acknowledged */
        return;
    }

    if (strcmp(packet, "qC") == 0) {
        char reply[24];
        snprintf(reply, sizeof(reply), "QC%x", s->cur_core + 1);
        send_str(s, reply);
        return;
    }

    if (strcmp(packet, "qfThreadInfo") == 0) {
        char   reply[64];
        size_t n = 0;
        reply[n++] = 'm';
        for (int i = 0; i < tricore_core_count(); i++) {
            n += (size_t)snprintf(reply + n, sizeof(reply) - n, "%s%x",
                                  i ? "," : "", tricore_core_index(i) + 1);
        }
        send_packet(s, reply, n);
        return;
    }
    if (strcmp(packet, "qsThreadInfo") == 0) {
        send_str(s, "l");
        return;
    }

    if (strncmp(packet, "qThreadExtraInfo,", 17) == 0) {
        const char *p = packet + 17;
        const int   core = thread_to_core(p);
        char        text[48];
        const int   len = snprintf(text, sizeof(text), "CPU%d, %s", core,
                                   tricore_is_halted(core) ? "halted" : "running");
        const size_t n = hex_encode(s_out, (const uint8_t *)text, (size_t)len);
        send_packet(s, s_out, n);
        return;
    }

    if (strcmp(packet, "qAttached") == 0) {
        /* 1: the target was already running and must be left running on detach.
         * That is exactly the situation here - this probe never started it. */
        send_str(s, "1");
        return;
    }

    send_empty(s);
}

static void handle_memory_read(rsp_session_t *s, const char *packet)
{
    const char *p = packet + 1;
    const uint32_t addr = parse_hex(&p);
    if (*p != ',') {
        send_error(s, 22);
        return;
    }
    p++;
    uint32_t len = parse_hex(&p);
    if (len > RSP_MAX_MEM_BYTES) {
        len = RSP_MAX_MEM_BYTES;
    }

    uint8_t buf[RSP_MAX_MEM_BYTES];
    if (tricore_read_mem(addr, buf, len) != ESP_OK) {
        send_error(s, 14);                  /* EFAULT, as GDB reads it */
        return;
    }
    const size_t n = hex_encode(s_out, buf, len);
    send_packet(s, s_out, n);
}

static void handle_memory_write(rsp_session_t *s, const char *packet)
{
    const char *p = packet + 1;
    const uint32_t addr = parse_hex(&p);
    if (*p != ',') {
        send_error(s, 22);
        return;
    }
    p++;
    uint32_t len = parse_hex(&p);
    if (*p != ':') {
        send_error(s, 22);
        return;
    }
    p++;
    if (len > RSP_MAX_MEM_BYTES) {
        send_error(s, 22);
        return;
    }

    uint8_t buf[RSP_MAX_MEM_BYTES];
    if (hex_decode(buf, p, len) != len) {
        send_error(s, 22);
        return;
    }
    if (tricore_write_mem(addr, buf, len) != ESP_OK) {
        send_error(s, 14);
        return;
    }
    send_ok(s);
}

static void handle_read_regs(rsp_session_t *s)
{
    uint32_t regs[TRICORE_NUM_REGS];

    if (!tricore_is_halted(s->cur_core)) {
        /*
         * GDB asks for registers as soon as it connects, and on a running core
         * every one of these reads bus-errors.  An error reply is the honest
         * answer and leaves the session usable for memory.
         */
        send_error(s, 1);
        return;
    }
    if (tricore_read_all_regs(s->cur_core, regs) != ESP_OK) {
        send_error(s, 1);
        return;
    }
    size_t n = 0;
    for (int i = 0; i < TRICORE_NUM_REGS; i++) {
        n += hex_u32_le(s_out + n, regs[i]);
    }
    send_packet(s, s_out, n);
}

static void handle_write_regs(rsp_session_t *s, const char *packet)
{
    const char *p = packet + 1;

    for (int i = 0; i < TRICORE_NUM_REGS; i++) {
        uint8_t bytes[4];
        if (hex_decode(bytes, p, 4) != 4) {
            break;
        }
        const uint32_t value = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                               ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
        tricore_write_reg(s->cur_core, i, value);
        p += 8;
    }
    send_ok(s);
}

static void handle_read_one_reg(rsp_session_t *s, const char *packet)
{
    const char *p = packet + 1;
    const uint32_t regno = parse_hex(&p);
    uint32_t value = 0;

    if (tricore_read_reg(s->cur_core, (int)regno, &value) != ESP_OK) {
        send_error(s, 1);
        return;
    }
    const size_t n = hex_u32_le(s_out, value);
    send_packet(s, s_out, n);
}

static void handle_write_one_reg(rsp_session_t *s, const char *packet)
{
    const char *p = packet + 1;
    const uint32_t regno = parse_hex(&p);
    if (*p != '=') {
        send_error(s, 22);
        return;
    }
    p++;
    uint8_t bytes[4];
    if (hex_decode(bytes, p, 4) != 4) {
        send_error(s, 22);
        return;
    }
    const uint32_t value = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
    if (tricore_write_reg(s->cur_core, (int)regno, value) != ESP_OK) {
        send_error(s, 1);
        return;
    }
    send_ok(s);
}

/*
 * Z/z.  Types 0 and 1 are both served from address triggers - see the header on
 * why software breakpoints are not worth it here - and 2/3/4 are watchpoints.
 *
 * A breakpoint goes on *every* core, not just the focused one.  The comparators
 * are per-core registers, so arming CPU0 does nothing for code CPU2 executes,
 * and nothing in the debug information says which core will run a function.
 */
static void handle_breakpoint(rsp_session_t *s, const char *packet, bool insert)
{
    const char *p = packet + 1;
    const uint32_t type = parse_hex(&p);
    if (*p != ',') {
        send_error(s, 22);
        return;
    }
    p++;
    const uint32_t addr = parse_hex(&p);
    if (*p != ',') {
        send_error(s, 22);
        return;
    }
    p++;
    uint32_t kind = parse_hex(&p);      /* for watchpoints, the length in bytes */
    if (kind == 0) {
        kind = 1;
    }

    int wanted = 0, achieved = 0;
    for (int i = 0; i < tricore_core_count(); i++) {
        const int core = tricore_core_index(i);
        wanted++;

        if (!insert) {
            achieved += tricore_bp_remove(core, addr) ? 1 : 0;
            continue;
        }
        int slot;
        switch (type) {
        case 0:     /* software breakpoint, served as hardware */
        case 1:
            slot = tricore_bp_add(core, addr, TRICORE_BP_USER);
            break;
        case 2:     /* write watchpoint */
            slot = tricore_bp_add_watch(core, addr, kind, false, true);
            break;
        case 3:     /* read watchpoint */
            slot = tricore_bp_add_watch(core, addr, kind, true, false);
            break;
        case 4:     /* access watchpoint */
            slot = tricore_bp_add_watch(core, addr, kind, true, true);
            break;
        default:
            send_empty(s);      /* an unsupported type, not an error */
            return;
        }
        achieved += (slot >= 0) ? 1 : 0;
    }

    if (!insert) {
        send_ok(s);             /* removing something absent is not an error */
        return;
    }
    if (achieved == 0) {
        /*
         * Out of triggers on every core.  E28 rather than a silent OK: a
         * breakpoint GDB thinks it set and the hardware never took is worse
         * than one that visibly failed.
         */
        ESP_LOGW(TAG, "no core could take a breakpoint at 0x%08" PRIX32
                      " - all %d triggers are in use", addr, TRICORE_NUM_TRIGGERS);
        send_error(s, 28);
        return;
    }
    if (achieved < wanted) {
        ESP_LOGW(TAG, "breakpoint at 0x%08" PRIX32 " armed on %d of %d cores",
                 addr, achieved, wanted);
    }
    send_ok(s);
}

/* ------------------------------------------------------------------------ */
/* Execution                                                                 */
/* ------------------------------------------------------------------------ */

/*
 * Resume and watch until something stops, the client interrupts, or the link
 * dies.  Every core is resumed, not just the focused one: a stop halts whatever
 * hit it and leaving the rest parked would be worse than not having stopped
 * them at all.
 */
static void do_continue(rsp_session_t *s)
{
    /*
     * Step off a breakpoint under the PC before resuming.
     *
     * A trigger armed at the current address fires again the instant the core
     * runs, so a plain resume would leave it exactly where it is - and GDB,
     * seeing an immediate stop at the same PC, would continue again, forever.
     * One step moves past it, and tricore_step() takes the trigger down and
     * puts it back around that step.
     */
    for (int i = 0; i < tricore_core_count(); i++) {
        const int core = tricore_core_index(i);
        uint32_t   pc  = 0;

        if (!tricore_is_halted(core) || tricore_read_pc(core, &pc) != ESP_OK) {
            continue;
        }
        const tricore_bp_t *bp = tricore_bp_at(core, pc);
        if (bp != NULL && bp->kind == TRICORE_BP_USER) {
            tricore_step_t stepped;
            tricore_step(core, RSP_STEP_TIMEOUT_MS, &stepped);
        }
    }

    /* The focused core goes first, so the others are not left stopped while it
     * runs ahead into code that expects its siblings to be alive. */
    tricore_request_resume(s->cur_core);
    for (int i = 0; i < tricore_core_count(); i++) {
        const int core = tricore_core_index(i);
        if (core != s->cur_core) {
            tricore_request_resume(core);
        }
    }

    /*
     * Wait for the resume to take before watching for the next stop.  Without
     * this the first poll runs before the core has restarted, reads the halt
     * state that is still set, and reports a stop that already happened - so
     * every continue would return instantly at the same PC.
     *
     * A core that never leaves halt within the window is reported as stopped,
     * which is the truth: something is holding it there.
     */
    const int64_t settle = esp_timer_get_time() + 100 * 1000;
    bool observed_running = false;
    while (esp_timer_get_time() < settle) {
        if (!tricore_is_halted(s->cur_core)) {
            observed_running = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (!observed_running) {
        ESP_LOGW(TAG, "CPU%d did not restart; reporting it as stopped", s->cur_core);
        send_stop_reply(s, s->cur_core, SIGTRAP);
        return;
    }

    s->running = true;
    for (;;) {
        for (int i = 0; i < tricore_core_count(); i++) {
            const int core = tricore_core_index(i);
            if (tricore_is_halted(core)) {
                s->running = false;
                s->cur_core = core;
                send_stop_reply(s, core, SIGTRAP);
                return;
            }
        }

        /* A packet arriving while running is either an interrupt or a client
         * that has given up; nothing else is legal here. */
        const int n = read_packet(s, s_in, sizeof(s_in), 10);
        if (n == -1) {
            s->running = false;
            return;                     /* the client went away */
        }
        if (n == -2) {
            if (tricore_halt(s->cur_core, RSP_HALT_LINE, RSP_HALT_TIMEOUT_MS) != ESP_OK) {
                ESP_LOGW(TAG, "CPU%d did not stop within the timeout", s->cur_core);
            }
            s->running = false;
            send_stop_reply(s, s->cur_core, SIGINT);
            return;
        }
        if (!s_should_run) {
            s->running = false;
            return;
        }
    }
}

static void do_step(rsp_session_t *s)
{
    tricore_step_t result;

    if (!tricore_is_halted(s->cur_core)) {
        send_error(s, 1);
        return;
    }
    if (tricore_step(s->cur_core, RSP_STEP_TIMEOUT_MS, &result) != ESP_OK) {
        ESP_LOGW(TAG, "step on CPU%d failed: %s", s->cur_core, result.note);
        send_stop_reply(s, s->cur_core, SIGTRAP);
        return;
    }
    if (result.note[0]) {
        ESP_LOGI(TAG, "step on CPU%d: %s", s->cur_core, result.note);
    }
    send_stop_reply(s, s->cur_core, SIGTRAP);
}

static void handle_vcont(rsp_session_t *s, const char *packet)
{
    if (strcmp(packet, "vCont?") == 0) {
        /* No 'r' (range stepping): it would need the same successor decoding
         * per instruction and buys nothing over plain 's' here. */
        send_str(s, "vCont;c;C;s;S;t");
        return;
    }
    /*
     * The action list is scanned for the first thing we can act on.  GDB sends
     * per-thread actions, but run control here is per-core and a stop halts the
     * core that hit it, so honouring the first action is the honest reading.
     */
    if (strchr(packet, 's') != NULL) {
        do_step(s);
        return;
    }
    if (strchr(packet, 't') != NULL) {
        tricore_halt(s->cur_core, RSP_HALT_LINE, RSP_HALT_TIMEOUT_MS);
        send_stop_reply(s, s->cur_core, SIGINT);
        return;
    }
    do_continue(s);
}

/* ------------------------------------------------------------------------ */
/* Session                                                                   */
/* ------------------------------------------------------------------------ */

static void serve(rsp_session_t *s)
{
    for (;;) {
        const int n = read_packet(s, s_in, sizeof(s_in), 1000);

        if (n == -1) {
            return;                         /* the client closed */
        }
        if (!s_should_run) {
            return;
        }
        if (n == -3) {
            continue;                       /* idle */
        }
        if (n == -2) {
            /* An interrupt while already stopped: report the stop again. */
            send_stop_reply(s, s->cur_core, SIGINT);
            continue;
        }

        switch (s_in[0]) {
        case 'q':
        case 'Q':
            handle_query(s, s_in);
            break;

        case '?':
            /* GDB's first real question.  A running target is reported as
             * stopped anyway, because RSP has no "still running" answer here -
             * and GDB's next move is to ask for registers, which tells it. */
            send_stop_reply(s, s->cur_core, SIGTRAP);
            break;

        case 'H':
            /* Hc / Hg: which thread the next operations address. */
            if (s_in[1] == 'g' || s_in[1] == 'c') {
                const int core = thread_to_core(s_in + 2);
                if (tricore_core_present(core)) {
                    s->cur_core = core;
                }
            }
            send_ok(s);
            break;

        case 'T': {
            /* Is this thread alive? */
            const int core = thread_to_core(s_in + 1);
            tricore_core_present(core) ? send_ok(s) : send_error(s, 1);
            break;
        }

        case 'g': handle_read_regs(s);            break;
        case 'G': handle_write_regs(s, s_in);     break;
        case 'p': handle_read_one_reg(s, s_in);   break;
        case 'P': handle_write_one_reg(s, s_in);  break;
        case 'm': handle_memory_read(s, s_in);    break;
        case 'M': handle_memory_write(s, s_in);   break;
        case 'Z': handle_breakpoint(s, s_in, true);  break;
        case 'z': handle_breakpoint(s, s_in, false); break;

        case 'c': do_continue(s); break;
        case 's': do_step(s);     break;

        case 'v':
            if (strncmp(s_in, "vCont", 5) == 0) {
                handle_vcont(s, s_in);
            } else {
                send_empty(s);
            }
            break;

        case 'D':
            /* Detach: leave the target as we found it - running, with no
             * triggers of ours left armed to halt it later. */
            for (int i = 0; i < tricore_core_count(); i++) {
                const int core = tricore_core_index(i);
                tricore_bp_clear_kind(core, TRICORE_BP_USER);
                tricore_bp_clear_kind(core, TRICORE_BP_STEP);
                tricore_bp_clear_kind(core, TRICORE_BP_WATCH);
                tricore_bp_clear_kind(core, TRICORE_BP_WATCH_HI);
                tricore_clear_debug_events(core);
                tricore_request_resume(core);
            }
            send_ok(s);
            return;

        case 'k':
            return;                         /* kill: just drop the connection */

        default:
            send_empty(s);                  /* an unknown packet is not an error */
            break;
        }
    }
}

static esp_err_t attach_target(void)
{
    dap_exchange_t x;

    esp_err_t err = dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DAP PHY unavailable on this board");
        return err;
    }
    err = dap_probe_attach(&x, 3);
    if (err != ESP_OK || x.reply != 0xAAAAAAAAu) {
        ESP_LOGE(TAG, "the target did not answer sync");
        return ESP_ERR_INVALID_STATE;
    }
    dap_probe_client_set(1, &x);
    dap_probe_clear_error_state();
    dap_probe_set_rw_mode(true);

    if (dap_probe_enable_ocds() != ESP_OK) {
        ESP_LOGE(TAG, "OCDS did not come up; run control needs it");
        return ESP_ERR_INVALID_STATE;
    }
    err = tricore_discover();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no TriCore cores answered");
        return err;
    }

    /* A previous debugger may have left the halt-after-reset trigger armed, and
     * with it every resume immediately re-halts at the reset vector. */
    tricore_disarm_reset_trigger();

    for (int i = 0; i < tricore_core_count(); i++) {
        const int core = tricore_core_index(i);
        /* Stop this core's timer while it is halted, so a breakpoint does not
         * silently kill a periodic task - see tricore.h. */
        tricore_freeze_timer(core, true);
    }
    ESP_LOGI(TAG, "attached: %d core(s)", tricore_core_count());
    return ESP_OK;
}

static void rsp_task(void *arg)
{
    (void)arg;

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(s_port),
    };

    s_listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_listen_sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        goto done;
    }
    int one = 1;
    setsockopt(s_listen_sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    if (bind(s_listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind to port %u: errno %d", s_port, errno);
        goto done;
    }
    if (listen(s_listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "listen: errno %d", errno);
        goto done;
    }
    ESP_LOGI(TAG, "listening on port %u - target remote <board>:%u", s_port, s_port);

    while (s_should_run) {
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(s_listen_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        const int client = accept(s_listen_sock, NULL, NULL);
        if (client < 0) {
            continue;                       /* timed out, or shutting down */
        }
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        ESP_LOGI(TAG, "client connected");
        s_connected = true;

        rsp_session_t session = {
            .sock     = client,
            .cur_core = tricore_core_index(0),
        };
        serve(&session);

        close(client);
        s_connected = false;
        ESP_LOGI(TAG, "client disconnected");
    }

done:
    if (s_listen_sock >= 0) {
        close(s_listen_sock);
        s_listen_sock = -1;
    }
    s_should_run = false;
    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t gdb_rsp_start(uint16_t port)
{
    if (s_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t err = attach_target();
    if (err != ESP_OK) {
        return err;
    }

    s_port = port ? port : GDB_RSP_DEFAULT_PORT;
    s_should_run = true;
    if (xTaskCreate(rsp_task, "gdb_rsp", 6144, NULL, 5, &s_task) != pdPASS) {
        s_should_run = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void gdb_rsp_stop(void)
{
    s_should_run = false;
}

bool gdb_rsp_running(void)   { return s_task != NULL; }
bool gdb_rsp_connected(void) { return s_connected; }
uint16_t gdb_rsp_port(void)  { return s_port; }
