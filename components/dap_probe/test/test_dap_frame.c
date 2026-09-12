/*
 * Host tests for the DAP frame layer.  Builds with plain gcc, no ESP-IDF and
 * no hardware: `make -C components/dap_probe/test run`.
 *
 * Every expected value here comes from the protocol documentation, not from
 * this implementation's output, so a passing run means the wire format agrees
 * with the target rather than with itself.  The residue check is the strongest
 * of them: it runs each generated frame through the manual's own checker,
 * which is what the silicon implements.
 */

#include "dap_frame.h"

#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(cond, fmt, ...)                                                 \
    do {                                                                      \
        if (cond) {                                                           \
            printf("  ok    " fmt "\n", ##__VA_ARGS__);                       \
        } else {                                                              \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__);                       \
            failures++;                                                       \
        }                                                                     \
    } while (0)

/* Build the payload the CRC covers, so a frame can be re-checked field by field. */
static size_t payload_of(uint8_t cmd, uint8_t len_field, uint64_t data,
                         size_t data_bits, uint8_t *out)
{
    size_t n = 0;
    for (size_t i = 0; i < 5; i++) out[n++] = (cmd >> i) & 1u;
    for (size_t i = 0; i < 6; i++) out[n++] = (len_field >> i) & 1u;
    for (size_t i = 0; i < data_bits; i++) out[n++] = (data >> i) & 1u;
    return n;
}

/* A frame's own residue must be zero under the manual's checker. */
static bool residue_of_frame(const dap_frame_t *f)
{
    /* Skip the start bit and drop the trailing zero: the CRC covers
     * CMD+LEN+DATA and is verified together with its own six bits. */
    return dap_crc6_residue_ok(&f->bit[1], f->len - 2);
}

static void test_documented_crc_vectors(void)
{
    printf("documented CRC6 vectors\n");

    uint8_t p[80];
    size_t n = payload_of(DAP_CMD_SYNC, 63, 0, 0, p);
    CHECK(dap_crc6(p, n) == 9, "sync, LEN=63 -> CRC 9 (got %u)", dap_crc6(p, n));

    n = payload_of(DAP_CMD_SYNC, 0, 0, 0, p);
    CHECK(dap_crc6(p, n) == 25, "sync, LEN=0  -> CRC 25 (got %u)", dap_crc6(p, n));
}

static void test_sync_wire_word(void)
{
    printf("first frame a probe ever sends\n");

    dap_frame_t f;
    CHECK(dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0), "sync frame builds");
    CHECK(f.len == 19, "sync is 19 bits (got %zu)", f.len);
    CHECK(dap_frame_word(&f) == 0x09FE1u, "sync wire word is 0x09FE1 (got 0x%05llX)",
          (unsigned long long)dap_frame_word(&f));
    CHECK(f.bit[0] == 1, "starts with the start bit");
    CHECK(f.bit[f.len - 1] == 0, "ends with the trailing zero");
    CHECK(residue_of_frame(&f), "residue zero under the manual's checker");
}

static void test_client_read_payloads(void)
{
    printf("client_read payloads, one byte each\n");

    CHECK(dap_client_read_payload(DAP_IO_READ_BYTE,  3) == 0x39,
          "8-bit  read -> 0x39 (got 0x%02X)", dap_client_read_payload(DAP_IO_READ_BYTE, 3));
    CHECK(dap_client_read_payload(DAP_IO_READ_HWORD, 4) == 0x47,
          "16-bit read -> 0x47 (got 0x%02X)", dap_client_read_payload(DAP_IO_READ_HWORD, 4));
    CHECK(dap_client_read_payload(DAP_IO_READ_WORD,  5) == 0x55,
          "32-bit read -> 0x55 (got 0x%02X)", dap_client_read_payload(DAP_IO_READ_WORD, 5));
}

static void test_bring_up_frames_are_self_consistent(void)
{
    printf("every bring-up frame passes the silicon's own check\n");

    struct { const char *name; uint8_t cmd; uint8_t len; uint64_t data; size_t bits; } cases[] = {
        { "sync (LEN 63)",          DAP_CMD_SYNC,         63, 0,     0  },
        { "sync (LEN 0)",           DAP_CMD_SYNC,          0, 0,     0  },
        { "dapisc read",            DAP_CMD_DAPISC,       16, 0,     16 },
        { "client_set(1)",          DAP_CMD_CLIENT_SET,    3, 1,     3  },
        { "client_read CLIENT_ID",  DAP_CMD_CLIENT_READ,   7, 0x0F,  7  },
        { "client_read 32-bit",     DAP_CMD_CLIENT_READ,   7, 0x55,  7  },
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        dap_frame_t f;
        bool built = dap_frame_build(&f, cases[i].cmd, cases[i].len,
                                     cases[i].data, cases[i].bits);
        CHECK(built && residue_of_frame(&f), "%s", cases[i].name);
    }
}

/* A generator that only agrees with itself is worthless; prove it catches damage. */
static void test_checker_rejects_corruption(void)
{
    printf("the checker actually rejects bad frames\n");

    dap_frame_t f;
    dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0);

    int caught = 0;
    for (size_t i = 1; i < f.len - 1; i++) {
        dap_frame_t bad = f;
        bad.bit[i] ^= 1u;
        if (!residue_of_frame(&bad)) {
            caught++;
        }
    }
    CHECK(caught == (int)(f.len - 2), "all %d single-bit flips rejected (caught %d)",
          (int)(f.len - 2), caught);
}

static void test_field_widths_are_enforced(void)
{
    printf("field widths\n");

    dap_frame_t f;
    CHECK(!dap_frame_build(&f, 0x20, 0, 0, 0),  "CMD above 5 bits refused");
    CHECK(!dap_frame_build(&f, DAP_CMD_SYNC, 64, 0, 0), "LEN above 6 bits refused");
    CHECK(!dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 64), "DATA above 63 bits refused");
}

int main(void)
{
    test_documented_crc_vectors();
    test_sync_wire_word();
    test_client_read_payloads();
    test_bring_up_frames_are_self_consistent();
    test_checker_rejects_corruption();
    test_field_widths_are_enforced();

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
