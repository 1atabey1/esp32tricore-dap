/*
 * Print the wire word for frames that carry data.
 *
 * The RTL testbench only ever checked frames with no DATA field - sync, and a
 * LEN 0 frame - and on hardware exactly those work while every frame with a
 * payload draws no reply.  So the expected bits have to come from somewhere
 * independent of the RTL, and this is that somewhere: the C frame builder,
 * which is checked against the documented vectors by test_dap_frame.
 */

#include <stdio.h>

#include "dap_frame.h"

static void show(const char *what, uint8_t cmd, uint8_t len_field,
                 uint64_t data, size_t data_bits)
{
    dap_frame_t f = {0};

    if (!dap_frame_build(&f, cmd, len_field, data, data_bits)) {
        printf("%-22s REFUSED\n", what);
        return;
    }

    printf("%-22s cmd=0x%02X len=%2u dbits=%2u  bits=%2u  word=0x%0*llX  ",
           what, cmd, len_field, (unsigned)data_bits, (unsigned)f.len,
           (int)((f.len + 3) / 4), (unsigned long long)dap_frame_word(&f));

    for (size_t i = 0; i < f.len; i++) {
        putchar(f.bit[i] ? '1' : '0');
    }
    putchar('\n');
}

int main(void)
{
    /* The frame that works on hardware, as a control. */
    show("sync", 0x10, 63, 0, 0);

    /*
     * The ones that do not: everything with a DATA field.
     *
     * These are the LEN and DBITS dap_probe_client_set and
     * dap_probe_client_read actually use - 3 and 7, not the round 4 and 8 an
     * earlier version of this file guessed at.  The guess produced a perfectly
     * good vector for a frame nothing sends, which the RTL was then checked
     * against; the frames below are the ones that go on the wire.
     */
    show("client_set(1)", 0x1C, 3, 1, 3);
    show("client_read CLIENT_ID", 0x1A, 7, ((uint64_t)4u << 4) | 0xFu, 7);
    show("client_write IOCONF", 0x08, 16, ((uint64_t)0x081u << 4) | 0x0u, 16);
    return 0;
}
