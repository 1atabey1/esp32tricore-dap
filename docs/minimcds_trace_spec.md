# TC3xx miniMCDS: implementer's spec for DTU_TC, WTU_MCX, TSU, DMC and the raw TRAM stream decoder

Source: AURIX TC3xx Target Specification V2.5.1, part 1, chapter 9 "Multi-Core Debug Solution (MINIMCDS)"
(module revision MINIMCDSV2.1.11, 2018-04). Text comes from the flat extraction `ts_part1.txt`,
cross-checked against a `pdftotext -layout` render of `C:\Users\a.onur\Downloads\AURIXTC3XX_ts_part1_V2.5.1.pdf`
(the layout render keeps table columns; Table 394 below was rebuilt from it).

Legend used below:
- **[V]** = verbatim quote or table taken directly from the spec.
- **[D]** = my derivation, with the reasoning given. Treat these as working hypotheses and check them on silicon.

---

## 0. Stream conventions every decoder needs (from 9.3.2; Tables 337-341 are known, summarised)

- [V] "Table 337 formally defines the message syntax (in Backus-Naur form, least significant bits to the right)."
- [V] Length rule: "Consistently in all messages the length always includes all bits of the message excluding the length itself."
- [V] `<body> ::= <trace_data> <trace_type> <core_ID>` so, reading from the LSB: **length, then core_ID (5), then trace_type (3), then trace_data**.
- [V] "<subtype> fields however are never compressed. They are placed adjacent to the <trace_type> for convenient message decoding."
- [V] Table 340 (LSB on the right):
  `TT4 <uncompr. data 2> <uncompressed data 1> <length 1>`, `TT6 <uncompr. data 2> <uncompressed data 1> <length 1> <subtype>`.
  Footnote: "If more than one data item is contained <length 1> gives the length of <data 1> only. The length of <data 2> can be calculated from the total length <L> of the message by subtracting the size of all other items."
- [V] "The leading zero suppression may be applied to <uncompressed data> fields as well, as the current length can be derived from the total message length."
- [V] Compression: "The data to be transmitted is XORed with the reconstructed data of the most recent message of the same type. Then leading zeros are stripped." Also "even the LSB can be stripped, leaving an empty data field (size 0)" and "Not all leading bits need to be stripped, if a standard length can be used otherwise."
- [V] "A global constant MAXCNT is defined, with a value smaller than 256 ... not important for customers" (so a decoder should not rely on its exact value).
- [V] "uncompressed messages may be used wherever compressed messages are allowed, but not vice versa."

**Bit order in memory, checked against Table 342** [V example, D rule]. `C7 00 20 04 00 -8` decodes as: byte[0] low nibble `0111` = L (symlen 40);
core ID `01100` spans byte[0] bits 7:4 and byte[1] bit 0; TT = byte[1] bits 3:1; TD = bits 4..35 of bytes 1..5 = 80004200H.
The next message's L is the **high nibble of byte[5]** (`2-`).
- [D] The TRAM is a little-endian bit stream: stream bit n = bit (n mod 8) of byte (n div 8). Messages are packed back to back at **nibble** granularity and are not byte-aligned.

Length-nibble dispatch [V Table 337 tokens]:

| first nibble (LSB) | meaning |
|---|---|
| `0000` | `<tick>` (4 bits, empty message) |
| `0001` | `<errormessage>` = `<error_cnt>(7) <core_ID>(5) 0001` (16 bits total) |
| `0010`..`1101` | `<symlength>` (Table 338: 12,16,20,24,32,40,44,56,64,76,92,112) |
| `1110` | `<multick>`, preceded (i.e. followed in bit order) by 8-bit `<ticount>` |
| `1111` | `<absize>`, followed by 8-bit `<bicount>` = body length in bits (`00000001`..`11110100`) ; `11111111 1111` = `<endoftrace>` |

- [V] `<ticount>` footnote: "Value 0 is used to make a 12 bit 'filler message' for memory alignment purposes only".
- [V] `<skip> ::= (0 | 1)* <ntn> 00000 <length>`. A body with core_ID `00000` is a skip. Footnote: "This length (encoded as <bicount> <absize>) is important for the paragraph adjustment. The bits following the <ntn> 00000B string are not interpreted by the tool." (`ntn` = Next Tile Now; the miniMCDS does not use it.)
- [V] Core IDs (Table 339): `01000` DCU_TCX, `01001` DTU0_TCX, `01011` DTU1_TCX, `01100` PTU_TCX, `01110` TSU_MCX, `01111` WTU_MCX, `00000` <skip>.
- [V] `<length 1>` codes (Table 341): 0000=0, 0001=4, 0010=8, 0011=12, 0100=16, 0101=20, 0110=24, 0111=28, 1000=32, 1001=36, 1010=40, 1011=44, 1100=64.

---

## 1. DTU_TC (core IDs DTU0_TCX = 01001B, DTU1_TCX = 01011B)

### 1.1 Message table [V Table 389]

```
Name   Maximum Length                 Content
DTW    90 = 44 + 32 + 2 + 4 + 3 + 5   write address (master ...1),absolute byte address), write data (8, 16, 32 bit) and data size
DTWA   54 = 44 + 2 + 3 + 5            write address (master ...1),absolute byte address)
DTWD   42 = 32 + 2 + 3 + 5            write data (8, 16, 32 bit) and data size
DTR    90 = 44 + 32 + 2 + 4 + 3 + 5   read address (master ...1),absolute byte address), read data (8, 16, 32 bit) and data size
DTRA   54 = 44 + 2 + 3 + 5            read address (master ...1),absolute byte address)
DTRD   44 = 32 + 2 + 2 + 3 + 5        read data (8, 16, 32 bit) and data size
ERR    12 = 7 + 5                     -
1) The 12 MSB correspond to the ACMSK bit fields described in the related register.
```
[V] "Note: The duplex architecture occupies two core IDs (DTU0_TC, DTU1_TC), but shares one register file."
DTA is missing from Table 389. By Table 369 it has the same layout as DTWA (54 bits max) [D].

### 1.2 Trace-type / subtype assignment [V Tables 363-370]

| Msg | TT (uncompr / compr) | subtype | fields (spec wording) |
|---|---|---|---|
| DTWD | TT0 / TT1 | none | uncompressed/compressed data = "data written" |
| DTA  | TT2 / TT3 | "subtype (2 bit)" = **0** | data = "destination address" |
| DTWA | TT2 / TT3 | "subtype (2 bit)" = **1** | data = "destination address" |
| DTRD | TT2 / TT3 | "subtype (2 bit)" = **2** | data = "data read" |
| DTRA | TT2 / TT3 | "subtype (2 bit)" = **3** | data = "source address" |
| DTW  | TT4 / TT5 | none | data 1 = "data written", data 2 = "destination address" |
| DTR  | TT6 / TT7 | "subtype not used (length 0)" | data 1 = "data read", data 2 = "source address" |
| ERR  | - | - | "err_cnt  Number of lost messages excluding DTA" |

### 1.3 Field order inside the body (TC3xx B-step layout)

[V] The B-step changes are:
- "The address and the Transaction Type information is swapped. In other words the address is appended to the Transaction Type like the master ID, for details see the respective *ACMSKi bit field definitions."
- "Data size is encoded in an additional 2-bit field which is prepended to the L1 length field. As before the L1 length contains the encoded length of the data not including the 2-bit data size field."
- "Modified layout for DTW, DTWD, DTR and DTRD messages e.g. DTW message: master ID + address + data + data size + L1 length + TT + Core ID + length (compared to address + master ID + data + L1 length + TT + Core ID + length on TC3xx A-Step)."
- "The Transaction Type can be masked (set to zero) and stripped off through leading zero suppression if not needed as in trace based measurement."

[D] Resulting bit layouts, listed LSB first after the length field. Widths come from Table 389, and the order follows the quote above, where MSB is on the left:

```
DTWD  (TT0/1): core_ID[5] | TT[3] | dsize[2] | data[L-10]               (32 max)
DTRD  (TT2/3): core_ID[5] | TT[3] | subtype[2]=2 | dsize[2] | data[L-12] (32 max)
DTA/DTWA/DTRA (TT2/3): core_ID[5] | TT[3] | subtype[2]=0/1/3 | addr44[L-10]
DTW   (TT4/5): core_ID[5] | TT[3] | L1[4] | dsize[2] | data1[len(L1)] | addr44[rest]
DTR   (TT6/7): core_ID[5] | TT[3] | (subtype 0 bits) | L1[4] | dsize[2] | data1[len(L1)] | addr44[rest]
       rest = L - 8 - 4 - 2 - len(L1)
ERR          : 0001 | core_ID[5] | err_cnt[7]
```
The dsize position in DTWD and DTRD is [D]. Table 389 gives the "+2" but not its position. The quoted layout puts the data size directly below the data, and a variable-length field has to be the most-significant one, so the length can be derived from L.

**Data size field** [V Table 362]:
```
Data Size  BYTE  HALFWORD  WORD  DOUBLE
Field      00B   01B       10B   11B
```
[V Table 360] "DOUBLE lower 32bits traced and treated as WORD access if properly aligned". [V] "only the lower 32bit of the TriCore data bus is routed to the MINIMCDS". Data values are "right justified" (BYTE = 8 bit, HALFWORD = 16 bit, WORD = 32 bit).

**44-bit address field** ("addr44"):
- [V] Footnote 1 of Table 389: "The 12 MSB correspond to the ACMSK bit fields described in the related register."
- [V] "The address information contained in the messages below consists of the effective address as received on the address port plus a prepended bit string containing selected information from the Transaction Type port."
- [V TCXACMSKj.MSK] "0B subchannel, master and SVM are included in DTU messages. 1B subchannel, master and SVM are set to zero and as such not included in DTU messages." "The MSK bits of a register group are ORed."
- [D] ACMSK bits 11:0 are SUBCHANNEL[11:5], MASTER[4:1], SVM[0], so
  `addr44[31:0] = effective byte address`, `addr44[32] = SVM`, `addr44[36:33] = MASTER (Table 385)`, `addr44[43:37] = SUBCHANNEL (Table 386)`.
  The spec does not state this order explicitly. It assumes the ACMSK order is kept. The direction bits WR/RD (ACMSK 12/13) are not among the 12 MSB, because the message type already encodes direction.
- [D] TCXACMSKj resets to 0000 7FFFH, so MSK = 1 by default. The prefix is then zero and leading-zero suppression strips it, leaving a plain 32-bit address. To get master IDs, clear MSK in **both** TCXACMSK0 and TCXACMSK1. Also open question: the spec does not say whether the MASTER/SUBCHANNEL mask bits (which feed the comparator) also mask the message content.

**Bus master encoding** [V Table 385]: 0000B Cerberus, 0001B CPU0 subsystem, 0010B CPU1 subsystem, 0011B CPU2 subsystem, 0100B CPU3 subsystem, 0101B CPU4 subsystem, 0110B CPU5 subsystem, 0111B DAM0 & DAM1, 1000B DMA (all Resource Partitions), 1001B-1011B (not used), 1100B HSSL0 & HSSL1, 1101B Ethernet, 1110B HSM, 1111B Reserved.
[V] "Note: The Bus Master Encoding is not valid in case CPUx (vs CPUx_MEMSlave) is selected in the MUX_TC_RC register."

**Submaster/Subchannel** [V Table 386]:
```
Master   Code                    Meaning
any DMA  0000000B … 1111111B     Channel 0 … Channel 127
any CPU  0000000B                DMI
         0001000B                DMI safe access path
         0010000B                PMI
         others                  reserved
others   d.c.                    reserved
```
[V, ch. 1.3.4.2] "Note: The data trace of the SRI slave does not include the DMA channel number." This conflicts with Table 386. Expect channel 0.

### 1.4 Compression engines [V]
- "Each DTU (i.e. each Core ID) has its own compression engine." (DTU0 and DTU1 therefore need separate decompression state. Also from 9.3.10: "Note that the compression is local to each DTU, so a separate decompression is required for each core ID.")
- **Address engine:** "Compression for DTA, DTRA, DTWA and the address part of DTW and DTR is based on a common memory." [D] The XOR base is the full 44-bit value, including the prefix.
- **Data engine:** "Compression for DTWD, DTRD and the data part of DTW and DTR is also based on a common memory. For compression purposes all data is extended with 0 up to the longest possible data size."
  Example [V]: "Previous message contained the data of a compressed word. The new data has the data size BYTE. So the byte is extended with 0 to data size DOUBLE and the compression is performed. In the new message just the byte part of the compression result is used and indicated through the 2-bit data size field."
  [D] Decoder: `val = (field XOR base) & mask(dsize)`, then `base = zero_extend(val)`. The base update is inferred: the spec says only that all data is zero-extended. Verify with a word write followed by a byte write.
- "Address and Data compression executed (independently) no matter whether data size differs or not."

### 1.5 When uncompressed messages are forced [V, 9.3.9]
- Data-bearing (DTWD, DTRD, and the data part of DTW/DTR): "as first message containing data (i.e. the first from the set DTWD, DTRD, DTW, DTR) generated for a new paragraph of buffer memory; when MAXCNT messages containing compressed data have been generated consecutively; as first message containing data after an ERR message."
- Address-bearing (DTA, DTWA, DTRA): "as first message containing an address (i.e. the first from the set DTA, DTWA, DTRA, DTW, DTR) generated for a new paragraph ...; when MAXCNT messages containing a compressed address have been generated consecutively; as first message containing an address after an ERR message."
- DTW/DTR: the union of both lists.
- Split: "When a DTW or DTR message is due ... but the physical size of the message would exceed the maximum length physically possible for the DMC of the device on hand, two messages with the same time stamp are generated instead: DTW is replaced by DTA+DTWD, DTR is replaced by DTA+DTRD. Note that DTA is always put first into the buffer trace memory ... the decoder can rely on getting a DTRD or DTWD after each DTA (unless Flush is asserted)."
- ERR: "generated when any of the other messages of the DTU is due but the FIFO is full ... only generated when there is again enough space in the FIFO for at least one other message; the time tag however is set to the instant the ERR message was due."

### 1.6 Which message is generated [V]
- DTWD: "dtu_wdat is asserted but dtu_wadr is deasserted, reconstructed data is available and WRITE is seen".
- DTWA: "dtu_wadr is asserted, dtu_wdat is deasserted or no reconstructed data is available, the reconstructed address is available and WRITE is seen".
- DTW: "dtu_wdat and dtu_wadr are asserted, reconstructed address and data are available and WRITE is seen". The read side (DTRD, DTRA, DTR) works the same way with dtu_rdat and dtu_radr.
- "a message is not generated when the information to be contained is not available ... it is therefore legal to have trace qualification inputs asserted continuously. All messages (apart from ERR) get the time tag of their creation."
- The four enables are MCXACT27 dtu_wdat, MCXACT28 dtu_wadr, MCXACT29 dtu_rdat and MCXACT30 dtu_radr, each with "Note: individual for each DTU of DTU2".

### 1.7 Comparator data alignment [V Table 371]
```
Size      31:24  23:16  15:8  7:0
BYTE      7:0    7:0    7:0   7:0
HALFWORD  15:0          15:0
WORD      31:0
```
[V] "if the data is shorter than the reconstruction register, the data is repeated (This implies that only 'naturally aligned' data is used.) A byte written to address 2 can therefore be seen and matched at [23:16], no matter with which granularity the memory or SFR was accessed. The trigger outputs (dtu_dat_trig) are asserted only while READ or WRITE is signalled on the transaction type port."
[D] Table 371 only describes the **comparator** view. The **message** data field is right-justified (Table 360). Take the byte lane from address[1:0].

### 1.8 Trigger behaviour relevant to watch-points [V]
- "The trigger outputs are sent to the TQU as dtu_ea_trig. These trigger signals are asserted by the address comparator until a transaction with a new address occurs."
- "The same holds true for the Transaction Type excluding the read/write indication itself ... They are called dtu_acc_trig ... The sending PAL takes care that the mode port of Transaction Type is READ or WRITE for exactly one clock cycle for each transaction. Therefore it is strongly recommended to combine the dtu_ea_trig and the dtu_acc_trig."
- [D] To get one pulse per write, program TCXACMSKj = 1000H (WR only), TCXACBNDj = 1000H and TCXACRNGj = 0. Then AND `dtu_acc_trig[j]` with `dtu_ea_trig[i]` in one MCXEVT (for example MCXEVT8 or MCXEVT10, which carry both; see §6).

### 1.9 DTU_TC comparator registers [V]

| Reg | Offset | Reset | Layout |
|---|---|---|---|
| TCXEABNDj (j=0-1) | 2400H+j*10H | 0000 0000H | BOUND 31:0 rw "Address Comparator range lower bound" |
| TCXEARNGj | 2404H+j*10H | FFFF FFFFH | RANGE 31:0 rw "Address Comparator range size" |
| TCXWDBNDj | 2480H+j*20H | 0000 0000H | BOUND 31:0 rw "Data Comparator range lower bound" |
| TCXWDRNGj | 2488H+j*20H | 0000 0000H | RANGE 31:0 rw |
| TCXWDMSKj | 2490H+j*20H | FFFF FFFFH | MASK 31:0 rw |
| TCXWDSGNj | 249CH+j*20H | 0000 0000H | SIGN 4:0 rw "Bit number (1…31) of sign bit"; 0 at 19:5, 31:20 |
| TCXACBNDj | 2500H+j*18H | 0000 0000H | BOUND 13:0 rw; 0 at 31:14 |
| TCXACRNGj | 2504H+j*18H | 0000 0000H | RANGE 13:0 rw; 0 at 31:14 |
| TCXACMSKj | 2508H+j*18H | 0000 7FFFH | see below |

TCXACMSKj [V]: `SVM 0 rw` ("0B Overwrite SVM with 0 for the comparison. 1B Use SVM as bit 0 for the comparison."), `MASTER 4:1 rw` ("ANDed with this bit pattern and the result used as bit [4:1] for the comparison"), `SUBCHANNEL 11:5 rw` (same, "bit [11:5]"), `WR 12 rw` ("1B Bit 12 of the compared pattern is set to 1 if the current transaction is a write"), `RD 13 rw` (the same for read, bit 13), `MSK 14 rw` (quoted in 1.3), `0 31:15 r`.
TCXACBNDj [V]: "The bit string, consisting of SVM, bus-master ID, sub-channel and direction, masked by TCXACMSKx, is compared to this lower bound."
Magnitude comparator semantics [V 9.3.3.1, Table 343]: match when `0 <= (value - BOUND) <= RANGE` (unsigned). Use cases: "x ≤ value ≤ y: BOUND x, RANGE y - x"; "value = x: x, 0"; "always: 0, all 1".
[V TCXEABND note] "When defining the comparators for the effective address keep in mind that many memory locations can be accessed under two different addresses, namely as cached or non-cached memory range." [D] Program the address alias the software actually uses, or use both EA comparators, one per alias, and OR them in the action.

Other DTU_TC-related notes [V, 9.4.3.3]: "The current implementation of the TriCore writes four core registers per clock cycle to the CSA during context switch. TCWD is based on an 64 bit wide internal bus, therefore only a subset of the saved registers can be seen." Table 388 maps address[5:0] 08H/18H/28H/38H to D8/PCX/D12/A12 for the upper context (CALL, interrupts, exceptions), and to D0/PCX/D4/A4 for the lower context (SVLCX, BISR). [D] The pipeline source (TC_MUX = CPU0) shows CSA spills as writes. Filter them out.

---

## 2. WTU_MCX (core ID 01111B)

[V Table 392]
```
Name  Maximum Length         Content
WPS   11 = 3 + 3 + 5         watch-point ID (unsigned 0…7)
WPM   16 = 8 + 0 + 3 + 5     watch-point IDs (bit 0…7 corresponds to watch-point 0…7)
EVC   32 = 16 + 4 + 4 + 3 + 5  counter ID (unsigned 0…7 corresponding to MCXCNT0…MCXCNT7) counter value (unsigned 16 bit)
ERR   12 = 7 + 5         -
```
[V Tables 372-374]: WPS uses TT0/TT1 with data = "watch-point ID (unsigned)". WPM uses TT2/TT3 with "subtype not used (length 0)" and data = "watch-point ID (bit set)". EVC uses TT4/TT5 with data 1 = "counter ID (unsigned)" and data 2 = "current count (unsigned)".

[D] Layouts, LSB first after length:
```
WPS (TT0/1): core_ID[5] | TT[3] | id[L-8]      (3 significant bits; L = 12 -> 4-bit field)
WPM (TT2/3): core_ID[5] | TT[3] | bitset[L-8]  (8 bits max)
EVC (TT4/5): core_ID[5] | TT[3] | L1[4] | cnt_id[len(L1)] (<=4) | count[rest] (<=16)
```
Compression and forced-uncompressed rules [V]:
- "Each WTU of each observation block has an own compression for its WPS messages. The uncompressed WPS may be sent if the length is very small e.g. 12bit (and compression does not make much sense) and must be sent as first WPS generated for a new paragraph of buffer memory; when MAXCNT consecutive compressed WPS messages have been sent; as first WPS after an ERR message." The same three rules apply to WPM ("own compression for its WPM messages") and to EVC ("own compression for its EVC messages").
- [D] So there are three independent XOR bases: WPS id, WPM bitset, and EVC (id and count, each XORed against the previous EVC's field).

Generation rules [V]:
- WPS: "generated when the indicated input wtu_enable gets asserted (this means edge detection) and every other wtu_enable input stays stable or is getting deasserted. Note that the wtu_cnt inputs have no influence."
- WPM: "generated if more than one wtu_enable gets asserted (edge detection on each input signal)." "LSB represents wtu_enable[0]".
- EVC: "If no wtu_enable gets asserted ... an EVC message is generated if any wtu_cnt gets asserted (edge detection); if any wtu_cnt was asserted but not served in a previous clock cycle (i.e. detected edges are stored). Of all pending wtu_cnt requests the one with the numerically lowest counter ID is served first." "Note that there is a buffer in the data path, so the EVC message contains the counter value at the point of time wtu_cnt[n] was asserted last time."
- Overflow: "All watch-points hit while the FIFO is full are collected and sent immediately after the ERR message, either as WPS or WPM ... If a watch-point is hit repeatedly while the FIFO is full however this information is lost." "EVC messages due before or during the time the FIFO is full are also remembered."
- Actions: MCXACT5..12 = wtu_enable[0..7] ("watch-point message request ID n"). MCXACT13..20 = wtu_cnt[0..7] ("counter message request (MCXCNTn)").
- [D] WPS fires on a **rising edge** of the action. Use LV=0 on the action input (a one-cycle pulse), and make sure the event itself returns low between hits (see 1.8).

---

## 3. TSU_MCX (core ID 01110B) and time reconstruction

### 3.1 Messages [V Tables 379-381, 391]
```
TSR  40 = 32 + 3 + 5      current counter value (emulation clock, 32 bit)   TT0 uncompressed / TT1 compressed
TSA  40 = 32 + 0 + 3 + 5  current counter value (reference clock, 32 bit)   TT2 / TT3, "subtype not used (length 0)"
ERR  12 = 7 + 5           err_cnt "Don't care"
```
[D] Layout: `core_ID[5] | TT[3] | value[L-8]`. There is one XOR base per counter: [V] "Compression is done separately for each counter."
Forced-uncompressed rules [V] (the same for TSR and TSA): "as first TSR generated after a Flush (i.e. start of trace); as first TSR generated for a new paragraph of buffer memory; as first TSR after an ERR message." The spec lists no MAXCNT rule for TSU.

Generation [V]:
- TSR: "generated when no TSA message is due and tsu_rel_en gets asserted (start of trace) or both tsu_rel_en and tsu_rel_sync are asserted (sync message)."
- TSA: "generated when tsu_abs_en gets asserted (start of trace); tsu_abs_en is asserted and tsu_abs_sync gets asserted (sync message). both tsu_abs_en and tsu_abs_sync are asserted and the counter value changes and the FIFO is not full (autostamp message)." "Note: TSAs of the auto-stamp mode never cause overflow."
- "As a general rule, only one time stamp per emulation clock is put into the trace memory by the DMC, highest priority given to TSA or TSR and lowest priority to <tick> messages."
- Actions: MCXACT0 tsu_rel_en, MCXACT1 tsu_rel_sync, MCXACT2 tsu_abs_en, MCXACT3 tsu_abs_sync, MCXACT4 tsu_rel ("TSU TSR message request to MCX"; "directly ORed into the MCX's tsu_rel_sync without delay").
- "The emulation clock always drives a circular 32 bit long binary counter. Thus two messages generated less than 4294967296 clocks (e.g. 28.6 s at 150 MHz) apart can be identified unambiguously." Footnote: "The least significant 8 bits of this counter are used as time tag in the primary and secondary FIFOs". "The reference clock always drives a circular 32 bit long binary counter."
- "Note: The circular counters are not affected by MCDS Reset ... Power On Reset however does work".
- Emulation clock = MCDS clock: "The MINIMCDS design is able to trace data up to twice the MCDS clock (same as BBB clock)".

### 3.2 TSU registers [V]

| Reg | Offset | Reset | Layout |
|---|---|---|---|
| TSUREFCNT | 0400H | PowerOn 0000 0000H | COUNT 31:0 rh (reference-clock counter; the source of TSA) |
| TSUPRSCL | 0404H | PowerOn 0000 0000H | RELOAD 31:0 rw |
| TSUEMUCNT | 0408H | PowerOn 0000 0000H | COUNT 31:0 rh (emulation-clock counter; the source of TSR) |

Generic PRSCL [V]: "This number is automatically loaded into the down counter on under-run (No change to the running divider!) ... A value of 1 will therefore activate the tc_trig output every second reference clock cycle (sequence 1->0->1->0...), while the value 0 will activate the trigger continuously." Generic CNT: "incremented on each rising clock edge starting with MCDS clock enabled. After reaching FFFF FFFFH it automatically wraps around ... The only way to clear it is a Power On Reset."
The prescaler output is the trigger `tsu_tc_trig` (Table 394). The reference clock is selected by MUX_TC_RC.RC (see §5). There are no separate TSUCTRL or TSUEMUCNT-control registers: Table 384 lists only these three TSU registers.

### 3.3 `<tick>` / `<multick>` [V facts, D semantics]
- [V] "If enabled (by tick_enable), the relative timestamps (<tick> and <multick>, see Table 337) are derived from the incoming tags inside DMC without additional message traffic through the FIFOs."
- [V] "Each message gets a time tag assigned in the clock cycle when it is generated ... All messages with the same tag value belong to the same sample point (= emulation clock cycle). In case there is a time stamp message (TSR or TSA) in the set belonging to a time tag it is stored first, followed by PTU messages. Messages from Duplex DTUs are stored according to their temporal order."
- [V] "If enabled by tick_enable ... a time stamp message of format <tick> or <multick> ... can be written as first message of each set belonging to a time tag. If tick_enable is set and no new message arrives for 255 clock cycles, a <multick> message is written automatically."
- [V] footnote: "<tick> messages need to be generated however as long as messages arrive in this phase [Flush], Once all FIFOs are empty no more <tick> or <multick> messages are generated until Flush is cleared again."
- [V] MCXACT21 tick_enable = "enable <tick> message generation by DMC". The action footnote says tick_enable is the only action **not** forced off by OCS suspension.
- [V] Grammar: `<ticount> ::= 00000000 | 00000100 | … | 11111111` (the second value is printed as `00000100`, which looks like a typo).
- [D] Working model, to be verified: a `<tick>` starts a new sample set exactly 1 emulation clock after the previous one, and `<ticount><multick>` means ticount clocks elapsed (1..255, with 0 = filler). A set that follows a TSR or TSA has no tick; the TSR carries the absolute value. Calibrate by enabling tick_enable together with a periodic TSR sync driven by `tsu_tc_trig`, then compare the cumulative ticks against the TSR deltas.

---

## 4. DMC / buffer registers (9.4.5; TRAM = 8 Kbyte, paragraphs of 1 Kbyte, 256-bit write word)

[V] "Up to 8 Kbyte of buffer memory addressable ... Organized in paragraphs of 1 Kbyte. Physical memory word is 256 bit for writing, 32 bit for reading." Table 399: for 8 Kbyte, the NOW / PRE / TOP pointer is [12:5] and the BOTTOM pointer is [12:10].

| Reg | Offset | Reset | Fields |
|---|---|---|---|
| FIFONOW | 0200H | PowerOn 0000 0000H | NOW 12:5 rh; 0 at 4:0, 31:13. "When tracing stops this register points to the buffer memory word containing the <end-of-trace> message." |
| FIFOBOT | 0204H | MCDS 0000 0000H | BOTTOM 12:10 rw; 0 at 9:0, 31:13 |
| FIFOPRE | 0208H | MCDS 0000 0000H | PRE 12:5 rw; 0 at 4:0, 31:13 |
| FIFOTOP | 020CH | MCDS 0000 1FFFH | TOP 12:5 rw; "1  4:0  rX  Reserved Read as 1; must be written with 1."; 0 at 31:13 |
| FIFOCTL | 0210H | PowerOn 0000 2002H | see below |
| FIFOWARNx (x=0-1) | 0214H+x*4 | MCDS 0000 0000H | WARN 12:5 rw; EN 31 rw ("0B Trigger generation is disabled 1B Trigger generation is enabled"); 0 at 4:0, 30:13 |
| FIFOOVRCNT | 021CH | PowerOn 0000 0000H | COUNT 7:0 rh; CLR 15 w ("1B Counter is cleared"); 0 at 14:8, 31:16 |

FIFOCTL [V]: `TRG 0 rh`, `FFE 1 rh`, `TRDIS 9 rh`, `TRON 10 w`, `TROFF 11 w`, `FLSH 13 rh`, `CLR 14 w`, `SET 15 w`, `0 8:2, 12, 31:16 r`.
- TRG: "0B No trace_done has been seen since the last CLR bit setting (still writing in the Pre-Trigger area). 1B The trace_done signal was asserted at least once since CLR was set last time".
- FFE: "all primary, secondary and DMC-internal FIFOs have been emptied ... After Power On Reset and MCDS Reset this bit will be set automatically".
- TRDIS: "0B Trigger processing active. 1B All Triggers masked." "This bit is automatically reset by MCDS Reset." TRON: "globally enable all trigger pools". TROFF: "forcing all trigger inputs of all TQUs (including the performance data to the Event Counters) to '0'".
- FLSH: "The Flush signal can be set by the SET bit and by the hardware (at end of trace). The Flush signal can only be cleared by the CLR bit. Note: This bit is automatically set by MCDS Reset. 0B Tracing active. 1B Tracing stopped."
- CLR: "the only way to start trace recording ... starting the trace recording from the address given in BOT register." SET: "the only way to set the Flush flag by software".
- Values: start = 4000H, stop = 8000H, TRON = 0400H, TROFF = 0800H.

FIFOOVRCNT.COUNT [V]: "The counter increments in case an ERR message is written to one of the Trace Unit FIFOs. In case multiple error messages are written the counter still increments just by one."

Generic semantics [V 9.3.15.5]:
- BOTTOM: "loaded into the DMC write pointer when Flush is cleared (start of tracing) and when the upper bound was reached and tracing continues ... The granularity is a paragraph."
- TOP: "the address of the last byte to be used as trace buffer. When the DMC write pointer reaches this value and tracing continues, the write pointer is reloaded with BOTTOM (wrap around). Note: TOP must be at least two memory words higher than BOTTOM".
- PRE: "When the trace_done signal from the central TQU is received tracing will continue until the complete buffer area between BOTTOM and TOP is filled, but PRE bytes already written before trace_done arrived are not overwritten ... 0D Post-Trigger-only mode ... TOP-BOTTOM, Pre-Trigger-only mode: Trace recording is stopped as soon as trace_done is received ... PRE must not be larger than (TOP-BOTTOM) ... PRE must not be less than two memory words (or 0)."
- WARN: "If the Trace Buffer write pointer (NOW) equals this value, the signal fifo_trg to TQU and/or OCDS is asserted. Note: At the upper end of the used memory range, the last 31 byte addresses immediately preceding FIFO_TOP are not reachable for FIFO_NOW." (9.3.15.4:) "If NOW equals WARN while tracing is ongoing (Flush not asserted), a signal (fifo_trg) is sent to the central TQU ... WARN should be set well below TOP if it is used to generate trace_done. The signal fifo_trg is automatically de-asserted when Flush gets active." FIFOWARN0 and FIFOWARN1 appear as fifo_trg[0] and fifo_trg[1] in Table 394.

### 4.1 Paragraphs, `<endoftrace>`, readout [V]
- "Each paragraph starts at a 1 Kbyte border. The message packer now has to make sure that each paragraph starts with a new message; each paragraph is completely filled (see <skip> message); only the last written paragraph may be incomplete; decoding then stops at the <end of trace> message; each active trace unit is asked to generate an uncompressed message when a new paragraph is started." Footnote: "There may be some 'hang-over' from the previous paragraph as messages already stored in the primary and secondary FIFOs can't be modified."
- "The shaded area in the second paragraph ... is unrecoverable by the decoder. This is caused by the loss of the compression bases through overwriting in the paragraph containing NOW. One paragraph worst case of buffer memory assigned to Pre-Trigger trace data is lost".
- sync_rq: "Pulsed high for one clock cycle at the beginning of a new paragraph of buffer memory" (event IDs 20…31 in Table 395; clear IDs 16…31 in Table 397).
- "Note: Tracing always starts at the beginning of a new paragraph of buffer memory."

Table 382 procedure [V, condensed; columns are Post-Trigger only / Pre- and Post-Trigger / Pre-Trigger only]:
1. Set BOTTOM to the "Address of first byte in first available paragraph". Set TOP to the "Address of last byte in last available paragraph".
2. Set PRE to `0` / "Fraction of (TOP - BOTTOM)" / "TOP - BOTTOM". Footnote: "PRE must not be less than two memory words."
3. "Initialize memory: Write <end of trace> message to the first memory location of each paragraph to be used." Footnote: "This is required to distinguish 'virgin' paragraphs from used ones."
4. "Start tracing: Clear the Flush flag in CTL".
5. "Wait for end of trace: Indicated by Flush flag being raised again". Footnote: "Manual stop is also possible by setting the Flush flag via CTL register."
6. "Check for errors: not supported".
7. Fetch: the oldest message is "at BOTTOM" in post-trigger mode. Otherwise: "a) Copy NOW to pointer variable p b) Advance p to next paragraph c) if p reaches TOP, p to BOTTOM d) Repeat from b) until p does not point to <end of trace> or p equals NOW e) start reading at p". The latest message is "in memory word pointed to by NOW (or just read on until <end of trace>)".

[D] `<endoftrace>` is the 12-bit pattern FFFH at the LSB of the paragraph. Writing FFFF FFFFH to the paragraph's first word is enough. The TRAM must also be ECC-initialised before any read ([V] "the memory has to be initialized from the SRI interface side e.g. using word (32bit) or dword (64bit) writes before reads can be attempted"). So fill the whole TRAM with FFFF FFFFH through B800 0000H.
[V] "During this initialisation phase the MINIMCDS does not perform any writes. Concurrent write accesses to TRAM are not a valid use case ... Once the MINIMCDS starts to write trace data into TRAM, any concurrent read access from the SRI interface side is delayed."
[D] DMC pointers are byte offsets in the 8 KB TRAM, so TRAM byte = B800 0000H + offset.

### 4.2 Circular / continuous tracing [V + D]
- Circular: Pre-Trigger (long): "tracing is started, but the trace_done is not reached before the trace buffer is completely filled for the first time. Due to the ring buffer behavior the write pointer rolls over and starts to overwrite the oldest data. This may happen many times." [D] Set PRE = TOP-BOTTOM, never assert trace_done, and stop with FIFOCTL.SET. Then wait for FFE=1 and read out with step 7.
- There is no live streaming. The documented continuous-trace aid is [V 9.4.4.4]: "suspend_out is sent to the Trigger Switch (OTGS) to cause suspend requests. The intended application is to suspend e.g. the processors when the Trace Buffer runs full in continuous trace applications". [D] That is: FIFOWARNx.EN → fifo_trg[x] → MCXEVT → MCXACT24 suspend_out (or MCXACT23 break_out). The tool drains the buffer and restarts, and each restart begins a new paragraph with uncompressed messages.
- [V] "It is strongly recommended to set the FIFOCTL.FLSH bit prior to turning the clock off."

---

## 5. Global registers (base FB71 8000H; TRAM B800 0000H non-cached / 9800 0000H cached)

[V Table 383] "(MINIMCDS) 98000000H 98001FFFH TRAM cached access via SRI; B8000000H B8001FFFH TRAM non cached access via SRI; MINIMCDS FB718000H FB71FFFFH sri slave interface".
[V 9.4.1] Access rules: "Write access to empty addresses: no bus error, CT.IWA set. Read access to empty addresses: no bus error, CT.IRA set. Write or read other than word: not supported. Read-modify-write access: not supported. Read when the MCDS clock is turned off: bus error (unless CLC, OCS). Write when the MCDS clock is turned off: no bus error, no update of registers. Write to readonly registers: no bus error". All accesses are SV and U, 32-bit.

### 5.1 Table 384 (complete) [V]
```
CLC        0000H        rw  PowerOn       OCS       0004H  rw  MCDS
ID         0008H        r   (Table 400)   CT        0010H  rw  (Table 401)
MUX        0014H        [Read column empty in the layout render; Write SV,U,32]  PowerOn
MUX_TC_RC  0020H        rw  PowerOn
FIFONOW    0200H r PowerOn | FIFOBOT 0204H rw MCDS | FIFOPRE 0208H rw MCDS | FIFOTOP 020CH rw MCDS
FIFOCTL    0210H rw PowerOn | FIFOWARNx 0214H+x*4 rw MCDS | FIFOOVRCNT 021CH r PowerOn
TSUREFCNT  0400H r PowerOn | TSUPRSCL 0404H rw PowerOn | TSUEMUCNT 0408H r PowerOn
MCXEVTx    0800H+x*4 rw MCDS (x=0-15) | MCXACTx 0880H+x*4 rw MCDS (x=0-41)
MCXCCLj    0A00H+j*10H rw MCDS | MCXLMTj 0A04H+j*10H rw MCDS | MCXCNTj 0A08H+j*10H r PowerOn (j=0-7)
TCXDCSTS   2000H r PowerOn | TCXCIP 2008H r PowerOn | TCXCFT 200CH rw MCDS
TCXEABNDj  2400H+j*10H | TCXEARNGj 2404H+j*10H
TCXWDBNDj  2480H+j*20H | TCXWDRNGj 2488H+j*20H | TCXWDMSKj 2490H+j*20H | TCXWDSGNj 249CH+j*20H
TCXACBNDj  2500H+j*18H | TCXACRNGj 2504H+j*18H | TCXACMSKj 2508H+j*18H
TCXIPBNDj  3000H+j*10H | TCXIPRNGj 3004H+j*10H        (all TCX comparators rw, MCDS Reset)
```
FIFOWARNx is x=0-1. FIFOOVRCNT is written through its CLR bit even though the Table 384 Write column is empty. MUX is documented as rw with protect bits. The empty Read cell looks like a table artefact, so check it by reading back.

### 5.2 CLC (0000H, PowerOn 0000 0003H) [V]
`DISR 0 rw`: "0B Module incl. TRAM enable (clock on) is requested. 1B Module incl. TRAM disable (clock off) is requested." "Note: Turn off requests are ignored while the Flush flag is not asserted." `DISS 1 rh`. `0 31:2 r`. "After Power On Reset the clock is turned off and needs to be enabled by the tool." Enable by writing 0000 0000H.

### 5.3 OCS (0004H, MCDS 0000 0000H) [V]
`SUS 27:24 rw`: "0H Will not suspend 1H Hard suspend: Not all outstanding messages in FIFOs are written to TRAM. After a few clock cycles messages are dropped. 2H Soft suspend: All outstanding messages in FIFOs are written to TRAM (until FIFOCTL.FFE is set). No messages are dropped." `SUS_P 28 w` ("SUS is only written when SUS_P is 1"). `SUSSTA 29 rh`. `0 23:0, 31:30 r`. The spec describes this as the Power-On-Reset "negotiated power down" path: "MINIMCDS will then store all remaining traces ... including a final <endoftrace> to TRAM ... Note: This requires an adapted upload algorithm as NOW is not retained."

### 5.4 ID (0008H) [V]
`MOD_REV 7:0 r`, `MOD_TYPE 15:8 r`, `MOD_NUMBER 31:16 r`. Table 400: PowerOn Reset **00D6 C007H**.

### 5.5 CT (0010H; PowerOn 0000 00A0H) [V]
`KOK 5 rh` ("always set"), `KAV 7 rh` ("always set"), `EN 13 rh`, `CLRE 14 w`, `SETE 15 w` ("the only way to set the MCDS enable flag"), `BED 16 rw` ("obsolete for miniMCDS"), `BED_P 19 w`, `IRA 21 rh`, `CLRI 22 w` ("IWA and IRA are reset"), `IWA 23 rh`, `RES 29 rh` (MCDS Reset in progress), `SETR 31 w` ("Request MCDS Reset"). Reserved: `4:0, 6, 12:8, 18:17, 20, 28:24, 30`.
"As long as this bit [EN] is not set no MCDS register (apart from CLC, OCS and CT) can be written even if the clock was turned on (via CLC)". Enable writes with CT = 0000 8000H. Request an MCDS reset with CT = 8000 0000H.

### 5.6 MUX (0014H, PowerOn 0000 0000H) [V]
`TMUX0 3:0 rw`, `TM0_P 7 w`, `TMUX1 11:8`, `TM1_P 15`, `TMUX2 19:16`, `TM2_P 23`, `TMUX3 27:24`, `TM3_P 31`, `0 6:4, 14:12, 22:20, 30:28 r`.
"TM1_P, TM2_P, TM3_P are not relevant for miniMCDS"; TMUX1..3 are "Not relevant for miniMCDS".
TMUX0: "This bit field can only be changed, if TM0_P is written '1' simultaneously." Values: "0H Nothing selected 1H CPU0 or CPU0_MEMSlave 2H CPU1 or CPU1_MEMSlave 3H CPU2 or CPU2_MEMSlave 4H CPU3 or CPU3_MEMSlave 5H CPU4 or CPU4_MEMSlave 6H CPU5 or CPU5_MEMSlave 9H LMU0 AH OLDA DH OTGB EH OTGBM (GTM) others, reserved".
To select CPU0, write MUX = 0000 0081H. For LMU0, write 0000 0089H.

### 5.7 MUX_TC_RC (0020H, PowerOn 0000 0000H) [V]
`TC_MUXz (z=0-5) 2*z+1:2*z rw`: "00B CPUz 01B CPUz 10B CPUz_MEMSlave (PSPR, DSPR ...) others, reserved". Writable only "if TC_TM_P is written '1' simultaneously". `TC_TM_P 15 w`. `RC 24 rw`: "0B Reference is fSOURCE0/24 (REFCLK1). 1B Reference is fSOURCE1/24 (REFCLK2)" (write it together with `RC_P 27 w`). "no reference clock faster than half the emulation clock must be used." Reserved: `14:12, 23:16, 26:25, 31:28`.
[V] "MUX_TC_RC register shall be configured first, before the MUX register."
Values: CPU0 pipeline = 0000 8000H. CPU0_MEMSlave = 0000 8002H. REFCLK2 = 0900 0000H.

### 5.8 What must be enabled [V]
- "The MINIMCDS subsystem is only accessible from Cerberus and CPU0 if OCDS is enabled and the clock is turned on in the CLC register. Otherwise an SRI bus error is reported". The memory map adds: "MINIMCDS SFR may only be accessed when OCDS is enabled."
- "Attention: All trace signals are only visible if the trace boundary is turned on. This must be actively done by the tool through setting of control bit CBS_OSTATE.EECTRC".
- CBS (base F000 0400H). OSTATE is at 0080H (F000 0480H) with `EECTRC 5 rh`: "Can be set and cleared with OCNTRL.OC4 only ... 1B All trace ports towards MCDS are turned on." OCNTRL is at 007CH (F000 047CH): `OCx_P 2*x w`, `OCx 2*x+1 w`, and "OSTATE.OC4 is also called OSTATE.EECTRC (OEN protected)". [D] Write CBS_OCNTRL = 0000 0300H (OC4_P=1 at bit 8, OC4=1 at bit 9) with OSTATE.OEN = 1. Then read OSTATE bit 5 back.
- Trace-source semantics [V ch.1.3.4.2]: "The trace multiplexer within the CPU Subsystem allows selection between: CPU pipeline; PSPR/DSPR/DLMU SRI slave".
  - [D] **CPU0 pipeline** (TC_MUX0 = 00/01, TMUX0 = 1): every load and store CPU0 executes, to any address (DSPR, LMU, SFR), at the address CPU0 used. Master bits are invalid.
  - **CPU0_MEMSlave** (TC_MUX0 = 10): accesses by any SRI master arriving at CPU0's PSPR/DSPR/DLMU, with the master and submaster fields valid ([V] "Transaction originator (bus master, etc.) information included for accesses to CPUx_MEMSLAVE").
  - **LMU0** (TMUX0 = 9H): accesses arriving at the LMU0 slave. The spec does not state the address format seen in the slave sources (local or global). Check empirically.
- [V 9.4.2] "In OTGB and OTGBM trace use cases the data is written to address 0 because miniMCDS supports only a 32 bit data trace."
- [V 9.1] "In case the fastest clock domain to trace is faster than 150MHz, the PLL setting must be 2:1 for the MCDS input side."

---

## 6. MCX (TQU_MCX): registers and Table 394

### 6.1 Registers [V]
- **MCXEVTx (x=0-15), 0800H+x*4, MCDS 0:** `EIQq (q=0-15) 2*q+1:2*q rw`: "how the trigger input, named in the row of Table 394 with value x in the column corresponding to this event definition, is connected to the AND gate. 00B Event is disabled (input replaced by 0). 01B Trigger x is inverted. 10B Trigger x is connected directly. 11B Trigger x is ignored (replaced by 1)." [D] The reset value 0 therefore means "never". Set every unused EIQ to 11B. For example, "only bit b" = FFFF FFFFH with field b = 10B.
- **MCXACTx (x=0-41), 0880H+x*4, MCDS 0:** `AISq 8*q+4:8*q` (event index per Table 395), `AIQq 8*q+6:8*q+5` ("00B Input is ignored (replaced by 0). 01B ... inverted. 10B ... connected directly. 11B Input is replaced by 1."), `LVq 8*q+7` ("0B Look at inactive to active transitions. 1B Look for active state." and "The action is active for one emulation clock cycle if LVq is programmed 0"). The four inputs are ORed. [D] Examples: level on MCXEVTn = `0xC0|n`; edge = `0x40|n`; always = `0x60`.
- **MCXCCLj (j=0-7), 0A00H+j*10H:** `INCq 16*q+6:16*q` (Table 396; "value 0 ... does select nothing"), `ILVq 16*q+7` (0 = count edges, 1 = count cycles while asserted), `CLRq 16*q+13:16*q+8` (Table 397), `CLVq 16*q+15` (0 = clear on edge "Concurrent increments are not lost", 1 = clear while asserted), `0 14, 30 r`.
- **MCXLMTj, 0A04H+j*10H:** `LIMIT 15:0 rw` ("If the counter is higher than this value the trigger output (cnt_trig) of the counter is asserted. Writing this register has the side effect of clearing the counter ... Setting this register to 0000 FFFFH will never raise the trigger."), `MODq q+30 rw`, `0 29:16`. "The counter does not overflow but remain at its physical end value (all '1') until cleared." "Cascaded counters up to a width of 31 bits (only) can be build. The lower counter shall be 15 bit wide."
- **MCXCNTj, 0A08H+j*10H, PowerOn:** `COUNT 15:0 rh`.
- Event pools [V]: Table 395 (action inputs): 0…15 = MCXEVT0…15, 16…19 = mcx_trig[0…3], 20…31 = sync_rq. Table 396 (count): 1…15 = MCXEVT1…15 ("MCXEVT0 is not available, 0 is the code for 'none'"). Table 397 (clear): 1…15 = MCXEVT1…15, 16…31 = sync_rq.
- Table 398 [V]: 0 tsu_rel_en, 1 tsu_rel_sync, 2 tsu_abs_en, 3 tsu_abs_sync, 4 tsu_rel, 5-12 wtu_enable[0-7], 13-20 wtu_cnt[0-7], 21 tick_enable, 22 trace_done, 23 break_out, 24 suspend_out, 25 dcu_enable, 26 dcu_sync, 27 dtu_wdat, 28 dtu_wadr, 29 dtu_rdat, 30 dtu_radr, 31 ptu_enable, 32 ptu_nesting, 33 ptu_sync, 34-37 mcx_trig[0-3], 38-41 trig_out[0-3].
- [V] Programming order: "it is recommended to program MINIMCDS ordered Action - Event - Comparator - Counter after MCDS Reset." Also: "As long as signal trigger_dis is asserted all inputs to the trigger pools and counter pools of all TQUs are forced '0' ... De-assertion of the signal will cause a concurrent rising edge of all triggers which still happen to be driven '1'" (TRDIS in FIFOCTL: "usually the last one written by the tool").

### 6.2 Table 394: Trigger Pool to Event Definition Allocation Matrix

Rows as extracted by the flat text (occupied cells only, left to right) [V]:
```
core_crevt 0 0 0 0 | core_trig0 1 0 | core_trig1 2 1 1 | core_trig2 2 0 2 | core_trig3 1 1 3 | core_trig4 2 1
core_trig5 2 | core_trig6 0 | core_trig7 1 | core_tr0evt 2 3 3 2 | core_tr1evt 3 4 4 3 | core_exevt 4 4 4
core_swevt 5 5 5 | ptu_trig[0] 3 6 6 0 0 0 0 0 0 0 | ptu_trig[1] 3 6 1 1 1 1 1 1 1
dtu_ea_trig[0] 2 2 2 2 2 2 2 | dtu_ea_trig[1] 3 3 3 3 3 3 3 | dtu_dat_trig[0] 4 4 4 4 4 4 | dtu_dat_trig[1] 5 5 4 5 5
dtu_acc_trig[0] 6 5 6 6 6 | dtu_acc_trig[1] 5 7 7 5 7 | dcu_sus 7 7 | dcu_idle 4 4 7 | dcu_halt 5 5 8 | dcu_isr 5 6 9
break_in 6 6 8 8 7 5 6 8 8 6 7 8 0 0 | fifo_trg[0] 7 7 9 9 8 6 7 9 7 9 1 1 | fifo_trg[1] 8 8 9 8 6 9 8 8 10 2 2
tsu_tc_trig 9 9 10 10 7 9 7 10 9 9 11 3 3 | mcx_trig[0] 10 10 11 11 8 10 8 10 10 4 4 | mcx_trig[1] 11 11 9 11 9 11 11 5 5
mcx_trig[2] 10 10 10 10 10 11 12 12 6 6 | mcx_trig[3] 11 11 11 11 11 13 13 7 7 | cnt_trig[0] 12 12 12 12 14 14 8 8
cnt_trig[1] 13 13 13 13 15 15 9 9 | cnt_trig[2] 14 14 14 14 12 12 12 12 10 10 | cnt_trig[3] 15 15 15 15 13 13 13 13 11 11
cnt_trig[4] 12 12 12 12 14 14 14 14 12 12 | cnt_trig[5] 13 13 13 13 15 15 15 15 13 13 | cnt_trig[6] 14 14 14 14 14 14
cnt_trig[7] 15 15 15 15 15 15
```

**Reconstructed column placement. Status: certain.** It was rebuilt from the PDF's own layout render (`pdftotext -layout`, which keeps character columns). Each number was snapped to the nearest header column, and all snaps fell within 2.5 characters. The resulting grid was then checked. It has 256 cells; every column MCXEVT0..15 contains each bit index 0..15 exactly once; every row keeps the flat-text order. Header: "Trigger [row] assigned to given bit number in MCXEVT[header]".

| Trigger | E0 | E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8 | E9 | E10 | E11 | E12 | E13 | E14 | E15 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| core_crevt | 0 | 0 |  | 0 | 0 |  |  |  |  |  |  |  |  |  |  |  |
| core_trig0 | 1 |  |  |  |  |  | 0 |  |  |  |  |  |  |  |  |  |
| core_trig1 | 2 | 1 |  |  |  |  | 1 |  |  |  |  |  |  |  |  |  |
| core_trig2 |  | 2 | 0 |  |  |  | 2 |  |  |  |  |  |  |  |  |  |
| core_trig3 |  |  | 1 | 1 |  |  | 3 |  |  |  |  |  |  |  |  |  |
| core_trig4 |  |  |  | 2 | 1 |  |  |  |  |  |  |  |  |  |  |  |
| core_trig5 |  |  |  |  | 2 |  |  |  |  |  |  |  |  |  |  |  |
| core_trig6 |  |  |  |  |  | 0 |  |  |  |  |  |  |  |  |  |  |
| core_trig7 |  |  |  |  |  | 1 |  |  |  |  |  |  |  |  |  |  |
| core_tr0evt |  |  | 2 | 3 | 3 | 2 |  |  |  |  |  |  |  |  |  |  |
| core_tr1evt |  |  | 3 | 4 | 4 | 3 |  |  |  |  |  |  |  |  |  |  |
| core_exevt |  |  | 4 |  |  | 4 | 4 |  |  |  |  |  |  |  |  |  |
| core_swevt |  |  | 5 | 5 |  | 5 |  |  |  |  |  |  |  |  |  |  |
| ptu_trig[0] | 3 |  | 6 |  |  | 6 |  | 0 | 0 | 0 | 0 | 0 | 0 | 0 |  |  |
| ptu_trig[1] |  | 3 |  | 6 |  |  |  | 1 | 1 | 1 | 1 | 1 | 1 | 1 |  |  |
| dtu_ea_trig[0] |  |  |  |  |  |  |  | 2 | 2 | 2 | 2 | 2 | 2 | 2 |  |  |
| dtu_ea_trig[1] |  |  |  |  |  |  |  | 3 | 3 | 3 | 3 | 3 | 3 | 3 |  |  |
| dtu_dat_trig[0] |  |  |  |  |  |  |  | 4 | 4 | 4 | 4 |  | 4 | 4 |  |  |
| dtu_dat_trig[1] |  |  |  |  |  |  |  |  | 5 |  | 5 | 4 | 5 | 5 |  |  |
| dtu_acc_trig[0] |  |  |  |  |  |  |  |  | 6 | 5 | 6 |  | 6 | 6 |  |  |
| dtu_acc_trig[1] |  |  |  |  |  |  |  | 5 | 7 |  | 7 | 5 |  | 7 |  |  |
| dcu_sus |  |  | 7 | 7 |  |  |  |  |  |  |  |  |  |  |  |  |
| dcu_idle | 4 | 4 |  |  |  | 7 |  |  |  |  |  |  |  |  |  |  |
| dcu_halt |  | 5 |  |  | 5 | 8 |  |  |  |  |  |  |  |  |  |  |
| dcu_isr | 5 |  |  |  | 6 | 9 |  |  |  |  |  |  |  |  |  |  |
| break_in | 6 | 6 | 8 | 8 | 7 |  | 5 | 6 | 8 |  | 8 | 6 | 7 | 8 | 0 | 0 |
| fifo_trg[0] | 7 | 7 | 9 | 9 | 8 |  | 6 | 7 | 9 |  |  | 7 |  | 9 | 1 | 1 |
| fifo_trg[1] | 8 | 8 |  |  | 9 |  |  | 8 |  | 6 | 9 | 8 | 8 | 10 | 2 | 2 |
| tsu_tc_trig | 9 | 9 | 10 | 10 |  |  | 7 | 9 |  | 7 | 10 | 9 | 9 | 11 | 3 | 3 |
| mcx_trig[0] | 10 | 10 | 11 | 11 |  |  | 8 | 10 |  | 8 |  | 10 | 10 |  | 4 | 4 |
| mcx_trig[1] | 11 | 11 |  |  |  |  | 9 | 11 |  | 9 |  | 11 | 11 |  | 5 | 5 |
| mcx_trig[2] |  |  |  |  | 10 | 10 | 10 |  | 10 | 10 | 11 |  | 12 | 12 | 6 | 6 |
| mcx_trig[3] |  |  |  |  | 11 | 11 | 11 |  | 11 | 11 |  |  | 13 | 13 | 7 | 7 |
| cnt_trig[0] | 12 | 12 | 12 | 12 |  |  |  |  |  |  |  |  | 14 | 14 | 8 | 8 |
| cnt_trig[1] | 13 | 13 | 13 | 13 |  |  |  |  |  |  |  |  | 15 | 15 | 9 | 9 |
| cnt_trig[2] | 14 | 14 | 14 | 14 |  |  |  |  | 12 | 12 | 12 | 12 |  |  | 10 | 10 |
| cnt_trig[3] | 15 | 15 | 15 | 15 |  |  |  |  | 13 | 13 | 13 | 13 |  |  | 11 | 11 |
| cnt_trig[4] |  |  |  |  | 12 | 12 | 12 | 12 | 14 | 14 | 14 | 14 |  |  | 12 | 12 |
| cnt_trig[5] |  |  |  |  | 13 | 13 | 13 | 13 | 15 | 15 | 15 | 15 |  |  | 13 | 13 |
| cnt_trig[6] |  |  |  |  | 14 | 14 | 14 | 14 |  |  |  |  |  |  | 14 | 14 |
| cnt_trig[7] |  |  |  |  | 15 | 15 | 15 | 15 |  |  |  |  |  |  | 15 | 15 |

Inverse view (EIQ bit to trigger), for the columns that carry DTU triggers:
```
MCXEVT7 : 0 ptu_trig0, 1 ptu_trig1, 2 ea0, 3 ea1, 4 dat0, 5 acc1, 6 break_in, 7 fifo0, 8 fifo1, 9 tsu_tc, 10 mcx0, 11 mcx1, 12-15 cnt4-7
MCXEVT8 : 0 ptu0, 1 ptu1, 2 ea0, 3 ea1, 4 dat0, 5 dat1, 6 acc0, 7 acc1, 8 break_in, 9 fifo0, 10 mcx2, 11 mcx3, 12-15 cnt2-5
MCXEVT9 : 0 ptu0, 1 ptu1, 2 ea0, 3 ea1, 4 dat0, 5 acc0, 6 fifo1, 7 tsu_tc, 8 mcx0, 9 mcx1, 10 mcx2, 11 mcx3, 12-15 cnt2-5
MCXEVT10: 0 ptu0, 1 ptu1, 2 ea0, 3 ea1, 4 dat0, 5 dat1, 6 acc0, 7 acc1, 8 break_in, 9 fifo1, 10 tsu_tc, 11 mcx2, 12-15 cnt2-5
MCXEVT11: 0 ptu0, 1 ptu1, 2 ea0, 3 ea1, 4 dat1, 5 acc1, 6 break_in, 7 fifo0, 8 fifo1, 9 tsu_tc, 10 mcx0, 11 mcx1, 12-15 cnt2-5
MCXEVT12: 0 ptu0, 1 ptu1, 2 ea0, 3 ea1, 4 dat0, 5 dat1, 6 acc0, 7 break_in, 8 fifo1, 9 tsu_tc, 10 mcx0, 11 mcx1, 12 mcx2, 13 mcx3, 14 cnt0, 15 cnt1
MCXEVT13: 0 ptu0, 1 ptu1, 2 ea0, 3 ea1, 4 dat0, 5 dat1, 6 acc0, 7 acc1, 8 break_in, 9 fifo0, 10 fifo1, 11 tsu_tc, 12 mcx2, 13 mcx3, 14 cnt0, 15 cnt1
MCXEVT14/15: 0 break_in, 1 fifo0, 2 fifo1, 3 tsu_tc, 4-7 mcx_trig0-3, 8-15 cnt_trig0-7
```
[D] Trigger sources, inferred from the names and the TQU_MCX feature list: ptu_trig[j] come from TCXIPBND/RNGj, dtu_ea_trig[j] from TCXEABND/RNGj, dtu_dat_trig[j] from TCXWD*j, and dtu_acc_trig[j] from TCXAC*j. fifo_trg[x] comes from FIFOWARNx, tsu_tc_trig is the TSU prescaler, mcx_trig[k] come from MCXACT34+k, and cnt_trig[j] is counter j > LIMIT. The revision history confirms one of these: "Trigger dtu_acc_wr/dtu_acc_rd renamed to dtu_acc_trig[0/1]". core_* are defined in Table 393 (EXEVT, SWEVT, CREVT, TR0EVT/TR1EVT, TRnADR).

---

## 7. Programming sequences

The spec gives no step-by-step example for data or watch-point trace. It gives only the fragments marked [V]. The sequence below is [D], assembled from those fragments.

**A. Data trace of a write range [lo, hi] on CPU0 (e.g. DSPR0 or LMU variables):**
1. CBS: OSTATE.OEN = 1 (already needed for DAP). CBS_OCNTRL = 0000 0300H sets EECTRC. Check that OSTATE bit 5 reads 1.
2. MINIMCDS CLC = 0 (clock on; poll DISS = 0). CT = 0000 8000H (SETE). Optionally CT = 8000 0000H (SETR) first for a clean MCDS reset, then poll CT.RES = 0 and set SETE again.
3. MUX_TC_RC = 0000 8000H (CPU0 pipeline) or 0000 8002H (CPU0_MEMSlave: DSPR accesses from any master). Then MUX = 0000 0081H (TMUX0 = CPU0). For LMU0 as a slave, use MUX = 0000 0089H.
4. FIFOCTL = 0000 0800H (TROFF) while configuring. [V] The FIFOCTL write that clears TRDIS is "usually the last one written by the tool".
5. Actions first ([V] order "Action - Event - Comparator - Counter"): MCXACT27 (dtu_wdat) = MCXACT28 (dtu_wadr) = 0000 00C7H (level, direct, MCXEVT7). For reads, use MCXACT29 and MCXACT30. Optionally MCXACT0 = 0000 0060H (tsu_rel_en always, giving one TSR at start) and MCXACT21 = 0000 0060H (tick_enable always).
6. Events: MCXEVT7 = FFFF FFEFH (EIQ2 = 10B dtu_ea_trig[0]; the rest are 11B).
7. Comparators: TCXEABND0 = lo, TCXEARNG0 = hi - lo. If master IDs are wanted in the address, set TCXACMSK0 = TCXACMSK1 = 0000 3FFFH (MSK = 0).
8. Buffer: FIFOBOT = 0, FIFOTOP = 0000 1FFFH. FIFOPRE = 0000 1FE0H for circular Pre-Trigger-only, or 0 for "fill once then stop"; see Table 382. Fill the TRAM through B800 0000H with FFFF FFFFH (this also writes `<endoftrace>` at each paragraph start).
9. FIFOCTL = 0000 0400H (TRON), then FIFOCTL = 0000 4000H (CLR, start). Combining both in one write is not documented.
10. Stop: FIFOCTL = 0000 8000H (SET). Poll FFE = 1. Read FIFONOW and the TRAM, then apply Table 382 step 7. Check FIFOOVRCNT and ERR messages.

**B. Watch-point (WPS) per write to one address:** TCXEABND0 = addr, TCXEARNG0 = 0. TCXACMSK0 = 0000 1000H, TCXACBND0 = 0000 1000H, TCXACRNG0 = 0 (dtu_acc_trig[0] = write cycle). MCXEVT8: EIQ2 (ea0) = 10B, EIQ6 (acc0) = 10B, the rest 11B, which gives FFFF EFEFH. MCXACT5 (wtu_enable[0]) = 0000 0048H (edge, direct, MCXEVT8). Each hit then produces a 12-bit WPS with ID 0. Add tick_enable or TSR sync for timing. For the value as well, keep the DTU enables of sequence A on the same event.

**C. Counter snapshot (EVC):** MCXCCL0.INC0 = event, ILV0 = 0. MCXLMT0 = FFFFH (never triggers; writing it also clears the counter). MCXACT13 (wtu_cnt[0]) is edge on the sampling event, for example `tsu_tc_trig` via MCXEVT14 bit 3.

---

## 8. Open items / not in the spec
- The exact semantics of `<tick>` vs `<multick>`/`<ticount>` (§3.3) and the `<ticount>` grammar typo.
- Where the dsize field sits in DTWD/DTRD, and the bit order of the 12-bit master/subchannel/SVM prefix inside addr44 (§1.3). Both are strongly implied but not drawn in the spec.
- The data XOR base update after a size change (§1.4).
- The address format (local, global, or offset) seen by the CPUx_MEMSlave and LMU0 sources, and whether the pipeline source reports D000xxxxH or 7000xxxxH style addresses.
- MAXCNT value (the spec says it is below 256 and "not important").
- There are no TSUCTRL/TSUPRSC registers beyond TSUREFCNT, TSUPRSCL and TSUEMUCNT. The only TSU "control" is through MCXACT0-4 and MUX_TC_RC.RC.
