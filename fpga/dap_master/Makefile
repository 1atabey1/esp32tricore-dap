# DAP master for the ICE40UP5K.
#
# `make sim`   iverilog testbenches - the protocol, with no hardware involved
# `make synth` yosys, to see what each block costs in logic cells
#
# `make bitstream` yosys + nextpnr + icepack -> build/dap_master.bin
#
# The fabric runs at 48 MHz from the internal oscillator, which is the fastest
# it offers.  Place and route closes just under 50, so the margin is thin and
# earned rather than free - see FABRIC_MHZ below.  Nothing here runs above its
# static timing because it happened to work on the bench; that is how a probe
# becomes intermittent months later.

RTL := rtl/dap_crc6.v rtl/dap_frame_tx.v rtl/dap_frame_rx.v \
       rtl/spi_slave.v rtl/dap_top.v
BUILD := build

TESTS := tb_dap_frame tb_dap_rx tb_spi_slave tb_dap_top
TOPS  := dap_frame_tx dap_frame_rx spi_slave dap_top

.PHONY: sim synth bitstream clean $(TESTS)

sim: $(TESTS)

$(TESTS): %: $(BUILD)/%
	@cd $(BUILD) && vvp $*

$(BUILD)/tb_%: tb/tb_%.v $(RTL) | $(BUILD)
	@iverilog -g2012 -o $@ $(RTL) $<

# 48 MHz, straight off SB_HFOSC with no divider.
#
# Worth the work twice over: the DAP clock and the SPI link are both fractions
# of this one number, so the wire and the drain each double.  Getting here took
# registered half-period strobes and terminal counts, a shifting transmitter
# instead of an indexed one, a register-read mux split in two behind a dummy
# byte, per-bit decisions lifted out of clock enables, and a short reset list.
#
# The margin is thin and worth knowing about: the best seed closes at 48.5 MHz
# against a 48 MHz oscillator, about 1%, on nextpnr's typical-corner model.
# That meets timing rather than comfortably exceeding it.  If this design grows,
# expect to have to re-earn the frequency rather than assume it.
FABRIC_MHZ := 48

# Pinned, because the placer is randomised and the spread between seeds is
# several MHz on a design this full - larger than the remaining logic
# differences.  Seed 1 gives 50.40 MHz, seed 2 gives 49.81 and seed 8 gives
# 49.78; seven of the first twelve clear 48.  A bitstream that only meets
# timing on a seed nobody recorded is not a design that meets timing.
# `bash seedsweep.sh build/board.json <seeds...>` re-checks - and it has to be
# re-run whenever the netlist changes, because a seed's advantage belongs to
# the netlist it was found on.  Wide mode made that concrete: seed 24 was the
# best seed at 48.47, and on the wide netlist it lands at 47.92 and fails.
#
# Use the sweep as a distribution, not a search for one lucky seed.  Only one
# seed in forty cleared 48 at one point during the wide-mode work, which said
# the design was over its limit rather than that the seed was good; the fix was
# to find the two paths responsible - see the capture tap in dap_top and the
# split control read mux - after which most seeds clear it.
SEED := 2

bitstream: $(BUILD)/dap_master.bin

$(BUILD)/board.json: $(RTL) rtl/dap_board_top.v | $(BUILD)
	yosys -q -p "read_verilog $(RTL) rtl/dap_board_top.v; \
	             synth_ice40 -top dap_board_top -json $@"

$(BUILD)/dap_master.asc: $(BUILD)/board.json dap_master.pcf
	nextpnr-ice40 --up5k --package sg48 --json $< --pcf dap_master.pcf \
	              --asc $@ --freq $(FABRIC_MHZ) --seed $(SEED)

$(BUILD)/dap_master.bin: $(BUILD)/dap_master.asc
	icepack $< $@
	@echo "bitstream: $@"

synth: | $(BUILD)
	@for top in $(TOPS); do \
		yosys -p "read_verilog $(RTL); synth_ice40 -top $$top -json $(BUILD)/$$top.json; stat" \
			2>/dev/null | grep -E "^=== $$top|Number of cells"; \
	done

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
