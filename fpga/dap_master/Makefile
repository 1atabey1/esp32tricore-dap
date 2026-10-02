# DAP master for the ICE40UP5K.
#
# `make sim`       iverilog testbenches, no hardware involved
# `make synth`     yosys, logic cells per block
# `make bitstream` yosys + nextpnr + icepack -> build/dap_master.bin

RTL := rtl/dap_crc6.v rtl/dap_frame_tx.v rtl/dap_frame_rx.v \
       rtl/spi_slave.v rtl/dap_top.v
BUILD := build

TESTS := tb_dap_frame tb_dap_rx tb_spi_slave tb_dap_top
TOPS  := dap_frame_tx dap_frame_rx spi_slave dap_top

.PHONY: sim synth bitstream clean $(TESTS)

sim: $(TESTS)

$(TESTS): %: $(BUILD)/%
	@cd $(BUILD) && vvp $*

# dap_top instantiates SB_IO; the benches take yosys's own model of it.
YOSYS_DATDIR ?= $(dir $(shell command -v yosys))../share/yosys
CELLS_SIM    := $(YOSYS_DATDIR)/ice40/cells_sim.v

$(BUILD)/tb_%: tb/tb_%.v $(RTL) | $(BUILD)
	@iverilog -g2012 -o $@ $(RTL) $(CELLS_SIM) $<

# 48 MHz, straight off SB_HFOSC with no divider.  Timing margin is thin
# (about 1% on nextpnr's typical corner); DAP clock and SPI both derive from it.
FABRIC_MHZ := 48

# Placement seed, pinned: the spread between seeds is several MHz.  Re-check
# with `bash seedsweep.sh build/board.json <seeds...>` whenever the netlist
# changes; if most seeds fail, fix the critical paths rather than hunt seeds.
SEED := 56

# The fast-mode (48 MHz DAP clock) netlist does not close: 46.7 MHz on its
# best seed of 120, master's 47-50.  Built anyway, as an experiment; drop
# this to have nextpnr refuse it again.
PNR_FLAGS := --timing-allow-fail

bitstream: $(BUILD)/dap_master.bin

$(BUILD)/board.json: $(RTL) rtl/dap_board_top.v | $(BUILD)
	yosys -q -p "read_verilog $(RTL) rtl/dap_board_top.v; \
	             synth_ice40 -top dap_board_top -json $@"

$(BUILD)/dap_master.asc: $(BUILD)/board.json dap_master.pcf
	nextpnr-ice40 --up5k --package sg48 --json $< --pcf dap_master.pcf \
	              --asc $@ --freq $(FABRIC_MHZ) --seed $(SEED) $(PNR_FLAGS)

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
