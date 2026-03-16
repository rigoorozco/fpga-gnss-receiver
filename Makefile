SHELL := /usr/bin/env bash

GNSS_DATA_URL := https://sourceforge.net/projects/gnss-sdr/files/data/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.tar.gz
GNSS_DATA_ARCHIVE := 2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.tar.gz
GNSS_DATA_DIR := 2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN
GNSS_DATA_FILE := 2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat
GNSS_DATA_SRC := $(GNSS_DATA_DIR)/$(GNSS_DATA_FILE)

.PHONY: help fetch-gnss-data lint-vhdl sim-smoke sim-unit sim-regress phase3-eval phase3-gate synth-check schematic schematic-local waves

help:
	@echo "Targets:"
	@echo "  make fetch-gnss-data - Download and extract GNSS-SDR sample data"
	@echo "  make lint-vhdl    - Run VHDL checks with NVC"
	@echo "  make sim-smoke    - Run smoke simulations"
	@echo "  make sim-unit     - Run unit-level VHDL testbenches"
	@echo "  make sim-regress  - Run regression suite"
	@echo "  make phase3-eval  - Run smoke + Phase 3 metric report (non-gating)"
	@echo "  make phase3-gate  - Run smoke + Phase 3 metric report (gating)"
	@echo "  make synth-check  - Run synthesis sanity check"
	@echo "  make schematic    - Generate top-level schematics via GHDL+Yosys (Docker)"
	@echo "  make schematic-local - Generate top-level schematics via local GHDL+Yosys"
	@echo "  make waves        - Print waveform artifact paths"

$(GNSS_DATA_ARCHIVE):
	@echo "Fetching GNSS sample archive..."
	@if command -v curl >/dev/null 2>&1; then \
		curl -L --fail -o $(GNSS_DATA_ARCHIVE) $(GNSS_DATA_URL); \
	elif command -v wget >/dev/null 2>&1; then \
		wget -O $(GNSS_DATA_ARCHIVE) $(GNSS_DATA_URL); \
	else \
		echo "error: neither curl nor wget is available."; \
		exit 1; \
	fi

fetch-gnss-data: $(GNSS_DATA_ARCHIVE)
	@echo "Extracting GNSS sample archive..."
	@tar -xzf $(GNSS_DATA_ARCHIVE)
	@test -f $(GNSS_DATA_SRC) || (echo "error: expected data file not found after extraction: $(GNSS_DATA_SRC)" && exit 1)
	@ln -sfn $(GNSS_DATA_SRC) $(GNSS_DATA_FILE)
	@echo "Data ready at: $(GNSS_DATA_SRC)"
	@echo "Linked as: $(GNSS_DATA_FILE)"

lint-vhdl:
	@./lint/lint_vhdl.sh

sim-smoke:
	@./sim/run_smoke.sh

sim-unit:
	@./sim/run_unit_tbs.sh

sim-regress:
	@./sim/run_regression.sh

phase3-eval:
	@./sim/run_phase3_eval.sh

phase3-gate:
	@PHASE3_STRICT=1 ./sim/run_phase3_eval.sh

synth-check:
	@./synth/check_synth.sh

schematic:
	@./synth/gen_schematics_ghdl_yosys_docker.sh

schematic-local:
	@./synth/gen_schematics_ghdl_yosys.sh

waves:
	@echo "VHDL waveform: sim/gps_l1_ca_phase2_tb.fst"
