SHELL := /usr/bin/env bash

GNSS_DATA_URL := https://sourceforge.net/projects/gnss-sdr/files/data/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.tar.gz
GNSS_DATA_ARCHIVE := 2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.tar.gz
GNSS_DATA_DIR := 2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN
GNSS_DATA_FILE := 2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat
GNSS_DATA_SRC := $(GNSS_DATA_DIR)/$(GNSS_DATA_FILE)

.PHONY: help fetch-gnss-data lint-vhdl sim-smoke sim-unit sim-chan-bank sim-chan-bank-nav-store sim-acq-file sim-acq-equiv sim-regress phase3-eval phase3-gate synth-check schematic schematic-local waves

help:
	@echo "Targets:"
	@echo "  make fetch-gnss-data - Download and extract GNSS-SDR sample data"
	@echo "  make lint-vhdl    - Run VHDL checks with NVC"
	@echo "  make sim-smoke    - Run smoke simulations"
	@echo "                     Set NVC_ENABLE_WAVE=0 to disable wave dump"
	@echo "  make sim-unit     - Run unit-level VHDL testbenches"
	@echo "  make sim-chan-bank - Run gps_l1_ca_chan_bank_tb"
	@echo "  make sim-chan-bank-nav-store - Run gps_l1_ca_chan_bank_nav_store_tb"
	@echo "  make sim-acq-file - Run gps_l1_ca_acq_tb with GNSS stimulus file replay"
	@echo "  make sim-acq-equiv - Run gps_l1_ca_acq_tb in TD and FFT modes and compare tuples"
	@echo "  make sim-regress  - Run regression suite"
	@echo "  make phase3-eval  - Run smoke + Phase 3 metric report (non-gating)"
	@echo "                     Set PHASE3_ENABLE_WAVE=0 to disable wave dump"
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

sim-chan-bank:
	@./lint/lint_vhdl.sh
	@mkdir -p "$$(dirname "$${CHAN_BANK_WAVE_FILE:-sim/gps_l1_ca_chan_bank_tb.fst}")"
	@nvc --std=2008 --stderr="$${NVC_STDERR_LEVEL:-none}" -e \
		-gG_USE_FILE_INPUT="$${CHAN_BANK_USE_FILE_INPUT:-true}" \
		-gG_INPUT_FILE="$${CHAN_BANK_INPUT_FILE:-2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat}" \
		-gG_FILE_SAMPLE_RATE_SPS="$${CHAN_BANK_FILE_SAMPLE_RATE_SPS:-4000000}" \
		-gG_DUT_SAMPLE_RATE_SPS="$${CHAN_BANK_DUT_SAMPLE_RATE_SPS:-2000000}" \
		-gG_MAX_FILE_SAMPLES="$${CHAN_BANK_MAX_FILE_SAMPLES:-3000000}" \
		gps_l1_ca_chan_bank_tb
	@nvc --std=2008 --stderr="$${NVC_STDERR_LEVEL:-none}" -r gps_l1_ca_chan_bank_tb \
		--stop-time="$${CHAN_BANK_TB_STOP_TIME:-80ms}" \
		--wave="$${CHAN_BANK_WAVE_FILE:-sim/gps_l1_ca_chan_bank_tb.fst}"

sim-chan-bank-nav-store:
	@./lint/lint_vhdl.sh
	@mkdir -p "$$(dirname "$${CHAN_BANK_NAV_STORE_WAVE_FILE:-sim/gps_l1_ca_chan_bank_nav_store_tb.fst}")"
	@nvc --std=2008 --stderr="$${NVC_STDERR_LEVEL:-none}" -e \
		-gG_RUN_MS="$${CHAN_BANK_NAV_STORE_RUN_MS:-40}" \
		-gG_USE_FILE_INPUT="$${CHAN_BANK_NAV_STORE_USE_FILE_INPUT:-true}" \
		-gG_INPUT_FILE="$${CHAN_BANK_NAV_STORE_INPUT_FILE:-2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat}" \
		-gG_FILE_SAMPLE_RATE_SPS="$${CHAN_BANK_NAV_STORE_FILE_SAMPLE_RATE_SPS:-4000000}" \
		-gG_DUT_SAMPLE_RATE_SPS="$${CHAN_BANK_NAV_STORE_DUT_SAMPLE_RATE_SPS:-2000000}" \
		gps_l1_ca_chan_bank_nav_store_tb
	@nvc --std=2008 --stderr="$${NVC_STDERR_LEVEL:-none}" -r gps_l1_ca_chan_bank_nav_store_tb \
		--stop-time="$${CHAN_BANK_NAV_STORE_TB_STOP_TIME:-80ms}" \
		--wave="$${CHAN_BANK_NAV_STORE_WAVE_FILE:-sim/gps_l1_ca_chan_bank_nav_store_tb.fst}"

sim-acq-file:
	@./sim/run_acq_file_tb.sh

sim-acq-equiv:
	@./sim/run_acq_td_fft_equiv.sh

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
	@echo "VHDL waveform: sim/gps_l1_ca_acq_tb.fst"
	@echo "VHDL waveform: sim/gps_l1_ca_chan_bank_tb.fst"
	@echo "VHDL waveform: sim/gps_l1_ca_chan_bank_nav_store_tb.fst"
