# make test   regenerate the vectors and run every testbench (stops on the first failure)
# make sim    run the testbenches without regenerating the vectors
# make clean  remove simulation outputs
#
# Recipe lines start with ">" instead of a tab (.RECIPEPREFIX), so this file
# survives copy/paste.

.RECIPEPREFIX = >
PYTHON ?= python3
GHDL   ?= ghdl
GFLAGS  = --std=08
TBS     = tb_mf tb_mag_sq tb_cfar tb_top

.PHONY: all test vectors sim clean

all: test

test: vectors sim

vectors:
> $(PYTHON) model/radar_model.py
> $(PYTHON) model/gen_coef_pkg.py

sim:
> $(GHDL) -i $(GFLAGS) rtl/*.vhd tb/*.vhd
> @for tb in $(TBS); do \
>   echo "=== $$tb ==="; \
>   $(GHDL) -m $(GFLAGS) $$tb > /dev/null || exit 1; \
>   $(GHDL) -r $(GFLAGS) $$tb > sim_$$tb.log 2>&1; rc=$$?; \
>   grep -E "PASS|FAIL|error" sim_$$tb.log; \
>   [ $$rc -eq 0 ] || { echo "$$tb failed, see sim_$$tb.log"; exit 1; }; \
> done
> @echo "All testbenches passed."

clean:
> rm -f *.cf *.ghw sim_*.log vectors/*_sim.txt
