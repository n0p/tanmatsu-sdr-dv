# Host build of the tetra_rx component: library, unit tests, end-to-end
# tests on synthetic signals, CLI decoder for IQ recordings.
#
#   make            build everything
#   make test       run all tests that need no downloaded files
#   make bench      CPU cost and memory footprint
#   make riscv      compile-only check with the ESP32-P4 toolchain

CC ?= cc
TOP := ..
OT := $(TOP)/third_party/osmo-tetra
B := build

INC := -I$(TOP)/include -I$(TOP)/src -I$(OT)/compat -I$(OT)/src
# the library is built exactly as strict as on the target
LIB_CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror -Wdouble-promotion -Wshadow $(INC)
HOST_CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror $(INC) -D_POSIX_C_SOURCE=200809L

LIB_SRC := $(wildcard $(TOP)/src/*.c) \
	$(OT)/src/lower_mac/crc_simple.c $(OT)/src/lower_mac/tch_reordering.c \
	$(OT)/src/lower_mac/tetra_conv_enc.c $(OT)/src/lower_mac/tetra_interleave.c \
	$(OT)/src/lower_mac/tetra_rm3014.c $(OT)/src/lower_mac/tetra_scramb.c \
	$(OT)/src/phy/tetra_burst.c \
	$(OT)/src/tetra_cmce_pdu.c $(OT)/src/tetra_common.c $(OT)/src/tetra_llc_pdu.c \
	$(OT)/src/tetra_mac_pdu.c $(OT)/src/tetra_mle_pdu.c $(OT)/src/tetra_mm_pdu.c \
	$(OT)/src/tetra_sndcp_pdu.c
LIB_OBJ := $(patsubst $(TOP)/%.c,$(B)/lib/%.o,$(LIB_SRC))

HOST_COMMON := dsp_util.c tetra_gen.c print_events.c
HOST_OBJ := $(patsubst %.c,$(B)/%.o,$(HOST_COMMON))
PROGS := $(B)/tetra_rx_file $(B)/test_unit $(B)/test_e2e $(B)/bench

all: $(PROGS)

$(B)/lib/%.o: $(TOP)/%.c $(wildcard $(TOP)/src/*.h $(TOP)/include/*.h)
	@mkdir -p $(dir $@)
	$(CC) $(LIB_CFLAGS) -c $< -o $@

$(B)/%.o: %.c $(wildcard *.h $(TOP)/src/*.h $(TOP)/include/*.h)
	@mkdir -p $(dir $@)
	$(CC) $(HOST_CFLAGS) -c $< -o $@

$(B)/libtetra_rx.a: $(LIB_OBJ)
	$(AR) rcs $@ $^

$(B)/%: $(B)/%.o $(HOST_OBJ) $(B)/libtetra_rx.a
	$(CC) -o $@ $^ -lm

test: $(B)/test_unit $(B)/test_e2e
	$(B)/test_unit
	$(B)/test_e2e

bench: $(B)/bench
	$(B)/bench

# compile-only sanity check with the ESP32-P4 flags
RISCV_CC ?= $(firstword $(wildcard $(HOME)/.espressif/tools/riscv32-esp-elf/*/riscv32-esp-elf/bin/riscv32-esp-elf-gcc))
RISCV_CFLAGS := -march=rv32imafc_zicsr_zifencei -mabi=ilp32f -std=gnu17 -O2 \
	-Wall -Wextra -Werror -Wdouble-promotion -fstack-usage $(INC)
riscv:
	@mkdir -p $(B)/riscv
	@for f in $(LIB_SRC); do \
		o=$(B)/riscv/$$(basename $$f .c).o; \
		echo "RISCV $$f"; \
		$(RISCV_CC) $(RISCV_CFLAGS) -c $$f -o $$o || exit 1; \
	done
	@$(dir $(RISCV_CC))riscv32-esp-elf-size -t $(B)/riscv/*.o | tail -1

clean:
	find $(B) -type f -delete

.PHONY: all test bench riscv clean
.SECONDARY:
