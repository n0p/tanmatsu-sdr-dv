# SPDX-License-Identifier: AGPL-3.0-only
# Host tests of the tetra_codec wrapper. Needs the ETSI archive (see ../README.md):
#   make test ZIP=/path/to/en_30039502v010301p0.zip
ZIP ?= ../../../en_30039502v010301p0.zip
CC  ?= gcc
B   := build
E   := $(B)/etsi
TETRA_RX := ../../tetra_rx

# Reference: the ETSI files compiled one by one, the way their own makefile does it.
REF      := tetra_op fbas_tet fexp_tet fmat_tet sub_dsp sub_sc_d sdec_tet scod_tet
REF_CHAN := sub_cd cdec_tet
REF_OBJS      := $(REF:%=$(B)/ref_%.o)
REF_CHAN_OBJS := $(REF_CHAN:%=$(B)/ref_%.o)

all: $(B)/test_codec $(B)/tetra_voice_file

$(E)/source.h: $(ZIP) ../prepare.py
	@mkdir -p $(B)
	python3 ../prepare.py $(ZIP) $(E)
	@touch $@

$(B)/ref_%.o: $(E)/source.h ref_adapt.h
	$(CC) -O2 -w -fcommon -include ref_adapt.h -I$(E) -c $(E)/$*.c -o $@

# The unit under test, built the way the firmware builds it.
$(B)/tetra_codec.o: ../src/tetra_codec.c ../include/tetra_codec.h $(E)/source.h
	$(CC) -O2 -Wall -fwhole-program -DTETRA_CODEC_ETSI -I../include -I$(E) -c $< -o $@

$(B)/test_codec: test_codec.c $(B)/tetra_codec.o $(REF_OBJS)
	$(CC) -O2 -Wall -I../include -I$(E) -include ref_adapt.h $^ -lm -o $@

$(TETRA_RX)/host/build/libtetra_rx.a:
	$(MAKE) -C $(TETRA_RX)/host

$(B)/tetra_voice_file: tetra_voice_file.c $(B)/tetra_codec.o $(REF_CHAN_OBJS) $(B)/ref_tetra_op.o $(TETRA_RX)/host/build/libtetra_rx.a
	$(CC) -O2 -Wall -I../include -I$(TETRA_RX)/include -I$(E) -include ref_adapt.h $^ -lm -o $@

REC := ../../../testdata/tetra_391262500_voicecall_72k.cs16

test: $(B)/test_codec $(B)/tetra_voice_file
	$(B)/test_codec
	$(B)/tetra_voice_file $(REC) $(B)/voicecall_8k.wav

clean:
	rm -rf $(B)

.PHONY: all test clean
