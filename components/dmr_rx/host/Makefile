# Host build of dmr_rx: library objects, unit tests, dmr_rx_file CLI.
#   make            build everything
#   make test       run all tests that need no downloads
#   make VOICE=0    build without the vocoder
ROOT   := ..
BUILD  ?= build
VOICE  ?= 1
CXX    ?= g++
CC     ?= gcc

WARN   := -Wall -Wextra -Werror -Wdouble-promotion
INC    := -I$(ROOT)/include -I$(ROOT)/src -I$(ROOT)/third_party/mmdvmhost \
          -I$(ROOT)/third_party/mbelib -I$(ROOT)/third_party/dsd
DEFS   :=
ifeq ($(VOICE),1)
DEFS   += -DDMR_RX_WITH_VOICE
endif
OPT    ?= -O2 -g
CXXFLAGS := -std=c++17 $(OPT) $(WARN) -fno-exceptions -fno-rtti $(INC) $(DEFS)
CFLAGS   := -std=c11 $(OPT) $(WARN) $(INC) $(DEFS)
# host-only code (tests, generator, CLI) may use the full language
HOSTCXXFLAGS := -std=c++17 $(OPT) $(WARN) $(INC) $(DEFS)

LIB_CXX := $(wildcard $(ROOT)/src/*.cpp) $(wildcard $(ROOT)/third_party/mmdvmhost/*.cpp)
LIB_C   := $(wildcard $(ROOT)/third_party/mbelib/*.c)
LIB_OBJ := $(patsubst $(ROOT)/%.cpp,$(BUILD)/%.o,$(LIB_CXX)) $(patsubst $(ROOT)/%.c,$(BUILD)/%.o,$(LIB_C))
LIB     := $(BUILD)/libdmr_rx.a

HOST_SRC := $(wildcard *.cpp)
BINS     := $(BUILD)/dmr_rx_file $(BUILD)/test_fec $(BUILD)/test_rx $(BUILD)/test_voice $(BUILD)/dmr_gen \
            $(BUILD)/bench $(BUILD)/test_data

all: $(BINS)

$(BUILD)/%.o: $(ROOT)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -c $< -o $@
$(BUILD)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@
$(BUILD)/host/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(HOSTCXXFLAGS) -MMD -c $< -o $@

$(LIB): $(LIB_OBJ)
	ar rcs $@ $^

$(BUILD)/dmr_rx_file: $(BUILD)/host/dmr_rx_file.o $(LIB)
	$(CXX) -o $@ $^ -lm
$(BUILD)/test_fec: $(BUILD)/host/test_fec.o $(BUILD)/host/dmr_gen_lib.o $(LIB)
	$(CXX) -o $@ $^ -lm
# packet data: block codings, checksums, reassembly, text message formats
$(BUILD)/test_data: $(BUILD)/host/test_data.o $(BUILD)/host/dmr_gen_lib.o $(LIB)
	$(CXX) -o $@ $^ -lm
# test_rx counts heap calls made from the library while it processes samples
$(BUILD)/test_rx: $(BUILD)/host/test_rx.o $(BUILD)/host/dmr_gen_lib.o $(LIB)
	$(CXX) -o $@ $^ -lm -Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc,--wrap=free
# speech decoder against the double precision reference (skips itself with VOICE=0)
$(BUILD)/test_voice: $(BUILD)/host/test_voice.o $(LIB)
	$(CXX) -o $@ $^ -lm
$(BUILD)/dmr_gen: $(BUILD)/host/dmr_gen.o $(BUILD)/host/dmr_gen_lib.o $(LIB)
	$(CXX) -o $@ $^ -lm
$(BUILD)/bench: $(BUILD)/host/bench.o $(BUILD)/host/dmr_gen_lib.o $(LIB)
	$(CXX) -o $@ $^ -lm

test: $(BUILD)/test_fec $(BUILD)/test_rx $(BUILD)/test_voice $(BUILD)/test_data
	$(BUILD)/test_fec
	$(BUILD)/test_data
	$(BUILD)/test_voice
	$(BUILD)/test_rx

clean:
	rm -rf $(BUILD)

-include $(shell find $(BUILD) -name '*.d' 2>/dev/null)
.PHONY: all test clean
