# The escs simulation (test/sim/README.md): the firmware's esc_session.h + fourway.h on the host.
#   make -C test/sim          build and run
ROOT := ../..
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra -Wno-unused-function -Wno-unused-parameter
INC := -Istubs -I. -I$(ROOT)/src/apps -I$(ROOT)/lib/blheli_bl -I$(ROOT)/lib/esc_setup -I$(ROOT)/lib/esc_lifecycle
DEFS := -DESC_BOARD_PANEL=1 -DESC_WIFI_STA=0 -DNUM_PIOS=2
OUT ?= /tmp/escs_sim

run: $(OUT)
	$(OUT)

$(OUT): test_escs_sim.cpp sim.cpp sim.h $(wildcard stubs/*.h stubs/*/*.h) $(ROOT)/src/apps/esc_session.h $(ROOT)/src/apps/fourway.h $(ROOT)/lib/esc_lifecycle/*.h $(ROOT)/lib/esc_setup/esc_setup.cpp
	$(CXX) $(CXXFLAGS) $(INC) $(DEFS) -o $@ test_escs_sim.cpp sim.cpp $(ROOT)/lib/esc_setup/esc_setup.cpp

.PHONY: run
