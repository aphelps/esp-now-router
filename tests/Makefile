# Host unit tests for the router-used logic (relay, leader election, attach protocol). The
# idiomatic way is
#   pio test -e native      (from inside WLED_dev)
# This Makefile is a lightweight alternative + the gcov coverage path (pio's custom runner doesn't
# do coverage). Run from inside WLED_dev so the sibling WLED wire header resolves.
#
#   make test       # build + run every suite with your host c++
#   make coverage   # + gcov line coverage of the router-used code

CXX      ?= c++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -I../../WLED/usermods/ampworks -I../src
SUITES    = test_relay/test_relay test_election/test_election test_attach/test_attach test_translate/test_translate test_coord_mode/test_coord_mode test_coord_phase/test_coord_phase test_hmtl_to_wled/test_hmtl_to_wled test_onboard_cfg/test_onboard_cfg test_candidate/test_candidate
OUTDIR    = /tmp

.PHONY: test coverage clean

test:
	@for s in $(SUITES); do \
	  echo "== $$s =="; \
	  $(CXX) $(CXXFLAGS) -o $(OUTDIR)/enr_`basename $$s` $$s.cpp && $(OUTDIR)/enr_`basename $$s` || exit 1; \
	done

coverage:
	@for s in $(SUITES); do \
	  $(CXX) $(CXXFLAGS) --coverage -O0 -o $(OUTDIR)/enr_`basename $$s`_cov $$s.cpp && $(OUTDIR)/enr_`basename $$s`_cov; \
	done
	@gcov -r -m test_relay/test_relay.cpp test_election/test_election.cpp test_attach/test_attach.cpp test_translate/test_translate.cpp test_coord_mode/test_coord_mode.cpp test_coord_phase/test_coord_phase.cpp test_hmtl_to_wled/test_hmtl_to_wled.cpp test_onboard_cfg/test_onboard_cfg.cpp test_candidate/test_candidate.cpp 2>/dev/null | grep -A1 -E 'router_relay.h|router_election.h|router_attach.h|wled_translate.h|coordinator_mode.h|coordinator_phase.h|hmtl_to_wled.h|wled_onboard_cfg.h|wled_candidate.h|sensor_sync_protocol.h' || \
	  echo "(install gcov to see per-file line coverage of the router-used functions)"

clean:
	rm -f $(OUTDIR)/enr_* *.gcno *.gcda *.gcov
