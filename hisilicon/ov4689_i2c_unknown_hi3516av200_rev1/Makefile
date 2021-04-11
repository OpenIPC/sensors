#
# sensor lib Makefile
#
	
include ../../../../Rules.make

TARGETS := libsns_ov2710.a
TARGETS_SO := libsns_ov2710.so
PRINTF := @printf "\033[31;1m\t%s\033[0m\n"

SRC :=$(wildcard *.c)
SRC += $(wildcard *.cpp)

OBJS := $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SRC)))

CXXFLAGS := -Wall -I./
CXXFLAGS += -I../../mpp_include

all:$(TARGETS) $(TARGETS_SO)

$(TARGETS):$(OBJS)
	$(PRINTF) "AR  $@";$(AR) rcs $@ $^
$(TARGETS_SO):$(OBJS)
	$(PRINTF) "CC  $@";$(CC) -shared -fPIC -o $@ $^

%.o: %.cpp
	$(PRINTF) "$(CXX) $@";$(CXX) -c $(CXXFLAGS) -o $@ $<

%.o: %.c
	$(PRINTF) "$(CC) $@";$(CC) -c $(CXXFLAGS) -o $@ $<

clean:
	$(PRINTF) "RM  $(TARGETS) $(TARGETS_SO)";rm -r $(TARGETS) $(TARGETS_SO);rm *.o

