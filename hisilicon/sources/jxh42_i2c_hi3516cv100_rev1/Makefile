#
# sensor lib Makefile
#
export TOP_DIR=../../../../	
include ../../../../Rules.make

TARGETS := libsns_jxh42_i2c.a
TARGETS_SO := libsns_jxh42_i2c.so

SRC :=$(wildcard *.c)
SRC += $(wildcard *.cpp)

OBJS := $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SRC)))

CXXFLAGS := -Wall -I./
CXXFLAGS += -I../../mpp_include

all:$(TARGETS) $(TARGETS_SO)

$(TARGETS):$(OBJS)
	$(AR) rcs $@ $^
$(TARGETS_SO):$(OBJS)
	$(CC) -shared -fPIC -o $@ $^

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

%.o: %.c
	$(CC) -c $(CXXFLAGS) -o $@ $<

clean:
	rm -r $(TARGETS) $(TARGETS_SO);rm *.o

