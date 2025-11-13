CC ?= gcc
CFLAGS += -std=c11 -Wall -Wextra -Wno-unused-parameter -Iinclude -I../Core/Inc

ifeq ($(OS),Windows_NT)
LDFLAGS +=
else
CFLAGS += -pthread
LDFLAGS += -pthread -lm
endif

CORE_SRCS = \
	../Core/Src/base_node.c \
	../Core/Src/camera_node.c \
	../Core/Src/conveyor_node.c \
	../Core/Src/piston_node.c \
	../Core/Src/safety_node.c

SIM_SRCS = \
	src/freertos_sim.c \
	src/validation_main.c

SRCS = $(SIM_SRCS) $(CORE_SRCS)

TARGET = esteira_sim

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET)
