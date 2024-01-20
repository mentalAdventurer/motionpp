# Compiler settings
CC = g++
CFLAGS = -Wall -fPIC

# Library settings
LDFLAGS = -lgsl -lgslcblas

# Output directory
OUT_DIR = ./lib

# Target library
TARGET = $(OUT_DIR)/libsimulator.so

# Source and header files
SRC = src/simulator.cpp
HEADER = src/simulator.h

# Default target
all: $(TARGET)

# Rule to create the dynamic library
$(TARGET): $(SRC) $(HEADER)
	@mkdir -p $(OUT_DIR)
	$(CC) $(CFLAGS) -shared -o $@ $(SRC) $(LDFLAGS)

# Clean up
clean:
	rm -rf $(OUT_DIR)

