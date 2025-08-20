CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2

# Executable name
TARGET = filter_test

# Source files
SRCS = filter_test.cpp filter.cpp matrix_utils.cpp models.cpp
OBJS = $(SRCS:.cpp=.o)

# Default rule
all: $(TARGET)

# Link step
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile step
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
