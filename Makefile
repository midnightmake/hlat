CXX = g++
CXXFLAGS = -std=c++20 -O3 -Wall -Wextra
LDFLAGS = -static
TARGET = hlat.exe
SRC = hlat.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

clean:
	rm -rf $(TARGET)

rebuild: clean all

.PHONY: all clean rebuild
