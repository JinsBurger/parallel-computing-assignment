CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I.  -g#-DDSTAR_VERBOSE
SRCS = main.cpp simulator.cpp schedular.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = MRTA

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
