CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic -pthread

WEB_TARGET = imu_web_publisher
SOCKET_TARGET = imu_socket_server
THREAD_TARGET = imu_multithread_server

.PHONY: all clean

all: $(WEB_TARGET) $(SOCKET_TARGET) $(THREAD_TARGET)

$(WEB_TARGET): imu_web_publisher.cpp imu_core.h
	$(CXX) $(CXXFLAGS) imu_web_publisher.cpp -o $(WEB_TARGET)

$(SOCKET_TARGET): imu_socket_server.cpp imu_core.h
	$(CXX) $(CXXFLAGS) imu_socket_server.cpp -o $(SOCKET_TARGET)

$(THREAD_TARGET): imu_multithread_server.cpp
	$(CXX) $(CXXFLAGS) imu_multithread_server.cpp -o $(THREAD_TARGET)

clean:
	rm -f $(WEB_TARGET) $(SOCKET_TARGET) $(THREAD_TARGET)
