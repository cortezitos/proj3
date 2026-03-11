CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic -pthread
TARGET = imu_server
PORT ?= 5555
WEB_OUTPUT ?= /var/www/html/imu-data/imu_quaternion.txt
LOG_FILE ?= /tmp/imu_server.log
RUN_ARGS = --port $(PORT) --web-output $(WEB_OUTPUT)

.PHONY: all clean run start stop restart status logs run_mock

all: $(TARGET)

$(TARGET): imu_multithread_server.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

run: $(TARGET)
	./$(TARGET) $(RUN_ARGS)

start: $(TARGET)
	pkill -x "$(TARGET)" || true
	umask 022 && nohup ./$(TARGET) $(RUN_ARGS) >$(LOG_FILE) 2>&1 </dev/null &
	@echo "Started $(TARGET)."
	@$(MAKE) status

stop:
	pkill -x "$(TARGET)" || true

restart: stop start

status:
	pgrep -ax "$(TARGET)" || echo "$(TARGET) is not running"

logs:
	test -f $(LOG_FILE) && tail -n 40 $(LOG_FILE) || echo "No log file at $(LOG_FILE)"

run_mock: $(TARGET)
	./$(TARGET) --mock --port $(PORT) --web-output /tmp/imu_quaternion.txt

clean:
	rm -f $(TARGET)
