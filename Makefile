SHELL := /bin/bash

PI_HOST ?= 192.168.0.179
PI_USER ?= azat
PI_ROOT ?= ~/proj3
PI_ROOT_ABS ?= /home/$(PI_USER)/proj3
PI_RPI_DIR := $(PI_ROOT)/rpi_server
PI_RPI_DIR_ABS := $(PI_ROOT_ABS)/rpi_server
IMU_PORT ?= 5555
WEB_OUTPUT ?= /var/www/html/imu-data/imu_quaternion.txt
CLIENT_HOST ?= $(PI_HOST)
SSH := ssh $(PI_USER)@$(PI_HOST)
LOCAL_ROOT := $(CURDIR)
LOCAL_RPI_DIR := $(LOCAL_ROOT)/rpi_server

IS_PI := $(shell if grep -qa "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then echo 1; else echo 0; fi)

ifeq ($(IS_PI),1)
SERVER_MODE_DEFAULT := local
else
SERVER_MODE_DEFAULT := remote
endif

SERVER_MODE ?= $(SERVER_MODE_DEFAULT)

.PHONY: help
.PHONY: task1 task2 task3 task4
.PHONY: task1-local task1-remote task2-build
.PHONY: task3-local task3-remote task3-client task3-build
.PHONY: task4-local task4-remote
.PHONY: remote-stop remote-status local-stop local-status

help:
	@echo "Project 3 top-level targets"
	@echo ""
	@echo "Main targets:"
	@echo "  make task1 [PI_HOST=<pi-ip>]"
	@echo "  make task2"
	@echo "  make task3 [PI_HOST=<pi-ip>]"
	@echo "  make task4 [PI_HOST=<pi-ip>]"
	@echo ""
	@echo "Behavior:"
	@echo "  On the Raspberry Pi: task1/task3/task4 run locally."
	@echo "  On your laptop/PC: task1/task3/task4 connect to the Pi over SSH."
	@echo ""
	@echo "Useful extra targets:"
	@echo "  make task3-client CLIENT_HOST=<pi-ip>"
	@echo "  make remote-status PI_HOST=<pi-ip>"
	@echo "  make remote-stop PI_HOST=<pi-ip>"
	@echo "  make local-status"
	@echo "  make local-stop"
	@echo ""
	@echo "Variables:"
	@echo "  SERVER_MODE=$(SERVER_MODE)   # local or remote"
	@echo "  PI_HOST=$(PI_HOST)"
	@echo "  PI_USER=$(PI_USER)"
	@echo "  CLIENT_HOST=$(CLIENT_HOST)"
	@echo "  IMU_PORT=$(IMU_PORT)"

task1:
ifeq ($(SERVER_MODE),local)
	@$(MAKE) task1-local
	@echo "Task 1 is running locally on this Pi."
	@echo "Open: http://$$(hostname -I | awk '{print $$1}')/imu_quaternion.cgi"
else
	@$(MAKE) task1-remote
	@echo "Task 1 is running on the Pi at $(PI_HOST)."
	@echo "Open: http://$(PI_HOST)/imu_quaternion.cgi"
endif

task2: task2-build
	@./qt_basics/build/qt_basics_mouse_tracker

task3:
ifeq ($(SERVER_MODE),local)
	@$(MAKE) task3-local
	@echo "Task 3 server is running locally on this Pi on port $(IMU_PORT)."
else
	@$(MAKE) task3-remote
	@$(MAKE) task3-client CLIENT_HOST="$(PI_HOST)"
endif

task4:
ifeq ($(SERVER_MODE),local)
	@$(MAKE) task4-local
	@echo "Task 4 threaded server is running locally on this Pi on port $(IMU_PORT)."
else
	@$(MAKE) task4-remote
	@$(MAKE) task3-client CLIENT_HOST="$(PI_HOST)"
endif

task2-build:
	@cmake -S qt_basics -B qt_basics/build
	@cmake --build qt_basics/build -j2

task3-build:
	@cmake -S qt_client -B qt_client/build
	@cmake --build qt_client/build -j2

task3-client: task3-build
	@IMU_HOST="$(CLIENT_HOST)" IMU_PORT="$(IMU_PORT)" ./run_task3_client.sh

task1-local:
	@cd rpi_server && make imu_web_publisher
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_socket_server( |\$$)" || true
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_multithread_server( |\$$)" || true
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_web_publisher( |\$$)" || true
	@nohup "$(LOCAL_RPI_DIR)/imu_web_publisher" --output "$(WEB_OUTPUT)" >/tmp/imu_web_publisher.log 2>&1 </dev/null &

task3-local:
	@cd rpi_server && make imu_socket_server
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_web_publisher( |\$$)" || true
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_multithread_server( |\$$)" || true
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_socket_server( |\$$)" || true
	@nohup "$(LOCAL_RPI_DIR)/imu_socket_server" --port "$(IMU_PORT)" --web-output "$(WEB_OUTPUT)" >/tmp/imu_socket_server.log 2>&1 </dev/null &

task4-local:
	@cd rpi_server && make imu_multithread_server
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_web_publisher( |\$$)" || true
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_socket_server( |\$$)" || true
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_multithread_server( |\$$)" || true
	@nohup "$(LOCAL_RPI_DIR)/imu_multithread_server" --port "$(IMU_PORT)" --web-output "$(WEB_OUTPUT)" >/tmp/imu_multithread_server.log 2>&1 </dev/null &

task1-remote:
	@$(SSH) "bash -lc 'set -e; cd $(PI_RPI_DIR_ABS); make imu_web_publisher; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_socket_server( |\$$)\" || true; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_multithread_server( |\$$)\" || true; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_web_publisher( |\$$)\" || true; nohup $(PI_RPI_DIR_ABS)/imu_web_publisher --output $(WEB_OUTPUT) >/tmp/imu_web_publisher.log 2>&1 </dev/null &'"

task3-remote:
	@$(SSH) "bash -lc 'set -e; cd $(PI_RPI_DIR_ABS); make imu_socket_server; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_web_publisher( |\$$)\" || true; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_multithread_server( |\$$)\" || true; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_socket_server( |\$$)\" || true; nohup $(PI_RPI_DIR_ABS)/imu_socket_server --port $(IMU_PORT) --web-output $(WEB_OUTPUT) >/tmp/imu_socket_server.log 2>&1 </dev/null &'"

task4-remote:
	@$(SSH) "bash -lc 'set -e; cd $(PI_RPI_DIR_ABS); make imu_multithread_server; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_web_publisher( |\$$)\" || true; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_socket_server( |\$$)\" || true; pkill -f \"^$(PI_RPI_DIR_ABS)/imu_multithread_server( |\$$)\" || true; nohup $(PI_RPI_DIR_ABS)/imu_multithread_server --port $(IMU_PORT) --web-output $(WEB_OUTPUT) >/tmp/imu_multithread_server.log 2>&1 </dev/null &'"

local-stop:
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_web_publisher( |\$$)" || true
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_socket_server( |\$$)" || true
	@pkill -f "^$(LOCAL_RPI_DIR)/imu_multithread_server( |\$$)" || true

local-status:
	@echo "Processes:"
	@pgrep -af 'imu_' || true
	@echo ""
	@echo "Quaternion file:"
	@ls -l "$(WEB_OUTPUT)" 2>/dev/null || true
	@sed -n '1,3p' "$(WEB_OUTPUT)" 2>/dev/null || true

remote-stop:
	@$(SSH) "pkill -f '^$(PI_RPI_DIR_ABS)/imu_web_publisher( |\$$)' || true; pkill -f '^$(PI_RPI_DIR_ABS)/imu_socket_server( |\$$)' || true; pkill -f '^$(PI_RPI_DIR_ABS)/imu_multithread_server( |\$$)' || true"

remote-status:
	@$(SSH) "echo 'Processes:'; pgrep -af 'imu_' || true; echo; echo 'Quaternion file:'; ls -l $(WEB_OUTPUT) 2>/dev/null || true; sed -n '1,3p' $(WEB_OUTPUT) 2>/dev/null || true"
