.PHONY: build flash monitor clean config port target help _port_saved esp32s3

IDF_PATH ?= $(HOME)/.espressif/v5.5.4/esp-idf
ROOT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
ENV_DIR := $(ROOT_DIR)/.env
ENV_PORT := $(ENV_DIR)/port
ENV_TARGET := $(ENV_DIR)/target

read_env = $(shell [ -f "$(1)" ] && tr -d '\n\r' < "$(1)" 2>/dev/null)

ifneq ($(port),)
  _PORT_VALUE := $(if $(filter /%,$(port)),$(port),/dev/$(port))
  _ := $(shell mkdir -p '$(ENV_DIR)' && printf '%s\n' '$(_PORT_VALUE)' > '$(ENV_PORT)')
  SERIAL_PORT := $(_PORT_VALUE)
else
  SERIAL_PORT := $(call read_env,$(ENV_PORT))
endif

ifeq ($(port),)
.DEFAULT_GOAL := help
else
.DEFAULT_GOAL := _port_saved
endif

build:
	. "$(IDF_PATH)/export.sh" >/dev/null 2>&1 && if [ -n "$(SERIAL_PORT)" ]; then "$(IDF_PATH)/tools/idf.py" -p "$(SERIAL_PORT)" build; else "$(IDF_PATH)/tools/idf.py" build; fi

flash:
	. "$(IDF_PATH)/export.sh" >/dev/null 2>&1 && if [ -n "$(SERIAL_PORT)" ]; then "$(IDF_PATH)/tools/idf.py" -p "$(SERIAL_PORT)" flash; else "$(IDF_PATH)/tools/idf.py" flash; fi

monitor:
	. "$(IDF_PATH)/export.sh" >/dev/null 2>&1 && if [ -n "$(SERIAL_PORT)" ]; then "$(IDF_PATH)/tools/idf.py" -p "$(SERIAL_PORT)" monitor; else "$(IDF_PATH)/tools/idf.py" monitor; fi

clean:
	. "$(IDF_PATH)/export.sh" >/dev/null 2>&1 && "$(IDF_PATH)/tools/idf.py" fullclean

config:
	. "$(IDF_PATH)/export.sh" >/dev/null 2>&1 && "$(IDF_PATH)/tools/idf.py" menuconfig

port:
	@if [ -z "$(SERIAL_PORT)" ]; then \
		echo "Uso: make port=/dev/ttyACM0"; \
		exit 1; \
	fi
	@mkdir -p "$(ENV_DIR)"
	@printf '%s\n' "$(SERIAL_PORT)" > "$(ENV_PORT)"
	@echo "Porta gravada em .env/port: $(SERIAL_PORT)"

_port_saved:
	@$(MAKE) --no-print-directory port

target:
	@mkdir -p "$(ENV_DIR)"
	@printf '%s\n' "esp32s3" > "$(ENV_TARGET)"
	@. "$(IDF_PATH)/export.sh" >/dev/null 2>&1 && "$(IDF_PATH)/tools/idf.py" set-target esp32s3
	@echo "Target gravado em .env/target: esp32s3"

esp32s3:
	@:

help:
	@echo "Available targets:"
	@echo "  make build   - build the project"
	@echo "  make flash   - flash the project"
	@echo "  make monitor - open serial monitor"
	@echo "  make clean   - full clean"
	@echo "  make config  - open menuconfig"
	@echo "  make port=/dev/ttyACM0 - save port to .env/port"
	@echo "  make target  - set and save target esp32s3"
