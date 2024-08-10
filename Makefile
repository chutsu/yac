SHELL:=/bin/bash
PREFIX=/opt/yac
BUILD_DIR=build
NUM_CPU=2

.PHONY: help deps debug release download_test_data tests

help:
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

${BUILD_DIR}:
	@mkdir -p ${BUILD_DIR}

${PREFIX}:
	@mkdir -p ${PREFIX}

deps: ## Install dependencies
	@echo "[Installing Dependencies]"
	@make -s -C deps

debug: ## Build in debug mode
	@mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make

release: ## Build in release mode
	@mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make
