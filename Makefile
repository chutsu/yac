MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
PREFIX := $(MKFILE_DIR)/third_party
NUM_PROCS := $(shell expr `nproc` / 2)

define cmake_build
	mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=$1 \
			-DCMAKE_PREFIX_PATH=$(PREFIX) \
			-DCMAKE_LIBRARY_PATH=$(PREFIX)\lib \
			.. \
		&& make -j$(NUM_PROCS)
endef

.PHONY: help third_party debug release

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

third_party: ## Build third party
	@echo "[Build Third Party]"
	@make -s -C third_party all

debug: ## Build in debug mode
	$(call cmake_build,Debug)

release: ## Build in release mode
	$(call cmake_build,Release)
