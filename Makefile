SHELL:=/bin/bash
PREFIX=/usr/local
BUILD_DIR=build
NUM_CPU=2

.PHONY: help deps lib_debug lib debug release download_test_data tests

define compile_yac
	@cd ${BUILD_DIR} \
		&& cmake ../yac \
		-DCMAKE_BUILD_TYPE=$(1) \
		-DCMAKE_INSTALL_PREFIX=${PREFIX} \
		&& time make -j${NUM_CPU}
endef

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

lib_debug: ${BUILD_DIR} ${PREFIX} ## Build libyac in debug mode
	$(call compile_yac,Debug)

lib_relwithdeb: ${BUILD_DIR} ${PREFIX} ## Build libyac in release with debug info
	$(call compile_yac,RelWithDebInfo)

lib: ${BUILD_DIR} ${PREFIX} ## Build libyac in release mode
	$(call compile_yac,Release)

install:
	cd ${BUILD_DIR} && make install

download_test_data: ## Download test data
	@bash ./scripts/download_test_data.bash

tests: ## Build and run tests
	@cd build \
		&& ./test_util_config \
		&& ./test_util_cv \
		&& ./test_util_data \
		&& ./test_util_fs \
		&& ./test_util_net \
		&& ./test_util_timeline \
		&& ./test_calib_camera \
		&& ./test_calib_data \
		&& ./test_calib_mocap \
		&& ./test_calib_vi

build_docker:
	docker build -f Dockerfile -t chutsu/yac .

run_docker:
	@xhost +local:docker
	@docker run -e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /data:/data \
		--network="host" \
		-it \
		--rm chutsu/yac
