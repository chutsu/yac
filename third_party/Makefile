MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
PREFIX := $(MKFILE_DIR)
SRC_PATH := $(PREFIX)/src
NUM_PROCS := $(shell expr `nproc` / 2)

define cmake_build
	@cd "$(SRC_PATH)/$1" \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_PROCS) \
		&& make install
endef

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

default: setup
.PHONY: setup clean
.PHONY: build_yaml build_abseil build_gtest
.PHONY: build_eigen build_suitesparse
.PHONY: build_ceres build_opencv build_apriltag build_apriltags

setup:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p log
	@mkdir -p share
	@mkdir -p src

clean: ## Clean
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf log
	@rm -rf share

build_yaml:  ## Build yaml
	$(call cmake_build,yaml-cpp)

build_abseil:  ## Build abseil
	$(call cmake_build,abseil-cpp)

build_gtest:  ## Build GTest
	$(call cmake_build,googletest)

build_eigen:  ## Build Eigen
	$(call cmake_build,eigen)

build_suitesparse: build_eigen  ## Build SuiteSparse
	$(call cmake_build,SuiteSparse)

build_ceres: build_abseil build_gtest build_eigen build_suitesparse  ## Build SuiteSparse
	$(call cmake_build,ceres-solver)

build_opencv: build_eigen  ## Build OpenCV
	$(call cmake_build,opencv)

build_apriltag: build_eigen build_opencv  ## Build AprilTag
	$(call cmake_build,apriltag)

build_apriltags: build_eigen build_opencv  ## Build AprilTags
	@cd src/apriltags && git apply ../../patches/apriltags.patch
	$(call cmake_build,apriltags)

BUILD_TARGETS := \
	build_yaml \
	build_abseil \
	build_gtest \
	build_eigen \
	build_suitesparse \
	build_ceres \
	build_opencv \
	build_apriltag
all: $(BUILD_TARGETS)  ## Build all
