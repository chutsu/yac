default: deps yac

.PHONY: deps yac

deps:
	@echo "Installing deps..."
	@sudo apt-get install -qqq -y libopencv-*
	@sudo apt-get install -qqq -y libceres1 libceres-dev
	@sudo apt-get install -qqq -y libyaml-cpp-dev
	@echo "Done!"

yac:
	@cd yac && mkdir -p build && cd build \
		&& cmake .. -DCMAKE_BUILD_TYPE=Release \
		&& make -j2
