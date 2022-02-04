default: dirs apriltag3 apriltag
.PHONY: dirs apriltag3 apriltag suitesparse ceres

dirs:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p share
	@mkdir -p log

clean:
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf share
	@rm -rf log

apriltag3:
	@echo "Building apriltag3 ..."
	@bash ./scripts/build_apriltag3.bash > log/build_apriltag3.log

apriltag:
	@echo "Building apriltag ..."
	@bash ./scripts/build_apriltag.bash > log/build_apriltag.log

suitesparse:
	@echo "Building SuiteSparse ..."
	@bash ./scripts/build_suitesparse.bash > log/build_suitesparse.log

ceres:
	@echo "Building Ceres-Solver ..."
	@bash ./scripts/build_ceres.bash > log/build_ceres.log
