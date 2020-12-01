default: ceres apriltag3 apriltags
.PHONY: ceres apriltag3 s apriltags suitesparse ceres

clean:
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf share

apriltag3:
	@echo "Building apriltag3 ..."
	@bash ./build_apriltag3.bash > build_apriltag3.log

apriltags:
	@echo "Building apriltags ..."
	@bash ./build_apriltags.bash > build_apriltags.log

suitesparse:
	@echo "Building SuiteSparse ..."
	@bash ./build_suitesparse.bash > build_suitesparse.log

ceres:
	@echo "Building Ceres-Solver ..."
	@bash ./build_ceres.bash > build_ceres.log
