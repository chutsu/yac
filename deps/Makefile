default: apriltag3 apriltags
.PHONY: apriltag3 s apriltags suitesparse ceres

clean:
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf share

apriltag3:
	@echo "Building AprilTag3 ..."
	@bash ./build_apriltag3.bash > build_apriltag3.log

apriltags:
	@echo "Building AprilTags ..."
	@bash ./build_apriltags.bash > build_apriltags.log

suitesparse:
	@echo "Building SuiteSparse ..."
	@bash ./build_suitesparse.bash > build_suitesparse.log

ceres:
	@echo "Building Ceres-Solver ..."
	@bash ./build_ceres.bash > build_ceres.log
