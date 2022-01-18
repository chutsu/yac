default: apriltag3 apriltag
.PHONY: apriltag3 apriltag suitesparse ceres

clean:
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf share

apriltag3:
	@echo "Building apriltag3 ..."
	@bash ./build_apriltag3.bash > build_apriltag3.log

apriltag:
	@echo "Building apriltag ..."
	@bash ./build_apriltag.bash > build_apriltag.log

suitesparse:
	@echo "Building SuiteSparse ..."
	@bash ./build_suitesparse.bash > build_suitesparse.log

ceres:
	@echo "Building Ceres-Solver ..."
	@bash ./build_ceres.bash > build_ceres.log
