default: dirs eigen apriltag apriltag3 suitesparse ceres
.PHONY: dirs eigen apriltag apriltag3 suitesparse ceres

dirs:
	@mkdir -p src
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p share
	@mkdir -p log

clean:
	@rm -rf src
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf share
	@rm -rf log

eigen:
	@echo "building eigen ..."
	@bash ./scripts/build_eigen.bash 2>&1 > log/build_eigen.log

apriltag:
	@echo "building apriltag ..."
	@bash ./scripts/build_apriltag.bash 2>&1 > log/build_apriltag.log

apriltag3:
	@echo "building apriltag3 ..."
	@bash ./scripts/build_apriltag3.bash 2>&1 > log/build_apriltag3.log

suitesparse:
	@echo "building suitesparse ..."
	@bash ./scripts/build_suitesparse.bash 2>&1 > log/build_suitesparse.log

ceres:
	@echo "building ceres ..."
	@bash ./scripts/build_ceres.bash 2>&1 > log/build_ceres.log
