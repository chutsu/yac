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
	@bash ./scripts/build_eigen.bash > log/build_eigen.log 2>&1

apriltag:
	@echo "building apriltag ..."
	@bash ./scripts/build_apriltag.bash > log/build_apriltag.log 2>&1

apriltag3:
	@echo "building apriltag3 ..."
	@bash ./scripts/build_apriltag3.bash > log/build_apriltag3.log 2>&1

suitesparse:
	@echo "building suitesparse ..."
	@bash ./scripts/build_suitesparse.bash > log/build_suitesparse.log 2>&1

ceres:
	@echo "building ceres ..."
	@bash ./scripts/build_ceres.bash > log/build_ceres.log 2>&1
