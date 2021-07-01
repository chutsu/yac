#ifndef YAC_CALIB_CAMERA_HPP
#define YAC_CALIB_CAMERA_HPP

#include <ceres/ceres.h>

#include "calib_data.hpp"
#include "calib_params.hpp"

namespace yac {

/** Camera Data **/
typedef std::map<timestamp_t, std::map<int, aprilgrid_t>> camera_data_t;

/** Reprojection Error */
struct reproj_error_t : ceres::CostFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const camera_geometry_t *cam_geom;
  const int cam_index;
  const int cam_res[2];
  const int tag_id;
  const int corner_idx;
  const vec3_t r_FFi;
  const vec2_t z;

  const mat2_t covar;
  const mat2_t info;
  const mat2_t sqrt_info;

  reproj_error_t(const camera_geometry_t *cam_geom_,
                 const int cam_index_,
                 const int cam_res_[2],
                 const int tag_id_,
                 const int corner_idx_,
                 const vec3_t &r_FFi_,
                 const vec2_t &z_,
                 const mat2_t &covar_)
      : cam_geom{cam_geom_},
        cam_index{cam_index_},
        cam_res{cam_res_[0], cam_res_[1]},
        tag_id{tag_id_},
        corner_idx{corner_idx_},
        r_FFi{r_FFi_},
        z{z_},
        covar{covar_},
        info{covar.inverse()},
        sqrt_info{info.llt().matrixL().transpose()} {
    set_num_residuals(2);
    auto block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(7);  // camera-fiducial relative pose
    block_sizes->push_back(7);  // camera-camera extrinsics
    block_sizes->push_back(8);  // camera parameters
  }

  bool Evaluate(double const * const *params,
                double *residuals,
                double **jacobians) const {
    // Map parameters out
    const mat4_t T_C0F = tf(params[0]);
    const mat4_t T_C0Ci = tf(params[1]);
    Eigen::Map<const vecx_t> cam_params(params[2], 8);

    // Transform and project point to image plane
    // -- Transform point from fiducial frame to camera-n
    const mat4_t T_CiC0 = T_C0Ci.inverse();
    const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);
    // -- Project point from camera frame to image plane
    vec2_t z_hat;
    bool valid = true;
    if (cam_geom->project(cam_res, cam_params, r_CiFi, z_hat) != 0) {
      valid = false;
    }

    // Residual
    Eigen::Map<vec2_t> r(residuals);
    r = sqrt_info * (z - z_hat);

    // Jacobians
    const matx_t Jh = cam_geom->project_jacobian(cam_params, r_CiFi);
    const matx_t J_cam_params = cam_geom->params_jacobian(cam_params, r_CiFi);
    const matx_t Jh_weighted = -1 * sqrt_info * Jh;

    if (jacobians) {
      // Jacobians w.r.t T_C0F
      if (jacobians[0]) {
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[0]);
        J.setZero();

        if (valid) {
          const mat3_t C_CiC0 = tf_rot(T_CiC0);
          const mat3_t C_C0F = tf_rot(T_C0F);
          J.block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
          J.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -skew(C_C0F * r_FFi);
        }
      }

      // Jacobians w.r.t T_C0Ci
      if (jacobians[1]) {
        Eigen::Map<mat_t<2, 7, row_major_t>> J(jacobians[1]);
        J.setZero();

        if (valid) {
          const mat3_t C_CiC0 = tf_rot(T_CiC0);
          const mat3_t C_C0Ci = C_CiC0.transpose();
          const vec3_t r_CiFi = tf_point(T_CiC0 * T_C0F, r_FFi);
          J.block(0, 0, 2, 3) = -1 * Jh_weighted * C_CiC0;
          J.block(0, 3, 2, 3) = -1 * Jh_weighted * C_CiC0 * -skew(C_C0Ci * r_CiFi);
        }
      }

      // Jacobians w.r.t cam params
      if (jacobians[2]) {
        Eigen::Map<mat_t<2, 8, row_major_t>> J(jacobians[2]);
        J.setZero();

        if (valid) {
          J.block(0, 0, 2, 8) = -1 * sqrt_info * J_cam_params;
        }
      }
    }

    return true;
  }
};

/**
 * Initialize camera intrinsics using AprilGrids `grids` observed by the camera,
 * the camera geometry `cam_geom`, and camera parameters `cam_params`.
 */
void initialize_camera(const aprilgrids_t &grids,
                       const camera_geometry_t *cam_geom,
                       camera_params_t &cam_params,
                       const bool verbose=false);

/**
 * Camera Chain Query Tool
 */
struct camchain_t {
	std::map<int, std::vector<int>> adj_list;   // CameraChain adjacency list
  std::map<int, std::map<int, mat4_t>> exts;  // Extrinsics

  camchain_t(const camera_data_t &cam_data,
             const std::map<int, camera_geometry_t *> &cam_geoms,
             const std::map<int, camera_params_t> &cam_params) {
		for (const auto &kv : cam_data) {
      const auto &ts = kv.first;

		  // Get all detected AprilGrid at ts
      std::vector<int> cam_indicies;
      std::vector<aprilgrid_t> grids;
      _get_aprilgrids(ts, cam_data, cam_indicies, grids);

			// Check we have more than 1
			if (cam_indicies.size() < 2) {
        continue;
			}

			// Add to camchain if we haven't already
			const int cam_i = cam_indicies[0];
			const auto params_i = cam_params.at(cam_i);
			const auto res_i = params_i.resolution;
			const auto geom_i = cam_geoms.at(cam_i);
			const auto &grid_i = grids[0];

      for (size_t i = 1; i < cam_indicies.size(); i++) {
        const auto cam_j = cam_indicies[i];
        const auto params_j = cam_params.at(cam_j);
        const auto res_j = params_j.resolution;
        const auto geom_j = cam_geoms.at(cam_j);
        const auto &grid_j = grids[i];

        if (contains(cam_i, cam_j) == false) {
          mat4_t T_CiF;
          if (grid_i.estimate(geom_i, res_i, params_i.param, T_CiF) != 0) {
            FATAL("Failed to estimate relative pose!");
          }

          mat4_t T_CjF;
          if (grid_j.estimate(geom_j, res_j, params_j.param, T_CjF) != 0) {
            FATAL("Failed to estimate relative pose!");
          }

          const mat4_t T_CiCj = T_CiF * T_CjF.inverse();
          insert(cam_i, cam_j, T_CiCj);
          print_matrix("T_CiCj", T_CiCj);
        }
      }  // Iter camera indicies
		}  // Iter timestamps
  }  // Constructor

  void _get_aprilgrids(const timestamp_t &ts,
                       const camera_data_t &cam_data,
                       std::vector<int> &cam_indicies,
                       std::vector<aprilgrid_t> &grids) {
    // Get all detected AprilGrid at ts
    for (const auto &kv : cam_data.at(ts)) {
      // Camera index and grid
      const auto cam_idx = kv.first;
      const auto &grid = kv.second;

      // Check aprilgrid is detected
      if (grid.detected == false && grid.nb_detections < 12) {
        continue;
      }

      // Update
      cam_indicies.push_back(cam_idx);
      grids.push_back(grid);
    }
  }

  void insert(const int cam_i, const int cam_j, const mat4_t &T_CiCj) {
    if (contains(cam_i, cam_j)) {
      return;
    }

    adj_list[cam_i].push_back(cam_j);
    adj_list[cam_j].push_back(cam_i);
    exts[cam_i][cam_j] = T_CiCj;
    exts[cam_j][cam_i] = T_CiCj.inverse();
  }

	bool contains(const int cam_i, const int cam_j) const {
		return (exts.count(cam_i) && exts.at(cam_i).count(cam_j));
	}

	int find(const int cam_i, const int cam_j, mat4_t &T_CiCj) const {
		// Straight-forward case
		if (cam_i == cam_j) {
			T_CiCj = I(4);
			return 0;
		}

		// Check if we have even inserted the cameras before
		if (exts.count(cam_i) == 0 || exts.count(cam_j) == 0) {
			return -1;
		}

		// Check if we already have the extrinsics pair
		if (contains(cam_i, cam_j)) {
			T_CiCj = exts.at(cam_i).at(cam_j);
			return 0;
		}

		// Iterative BFS - To get path from cam_i to cam_j
		bool found_target = false;
		std::deque<int> queue;
		std::map<int, bool> visited;
		std::map<int, int> path_map;

		queue.push_back(cam_i);
		while (!queue.empty()) {
			const auto parent = queue.front();
			queue.pop_front();
			visited.at(parent) = true;

			for (const int& child : adj_list.at(parent)) {
				if (visited.at(child)) {
					continue;
				}

				queue.push_back(child);
				path_map[child] = parent;

				if (child == cam_j) {
					found_target = true;
					break;
				}
			}
		}

		// Check if we've found the target
		if (found_target == false) {
			return -2;
		}

		// Traverse the path backwards and chain the transforms
		mat4_t T_CjCi = I(4);
		int child = cam_j;
		while (path_map.count(child)) {
			const int parent = path_map[child];
			T_CjCi = T_CjCi * exts.at(child).at(parent);
			child = parent;
		}
		T_CiCj = T_CjCi.inverse();

		return 0;
	}

	void clear() {
		adj_list.clear();
		exts.clear();
	}
};

/* Calibration View */
struct calib_view_t {
  aprilgrid_t grid;
  const camera_params_t &cam;
  const pose_t &T_C0F;
  const pose_t &T_C0Ci;

  std::vector<ceres::ResidualBlockId> res_ids;
  std::vector<reproj_error_t *> cost_fns;

  calib_view_t(const aprilgrid_t &grid_,
               const camera_params_t &cam_,
               const pose_t &T_C0F_,
               const pose_t &T_C0Ci_)
    : grid{grid_},
      cam{cam_},
      T_C0F{T_C0F_},
      T_C0Ci{T_C0Ci_} {}
};

/** Camera Calibrator **/
struct calib_camera_t {
  // Problem
  ceres::Problem::Options prob_options;
  ceres::Problem *problem;
  PoseLocalParameterization pose_plus;

  // State variables
  std::map<int, camera_params_t> cam_params;
  std::map<int, extrinsics_t> cam_exts;
  std::map<timestamp_t, pose_t> poses;

  // Data
  std::map<int, camera_geometry_t *> cam_geoms;
  calib_target_t target;
  std::set<timestamp_t> timestamps;
  camera_data_t cam_data;

  // Camera geometries
  pinhole_radtan4_t pinhole_radtan4;
  pinhole_equi4_t pinhole_equi4;

  calib_camera_t() {
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.enable_fast_removal = true;
    problem = new ceres::Problem(prob_options);
  }

  ~calib_camera_t() {
    delete problem;
  }

  int nb_cams() {
    return cam_params.size();
  }

  aprilgrids_t get_cam_data(const int cam_idx) {
    aprilgrids_t grids;
    for (const auto &ts : timestamps) {
      grids.push_back(cam_data[ts][cam_idx]);
    }
    return grids;
  }

  mat4_t get_camera_extrinsics(const int cam_idx) {
    return cam_exts[cam_idx].tf();
  }

  void add_calib_target(const calib_target_t &target_) {
    target = target_;
  }

  void add_camera_data(const int cam_idx, const aprilgrids_t &grids) {
    for (const auto &grid : grids) {
      const auto ts = grid.timestamp;
      timestamps.insert(ts);
      cam_data[ts][cam_idx] = grid;
    }
  }

  void add_camera(const int cam_idx,
                  const int cam_res[2],
                  const std::string &proj_model,
                  const std::string &dist_model,
                  const bool fixed=false) {
    // Projection params
    vecx_t proj_params;
    if (proj_model == "pinhole") {
      const double fx = pinhole_focal(cam_res[0], 90);
      const double fy = pinhole_focal(cam_res[0], 90);
      const double cx = cam_res[0] / 2.0;
      const double cy = cam_res[1] / 2.0;
      proj_params.resize(4);
      proj_params << fx, fy, cx, cy;
    } else {
      FATAL("Unsupported [%s]!", proj_model.c_str());
    }

    // Distortion params
    vecx_t dist_params;
    if (dist_model == "radtan4" || dist_model == "equi4") {
      dist_params = zeros(4, 1);
    } else {
      FATAL("Unsupported [%s]!", dist_model.c_str());
    }

    // Camera parameters
    camera_params_t params(0, cam_idx, cam_res,
                           proj_model, dist_model,
                           proj_params, dist_params,
                           fixed);
    cam_params[cam_idx] = params;

    // Camera geometry
    if (proj_model == "pinhole" && dist_model == "radtan4") {
      cam_geoms[cam_idx] = &pinhole_radtan4;
    } else if (proj_model == "pinhole" && dist_model == "equi4") {
      cam_geoms[cam_idx] = &pinhole_equi4;
    }

    // Add parameter to problem
    int params_size = proj_params.size() + dist_params.size();
    problem->AddParameterBlock(cam_params[cam_idx].data(), params_size);
    if (fixed) {
      problem->SetParameterBlockConstant(cam_params[cam_idx].data());
    }
  }

  void add_camera_extrinsics(const int cam_idx,
                             const mat4_t &ext=I(4),
                             const bool fixed=false) {
    cam_exts[cam_idx] = extrinsics_t{cam_idx, ext};
    problem->AddParameterBlock(cam_exts[cam_idx].data(), 7);
    problem->SetParameterization(cam_exts[cam_idx].data(), &pose_plus);
    if (cam_idx == 0 || fixed) {
      problem->SetParameterBlockConstant(cam_exts[cam_idx].data());
    }
  }

  pose_t &add_pose(const timestamp_t &ts,
                   const mat4_t &T,
                   const bool fixed=false) {
    poses[ts] = pose_t{0, ts, T};
    problem->AddParameterBlock(poses[ts].data(), 7);
    problem->SetParameterization(poses[ts].data(), &pose_plus);
    if (fixed) {
      problem->SetParameterBlockConstant(poses[ts].data());
    }
    return poses[ts];
  }

  void _initialize_intrinsics() {
    for (int cam_idx = 0; cam_idx < nb_cams(); cam_idx++) {
      printf("initializing cam%d intrinsics ...\n", cam_idx);
      const aprilgrids_t grids = get_cam_data(cam_idx);
      initialize_camera(grids, cam_geoms[cam_idx], cam_params[cam_idx]);
    }
  }

  void _initialize_extrinsics() {
    printf("initializing extrinsics ...\n");
    add_camera_extrinsics(0);
    if (nb_cams() == 1) {
      return;
    }

		camchain_t camchain{cam_data, cam_geoms, cam_params};
		for (int j = 1; j < nb_cams(); j++) {
		  mat4_t T_C0Cj;
      if (camchain.find(0, j, T_C0Cj) != 0) {
        FATAL("Failed to initialze T_C0C%d", j);
      }

      add_camera_extrinsics(j, T_C0Cj);
		}
  }

  void _setup_problem() {
    mat2_t covar = I(2);

    for (const auto &ts: timestamps) {
      for (const auto &kv: cam_data[ts]) {
        const auto cam_idx = kv.first;
        const auto &grid = kv.second;
        if (grid.detected == false) {
          continue;
        }

        // Estimate relative pose T_C0F
        const auto cam_geom = cam_geoms[cam_idx];
        const auto param = cam_params[cam_idx].param;
        const auto res = cam_params[cam_idx].resolution;
        mat4_t T_CiF;
        if (grid.estimate(cam_geom, res, param, T_CiF) != 0) {
          FATAL("Failed to estimate relative pose!");
        }

        // Add relative pose T_C0F
        const mat4_t T_C0Ci = get_camera_extrinsics(cam_idx);
        const mat4_t T_C0F = T_C0Ci * T_CiF;
        if (poses.count(ts) == 0) {
          add_pose(ts, T_C0F);
        }

        // Add reprojection factors to problem
        std::vector<int> tag_ids;
        std::vector<int> corner_idxs;
        vec2s_t kps;
        vec3s_t pts;
        grid.get_measurements(tag_ids, corner_idxs, kps, pts);

        for (size_t i = 0; i < tag_ids.size(); i++) {
          const int tag_id = tag_ids[i];
          const int corner_idx = corner_idxs[i];
          const vec2_t z = kps[i];
          const vec3_t r_FFi = pts[i];

          auto cost_fn = new reproj_error_t{cam_geom, cam_idx, res,
                                            tag_id, corner_idx,
                                            r_FFi, z, covar};
          auto res_id = problem->AddResidualBlock(cost_fn,
                                                  NULL,
                                                  poses[ts].data(),
                                                  cam_exts[cam_idx].data(),
                                                  cam_params[cam_idx].data());
        }
      }
    }
  }

  void solve() {
    _initialize_intrinsics();
    _initialize_extrinsics();
    _setup_problem();

    // Solver options
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;

    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << std::endl;
  }
};

} //  namespace yac
#endif // YAC_CALIB_CAMERA_HPP
