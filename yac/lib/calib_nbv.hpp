#ifndef YAC_CALIB_NBV_HPP
#define YAC_CALIB_NBV_HPP

#include "core.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"
#include "calib_mono.hpp"

namespace yac {

// Check number of apriltags observed by camera
template <typename CAMERA>
bool check_fully_observable(const calib_target_t &target,
													  const camera_params_t &cam_params,
														const mat4_t &T_FC) {
	const int tag_rows = target.tag_rows;
	const int tag_cols = target.tag_cols;
	const double tag_size = target.tag_size;
	const double tag_spacing = target.tag_spacing;
  aprilgrid_t grid{0, tag_rows, tag_cols, tag_size, tag_spacing};

	const auto cam_res = cam_params.resolution;
	const auto proj_params = cam_params.proj_params();
	const auto dist_params = cam_params.dist_params();
	CAMERA camera{cam_res, proj_params, dist_params};

	const mat4_t T_CF = T_FC.inverse();
  const int nb_tags = (grid.tag_rows * grid.tag_cols);

  for (int tag_id = 0; tag_id < nb_tags; tag_id++) {
		for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
			const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);
			const vec3_t r_CFi = tf_point(T_CF, r_FFi);

			vec2_t z_hat;
			if (camera.project(r_CFi, z_hat) != 0) {
				return false;
			}
		}
  }

  return true;
}

/** Calculate target origin (O) w.r.t. fiducial (F) T_FO **/
template <typename CAMERA>
mat4_t calib_target_origin(const calib_target_t &target,
													 const camera_params_t &cam_params,
												   const double target_scale=0.5) {
  // Calculate target center
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double target_width = tag_cols * tag_size + spacing_x;
  const double target_height = tag_rows * tag_size + spacing_y;
  const vec2_t center{target_width / 2.0, target_height / 2.0};

	double scale = target_scale;
  int retry = 5;
start:
  // Calculate distance away from target center
  const double image_width = cam_params.resolution[0];
  const double target_half_width =  target_width / 2.0;
  const double target_half_resolution_x = image_width / 2.0;
  const auto fx = cam_params.proj_params()[0];
  const auto z_FO = fx * target_half_width / (target_half_resolution_x * scale);

  // Form transform of calibration origin (O) wrt fiducial target (F) T_FO
  const mat3_t C_FO = I(3);
  const vec3_t r_FO{center(0), center(1), z_FO};
  const mat4_t T_FO = tf(C_FO, r_FO);

  // Rotate the camera around to see if camera can actually observe the target
  const vec3_t rpy = deg2rad(vec3_t{-180.0, 0.0, 0.0});
  const mat3_t C_FC = euler321(rpy);
  const mat4_t T_FC = tf(C_FC, r_FO);
  if (check_fully_observable<CAMERA>(target, cam_params, T_FC) == false) {
    scale -= 0.1;
    retry--;

    if (retry == 0) {
      FATAL("Failed to find calibration origin! Check camera params?");
    }
    goto start;
  }

  return T_FO;
}

template <typename CAMERA>
mat4s_t calib_init_poses(const calib_target_t &target,
												 const camera_params_t &cam_params) {
  // Target
	const mat4_t T_FO = calib_target_origin<CAMERA>(target, cam_params, 0.5);
	const vec3_t r_FO = tf_trans(T_FO);
	const double target_width = r_FO(0) * 2;
	const double target_height = r_FO(1) * 2;
	const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  // Pose settings
  const double h_width = target_width / 2.0;
  const double h_height = target_height / 2.0;
  const double xy_scale = 1.2;
  const double range_w[2] = {-h_width * xy_scale, h_width * xy_scale};
  const double range_h[2] = {-h_height * xy_scale, h_height * xy_scale};
  const auto x_range = linspace(range_w[0], range_w[1], 2);
  const auto y_range = linspace(range_h[0], range_h[1], 2);
	const double z = r_FO(2);
  mat4s_t poses;

  // Push first pose to be infront of target center
  const vec3_t r_FC{target_center(0), target_center(1), z};
  const mat4_t T_FC = lookat(r_FC, target_center);
  poses.push_back(T_FC);

  // Generate camera positions infront of the AprilGrid target in the fiducial
  // frame, r_FC.
  vec3s_t cam_pos;
  cam_pos.emplace_back(vec3_t{range_w[0], range_h[1], z} + target_center);
  cam_pos.emplace_back(vec3_t{range_w[0], range_h[0], z} + target_center);
  cam_pos.emplace_back(vec3_t{range_w[1], range_h[0], z} + target_center);
  cam_pos.emplace_back(vec3_t{range_w[1], range_h[1], z} + target_center);

  // For each position create a camera pose that "looks at" the fiducial center
  // in the fiducial frame, T_FC.
  for (const auto &r_FC : cam_pos) {
    mat4_t T_FC = lookat(r_FC, target_center);
    poses.push_back(T_FC);
  }

  return poses;
}

template <typename CAMERA>
mat4s_t calib_nbv_poses(const calib_target_t &target,
                        const camera_params_t &cam_params,
                        const int range_x_size = 5,
                        const int range_y_size = 5,
                        const int range_z_size = 5) {
  // Target
	const mat4_t T_FO = calib_target_origin<CAMERA>(target, cam_params, 0.8);
	const vec3_t r_FO = tf_trans(T_FO);
	const double target_width = r_FO(0) * 2;
	const double target_height = r_FO(1) * 2;
	const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  // Pose settings
  const double xy_scale = 2.0;
  const double d_start = r_FO(2);
  const double d_end = d_start * 4.0;
  const double h_width = target_width / 2.0;
  const double h_height = target_height / 2.0;
  const double range_w[2] = {-h_width * xy_scale, h_width * xy_scale};
  const double range_h[2] = {-h_height * xy_scale, h_height * xy_scale};
  const double range_d[2] = {d_start, d_end};
  const auto x_range = linspace(range_w[0], range_w[1], range_x_size);
  const auto y_range = linspace(range_h[0], range_h[1], range_y_size);
  const auto z_range = linspace(range_d[0], range_d[1], range_z_size);

  // Generate camera positions infront of the AprilGrid target in the fiducial
  // frame, r_FC.
  vec3s_t cam_positions;
  for (const auto &x : x_range) {
    for (const auto &y : y_range) {
      for (const auto &z : z_range) {
        const vec3_t r_FC = vec3_t{x, y, z} + target_center;
        cam_positions.push_back(r_FC);
      }
    }
  }

  // For each position create a camera pose that "looks at" the fiducial center
  // in the fiducial frame, T_FC.
  mat4s_t poses;
  for (const auto &cam_pos : cam_positions) {
    mat4_t T_FC = lookat(cam_pos, target_center);
    poses.push_back(T_FC);
  }

  return poses;
}

template <typename T>
void calib_orbit_trajs(const calib_target_t &target,
                       const camera_params_t &cam0,
                       const camera_params_t &cam1,
                       const mat4_t &T_C0C1,
                       const mat4_t &T_WF,
                       const mat4_t &T_FO,
                       const timestamp_t &ts_start,
                       const timestamp_t &ts_end,
                       ctrajs_t &trajs) {
  // Calculate target width and height
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;

  // Target center (Fc) w.r.t. Target origin (F)
  const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};

  // Trajectory parameters
  double scale = 0.1;
  const double lat_min = deg2rad(0.0);
  const double lat_max = deg2rad(360.0);
  const double lon_min = deg2rad(0.0);
  const double lon_max = deg2rad(80.0);

  int retry = 20;
  ctrajs_t orbit_trajs;
start:
  double rho = (calib_width * scale);  // Sphere radius

  // Adjust the calibration origin such that trajectories are valid
  mat4_t calib_origin = T_FO;
  calib_origin(2, 3) = rho;

  // Orbit trajectories. Imagine a half sphere coming out from the
  // calibration target center. The trajectory would go from the pole of the
  // sphere to the sides of the sphere. While following the trajectory in a
  // tangent manner the camera view focuses on the target center.
	const auto nb_trajs = 8.0;
	const auto dlat = lat_max / nb_trajs;
	auto lat = lat_min;
  for (int i = 0; i < nb_trajs; i++) {
    vec3s_t positions;
    quats_t attitudes;

    // Create sphere point and transform it into world frame
    for (const auto &lon : linspace(lon_min, lon_max, 10)) {
      const vec3_t p = sphere(rho, lon, lat);
      const vec3_t r_FJ = tf_point(calib_origin, p);
      const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
      const mat4_t T_FC1 = T_FC0 * T_C0C1;
      const mat4_t T_WC0 = T_WF * T_FC0;

      if (check_fully_observable<T>(target, cam0, T_FC0) == false) {
        orbit_trajs.clear();
        scale += 0.1;
        retry--;
        if (retry == 0){
          FATAL("Failed to generate orbit trajectory!");
        }
        goto start;
      }
      if (check_fully_observable<T>(target, cam1, T_FC1) == false) {
        orbit_trajs.clear();
        scale += 0.1;
        retry--;
        if (retry == 0){
          FATAL("Failed to generate orbit trajectory!");
        }
        goto start;
      }

      positions.push_back(tf_trans(T_WC0));
      attitudes.emplace_back(tf_rot(T_WC0));
    }

    // Update
    const auto timestamps = linspace(ts_start, ts_end, 10);
    orbit_trajs.emplace_back(timestamps, positions, attitudes);
		lat += dlat;
  }

  // Append to results
  for (const auto &traj : orbit_trajs) {
    trajs.emplace_back(traj.timestamps, traj.positions, traj.orientations);
  }
}

template <typename T>
void calib_pan_trajs(const calib_target_t &target,
                     const camera_params_t &cam0,
										 const camera_params_t &cam1,
                     const mat4_t &T_C0C1,
                     const mat4_t &T_WF,
                     const mat4_t &T_FO,
                     const timestamp_t &ts_start,
                     const timestamp_t &ts_end,
                     ctrajs_t &trajs) {
  // Tag width
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;

  // Target center (Fc) w.r.t. Target origin (F)
  const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};

  // Calibration origin
  const mat4_t calib_origin = T_FO;

  // Create rotation matrix that is z-forard (i.e. camera frame)
  // const vec3_t rpy_WC{-90.0, 0.0, -90.0};
  // const mat3_t C_WC = euler321(deg2rad(rpy_WC));
  // const mat4_t T_WO = T_WF * T_FO;
  // const mat4_t T_FW = T_WF.inverse();
  // const mat3_t C_FW = tf_rot(T_FW);
  // const mat3_t C_FC = C_FW * C_WC;

  double scale = 1.0;
  ctrajs_t pan_trajs;
//   int retry = 20;
// start:
  // Trajectory parameters
  const int nb_trajs = 4;
  const int nb_control_points = 5;
  const double pan_length = (calib_width / 2.0) * scale;
  const double theta_min = deg2rad(0.0);
  const double theta_max = deg2rad(270.0);

  // Pan trajectories. Basically simple pan trajectories with the camera
  // looking forwards. No attitude changes.
  for (const auto &theta : linspace(theta_min, theta_max, nb_trajs)) {
    vec3s_t positions;
    quats_t attitudes;

		for (const auto &r: linspace(0.0, pan_length, nb_control_points)) {
			const vec2_t x = circle(r, theta);
			const vec3_t p{x(0), x(1), 0.0};

      const vec3_t r_FJ = tf_point(calib_origin, p);
      const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
      // const mat4_t T_FC1 = T_FC0 * T_C0C1;
      const mat4_t T_WC0 = T_WF * T_FC0;

      // if (check_fully_observable<T>(target, cam0, T_FC0) == false) {
      //   pan_trajs.clear();
      //   scale -= 0.05;
      //   retry--;
      //   if (retry == 0){
      //     FATAL("Failed to generate orbit trajectory!");
      //   }
      //   goto start;
      // }
      // if (check_fully_observable<T>(target, cam1, T_FC1) == false) {
      //   pan_trajs.clear();
      //   scale -= 0.05;
      //   retry--;
      //   if (retry == 0){
      //     FATAL("Failed to generate orbit trajectory!");
      //   }
      //   goto start;
      // }

			positions.emplace_back(tf_trans(T_WC0));
			attitudes.emplace_back(tf_rot(T_WC0));
		}

    // Add to trajectories
    const auto timestamps = linspace(ts_start, ts_end, nb_control_points);
    pan_trajs.emplace_back(timestamps, positions, attitudes);
  }

  // Append to results
  for (const auto &traj : pan_trajs) {
    trajs.emplace_back(traj.timestamps, traj.positions, traj.orientations);
  }
}

template <typename CAMERA>
void nbv_draw(const calib_target_t &target,
              const camera_params_t &cam_params,
              const mat4_t &T_FC,
              cv::Mat &image) {
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;

  const vec3_t r_FF0{0.0, 0.0, 0.0};
  const vec3_t r_FF1{calib_width, 0.0, 0.0};
  const vec3_t r_FF2{calib_width, calib_height, 0.0};
  const vec3_t r_FF3{0.0, calib_height, 0.0};

  const mat4_t T_CF = T_FC.inverse();
  const vec3_t r_CF0 = tf_point(T_CF, r_FF0);
  const vec3_t r_CF1 = tf_point(T_CF, r_FF1);
  const vec3_t r_CF2 = tf_point(T_CF, r_FF2);
  const vec3_t r_CF3 = tf_point(T_CF, r_FF3);

  const auto cam_res = cam_params.resolution;
  const vec4_t proj_params = cam_params.proj_params();
  const vec4_t dist_params = cam_params.dist_params();
	CAMERA camera{cam_res, proj_params, dist_params};

  vec2_t p0;
  vec2_t p1;
  vec2_t p2;
  vec2_t p3;
  camera.project(r_CF0, p0);
  camera.project(r_CF1, p1);
  camera.project(r_CF2, p2);
  camera.project(r_CF3, p3);

  const cv::Point2f pt0(p0(0), p0(1));
  const cv::Point2f pt1(p1(0), p1(1));
  const cv::Point2f pt2(p2(0), p2(1));
  const cv::Point2f pt3(p3(0), p3(1));
  const cv::Scalar color(0, 0, 255);
  const int thickness = 2;
  cv::line(image, pt0, pt1, color, thickness);
  cv::line(image, pt1, pt2, color, thickness);
  cv::line(image, pt2, pt3, color, thickness);
  cv::line(image, pt3, pt0, color, thickness);

  const auto corner_color = cv::Scalar(0, 0, 255);
  cv::circle(image, pt0, 1.0, corner_color, 5, 8);
  cv::circle(image, pt1, 1.0, corner_color, 5, 8);
  cv::circle(image, pt2, 1.0, corner_color, 5, 8);
  cv::circle(image, pt3, 1.0, corner_color, 5, 8);
}


template <typename CAMERA>
aprilgrids_t calib_simulate(const calib_target_t &target,
                            const mat4s_t &rel_poses,
                            const camera_params_t &cam_params) {
	const auto cam_res = cam_params.resolution;
	const auto proj_params = cam_params.proj_params();
	const auto dist_params = cam_params.dist_params();
	const CAMERA camera{cam_res, proj_params, dist_params};

  const int tag_rows = target.tag_rows;
  const int tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;
  const mat4s_t nbv_poses = calib_nbv_poses<CAMERA>(target, cam_params);

  aprilgrids_t grids;
  for (const mat4_t &T_FC : rel_poses) {
    aprilgrid_t grid(0, tag_rows, tag_cols, tag_size, tag_spacing);
    const mat4_t T_CF = T_FC.inverse();

    for (int tag_id = 0; tag_id < (tag_rows * tag_cols); tag_id++) {
      for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
        const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);
        const vec3_t r_CFi = tf_point(T_CF, r_FFi);

        vec2_t z_hat{0.0, 0.0};
        if (camera.project(r_CFi, z_hat) == 0) {
          grid.add(tag_id, corner_idx, z_hat);
        }
      }
    }

    grids.push_back(grid);
  }

  return grids;
}

// Returns reprojection uncertainty stddev in pixels
template <typename CAMERA>
int nbv_find(const calib_target_t &target,
             calib_mono_data_t &data,
             mat4_t &T_FC) {
  assert(data.problem);

  camera_params_t &cam_params = data.cam_params;
  ceres::Problem &problem = *data.problem;
  PoseLocalParameterization pose_plus;

  // Create NBVs and simulate observations
  const mat4s_t nbv_poses = calib_nbv_poses<CAMERA>(target, cam_params);
  const auto grids = calib_simulate<CAMERA>(target, nbv_poses, cam_params);

  // Evaluate observations
  const auto cam_res = cam_params.resolution;
  bool success = false;
  int best_index = 0;
  double best_score = std::numeric_limits<double>::max();

  for (size_t k = 0; k < nbv_poses.size(); k++) {
    const auto grid = grids[k];
    if (grid.detected == 0) {
      continue;
    }
    const mat4_t T_FC = nbv_poses[k];
    const mat4_t T_CF = T_FC.inverse();

    // Add current view residual blocks to problem
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    vec2s_t keypoints;
    vec3s_t object_points;
    grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

    pose_t pose{0, 0, T_CF};
    std::vector<calib_mono_residual_t<CAMERA> *> cost_fns;
    std::vector<ceres::ResidualBlockId> res_blocks;
    for (size_t i = 0; i < tag_ids.size(); i++) {
      const int tag_id = tag_ids[i];
      const int corner_idx = corner_indicies[i];
      const vec2_t z = keypoints[i];
      const vec3_t r_FFi = object_points[i];
      const mat2_t covar = I(2);

      auto cost_fn = new calib_mono_residual_t<CAMERA>{
        cam_res, tag_id, corner_idx,
        r_FFi, z, covar
      };
      auto res_id = problem.AddResidualBlock(cost_fn,
                                             NULL,
                                             pose.param.data(),
                                             cam_params.param.data());

      cost_fns.push_back(cost_fn);
      res_blocks.push_back(res_id);
    }
    problem.SetParameterization(pose.param.data(), &pose_plus);

    // Evaluate NBV that has the least uncertainty
    matx_t covar = std::numeric_limits<double>::max() * I(8);
    int retval = calib_mono_covar(cam_params, problem, covar);

    // Remove view from problem
    for (size_t i = 0; i < cost_fns.size(); i++) {
      problem.RemoveResidualBlock(res_blocks[i]);
      delete cost_fns[i];
    }
    problem.RemoveParameterBlock(pose.param.data());

    if (retval == 0 && covar.trace() < best_score) {
      best_score = covar.trace();
      best_index = k;
      success = true;
    }
  }

  // Set NBV pose
  T_FC = nbv_poses[best_index];

  return success ? 0 : -1;
}

struct nbv_test_grid_t {
  const int grid_rows = 5;
  const int grid_cols = 5;
  const double grid_depth = 1.5;
  const size_t nb_points = grid_rows * grid_cols;
  vec3s_t object_points;
  vec2s_t keypoints;

  template <typename T>
  nbv_test_grid_t(const T &cam) {
		const int img_w = cam.resolution[0];
		const int img_h = cam.resolution[1];
    const double dx = img_w / (grid_cols + 1);
    const double dy = img_h / (grid_rows + 1);
    double kp_x = 0;
    double kp_y = 0;

    for (int i = 1; i < (grid_cols + 1); i++) {
      kp_y += dy;
      for (int j = 1; j < (grid_rows + 1); j++) {
        kp_x += dx;

        // Keypoint
        const vec2_t kp{kp_x, kp_y};
        keypoints.push_back(kp);

        // Object point
        vec3_t ray;
        cam.back_project(kp, ray);
        object_points.push_back(ray * grid_depth);
      }
      kp_x = 0;
    }
  }
};

// // Returns reprojection uncertainty stddev in pixels
// template <typename T>
// double nbv_eval(const matx_t &calib_covar,
//                 const camera_params_t &cam0,
//                 const camera_params_t &cam1,
//                 const pose_t &extrinsics,
//                 cv::Mat *covar_map=nullptr) {
// 	// Setup
// 	// -- cam0
// 	const auto cam_res = cam0.resolution;
// 	const auto cam0_proj = cam0.proj_params();
// 	const auto cam0_dist = cam0.dist_params();
// 	const pinhole_radtan4_t cam0_geom{cam_res, cam0_proj, cam0_dist};
// 	// -- cam1
// 	const auto cam1_proj = cam1.proj_params();
// 	const auto cam1_dist = cam1.dist_params();
// 	const pinhole_radtan4_t cam1_geom{cam_res, cam1_proj, cam1_dist};
// 	// -- Test grid
//   const nbv_test_grid_t test_grid(cam0_geom);
//
//   if (covar_map) {
//     auto rows = cam0_geom.resolution[0];
//     auto cols = cam0_geom.resolution[1];
//     *covar_map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
//   }
//
//   // Evaluate reprojection uncertainty
//   double max_eigval = 0.0;
//   for (size_t i = 0; i < test_grid.nb_points; i++) {
//     const vec3_t r_CFi = test_grid.object_points[i];
//     const vec2_t kp = test_grid.keypoints[i];
//
//     // Project and camera params jacobian
//     vec2_t image_point;
//     mat_t<2, 3> Jh;
//     if (cam0_geom.project(r_CFi, image_point) != 0) {
//       FATAL("Failed to project point!");
//       continue;
//     }
//
//     // Input Jacobian
//     const vec2_t p{r_CFi(0) / r_CFi(2), r_CFi(1) / r_CFi(2)};
//     const matx_t J_cam0_params = cam0_geom.J_params(p);
//     mat_t<2, 8> Jx = J_cam0_params;
//
//     // Output covariance (First order error progagtion)
// 		const matx_t covar_x = calib_covar;
//     const matx_t covar_y = Jx * covar_x * Jx.transpose();
//
//     // Check covar_y
//     if (covar_y(0, 0) < 0 || covar_y(1, 1) < 0 || covar_y.determinant() < 0) {
//       LOG_WARN("Bad output covar");
//       print_matrix("covar_y", covar_y);
//       return INFINITY;
//     }
//
// 		// Find max eigen-value. The largest eigen-value represents the largest
// 		// "spread" or variance.
//     Eigen::SelfAdjointEigenSolver<matx_t> eigensolver(covar_y);
//     max_eigval = std::max(max_eigval, eigensolver.eigenvalues().maxCoeff());
//
//     if (covar_map) {
//       cv::Point p(kp(0), kp(1));
//       int radius = 5;
//       double percentage = 1.0 - eigensolver.eigenvalues().maxCoeff();
//
//       cv::Scalar color;
//       if (percentage < 0.4) {
//         color = cv::Scalar(0, 0, 255);    // Red
//       } else if (percentage < 0.9) {
//         color = cv::Scalar(0, 128, 255);  // Orage
//       } else if (percentage <= 1.0) {
//         color = cv::Scalar(0, 255, 0);    // Green
//       }
//
//       cv::circle(*covar_map, p, radius, color, cv::FILLED);
//     }
//   }
//
//   // Calculate sqrt(max variance), ie. stddev in pixels
//   return sqrt(max_eigval);
// }

} //  namespace yac
#endif // YAC_CALIB_NBV_HPP
