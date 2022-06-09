#include "calib_nbv.hpp"

namespace yac {

double shannon_entropy(const matx_t &covar) {
  assert(covar.rows() == covar.cols());
  const auto k = pow(2.0 * M_PI * std::exp(1.0), covar.rows());
  return 0.5 * std::log(k * covar.determinant());
}

bool check_fully_observable(const calib_target_t &target,
                            const camera_geometry_t *cam_geom,
                            const camera_params_t *cam_params,
                            const mat4_t &T_FCi) {
  const int tag_rows = target.tag_rows;
  const int tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;
  aprilgrid_t grid{0, tag_rows, tag_cols, tag_size, tag_spacing};

  const auto cam_res = cam_params->resolution;
  const vecx_t &params = cam_params->param;
  const mat4_t T_CiF = T_FCi.inverse();
  const int nb_tags = (grid.tag_rows * grid.tag_cols);

  for (int tag_id = 0; tag_id < nb_tags; tag_id++) {
    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);
      const vec3_t r_CiFi = tf_point(T_CiF, r_FFi);

      vec2_t z_hat;
      if (cam_geom->project(cam_res, params, r_CiFi, z_hat) != 0) {
        return false;
      }
    }
  }

  return true;
}

bool nbv_reached(const aprilgrid_t &nbv_target,
                 const aprilgrid_t &grid,
                 const real_t nbv_reproj_threshold,
                 std::vector<double> &reproj_errors) {
  // Pre-check
  if (grid.detected == false) {
    return false;
  }

  // Get measurements
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  vec2s_t keypoints;
  vec3s_t object_points;
  grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  // See if measured grid matches any NBV grid keypoints
  reproj_errors.clear();
  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indicies[i];
    if (nbv_target.has(tag_id, corner_idx) == false) {
      continue;
    }

    const vec2_t z_measured = keypoints[i];
    const vec2_t z_desired = nbv_target.keypoint(tag_id, corner_idx);
    reproj_errors.push_back((z_desired - z_measured).norm());
  }

  // Check if NBV is reached using reprojection errors
  const auto nbv_reproj_err = mean(reproj_errors);
  if (nbv_reproj_err > nbv_reproj_threshold) {
    return false;
  }

  return true;
}

/** Calculate target origin (O) w.r.t. fiducial (F) T_FO **/
int calib_target_origin(mat4_t &T_FO,
                        const calib_target_t &target,
                        const camera_geometry_t *cam_geom,
                        const camera_params_t *cam_params,
                        const double target_scale) {
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
  const double image_width = cam_params->resolution[0];
  const double target_half_width = target_width / 2.0;
  const double target_half_resolution_x = image_width / 2.0;
  const auto fx = cam_params->proj_params()[0];
  const auto z_FO = fx * target_half_width / (target_half_resolution_x * scale);

  // Form transform of calibration origin (O) wrt fiducial target (F) T_FO
  const mat3_t C_FO = I(3);
  const vec3_t r_FO{center(0), center(1), z_FO};
  T_FO = tf(C_FO, r_FO);

  // Rotate the camera around to see if camera can actually observe the target
  const vec3_t rpy = deg2rad(vec3_t{-180.0, 0.0, 0.0});
  const mat3_t C_FC = euler321(rpy);
  const mat4_t T_FC = tf(C_FC, r_FO);
  if (check_fully_observable(target, cam_geom, cam_params, T_FC) == false) {
    scale -= 0.1;
    retry--;

    if (retry == 0) {
      // FATAL("Failed to find calibration origin! Check camera params?");
      return -1;
    }
    goto start;
  }

  return 0;
}

int calib_init_poses(mat4s_t &poses,
                     const calib_target_t &target,
                     const camera_geometry_t *cam_geom,
                     const camera_params_t *cam_params) {
  // Target
  mat4_t T_FO;
  if (calib_target_origin(T_FO, target, cam_geom, cam_params, 0.5) != 0) {
    return -1;
  }
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

  // Push first pose to be infront of target center
  const vec3_t r_FC{target_center(0), target_center(1), z};
  const mat4_t T_FC = lookat(r_FC, target_center);
  poses.push_back(T_FC);

  // Generate camera positions infront of the AprilGrid target in the
  // fiducial frame, r_FC.
  vec3s_t cam_pos;
  cam_pos.emplace_back(vec3_t{range_w[0], range_h[1], z} + target_center);
  cam_pos.emplace_back(vec3_t{range_w[0], range_h[0], z} + target_center);
  cam_pos.emplace_back(vec3_t{range_w[1], range_h[0], z} + target_center);
  cam_pos.emplace_back(vec3_t{range_w[1], range_h[1], z} + target_center);

  // For each position create a camera pose that "looks at" the fiducial
  // center in the fiducial frame, T_FC.
  poses.clear();
  for (const auto &r_FC : cam_pos) {
    const mat4_t T_FC = lookat(r_FC, target_center);
    poses.push_back(T_FC);
  }

  return 0;
}

int calib_nbv_poses(mat4s_t &nbv_poses,
                    const calib_target_t &target,
                    const camera_geometry_t *cam_geom,
                    const camera_params_t *cam_params,
                    const int range_x_size,
                    const int range_y_size,
                    const int range_z_size) {
  // Target
  mat4_t T_FO;
  if (calib_target_origin(T_FO, target, cam_geom, cam_params, 0.8) != 0) {
    return -1;
  }
  const vec3_t r_FO = tf_trans(T_FO);
  const double target_width = r_FO(0) * 2;
  const double target_height = r_FO(1) * 2;
  const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  // Pose settings
  const double xy_scale = 1.6;
  const double d_start = r_FO(2);
  const double d_end = d_start * 1.5;
  const double h_width = target_width / 2.0;
  const double h_height = target_height / 2.0;
  const double range_w[2] = {-h_width * xy_scale, h_width * xy_scale};
  const double range_h[2] = {-h_height * xy_scale, h_height * xy_scale};
  const double range_d[2] = {d_start, d_end};
  const auto x_range = linspace(range_w[0], range_w[1], range_x_size);
  const auto y_range = linspace(range_h[0], range_h[1], range_y_size);
  const auto z_range = linspace(range_d[0], range_d[1], range_z_size);

  // Generate camera positions infront of the AprilGrid target in the fiducial
  // frame, r_FCi.
  vec3s_t cam_positions;
  for (const auto &x : x_range) {
    for (const auto &y : y_range) {
      for (const auto &z : z_range) {
        const vec3_t r_FCi = vec3_t{x, y, z} + target_center;
        cam_positions.push_back(r_FCi);
      }
    }
  }

  // For each position create a camera pose that "looks at" the fiducial
  // center in the fiducial frame, T_FCi.
  nbv_poses.clear();
  for (const auto &cam_pos : cam_positions) {
    const mat4_t T_FCi = lookat(cam_pos, target_center);
    nbv_poses.push_back(T_FCi);

    // Randomly perturb the pose a little
    // const auto roll = deg2rad(randf(-5.0, 5.0));
    // const auto pitch = deg2rad(randf(-5.0, 5.0));
    // const auto yaw = deg2rad(randf(-5.0, 5.0));
    // const vec3_t rpy{roll, pitch, yaw};
    // const mat3_t dC = euler321(rpy);
    // const mat4_t dT = tf(dC, zeros(3, 1));
    // poses.push_back(T_FCi * dT);
  }

  return 0;
}

int calib_nbv_poses(std::map<int, mat4s_t> &nbv_poses,
                    const calib_target_t &target,
                    const CamIdx2Geometry &cam_geoms,
                    const CamIdx2Parameters &cam_params,
                    const CamIdx2Extrinsics &cam_exts,
                    const int range_x_size,
                    const int range_y_size,
                    const int range_z_size) {
  nbv_poses.clear();
  for (const auto [cam_idx, _] : cam_geoms) {
    UNUSED(_);
    const auto geom = cam_geoms.at(cam_idx).get();
    const auto cam = cam_params.at(cam_idx).get();
    const auto ext = cam_exts.at(cam_idx);
    const mat4_t T_C0Ci = ext->tf();
    const mat4_t T_CiC0 = T_C0Ci.inverse();

    mat4s_t poses;
    auto retval = calib_nbv_poses(poses,
                                  target,
                                  geom,
                                  cam,
                                  range_x_size,
                                  range_y_size,
                                  range_z_size);
    if (retval != 0) {
      return -1;
    }

    for (const mat4_t T_FCi : poses) {
      nbv_poses[cam_idx].push_back(T_FCi * T_CiC0);
    }
  }

  // Note NBV poses are all relative to cam0
  return 0;
}

aprilgrid_t nbv_target_grid(const calib_target_t &target,
                            const camera_geometry_t *cam_geom,
                            const camera_params_t *cam_params,
                            const timestamp_t nbv_ts,
                            const mat4_t &T_FCi) {
  const int tag_rows = target.tag_rows;
  const int tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  const auto cam_res = cam_params->resolution;
  const vecx_t &params = cam_params->param;
  aprilgrid_t grid{nbv_ts, tag_rows, tag_cols, tag_size, tag_spacing};
  const mat4_t T_CiF = T_FCi.inverse();
  const int nb_tags = (grid.tag_rows * grid.tag_cols);

  for (int tag_id = 0; tag_id < nb_tags; tag_id++) {
    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      const vec3_t r_FFi = grid.object_point(tag_id, corner_idx);
      const vec3_t r_CiFi = tf_point(T_CiF, r_FFi);

      vec2_t z_hat{0.0, 0.0};
      if (cam_geom->project(cam_res, params, r_CiFi, z_hat) == 0) {
        grid.add(tag_id, corner_idx, z_hat);
      }
    }
  }

  return grid;
}

vec2s_t nbv_draw(const calib_target_t &target,
                 const camera_geometry_t *cam_geom,
                 const camera_params_t *cam_param,
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

  const auto cam_res = cam_param->resolution;
  const vecx_t params = cam_param->param;

  vec2_t p0;
  vec2_t p1;
  vec2_t p2;
  vec2_t p3;
  cam_geom->project(cam_res, params, r_CF0, p0);
  cam_geom->project(cam_res, params, r_CF1, p1);
  cam_geom->project(cam_res, params, r_CF2, p2);
  cam_geom->project(cam_res, params, r_CF3, p3);

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

  const auto red = cv::Scalar(0, 0, 255);
  const auto green = cv::Scalar(0, 255, 0);
  const auto blue = cv::Scalar(255, 0, 0);
  const auto yellow = cv::Scalar(0, 255, 255);
  cv::circle(image, pt0, 3.0, red, 5, 8);    // bottom left
  cv::circle(image, pt1, 3.0, green, 5, 8);  // bottom right
  cv::circle(image, pt2, 3.0, blue, 5, 8);   // top right
  cv::circle(image, pt3, 3.0, yellow, 5, 8); // top left

  return {p0, p1, p2, p3};
}

} // namespace yac
