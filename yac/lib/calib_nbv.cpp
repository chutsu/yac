#include "calib_nbv.hpp"

namespace yac {

mat4_t calib_target_origin(const calib_target_t &target,
                           const vec2_t &cam_res,
                           const double hfov) {
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  // Standard pinhole camera model equation
  // x = f * X / Z
  // x / f * X = 1 / Z
  // Z = f * X / x

  // Calculate target center
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double width = tag_cols * tag_size + spacing_x;
  const double height = tag_rows * tag_size + spacing_y;
  const vec2_t center{width / 2.0, height / 2.0};

  // Calculate distance away from target center
  const double half_width =  width / 2.0;
  const double half_resolution_x = cam_res(0) / 2.0;
  const double scale = 0.5; // Target width to be 50% of image space at T_TO
  const double image_width = cam_res(0);
  const auto fx = pinhole_focal(image_width, hfov);
  const auto z_FO = fx * half_width / (half_resolution_x * scale);

  // Form transform of calibration origin (O) wrt fiducial target (F) T_FO
  const mat3_t C_FO = I(3);
  const vec3_t r_FO{center(0), center(1), z_FO};
  const mat4_t T_FO = tf(C_FO, r_FO);

  return T_FO;
}

mat4s_t calib_init_poses(const calib_target_t &target) {
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double target_width = tag_cols * tag_size + spacing_x;
  const double target_height = tag_rows * tag_size + spacing_y;
  const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  // Pose settings
  const double h_width = target_width / 2.0;
  const double h_height = target_height / 2.0;
  const double xy_scale = 1.2;
  const double range_w[2] = {-h_width * xy_scale, h_width * xy_scale};
  const double range_h[2] = {-h_height * xy_scale, h_height * xy_scale};
  const auto x_range = linspace(range_w[0], range_w[1], 2);
  const auto y_range = linspace(range_h[0], range_h[1], 2);
  const double z = h_width * 2;
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

mat4s_t calib_nbv_poses(const calib_target_t &target,
                        const int range_x_size,
                        const int range_y_size,
                        const int range_z_size) {
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double target_width = tag_cols * tag_size + spacing_x;
  const double target_height = tag_rows * tag_size + spacing_y;
  const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  const double h_width = target_width / 2.0;
  const double h_height = target_height / 2.0;

  // Pose settings
  const double xy_scale = 1.2;
  const double d_scale[2] = {0.6, 2.0};
  const double range_w[2] = {-h_width * xy_scale, h_width * xy_scale};
  const double range_h[2] = {-h_height * xy_scale, h_height * xy_scale};
  const double range_d[2] = {h_width * d_scale[0], h_width * d_scale[1]};
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

} // namespace yac
