#include "nbv.hpp"

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

mat4s_t calib_generate_poses(const calib_target_t &target) {
  // const real_t target_width = (target.tag_rows - 1.0) * target.tag_size;
  // const real_t target_height = (target.tag_cols - 1.0) * target.tag_size;
  // const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

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
  const auto x_range = linspace(-0.1, 0.1, 5);
  const auto y_range = linspace(-0.1, 0.1, 5);
  const auto z_range = linspace(0.3, 0.5, 5);

  // Generate camera positions infront of the AprilGrid target in the target
  // frame, r_TC.
  vec3s_t cam_positions;
  for (const auto &x : x_range) {
    for (const auto &y : y_range) {
      for (const auto &z : z_range) {
        const vec3_t r_TC = vec3_t{x, y, z} + target_center;
        cam_positions.push_back(r_TC);
      }
    }
  }

  // For each position create a camera pose that "looks at" the AprilGrid
  // center in the target frame, T_TC.
  mat4s_t poses;
  for (const auto &cam_pos : cam_positions) {
    mat4_t T_TC = lookat(cam_pos, target_center);

    // // Perturb rotation
    // mat3_t C_TC = tf_rot(T_TC);
    // const vec3_t rpy{randf(-0.4, 0.4), randf(-0.4, 0.4), randf(-0.4, 0.4)};
    // const mat3_t C_perturb = euler321(rpy);
    // C_TC = C_perturb * C_TC;
    // T_TC.block(0, 0, 3, 3) = C_TC;

    // mat4_t T_TC = tf(I(3), cam_position);
    poses.push_back(T_TC);
  }

  return poses;
}

mat4s_t generate_initial_poses(const calib_target_t &target) {
  // Calculate target width and height
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
  const double calib_depth = calib_width * 1.2;

  // Generate intial poses
  mat4s_t poses;
  // -- First pose is right infront of calibration target
  const vec3_t calib_start{calib_width / 2.0, calib_height / 2.0, calib_depth};
  poses.push_back(lookat(calib_start, calib_center));
  // -- Other 4 are the 4 corners of the aprilgrid
  for (const auto &x : linspace(0.0, calib_width * 1.5, 2)) {
    for (const auto &y : linspace(0.0, calib_height * 1.5, 2)) {
      vec3_t cam_pos{x, y, calib_depth};
      poses.push_back(lookat(cam_pos - (0.5 * calib_center), calib_center));
    }
  }

  return poses;
}

mat4s_t generate_nbv_poses(const calib_target_t &target) {
  // const vec2_t cam_res{640, 480};
  // const double hfov = 90.0;
  // mat4_t T_FO = calib_target_origin(target, cam_res, hfov);

  // Calculate target width and height
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
  const double calib_depth = calib_width * 1.2;

  mat4s_t nbv_poses;
  for (const auto &x : linspace(0.0, calib_width * 1.5, 5)) {
    for (const auto &y : linspace(0.0, calib_height * 1.5, 5)) {
      for (const auto &z : linspace(calib_depth, calib_depth * 1.5, 5)) {
        vec3_t cam_pos{x, y, z};
        nbv_poses.push_back(lookat(cam_pos - (0.5 * calib_center), calib_center));
      }
    }
  }

  return nbv_poses;

  // // Trajectory parameters
  // const double rho = (calib_width / 2.0) * 0.9;  // Sphere radius
  // const double lat_min = deg2rad(0.0);
  // const double lat_max = deg2rad(360.0);
  // const double lon_min = deg2rad(0.0);
  // const double lon_max = deg2rad(80.0);

  // // Form Sphere offset (J) w.r.t. calibration origin (O)
  // // The sphere origin is at the center of the sphere (duh), but we actually
  // // want the north pole of the sphere to be the trajectory start point
  // // therefore we need this offset.
  // const mat3_t C_OJ = I(3);
  // const vec3_t r_OJ{0.0, 0.0, -rho};
  // const mat4_t T_OJ = tf(C_OJ, r_OJ);
  //
  // // // Create rotation matrix that converts z-forward to x-forward
  // // const auto rpy = deg2rad(vec3_t{-90.0, 0.0, -90.0});
  // // const auto C_BC = euler321(rpy);
  // // const auto r_BC = zeros(3, 1);
  // // const auto T_BC = tf(C_BC, r_BC);
  // // const auto T_CB = T_BC.inverse();
  //
  // // Target center (Fc) w.r.t. Target origin (F)
  // const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};
  //
  // // Orbit poses. Imagine a half sphere coming out from the calibration target
  // // center. The trajectory would go from the pole of the sphere to the sides
  // // of the sphere. While following the trajectory in a tangent manner the
  // // camera view focuses on the target center.
  // mat4s_t nbv_poses;
  // for (const auto &lat : linspace(lat_min, lat_max, 9)) {
  //   // Create sphere point and transform it into world frame
  //   for (const auto &lon : linspace(lon_min, lon_max, 10)) {
  //     const vec3_t p = sphere(rho, lon, lat);
  //     const vec4_t hr_FJ = T_FO * T_OJ * p.homogeneous();
  //     const vec3_t r_FJ = hr_FJ.head(3);
  //     const mat4_t T_FC = lookat(r_FJ, r_FFc);
  //
  //     // Offset from camera (z-forward) to whatever imu frame is forward
  //     vec3_t rpy{0.0, 0.0, - M_PI / 2.0};
  //     mat3_t C = euler321(rpy);
  //     mat4_t T_offset = tf(C, zeros(3, 1));
  //
  //     // Form nbv pose
  //     nbv_poses.emplace_back(T_FC * T_offset);
  //   }
  // }

  // return nbv_poses;
}

} // namespace yac
