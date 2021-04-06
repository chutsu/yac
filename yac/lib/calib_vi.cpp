#include "calib_vi.hpp"

namespace yac {

void calib_vi_init_poses(const calib_target_t &target,
                         const mat4_t &T_FO,
                         std::deque<mat4_t> &poses) {
  // Tag width
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t r_FFc{calib_width / 2.0, calib_height / 2.0, 0.0};

  // Create initial poses
  const auto angle = 30.0;
  // -- First position (left of calibration origin)
  {
    // const vec3_t r_OP{-calib_width / 2.0, 0.0, 0.0};
    const vec3_t r_OP{0.0, 0.0, 0.0};
    const vec3_t r_FJ = tf_point(T_FO, r_OP);
    const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
    const mat3_t C = euler321(deg2rad(vec3_t{0.0, 0.0, -angle}));
    const mat4_t T = tf(C, zeros(3, 1));
    poses.push_back(T_FC0 * T);
  }
  // -- Second position (at calibration origin)
  {
    // const vec3_t r_OP{calib_width / 2.0, 0.0, 0.0};
    const vec3_t r_OP{0.0, 0.0, 0.0};
    const vec3_t r_FJ = tf_point(T_FO, r_OP);
    const mat4_t T_FC0 = lookat(r_FJ, r_FFc);
    const mat3_t C = euler321(deg2rad(vec3_t{0.0, 0.0, angle}));
    const mat4_t T = tf(C, zeros(3, 1));
    poses.push_back(T_FC0 * T);
  }
}

} // namespace yac
