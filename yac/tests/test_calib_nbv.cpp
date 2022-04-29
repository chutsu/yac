#include "munit.hpp"
#include "calib_nbv.hpp"

namespace yac {

static std::vector<camera_params_t> setup_cameras() {
  const int img_w = 640;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";

  const double fx = pinhole_focal(img_w, 69.4);
  const double fy = pinhole_focal(img_h, 42.5);
  const double cx = 640.0 / 2.0;
  const double cy = 480.0 / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.001, 0.001};

  // clang-format off
  camera_params_t cam0{0, cam_res, proj_model, dist_model, proj_params, dist_params};
  camera_params_t cam1{1, cam_res, proj_model, dist_model, proj_params, dist_params};
  // clang-format on

  return {cam0, cam1};
}

static void setup_calib_target(const camera_params_t &cam,
                               calib_target_t &target,
                               mat4_t &T_FO,
                               mat4_t *T_WF = nullptr) {
  // Create calibration origin
  pinhole_radtan4_t cam_geom;
  target = calib_target_t{"aprilgrid", 6, 6, 0.088, 0.3};
  calib_target_origin(T_FO, target, &cam_geom, &cam);

  // Calibration target pose
  if (T_WF) {
    const vec3_t rpy = deg2rad(vec3_t{90.0, 0.0, -90.0});
    const mat3_t C_WF = euler321(rpy);
    *T_WF = tf(C_WF, zeros(3, 1));
  }
}

int test_calib_target_origin() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  pinhole_radtan4_t cam_geom;
  camera_params_t cam_params{0,
                             cam_res,
                             proj_model,
                             dist_model,
                             proj_params,
                             dist_params};

  mat4_t T_FO;
  MU_CHECK(calib_target_origin(T_FO, target, &cam_geom, &cam_params) == 0);
  print_matrix("T_FO", T_FO);

  return 0;
}

int test_calib_init_poses() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  pinhole_radtan4_t cam_geom;
  camera_params_t cam_params{0,
                             cam_res,
                             proj_model,
                             dist_model,
                             proj_params,
                             dist_params};

  mat4s_t poses;
  MU_CHECK(calib_init_poses(poses, target, &cam_geom, &cam_params) == 0);
  const std::string save_path = "/tmp/calib_poses.csv";
  timestamps_t timestamps;
  for (size_t k = 0; k < poses.size(); k++) {
    timestamps.push_back(0);
  }
  save_poses(save_path, timestamps, poses);

  return 0;
}

int test_calib_nbv_poses() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  pinhole_radtan4_t cam_geom;
  camera_params_t cam_params{0,
                             cam_res,
                             proj_model,
                             dist_model,
                             proj_params,
                             dist_params};

  mat4s_t poses;
  MU_CHECK(calib_nbv_poses(poses, target, &cam_geom, &cam_params) == 0);
  const std::string save_path = "/tmp/calib_poses.csv";
  timestamps_t timestamps;
  for (size_t k = 0; k < poses.size(); k++) {
    timestamps.push_back(0);
  }
  save_poses(save_path, timestamps, poses);

  return 0;
}

int test_nbv_draw() {
  // Setup Camera
  const int cam_idx = 0;
  const int img_w = 752;
  const int img_h = 480;
  const int cam_res[2] = {img_w, img_h};
  const double lens_hfov = 90.0;
  const double lens_vfov = 90.0;
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(img_w, lens_hfov);
  const double fy = pinhole_focal(img_h, lens_vfov);
  const double cx = img_w / 2.0;
  const double cy = img_h / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
  const pinhole_radtan4_t cam_geom;
  const camera_params_t cam_params{cam_idx,
                                   cam_res,
                                   proj_model,
                                   dist_model,
                                   proj_params,
                                   dist_params};

  // Setup nbv poses
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  mat4s_t poses;
  MU_CHECK(calib_init_poses(poses, target, &cam_geom, &cam_params) == 0);
  const std::string save_path = "/tmp/calib_poses.csv";
  timestamps_t timestamps;
  for (size_t k = 0; k < poses.size(); k++) {
    timestamps.push_back(0);
  }
  save_poses(save_path, timestamps, poses);

  for (const auto &T_FC : poses) {
    cv::Mat image(cam_res[1], cam_res[0], CV_8UC3, cv::Scalar(255, 255, 255));
    nbv_draw(target, &cam_geom, &cam_params, T_FC, image);
    cv::imshow("image", image);
    cv::waitKey(0);
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_target_origin);
  MU_ADD_TEST(test_calib_init_poses);
  MU_ADD_TEST(test_calib_nbv_poses);
  MU_ADD_TEST(test_nbv_draw);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
