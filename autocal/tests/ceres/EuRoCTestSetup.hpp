#ifndef EUROC_TEST_SETUP_HPP
#define EUROC_TEST_SETUP_HPP

#include "autocal/common/Core.hpp"
#include "autocal/common/AprilGrid.hpp"
#include "autocal/common/CalibData.hpp"
#include "autocal/common/EuRoC.hpp"
#include "autocal/common/Timeline.hpp"
#include "autocal/common/Parameters.hpp"
#include "autocal/common/Measurements.hpp"
#include "autocal/cv/CameraGeometry.hpp"
#include "autocal/param/ParameterBlock.hpp"
#include "autocal/param/PoseParameterBlock.hpp"
#include "autocal/param/SpeedAndBiasParameterBlock.hpp"
#include "autocal/param/TimeDelayParameterBlock.hpp"
#include "autocal/param/CameraParameterBlock.hpp"
#include "autocal/param/FiducialParameterBlock.hpp"

using namespace autocal;

vecx_t get_cam0_params_gnd() {
  // Euroc
  const double fx = 458.654;
  const double fy = 457.296;
  const double cx = 367.215;
  const double cy = 248.375;
  const double k1 = -0.28340811;
  const double k2 = 0.07395907;
  const double p1 = 0.00019359;
  const double p2 = 1.76187114e-05;

  // // Autocal
  // const double fx = 458.7566583;
  // const double fy = 457.4404671;
  // const double cx = 366.2358242;
  // const double cy = 248.3385405;
  // const double k1 = -0.2873913;
  // const double k2 = 0.0774326;
  // const double p1 = 0.0002425;
  // const double p2 = 7.2e-05;

  vecx_t params(8);
  params << fx, fy, cx, cy, k1, k2, p1, p2;

  return params;
}

vecx_t get_cam0_params_est() {
  const double fx = pinhole_focal(752.0, 90.0);
  const double fy = pinhole_focal(480.0, 73.0);
  const double cx = 752.0 / 2.0;
  const double cy = 480.0 / 2.0;
  const double k1 = 0.0;
  const double k2 = 0.0;
  const double p1 = 0.0;
  const double p2 = 0.0;

  vecx_t params(8);
  params << fx, fy, cx, cy, k1, k2, p1, p2;

  return params;
}

vecx_t get_cam1_params_gnd() {
  // Euroc
  const double fx = 457.587;
  const double fy = 456.134;
  const double cx = 379.999;
  const double cy = 255.238;
  const double k1 = -0.28368365;
  const double k2 = 0.07451284;
  const double p1 = -0.00010473;
  const double p2 = -3.55590700e-05;

  // // Autocal
  // const double fx = 457.3817361;
  // const double fy = 456.0005882;
  // const double cx = 378.9001604;
  // const double cy = 255.2982812;
  // const double k1 = -0.2847054;
  // const double k2 = 0.075288;
  // const double p1 = -4.57e-05;
  // const double p2 = 6.26e-05;

  vecx_t params(8);
  params << fx, fy, cx, cy, k1, k2, p1, p2;

  return params;
}

vecx_t get_cam1_params_est() {
  const double fx = pinhole_focal(752.0, 90.0);
  const double fy = pinhole_focal(480.0, 73.0);
  const double cx = 752.0 / 2.0;
  const double cy = 480.0 / 2.0;
  const double k1 = 0.0;
  const double k2 = 0.0;
  const double p1 = 0.0;
  const double p2 = 0.0;

  vecx_t params(8);
  params << fx, fy, cx, cy, k1, k2, p1, p2;

  return params;
}

PinholeRadtan get_cam0_geometry() {
  const int image_width = 752;
  const int image_height = 480;
  auto cam0_params = get_cam0_params_gnd();
  // auto cam0_params = get_cam0_params_est();

  return PinholeRadtan(image_width, image_height,
                       cam0_params(0), cam0_params(1), cam0_params(2), cam0_params(3),
                       {cam0_params(4), cam0_params(5), cam0_params(6), cam0_params(7)});
}

PinholeRadtan get_cam1_geometry() {
  const int image_width = 752;
  const int image_height = 480;
  auto cam1_params = get_cam1_params_gnd();
  // auto cam1_params = get_cam1_params_est();

  return PinholeRadtan(image_width, image_height,
                       cam1_params(0), cam1_params(1), cam1_params(2), cam1_params(3),
                       {cam1_params(4), cam1_params(5), cam1_params(6), cam1_params(7)});
}

mat4_t get_T_SC0_gnd() {
  // mat4_t T_SC0;
  // // clang-format off
  // T_SC0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //          0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //          0.0, 0.0, 0.0, 1.0;
  // // clang-format on
  // return T_SC0;

  mat4_t T_SC0;
  // clang-format off
  T_SC0 << 0.0, -1.0, 0.0, 0.0,
           1.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 1.0;
  // clang-format on
  return T_SC0;
}

mat4_t get_T_SC1_gnd() {
  // mat4_t T_C1C0;
  // // clang-format off
  // T_C1C0 << 0.999997256477881, 0.002312067192424, 0.000376008102415, -0.110073808127187,
  //           -0.002317135723281, 0.999898048506644, 0.014089835846648, 0.000399121547014,
  //           -0.000343393120525, -0.014090668452714, 0.999900662637729, -0.000853702503357,
  //           0.0, 0.0, 0.0, 1.0;
  // // clang-format on
  // mat4_t T_C0C1 = T_C1C0.inverse();
  // const auto T_SC0 = get_T_SC0_gnd();
  // return T_SC0 * T_C0C1;

  mat4_t T_SC1;
  // clang-format off
  T_SC1 << 0.0, -1.0, 0.0, 0.0,
           1.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 1.0;
  // clang-format on
  return T_SC1;
}

autocal::ImuParameters get_imu_params() {
  autocal::ImuParameters imu_params;

  imu_params.T_BS.setIdentity();  // Transformation from body to imu
  imu_params.a_max = 176.0;       // Acceleration saturation [m/s^2]
  imu_params.g_max = 7.8;         // Gyro saturation [rad/s]

  imu_params.sigma_bg = 0.03;     // Gyro bias prior [rad/s]
  imu_params.sigma_ba = 0.1;      // Accel bias prior [m/s^2]

  imu_params.sigma_a_c = 0.002;  // Accel noise density [m/s^2/sqrt(Hz)]
  imu_params.sigma_aw_c = 0.003; // Accel drift noise density [m/s^2/sqrt(Hz)]

  imu_params.sigma_g_c = 1.6968e-04; // Gyro noise density [rad/s/sqrt(Hz)]
  imu_params.sigma_gw_c = 1.9393e-05; // Gyro drift noise density [rad/s^s/sqrt(Hz)]

  imu_params.tau = 3600.0; // Reversion time constant, currently not in use [s]
  imu_params.g = 9.81007;  // Earth's acceleration due to gravity [m/s^2]
  imu_params.a0.setZero();
  imu_params.rate = 200; // Hz

  return imu_params;
}

struct euroc_test_data_t {
  // cam0
  vecx_t cam0_params;
  matx_t cam0_K;
  vecx_t cam0_D;
  PinholeRadtan cam0_geometry;

  // cam1
  vecx_t cam1_params;
  matx_t cam1_K;
  vecx_t cam1_D;
  PinholeRadtan cam1_geometry;

  // imu0
  autocal::ImuParameters imu0_params;

  // AprilGrids
  calib_target_t target_params;
  aprilgrids_t cam0_grids;
  aprilgrids_t cam1_grids;

  // Camera images
  std::vector<std::string> cam0_imgs;
  std::vector<std::string> cam1_imgs;

  // Extrinsics
  mat4_t T_SC0 = I(4);
  mat4_t T_SC1 = I(4);

  // Sensor poses and fiducial pose
  mat4s_t T_WS;
  mat4_t T_WF;
  timestamp_t t0;

  // Timeline
  timeline_t<timestamp_t> timeline;
};

euroc_test_data_t load_euroc_test_data(const std::string &data_path) {
  printf("Loading EuRoC dataset [%s]\n", data_path.c_str());
  euroc_calib_t data{data_path};
  if (data.ok == false) {
    FATAL("Failed to load data [%s]!", data_path.c_str());
  }

  // Setup test data
  euroc_test_data_t test_data;
  // -- cam0
  test_data.cam0_params = get_cam0_params_gnd();
  test_data.cam0_K = pinhole_K(test_data.cam0_params.head(4));
  test_data.cam0_D = test_data.cam0_params.tail(4);
  test_data.cam0_geometry = get_cam0_geometry();
  test_data.cam0_imgs = data.cam0_data.image_paths;
  // -- cam1
  test_data.cam1_params = get_cam1_params_gnd();
  test_data.cam1_K = pinhole_K(test_data.cam1_params.head(4));
  test_data.cam1_D = test_data.cam1_params.tail(4);
  test_data.cam1_geometry = get_cam1_geometry();
  test_data.cam1_imgs = data.cam1_data.image_paths;
  // -- imu0
  test_data.imu0_params = get_imu_params();
  // -- Calib target
  test_data.target_params.target_type = "aprilgrid";
  test_data.target_params.tag_rows = 6;
  test_data.target_params.tag_cols = 6;
  test_data.target_params.tag_size = 0.088;
  test_data.target_params.tag_spacing = 0.3;
  // -- Load aprilgrid data
  const std::string output_path = "/tmp";
  process_stereo_images(data,
                        output_path,
                        test_data.cam0_K,
                        test_data.cam0_D,
                        test_data.cam1_K,
                        test_data.cam1_D,
                        test_data.cam0_grids,
                        test_data.cam1_grids);
  // -- Create timeline
  test_data.T_SC0 = get_T_SC0_gnd();
  test_data.T_SC1 = get_T_SC1_gnd();
  test_data.timeline = create_timeline(data,
                                       test_data.cam0_grids,
                                       test_data.cam1_grids,
                                       test_data.T_SC0,
                                       test_data.T_WS,
                                       test_data.T_WF,
                                       test_data.t0);

  return test_data;
}

CameraParameterBlock
create_cam_block(const vecx_t &cam_params, int &param_id) {
  CameraParameterBlock cam_block(cam_params, param_id);
  param_id++;
  return cam_block;
}

PoseParameterBlock create_pose_block(const mat4_t &pose,
                                                     int &param_id) {
  PoseParameterBlock
      pose_block(autocal::Transformation{pose}, param_id);
  param_id++;
  return pose_block;
}

SpeedAndBiasParameterBlock create_speed_bias_block(
    const timestamp_t &ts, const double *state_data, int &param_id) {
  autocal::Time timestamp{ns2sec(ts)};
  autocal::SpeedAndBias state{state_data};
  SpeedAndBiasParameterBlock param_block(state,
                                                         param_id,
                                                         timestamp);
  param_id++;
  return param_block;
}

TimeDelayParameterBlock
create_time_delay_block(const double time_delay, int &param_id) {
  TimeDelayParameterBlock param_block(time_delay, param_id);
  param_id++;
  return param_block;
}

FiducialParameterBlock
create_fiducial_block(const mat4_t &pose, int &param_id) {
  FiducialParameterBlock block(
    autocal::Transformation{pose}, param_id);
  param_id++;
  return block;
}

void save_sensor_poses(const std::string &save_path,
                       const mat4s_t &sensor_poses) {
  matx_t cam_pose_mat = matx_t::Zero(sensor_poses.size() * 4, 4);
  for (size_t i = 0; i < sensor_poses.size(); i++) {
    const mat4_t T_WC = sensor_poses[i];
    cam_pose_mat.block<4, 4>(i * 4, 0) = T_WC;
  }
  if (mat2csv(save_path, cam_pose_mat) != 0) {
    FATAL("Failed to save cam poses!");
  }
}

// void save_estimated_sensor_poses(
//     const std::string &save_path,
//     const Calibrator<PinholeRadtan> &calib) {
//   mat4s_t pose_est;
//   for (const auto &param_block : calib.T_WS_blocks_) {
//     pose_est.emplace_back(param_block.get()->estimate().T());
//   }
//   save_sensor_poses(save_path, pose_est);
// }
//
// void save_estimated_states(
//     const std::string &save_path,
//     const Calibrator<PinholeRadtan> &calib) {
//   size_t nb_states = calib.state_blocks_.size();
//   matx_t state_mat = matx_t::Zero(nb_states, 9);
//   for (size_t i = 0; i < nb_states; i++) {
//     const auto param_block = calib.state_blocks_[i].get();
//     state_mat.block<1, 9>(i, 0) = param_block->estimate().transpose();
//   }
//   if (mat2csv(save_path, state_mat) != 0) {
//     FATAL("Failed to save states!");
//   }
// }

void save_cam_timestamps(const std::string &save_path,
                         const euroc_calib_t &calib_data) {
  std::ofstream outfile(save_path);
  const size_t nb_timestamps = calib_data.cam0_data.timestamps.size();
  for (size_t i = 0; i < nb_timestamps; i++) {
    outfile << calib_data.cam0_data.timestamps[i] << std::endl;
  }
  outfile.close();
}

void save_imu_timestamps(const std::string &save_path,
                         const euroc_calib_t &calib_data) {
  const size_t nb_timestamps = calib_data.imu_data.timestamps.size();
  std::ofstream outfile(save_path);
  for (size_t i = 0; i < nb_timestamps; i++) {
    const timestamp_t ts = calib_data.imu_data.timestamps[i];
    outfile << ts << std::endl;
  }
  outfile.close();
}

void print_camera_params_summary(const size_t cam_id,
                                 const vecx_t cam_params_init,
                                 const vecx_t cam_params_gnd,
                                 const vecx_t cam_params_est) {
  printf("Camera %d parameters:\n", (int) cam_id);
  printf("----------------------------------------\n");

  printf("-- Intrinsics:\n");
  printf("fx (init): %.5f\t", cam_params_init[0]);
  printf("fx  (gnd): %.5f\t", cam_params_gnd[0]);
  printf("fx  (est): %.5f\n", cam_params_est[0]);

  printf("fy (init): %.5f\t", cam_params_init[1]);
  printf("fy  (gnd): %.5f\t", cam_params_gnd[1]);
  printf("fy  (est): %.5f\n", cam_params_est[1]);

  printf("cx (init): %.5f\t", cam_params_init[2]);
  printf("cx  (gnd): %.5f\t", cam_params_gnd[2]);
  printf("cx  (est): %.5f\n", cam_params_est[2]);

  printf("cy (init): %.5f\t", cam_params_init[3]);
  printf("cy  (gnd): %.5f\t", cam_params_gnd[3]);
  printf("cy  (est): %.5f\n", cam_params_est[3]);

  printf("-- Distortion:\n");
  printf("k1 (init): %.5f\t", cam_params_init[4]);
  printf("k1  (gnd): %.5f\t", cam_params_gnd[4]);
  printf("k1  (est): %.5f\n", cam_params_est[4]);

  printf("k2 (init): %.5f\t", cam_params_init[5]);
  printf("k2  (gnd): %.5f\t", cam_params_gnd[5]);
  printf("k2  (est): %.5f\n", cam_params_est[5]);

  printf("p1 (init): %.5f\t", cam_params_init[6]);
  printf("p1  (gnd): %.5f\t", cam_params_gnd[6]);
  printf("p1  (est): %.5f\n", cam_params_est[6]);

  printf("p2 (init): %.5f\t", cam_params_init[6]);
  printf("p2  (gnd): %.5f\t", cam_params_gnd[6]);
  printf("p2  (est): %.5f\n", cam_params_est[6]);
}

void print_sensor_camera_extrinsics_summary(const size_t cam_id,
                                            const mat4_t &T_SC_init,
                                            const mat4_t &T_SC_gnd,
                                            const mat4_t &T_SC_est) {
  const std::string T_SC_str = "T_SC" + std::to_string(cam_id);

  printf("Sensor Camera Extrinsics Results:\n");
  printf("----------------------------------------\n");
  std::cout << "init " << T_SC_str << ":\n"
            << T_SC_init << std::endl
            << std::endl;
  std::cout << "gnd " << T_SC_str << ":\n"
            << T_SC_gnd << std::endl
            << std::endl;
  std::cout << "est " << T_SC_str << ":\n"
            << T_SC_est << std::endl
            << std::endl;
  std::cout << std::endl;

  const auto q_SC_init = quat_t{tf_rot(T_SC_init)};
  const auto q_SC_gnd = quat_t{tf_rot(T_SC_gnd)};
  const auto q_SC_est = quat_t{tf_rot(T_SC_est)};
  const auto rpy_init =
      rad2deg(quat2euler(q_SC_init)).transpose();
  const auto rpy_gnd = rad2deg(quat2euler(q_SC_gnd)).transpose();
  const auto rpy_est = rad2deg(quat2euler(q_SC_est)).transpose();
  std::cout << "init rpy_SC" << cam_id << ": " << rpy_init << std::endl;
  std::cout << "gnd rpy_SC" << cam_id << ": " << rpy_gnd << std::endl;
  std::cout << "est rpy_SC" << cam_id << ": " << rpy_est << std::endl;
  std::cout << "diff: " << (rpy_est.transpose() - rpy_gnd.transpose()).norm()
            << std::endl;
  std::cout << std::endl;

  const auto t_SC_init = tf_trans(T_SC_init);
  const auto t_SC_gnd = tf_trans(T_SC_gnd);
  const auto t_SC_est = tf_trans(T_SC_est);
  std::cout << "init t_SC: " << t_SC_init.transpose() << std::endl;
  std::cout << "gnd t_SC: " << t_SC_gnd.transpose() << std::endl;
  std::cout << "est t_SC: " << t_SC_est.transpose() << std::endl;
  std::cout << "diff t_SC: " << (t_SC_est - t_SC_gnd).norm() << std::endl;
  std::cout << std::endl;
}

void print_stereo_extrinsics_summary(const mat4_t &T_SC0_gnd,
                                     const mat4_t &T_SC1_gnd,
                                     const mat4_t &T_SC0_est,
                                     const mat4_t &T_SC1_est) {
  const auto T_C0C1_gnd = T_SC0_gnd.inverse() * T_SC1_gnd;
  const auto T_C0C1_est = T_SC0_est.inverse() * T_SC1_est;
  printf("Stereo Extrinsics Results:\n");
  printf("----------------------------------------\n");
  std::cout << "T_C0C1_gnd:\n" << T_C0C1_gnd << std::endl << std::endl;
  std::cout << "T_C0C1_est:\n" << T_C0C1_est << std::endl << std::endl;
  std::cout << std::endl;
}

// void print_summary(
//     const Calibrator<PinholeRadtan> &calib,
//     const timeline_t<timestamp_t> &timeline) {
//   // Print cam0 params
//   if (calib.cam_blocks_.size()) {
//     const auto cam0_params_init = calib.cam_params_.at(0).params();
//     const auto cam0_params_gnd = get_cam0_params_gnd();
//     const auto cam0_params_est = calib.cam_blocks_.at(0).get()->estimate();
//     print_camera_params_summary(0,
//                                 cam0_params_init,
//                                 cam0_params_gnd,
//                                 cam0_params_est);
//     printf("\n");
//   }
//
//   // Print cam1 params
//   if (calib.cam_blocks_.size() == 2) {
//     const auto cam1_params_init = calib.cam_params_.at(1).params();
//     const auto cam1_params_gnd = get_cam1_params_gnd();
//     const auto cam1_params_est = calib.cam_blocks_.at(1).get()->estimate();
//     print_camera_params_summary(1,
//                                 cam1_params_init,
//                                 cam1_params_gnd,
//                                 cam1_params_est);
//     printf("\n");
//   }
//   printf("========================================\n\n");
//
//   // Print sensor to cam0 extrinsics
//   const auto T_SC0_init = calib.extrinsic_params_.at(0).T();
//   const auto T_SC0_gnd = get_T_SC0_gnd();
//   const auto T_SC0_est =
//       calib.sensor_camera_extrinsic_blocks_.at(0).get()->estimate().T();
//   print_sensor_camera_extrinsics_summary(0, T_SC0_init, T_SC0_gnd, T_SC0_est);
//   printf("========================================\n\n");
//
//   // Print sensor to cam1 extrinsics
//   if (calib.sensor_camera_extrinsic_blocks_.size() == 2) {
//     const auto T_SC1_init = calib.extrinsic_params_.at(1).T();
//     const auto T_SC1_gnd = get_T_SC1_gnd();
//     const auto T_SC1_est =
//         calib.sensor_camera_extrinsic_blocks_.at(1).get()->estimate().T();
//     print_sensor_camera_extrinsics_summary(1, T_SC1_init, T_SC1_gnd, T_SC1_est);
//   }
//   printf("========================================\n\n");
//
//   // Print stereo extrinsics
//   if (calib.sensor_camera_extrinsic_blocks_.size() == 2) {
//     const auto T_SC0_gnd = get_T_SC0_gnd();
//     const auto T_SC0_est =
//         calib.sensor_camera_extrinsic_blocks_.at(0).get()->estimate().T();
//     const auto T_SC1_gnd = get_T_SC1_gnd();
//     const auto T_SC1_est =
//         calib.sensor_camera_extrinsic_blocks_.at(1).get()->estimate().T();
//     print_stereo_extrinsics_summary(T_SC0_gnd, T_SC1_gnd, T_SC0_est, T_SC1_est);
//   }
//   printf("========================================\n\n");
//
//   // Print time delay
//   if (calib.time_delay_block_) {
//     printf("IMU-Camera Time Delay Results:\n");
//     printf("----------------------------------------\n");
//     printf("time delay [s]: %f\n", calib.time_delay_block_.get()->estimate());
//     printf("\n");
//   }
//   printf("========================================\n\n");
//
//   // Optimization blocks
//   printf("Optimization Summary:\n");
//   printf("----------------------------------------\n");
//   printf("pose residual blocks: %zu\n", calib.T_WS_blocks_.size());
//   printf("cam0 residual blocks: %zu\n", calib.cam_errors_.at(0).size());
//   printf("cam1 residual blocks: %zu\n", calib.cam_errors_.at(1).size());
//   printf("imu residual blocks: %zu\n", calib.imu_errors_.size());
//
//   // Final reprojection error
//   std::vector<double> r;
//   std::vector<grid_outlier_t> outliers;
//   const double cam0_reproj_error =
//       calib.calcReprojError(timeline, 0, r, outliers);
//   const double cam1_reproj_error =
//       calib.calcReprojError(timeline, 1, r, outliers);
//   printf("RMSE Reprojection Error [cam0]: %f\n", cam0_reproj_error);
//   printf("RMSE Reprojection Error [cam1]: %f\n", cam1_reproj_error);
//   printf("\n");
// }

#endif // EUROC_TEST_SETUP
