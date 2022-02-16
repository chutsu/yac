#include "calib_camera.hpp"

std::map<int, yac::aprilgrids_t> load_test_dataset() {
  const std::string cam0_dir = "/tmp/yac_data/cam0";
  const std::string cam1_dir = "/tmp/yac_data/cam1";
  std::vector<std::string> cam0_files;
  std::vector<std::string> cam1_files;
  yac::list_files(cam0_dir, cam0_files);
  yac::list_files(cam1_dir, cam1_files);

  std::map<int, yac::aprilgrids_t> test_data;
  for (auto grid_path : cam0_files) {
    test_data[0].emplace_back(cam0_dir + "/" + grid_path);
  }
  for (auto grid_path : cam1_files) {
    test_data[1].emplace_back(cam1_dir + "/" + grid_path);
  }

  return test_data;
}

std::map<int, yac::aprilgrids_t> load_validation_dataset() {
  // Load test data for validation
  const yac::calib_target_t calib_target;
  const std::string grids_path = "/data/euroc/imu_april/mav0/grid0";
  const std::string data_path = "/data/euroc/imu_april";
  std::map<int, std::string> cam_paths;
  cam_paths[0] = data_path + "/mav0/cam0/data";
  cam_paths[1] = data_path + "/mav0/cam1/data";
  return yac::calib_data_preprocess(calib_target, cam_paths, grids_path);
}

int main() {
  // Setup
  const auto test_data = load_test_dataset();
  const auto valid_data = load_validation_dataset();

  // clang-format off
  const yac::calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  yac::vec4_t cam0_proj_params{460.28579839, 459.06697967, 368.29981211, 244.66090975};
  yac::vec4_t cam0_dist_params{-0.27615787, 0.06777914, 0.00075657, -0.00029946};
  yac::vec4_t cam1_proj_params{458.931838, 457.621130, 378.783646, 251.341323};
  yac::vec4_t cam1_dist_params{-0.272847, 0.064882, 0.000506, -0.000210};

  const yac::vec3_t r_C1C0{-0.109948, 0.000514, -0.000241};
  const yac::quat_t q_C1C0{0.999970, -0.007147, 0.002650, -0.001263};
  const yac::mat4_t T_C1C0 = yac::tf(q_C1C0, r_C1C0);
  const yac::mat4_t T_C0C1 = T_C1C0.inverse();

  yac::calib_camera_t calib{target};
  calib.add_camera_data(test_data);
  calib.add_camera(0, cam_res, proj_model, dist_model, cam0_proj_params, cam0_dist_params);
  calib.add_camera(1, cam_res, proj_model, dist_model, cam1_proj_params, cam1_dist_params);
  calib.add_camera_extrinsics(0);
  calib.add_camera_extrinsics(1, T_C0C1);

  calib.initialized = true;
  calib.enable_nbv = true;
  calib.enable_nbv_filter = false;
  calib.enable_outlier_filter = true;
  calib.solve();
  calib.save_results("/tmp/calib-cameras.yaml");
  calib.validate(valid_data);

  // FILE *views_csv = fopen("/tmp/yac_views.csv", "w");
  // for (const auto view : calib.calib_views) {
  //   fprintf(views_csv, "%ld\n", view->ts);
  // }
  // fclose(views_csv);
  // clang-format on

  return 0;
}
