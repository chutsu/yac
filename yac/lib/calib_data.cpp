#include "calib_data.hpp"

namespace yac {

// // CALIBRATION CONFIG
// //////////////////////////////////////////////////////////
//
// calib_config_t::calib_config_t(const std::string &config_file)
//     : config{config_file} {}
//
// int calib_config_t::get_num_cams() const {
//   const int max_cameras = 100;
//   for (int i = 0; i < max_cameras; i++) {
//     const std::string key = "cam" + std::to_string(i);
//     if (yaml_has_key(config, key) == 0) {
//       return i;
//     }
//   }
//
//   return 0;
// }
//
// int calib_config_t::get_num_imus() const {
//   const int max_imus = 100;
//   for (int i = 0; i < max_imus; i++) {
//     const std::string key = "imu" + std::to_string(i);
//     if (yaml_has_key(config, key) == 0) {
//       return i;
//     }
//   }
//
//   return 0;
// }
//
// calib_target_t calib_config_t::get_calib_target(const std::string &prefix,
//                                                 bool verbose) const {
//   calib_target_t calib_target;
//
//   const auto parent = (prefix == "") ? "" : prefix + ".";
//   parse(config, parent + "target_type", calib_target.target_type);
//   parse(config, parent + "tag_rows", calib_target.tag_rows);
//   parse(config, parent + "tag_cols", calib_target.tag_cols);
//   parse(config, parent + "tag_size", calib_target.tag_size);
//   parse(config, parent + "tag_spacing", calib_target.tag_spacing);
//
//   if (verbose) {
//     LOG_INFO("Calibration Target Parameters");
//     LOG_INFO("----------------------------------------");
//     LOG_INFO("target type: %s", calib_target.target_type.c_str());
//     LOG_INFO("tag rows: %d", calib_target.tag_rows);
//     LOG_INFO("tag cols: %d", calib_target.tag_cols);
//     LOG_INFO("tag size: %f", calib_target.tag_size);
//     LOG_INFO("tag spacing: %f", calib_target.tag_spacing);
//     LOG_INFO("");
//   }
//
//   return calib_target;
// }
//
// imu_params_t calib_config_t::get_imu_params(const int index,
//                                             bool verbose) const {
//   imu_params_t imu_params;
//   const std::string imu_key = "imu" + std::to_string(index);
//   parse(config, imu_key + ".rate", imu_params.rate);
//   parse(config, imu_key + ".a_max", imu_params.a_max);
//   parse(config, imu_key + ".g_max", imu_params.g_max);
//   parse(config, imu_key + ".sigma_g_c", imu_params.sigma_g_c);
//   parse(config, imu_key + ".sigma_a_c", imu_params.sigma_a_c);
//   parse(config, imu_key + ".sigma_gw_c", imu_params.sigma_gw_c);
//   parse(config, imu_key + ".sigma_aw_c", imu_params.sigma_aw_c);
//   parse(config, imu_key + ".g", imu_params.g);
//
//   if (verbose) {
//     LOG_INFO("IMU Parameters");
//     LOG_INFO("----------------------------------------");
//     LOG_INFO("rate: %f", imu_params.rate);
//     LOG_INFO("a_max: %f", imu_params.a_max);
//     LOG_INFO("g_max: %f", imu_params.g_max);
//     LOG_INFO("sigma_g_c: %f", imu_params.sigma_g_c);
//     LOG_INFO("sigma_a_c: %f", imu_params.sigma_a_c);
//     LOG_INFO("sigma_gw_c: %f", imu_params.sigma_gw_c);
//     LOG_INFO("sigma_aw_c: %f", imu_params.sigma_aw_c);
//     LOG_INFO("sigma_ba: %f", imu_params.sigma_ba);
//     LOG_INFO("sigma_bg: %f", imu_params.sigma_bg);
//     LOG_INFO("g: %f", imu_params.g);
//     LOG_INFO("");
//   }
//
//   return imu_params;
// }
//
// camera_params_t calib_config_t::get_cam_params(const int cam_idx,
//                                                bool verbose) const {
//   // Load camera calibration
//   const std::string cam_str = "cam" + std::to_string(cam_idx);
//   std::vector<int> cam_res;
//   std::string proj_model;
//   std::string dist_model;
//   vec4_t proj_params = zeros(4, 1);
//   vec4_t dist_params = zeros(4, 1);
//   // -- Parse resolution, camera and distortion model
//   parse(config, cam_str + ".resolution", cam_res);
//   parse(config, cam_str + ".proj_model", proj_model);
//   parse(config, cam_str + ".dist_model", dist_model);
//   // -- Parse intrinsics
//   if (yaml_has_key(config, cam_str + ".proj_params")) {
//     parse(config, cam_str + ".proj_params", proj_params);
//   }
//   // -- Parse distortion
//   if (yaml_has_key(config, cam_str + ".dist_params")) {
//     parse(config, cam_str + ".dist_params", dist_params);
//   }
//
//   if (verbose) {
//     LOG_INFO("Camera[%d] Parameters", cam_idx);
//     LOG_INFO("----------------------------------------");
//     LOG_INFO("proj_params: [%f, %f, %f, %f]",
//              proj_params(0),
//              proj_params(1),
//              proj_params(2),
//              proj_params(3));
//     LOG_INFO("dist_params: [%f, %f, %f, %f]",
//              dist_params(0),
//              dist_params(1),
//              dist_params(2),
//              dist_params(3));
//     LOG_INFO("");
//   }
//
//   return camera_params_t{cam_idx,
//                          cam_res.data(),
//                          proj_model,
//                          dist_model,
//                          proj_params,
//                          dist_params};
// }
//
// extrinsics_t calib_config_t::get_cam_exts(const int cam_idx,
//                                           bool verbose) const {
//   const std::string cam_key = "cam" + std::to_string(cam_idx);
//   const std::string key = "T_body0_" + cam_key;
//   mat4_t exts = I(4);
//   if (yaml_has_key(config, key)) {
//     parse(config, key, exts);
//   }
//
//   if (verbose) {
//     LOG_INFO("Camera Extrinsics: T_BC%d", cam_idx);
//     LOG_INFO("----------------------------------------");
//     LOG_INFO("[%f, %f, %f, %f,",
//              exts(0, 0),
//              exts(0, 1),
//              exts(0, 2),
//              exts(0, 3));
//     LOG_INFO(" %f, %f, %f, %f,",
//              exts(1, 0),
//              exts(1, 1),
//              exts(1, 2),
//              exts(1, 3));
//     LOG_INFO(" %f, %f, %f, %f,",
//              exts(2, 0),
//              exts(2, 1),
//              exts(2, 2),
//              exts(2, 3));
//     LOG_INFO(" %f, %f, %f, %f]",
//              exts(3, 0),
//              exts(3, 1),
//              exts(3, 2),
//              exts(3, 3));
//     LOG_INFO("");
//   }
//
//   return extrinsics_t{exts};
// }
//
// // CALIBRATION DATA
// ////////////////////////////////////////////////////////////
//
// calib_data_t::calib_data_t() {
//   // Problem
//   prob_options.local_parameterization_ownership =
//   ceres::DO_NOT_TAKE_OWNERSHIP;
//   // prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
//   prob_options.enable_fast_removal = true;
//   problem = new ceres::Problem(prob_options);
// }
//
// calib_data_t::calib_data_t(const std::string &config_file) {
//   // Problem
//   prob_options.local_parameterization_ownership =
//   ceres::DO_NOT_TAKE_OWNERSHIP; prob_options.cost_function_ownership =
//   ceres::DO_NOT_TAKE_OWNERSHIP; prob_options.enable_fast_removal = true;
//   problem = new ceres::Problem(prob_options);
//
//   // Load config
//   calib_config_t config{config_file};
//
//   // Calibration target
//   calib_target = config.get_calib_target("calib_target", true);
//
//   // Load Imu
//   nb_imus = config.get_num_imus();
//   if (nb_imus > 1) {
//     FATAL("YAC currently does not support more than 1 IMU!");
//   } else if (nb_imus == 1) {
//     imu_params = config.get_imu_params(0, true);
//   }
//
//   // Load cameras
//   nb_cams = config.get_num_cams();
//   for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//     cam_params[cam_idx] = config.get_cam_params(cam_idx, true);
//     cam_exts[cam_idx] = config.get_cam_exts(cam_idx, true);
//     vision_errors[cam_idx] = std::vector<double>();
//   }
// }
//
// calib_data_t::~calib_data_t() {
//   if (problem) {
//     delete problem;
//     problem = nullptr;
//   }
//   if (loss) {
//     delete loss;
//     loss = nullptr;
//   }
// }
//
// void calib_data_t::add_calib_target(const calib_target_t &calib_target_) {
//   calib_target = calib_target_;
// }
//
// void calib_data_t::add_imu(const imu_params_t &params) { imu_params = params;
// }
//
// void calib_data_t::add_camera(const camera_params_t &params) {
//   // Camera geometry
//   const auto cam_index = params.cam_index;
//   const auto proj_model = params.proj_model;
//   const auto dist_model = params.dist_model;
//   if (proj_model == "pinhole" && dist_model == "radtan4") {
//     cam_geoms[cam_index] = &pinhole_radtan4;
//   } else if (proj_model == "pinhole" && dist_model == "equi4") {
//     cam_geoms[cam_index] = &pinhole_equi4;
//   } else {
//     FATAL("[%s-%s] unsupported!", proj_model.c_str(), dist_model.c_str());
//   }
//
//   cam_params[cam_index] = params;
//   nb_cams += 1;
//   vision_errors[cam_index] = std::vector<double>();
//
//   problem->AddParameterBlock(cam_params[cam_index].param.data(), 8);
// }
//
// void calib_data_t::add_camera_extrinsics(const int cam_idx, const mat4_t
// &ext) {
//   cam_exts[cam_idx] = extrinsics_t{ext};
//   problem->AddParameterBlock(cam_exts[cam_idx].param.data(), 7);
//   problem->SetParameterization(cam_exts[cam_idx].param.data(), &pose_plus);
//   if (cam_idx == 0) {
//     problem->SetParameterBlockConstant(cam_exts[cam_idx].param.data());
//   }
// }
//
// void calib_data_t::add_grids(const int cam_idx, const aprilgrids_t &grids) {
//   for (auto &grid : grids) {
//     const timestamp_t ts = grid.timestamp;
//     timestamps.insert(ts);
//     cam_grids.insert({ts, {cam_idx, grid}});
//   }
// }
//
// pose_t &calib_data_t::add_pose(const timestamp_t &ts, const mat4_t &T) {
//   poses[ts] = pose_t{ts, T};
//   problem->AddParameterBlock(poses[ts].param.data(), 7);
//   problem->SetParameterization(poses[ts].param.data(), &pose_plus);
//   return poses[ts];
// }
//
// // void calib_data_t::check_data() {
// //   bool ok = true;
// //
// //   auto nb_grids = cam_grids[0].size();
// //   for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
// //     if (nb_grids != cam_grids[cam_idx].size()) {
// //       FATAL("grids[%d].size() != nb_grids!", cam_idx);
// //       ok = false;
// //     }
// //   }
// //
// //   std::vector<timestamp_t> timestamps;
// //   for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
// //     for (auto &grid : cam_grids[cam_idx]) {
// //       timestamps.push_back(grid.timestamp);
// //     }
// //   }
// //
// //   for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
// //     for (size_t i = 0; i < cam_grids[cam_idx].size(); i++) {
// //       auto &grid = cam_grids[cam_idx][i];
// //       if (grid.timestamp != timestamps[i]) {
// //         LOG_ERROR("grid.timestamp != timestamps[i]!");
// //         ok = false;
// //         break;
// //       }
// //
// //       if (grid.tag_rows != target.tag_rows) {
// //         LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
// //         LOG_ERROR("grid.tag_rows != target.tag_rows!");
// //         ok = false;
// //         break;
// //       } else if (grid.tag_cols != target.tag_cols) {
// //         LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
// //         LOG_ERROR("grid.tag_cols != target.tag_cols!");
// //         ok = false;
// //         break;
// //       } else if (grid.tag_size != target.tag_size) {
// //         LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
// //         LOG_ERROR("grid.tag_size != target.tag_size!");
// //         ok = false;
// //         break;
// //       } else if (grid.tag_spacing != target.tag_spacing) {
// //         LOG_ERROR("grid.timestamp: %ld", grid.timestamp);
// //         LOG_ERROR("grid.tag_spacing != target.tag_spacing!");
// //         ok = false;
// //         break;
// //       }
// //     }
// //   }
// //
// //   if (ok == false) {
// //     FATAL("Bad data!");
// //   }
// // }
//
// int calib_data_t::get_number_of_grids(const timestamp_t &ts) {
//   const auto range = cam_grids.equal_range(ts);
//   return std::distance(range.first, range.second);
// }
//
// std::vector<std::pair<int, aprilgrid_t>>
// calib_data_t::get_grids(const timestamp_t &ts) {
//   std::vector<std::pair<int, aprilgrid_t>> grids;
//
//   const auto range = cam_grids.equal_range(ts);
//   for (auto it = range.first; it != range.second; it++) {
//     grids.push_back(it->second);
//   }
//
//   return grids;
// }
//
// void calib_data_t::show_results() {
//   // Show results
//   const auto cam0_errs = vision_errors.at(0);
//   const auto cam1_errs = vision_errors.at(1);
//   const auto &cam0 = cam_params.at(0);
//   const auto &cam1 = cam_params.at(1);
//   const mat4_t T_BC0 = cam_exts.at(0).tf();
//   const mat4_t T_BC1 = cam_exts.at(1).tf();
//   const mat4_t T_C1C0 = T_BC1.inverse() * T_BC0;
//
//   printf("Optimization results:\n");
//   printf("---------------------\n");
//   for (int cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
//     const auto errs = vision_errors.at(cam_idx);
//     printf("cam%d reproj_error [px]", cam_idx);
//     printf("[");
//     printf("rmse: %f, ", rmse(errs));
//     printf("mean: %f, ", mean(errs));
//     printf("median: %f", median(errs));
//     printf("]\n");
//   }
//   printf("\n");
//
//   print_vector("cam0.proj_params", cam0.proj_params());
//   print_vector("cam0.dist_params", cam0.dist_params());
//   print_vector("cam1.proj_params", cam1.proj_params());
//   print_vector("cam1.dist_params", cam1.dist_params());
//   printf("\n");
//   print_matrix("T_C1C0", T_C1C0);
//   printf("\n");
// }
//
// int calib_data_t::preprocess_camera_data(const std::string &image_dir,
//                                          const std::string &output_dir,
//                                          const bool show_progress) {
//   int retval = 0;
//
//   // Get camera image paths
//   std::vector<std::string> image_paths;
//   if (list_files(image_dir, image_paths) != 0) {
//     return -1;
//   }
//
//   // Detect AprilGrid
//   aprilgrid_detector_t detector{calib_target.tag_rows,
//                                 calib_target.tag_cols,
//                                 calib_target.tag_size,
//                                 calib_target.tag_spacing};
//
// #pragma omp parallel for
//   for (size_t i = 0; i < image_paths.size(); i++) {
//     // Print progress
//     if (show_progress && i % 10 == 0) {
//       printf(".");
//       fflush(stdout);
//     }
//
//     // Create output file path
//     auto output_file = parse_fname(image_paths[i]);
//     const timestamp_t ts = std::stoull(output_file);
//     output_file = remove_ext(output_file);
//     output_file += ".csv";
//     const auto save_path = paths_join(output_dir, output_file);
//
//     // Skip if already preprocessed
//     if (file_exists(save_path)) {
//       continue;
//     }
//
//     // Read image
//     const auto image_path = paths_join(image_dir, image_paths[i]);
//     const cv::Mat frame = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
//
//     // -- Save AprilGrid
//     const auto grid = detector.detect(ts, frame);
//     if (grid.save(save_path) != 0) {
//       retval = -1;
//     }
//   }
//
//   // Print newline after print progress has finished
//   if (show_progress) {
//     printf("\n");
//   }
//
//   return retval;
// }
//
// // CALIBRATION FUNCTIONS
// ///////////////////////////////////////////////////////
//
// int load_camera_calib_data(const std::string &data_dir,
//                            aprilgrids_t &aprilgrids,
//                            bool detected_only) {
//   // Check image dir
//   if (dir_exists(data_dir) == false) {
//     LOG_ERROR("Image dir [%s] does not exist!", data_dir.c_str());
//     return -1;
//   }
//
//   // Get detection data
//   std::vector<std::string> data_paths;
//   if (list_dir(data_dir, data_paths) != 0) {
//     LOG_ERROR("Failed to traverse dir [%s]!", data_dir.c_str());
//     return -1;
//   }
//   std::sort(data_paths.begin(), data_paths.end());
//
//   // Load AprilGrid data
//   for (size_t i = 0; i < data_paths.size(); i++) {
//     // Load
//     const auto data_path = paths_join(data_dir, data_paths[i]);
//     aprilgrid_t grid{data_path};
//
//     // Make sure aprilgrid is actually detected
//     if (grid.detected || detected_only == false) {
//       aprilgrids.emplace_back(grid);
//     }
//   }
//
//   return 0;
// }
//
// cv::Mat draw_calib_validation(const cv::Mat &image,
//                               const vec2s_t &measured,
//                               const vec2s_t &projected,
//                               const cv::Scalar &measured_color,
//                               const cv::Scalar &projected_color) {
//   // Make an RGB version of the input image
//   cv::Mat image_rgb = gray2rgb(image);
//
//   // Draw measured points
//   for (const auto &p : measured) {
//     cv::circle(image_rgb,               // Target image
//                cv::Point2f(p(0), p(1)), // Center
//                1,                       // Radius
//                measured_color,          // Colour
//                CV_FILLED,               // Thickness
//                8);                      // Line type
//   }
//
//   // Draw projected points
//   for (const auto &p : projected) {
//     cv::circle(image_rgb,               // Target image
//                cv::Point2f(p(0), p(1)), // Center
//                1,                       // Radius
//                projected_color,         // Colour
//                CV_FILLED,               // Thickness
//                8);                      // Line type
//   }
//
//   // Calculate reprojection error and show in image
//   const real_t rmse = reprojection_error(measured, projected);
//   // -- Convert rmse to string
//   std::stringstream stream;
//   stream << std::fixed << std::setprecision(2) << rmse;
//   const std::string rmse_str = stream.str();
//   // -- Draw text
//   const auto text = "RMSE Reprojection Error: " + rmse_str;
//   const auto origin = cv::Point(0, 18);
//   const auto red = cv::Scalar(0, 0, 255);
//   const auto font = cv::FONT_HERSHEY_SIMPLEX;
//   cv::putText(image_rgb, text, origin, font, 0.6, red, 2);
//
//   return image_rgb;
// }

} //  namespace yac
