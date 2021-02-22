#include "Calibrator.hpp"

#include <limits>
#include <iomanip>
#include <thread>
#include <future>

namespace yac {

real_t covar_recover(const long i, const long l,
                     const sp_mat_t &L, const vecx_t &diag,
                     mat_hash_t &hash) {
  // Check if covar at (i, l) has already been computed
  if (hash.count(i) == 1 && hash[i].count(l) == 1) {
    return hash[i][l];
  } else if (hash.count(l) == 1 && hash[l].count(i) == 1) {
    return hash[l][i];
  }

  // Sum over sparse entries of row i in U
  Eigen::SparseVector<double> L_row = L.col(i);
  const auto sum_row = [&](const long i) {
    real_t sum = 0;

    for (Eigen::SparseVector<double>::InnerIterator it(L_row); it; ++it){
      long j = it.index();
      real_t U_ij = it.value();

      if (j != i) {
        real_t covar_lj = 0;
        if (j > l) {
          covar_lj = covar_recover(l, j, L, diag, hash);
        } else {
          covar_lj = covar_recover(j, l, L, diag, hash);
        }
        sum += U_ij * covar_lj;
      }
    }

    return sum;
  };

  // Compute covar at (i, l)
  real_t covar_il;
  if (i == l) {
    // Diagonals
    covar_il = diag[l] * (diag[l] - sum_row(l));
  } else {
    // Off-diagonals
    covar_il = (-sum_row(i) * diag[i]);
  }
  hash[i][l] = covar_il;

  return covar_il;
}

mat_hash_t covar_recover(const matx_t &H, const mat_indicies_t &indicies) {
  // Decompose H to LL^t
  const Eigen::LLT<matx_t> llt(H);
  if (llt.info() != Eigen::Success) {
    FATAL("Failed to decompose Hessian H!");
  }
  const matx_t L = llt.matrixL();

  // Pre-calculate diagonal inverses
  vecx_t diag(L.rows());
  size_t nb_rows = L.rows();
  for (size_t i = 0; i < nb_rows; i++) {
    diag(i) = 1.0 / L(i, i);
  }

  // Recover values of covariance matrix
  mat_hash_t hash;
  mat_hash_t retval;
  const sp_mat_t L_sparse = L.sparseView();
  for (auto &index : indicies) {
    const long i = index.first;
    const long j = index.second;
    const real_t covar_ij = covar_recover(i, j, L_sparse, diag, hash);
    retval[i][j] = covar_ij;
  }

  return retval;
}

SpeedAndBias sb_init(const vec3_t &v, const vec3_t &bg, const vec3_t &ba) {
  SpeedAndBias sb;
  sb << v, bg, ba;
  return sb;
}

int config_nb_cameras(const std::string &config_file) {
  config_t config{config_file};

  const int max_cameras = 100;
  for (int i = 0; i < max_cameras; i++) {
    const std::string key = "cam" + std::to_string(i);
    if (yaml_has_key(config, key) != 0) {
      return i;
    }
  }

  return max_cameras;
}

int config_nb_imus(const std::string &config_file) {
  config_t config{config_file};

  const int max_imus = 100;
  for (int i = 0; i < max_imus; i++) {
    const std::string key = "imu" + std::to_string(i);
    if (yaml_has_key(config, key) != 0) {
      return i;
    }
  }

  return max_imus;
}

void load_calib_target_params(const config_t &config,
                              calib_target_t &params,
                              bool verbose) {
  parse(config, "calib_target.target_type", params.target_type);
  parse(config, "calib_target.tag_rows", params.tag_rows);
  parse(config, "calib_target.tag_cols", params.tag_cols);
  parse(config, "calib_target.tag_size", params.tag_size);
  parse(config, "calib_target.tag_spacing", params.tag_spacing);

  if (verbose) {
    LOG_INFO("Calibration Target Parameters");
    LOG_INFO("----------------------------------------");
    LOG_INFO("target type: %s", params.target_type.c_str());
    LOG_INFO("tag rows: %d", params.tag_rows);
    LOG_INFO("tag cols: %d", params.tag_cols);
    LOG_INFO("tag size: %f", params.tag_size);
    LOG_INFO("tag spacing: %f", params.tag_spacing);
    LOG_INFO("");
  }
}

void load_camera_params(const config_t &config,
                        const int index,
                        CameraParameters &params,
                        bool verbose) {
  // Load camera calibration
  const std::string cam_str = "cam" + std::to_string(index);
  // -- Parse resolution, camera and distortion model
  parse(config, cam_str + ".resolution", params.resolution);
  parse(config, cam_str + ".camera_model", params.camera_model);
  parse(config, cam_str + ".distortion_model", params.distortion_model);
  // -- Parse intrinsics
  if (yaml_has_key(config, cam_str + ".intrinsics") == 0) {
    parse(config, cam_str + ".intrinsics", params.intrinsics);
  } else {
    // Estimate intrinsics from resolution and field of view
    double hfov = 0.0;
    double vfov = 0.0;
    parse(config, cam_str + ".lens_hfov", hfov);
    parse(config, cam_str + ".lens_vfov", vfov);

    const double image_width = params.resolution(0);
    const double image_height = params.resolution(1);
    const double fx = pinhole_focal(image_width, hfov);
    const double fy = pinhole_focal(image_height, vfov);
    const double cx = params.resolution(0) / 2.0;
    const double cy = params.resolution(1) / 2.0;

    params.intrinsics.resize(4);
    params.intrinsics << fx, fy, cx, cy;
  }
  // -- Parse distortion
  params.distortion.resize(4);
  params.distortion << zeros(4, 1);
  if (yaml_has_key(config, cam_str + ".distortion") == 0) {
    parse(config, cam_str + ".distortion", params.distortion);
  } else if (params.distortion_model == "radtan") {
    params.distortion << 0.0, 0.0, 0.0, 0.0;
  } else {
    FATAL("Not implemented. Supported distortions: [radtan]");
  }

  if (verbose) {
    LOG_INFO("Camera[%d] Parameters", index);
    LOG_INFO("----------------------------------------");
    LOG_INFO("intrinsics: [%f, %f, %f, %f]",
            params.intrinsics(0),
            params.intrinsics(1),
            params.intrinsics(2),
            params.intrinsics(3));
    LOG_INFO("distortion: [%f, %f, %f, %f]",
            params.distortion(0),
            params.distortion(1),
            params.distortion(2),
            params.distortion(3));
    LOG_INFO("");
  }
}

void load_imu_params(const config_t &config,
                     const int index,
                     ImuParameters &imu_params,
                     bool verbose) {
  const std::string imu_key = "imu" + std::to_string(index);
  parse(config, imu_key + ".a_max", imu_params.a_max);
  parse(config, imu_key + ".g_max", imu_params.g_max);
  parse(config, imu_key + ".sigma_g_c", imu_params.sigma_g_c);
  parse(config, imu_key + ".sigma_a_c", imu_params.sigma_a_c);
  parse(config, imu_key + ".sigma_bg", imu_params.sigma_bg);
  parse(config, imu_key + ".sigma_ba", imu_params.sigma_ba);
  parse(config, imu_key + ".sigma_gw_c", imu_params.sigma_gw_c);
  parse(config, imu_key + ".sigma_aw_c", imu_params.sigma_aw_c);
  parse(config, imu_key + ".tau", imu_params.tau);
  parse(config, imu_key + ".g", imu_params.g);
  parse(config, imu_key + ".a0", imu_params.a0);
  parse(config, imu_key + ".update_rate", imu_params.rate);
  imu_params.T_BS.setIdentity();  // Transformation from body to imu

  if (verbose) {
    LOG_INFO("IMU Parameters");
    LOG_INFO("----------------------------------------");
    LOG_INFO("a_max: %f", imu_params.a_max);
    LOG_INFO("g_max: %f", imu_params.g_max);
    LOG_INFO("sigma_g_c: %f", imu_params.sigma_g_c);
    LOG_INFO("sigma_a_c: %f", imu_params.sigma_a_c);
    LOG_INFO("sigma_bg: %f", imu_params.sigma_bg);
    LOG_INFO("sigma_ba: %f", imu_params.sigma_ba);
    LOG_INFO("sigma_gw_c: %f", imu_params.sigma_gw_c);
    LOG_INFO("sigma_aw_c: %f", imu_params.sigma_aw_c);
    LOG_INFO("tau: %f", imu_params.tau);
    LOG_INFO("g: %f", imu_params.g);
    LOG_INFO("a0: [%f, %f, %f]", imu_params.a0(0),
                                  imu_params.a0(1),
                                  imu_params.a0(2));
    LOG_INFO("update_rate: %d", imu_params.rate);
    LOG_INFO("");
  }
}

void load_extrinsic_params(const config_t &config,
                           const int imu_idx,
                           const int cam_idx,
                           mat4_t &T_SC,
                           bool verbose) {
  const std::string imu_key = "imu" + std::to_string(imu_idx);
  const std::string cam_key = "cam" + std::to_string(cam_idx);
  const std::string key = "T_" + imu_key + "_" + cam_key;
  parse(config, key, T_SC);

  if (verbose) {
    LOG_INFO("Sensor-Camera Extrinsics: T_SC%d", cam_idx);
    LOG_INFO("----------------------------------------");
    LOG_INFO("[%f, %f, %f, %f,", T_SC(0, 0), T_SC(0, 1), T_SC(0, 2), T_SC(0, 3));
    LOG_INFO(" %f, %f, %f, %f,", T_SC(1, 0), T_SC(1, 1), T_SC(1, 2), T_SC(1, 3));
    LOG_INFO(" %f, %f, %f, %f,", T_SC(2, 0), T_SC(2, 1), T_SC(2, 2), T_SC(2, 3));
    LOG_INFO(" %f, %f, %f, %f]", T_SC(3, 0), T_SC(3, 1), T_SC(3, 2), T_SC(3, 3));
    LOG_INFO("");
  }
}

void load_optimization_settings(const config_t &config,
                                OptSettings &opt) {
  parse(config, "settings.verbose", opt.verbose);
  parse(config, "settings.save_estimates", opt.save_estimates);
  parse(config, "settings.save_costs", opt.save_costs);
  parse(config, "settings.window_size", opt.window_size);
  parse(config, "settings.grid_limit", opt.grid_limit);
  parse(config, "settings.nb_iters", opt.nb_iters);
  parse(config, "settings.nb_threads", opt.nb_threads);
  parse(config, "settings.sigma_vision", opt.sigma_vision);
  parse(config, "settings.fix_intrinsics", opt.fix_intrinsics);

  LOG_INFO("Optimization Settings");
  LOG_INFO("----------------------------------------");
  LOG_INFO("verbose: %s", (opt.verbose) ? "true" : "false");
  LOG_INFO("save_estimates: %s", (opt.save_estimates) ? "true" : "false");
  LOG_INFO("save_costs: %s", (opt.save_costs) ? "true" : "false");
  LOG_INFO("");
  LOG_INFO("window_size: %ld", opt.window_size);
  LOG_INFO("grid_limit: %d", opt.grid_limit);
  LOG_INFO("nb_iters: %ld", opt.nb_iters);
  LOG_INFO("nb_threads: %ld", opt.nb_threads);
  LOG_INFO("");
  LOG_INFO("sigma_vision: %f", opt.sigma_vision);
  LOG_INFO("fix_intrinsics: %d", opt.fix_intrinsics);

  LOG_INFO("");
}

void trim_imu_data(ImuMeasurementDeque &imu_data, const timestamp_t t1) {
  // Makesure the trime timestamp is after the first imu measurement
  // if (t1 < imu_data.front().timeStamp.toNSec()) {
  //   return;
  // }

  // Trim IMU measurements
  ImuMeasurementDeque prev_meas;
  ImuMeasurementDeque imu_data_cropped;
  for (const auto &meas : imu_data) {
    if (meas.timeStamp.toNSec() > t1) {
      if (imu_data_cropped.size() == 0) {
        imu_data_cropped.push_back(prev_meas.back());
      }
      imu_data_cropped.push_back(meas);

    } else {
      prev_meas.push_back(meas);
    }
  }

  // Set estimates
  if (imu_data_cropped.size()) {
    imu_data = imu_data_cropped;
  }
}

} // namespace yac
