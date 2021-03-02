#ifndef YAC_CALIBRATOR_HPP
#define YAC_CALIBRATOR_HPP

#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "core.hpp"
#include "calib_data.hpp"
#include "common/Parameters.hpp"
#include "common/Measurements.hpp"
#include "common/Transformation.hpp"

namespace yac {

struct CameraParameters {
  vec2_t resolution{0.0, 0.0};
  std::string camera_model;
  std::string distortion_model;
  vecx_t intrinsics;
  vecx_t distortion;

  vecx_t params() const {
    vecx_t retval(8);
    retval.head(4) = intrinsics;
    retval.tail(4) = distortion;
    return retval;
  }
};

struct OptSettings {
	bool verbose = false;
  bool save_estimates = false;
  bool save_costs = false;

  size_t window_size = 3;
  int grid_limit = 20;
	size_t nb_iters = 10;
	size_t nb_threads = 4;

  double sigma_vision = 1.0;
  bool fix_intrinsics = false;
};

typedef Eigen::SparseMatrix<real_t> sp_mat_t;
typedef Eigen::SparseVector<real_t> sp_vec_t;
typedef std::unordered_map<long, std::unordered_map<long, real_t>> mat_hash_t;
typedef std::vector<std::pair<long int, long int>> mat_indicies_t;

real_t covar_recover(const long i, const long l,
                     const sp_mat_t &L, const vecx_t &diag,
                     mat_hash_t &hash);

mat_hash_t covar_recover(const matx_t &H, const mat_indicies_t &indicies);

/**
 * Setup speed and bias vector;
 */
SpeedAndBias sb_init(const vec3_t &v, const vec3_t &bg, const vec3_t &ba);

/**
 * Obtain number of cameras just by looking at the config file.
 */
int config_nb_cameras(const std::string &config_file);

/**
 * Obtain number of imus just by looking at the config file.
 */
int config_nb_imus(const std::string &config_file);

/**
 * Load calibration target parameters
 *
 * @param[in] config Config
 * @param[out] param Calibration target parameters
 */
void load_calib_target_params(const config_t &config,
                              calib_target_t &params,
                              bool verbose=true);

/**
 * Load camera parameters
 *
 * @param[in] config Config
 * @param[in] index Camera index
 * @param[out] param Camera parameters
 */
void load_camera_params(const config_t &config,
                        const int index,
                        CameraParameters &params,
                        bool verbose=true);

/**
 * Load Imu parameters
 *
 * @param[in] config Config
 * @param[in] index Camera index
 * @param[out] param Camera parameters
 */
void load_imu_params(const config_t &config,
                     const int index,
                     ImuParameters &params,
                     bool verbose=true);

/**
 * Load sensor-camera extrinsic parameters
 *
 * @param[in] config Config
 * @param[in] imu_idx Imu index
 * @param[in] cam_idx Camera index
 * @param[out] extrinsic Extrinsic
 */
void load_extrinsic_params(const config_t &config,
                           const int imu_idx,
                           const int cam_idx,
                           mat4_t &T_SC,
                           bool verbose=false);

/**
 * Load optimization settings
 *
 * @param[in] config Config
 */
void load_optimization_settings(const config_t &config,
                                OptSettings &opt);

} // namespace autocal
#endif /* YAC_CALIBRATOR_HPP */
