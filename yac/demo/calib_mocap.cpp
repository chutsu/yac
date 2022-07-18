#include "calib_mocap.hpp"
/**
 * Calibrate the extrinsics T_MC between object mocap frame (M) and camera
 * frame (C). The configuration file `config_file` is expected to be of the
 * form:
 *
 *     settings:
 *       fix_intrinsics: true
 *       fix_mocap_poses: true
 *       fix_fiducial_pose: false
 *       outlier_threshold: 4.0
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.088           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tag_size
 *                                 # Example: tag_size=2m, spacing=0.5m
 *                                 # tag_spacing=0.25
 *
 *     cam0:
 *       resolution: [640, 480]
 *       proj_model: "pinhole"
 *       dist_model: "radtan4"
 *
 * Data directory `data_path` is expected to follow a similiar pattern to EuRoC
 * format:
 *
 *     data_path/
 *       body0/
 *         data.csv
 *
 *       cam0/
 *         data/
 *           1658156653981475879.png
 *           1658156654189449268.png
 *           1658156654397423877.png
 *           ...
 *         data.csv
 *
 *       target0/
 *         data.csv
 *
 * where each `data.csv` file has contains pose data:
 *
 *     timestamp [ns],x,y,z,qx,qy,qz,qw
 *
 */
int main(int argc, char *argv[]) {
  // Check arguments
  if (argc != 3) {
    printf("usage: %s <config_file> <data_path>\n", argv[0]);
    return -1;
  }

  // Calibrate
  const std::string config_file = argv[1];
  const std::string data_path = argv[2];
  yac::calib_mocap_t calib{config_file, data_path};
  calib.solve();
  calib.save_results(data_path + "/calib_mocap-results.yaml");

  return 0;
}
