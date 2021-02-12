#include "calib_mocap.hpp"

namespace yac {

int calib_mocap_solve(const std::string &config_file) {
  calib_mocap_data_t data{config_file};

  std::string proj_model = data.cam0.proj_model;
  std::string dist_model = data.cam0.dist_model;
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    return calib_mocap_solve<pinhole_radtan4_t>(data);
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    return calib_mocap_solve<pinhole_equi4_t>(data);
  } else {
    FATAL("Unsupported [%s-%s]!", proj_model.c_str(), dist_model.c_str());
  }

  return -1;
}

} //  namespace yac
