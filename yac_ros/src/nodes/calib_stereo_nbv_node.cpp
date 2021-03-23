#include <signal.h>
#include <thread>
#include <termios.h>

#include "yac.hpp"
#include "ros.hpp"
#include "ros_calib.hpp"

int main(int argc, char *argv[]) {
  // Load config
  yac::ros_config_t config(argc, argv);
  yac::calib_data_t calib_data{config.calib_file};

  // Stereo NBV
  const auto proj_model = calib_data.cam_params[0].proj_model;
  const auto dist_model = calib_data.cam_params[0].dist_model;
  if (proj_model == "pinhole" && dist_model == "radtan4") {
    yac::calib_stereo_nbv_t<yac::pinhole_radtan4_t> calib{config, calib_data};
  } else if (proj_model == "pinhole" && dist_model == "equi4") {
    yac::calib_stereo_nbv_t<yac::pinhole_equi4_t> calib{config, calib_data};
  } else {
    FATAL("Unsupported projection-distorion type [%s-%s]!",
          proj_model.c_str(), dist_model.c_str());
  }

  return 0;
}
