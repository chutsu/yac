#include "munit.hpp"
#include "euroc.hpp"
#include "calib_mono.hpp"
#include "calib_nbv.hpp"

namespace yac {

struct test_data_t {
	const std::string data_path = "/data/euroc/calib/cam_april";
	const std::string grids0_path = data_path + "/grid0/cam0";
	const std::string grids1_path = data_path + "/grid0/cam1";
  euroc_calib_t data{data_path};
  aprilgrids_t grids0;
  aprilgrids_t grids1;
};

test_data_t setup_test_data() {
	test_data_t test_data;
	LOG_INFO("Loading EuRoC data [%s]", test_data.data_path.c_str());

	// preprocess aprilgrids
	LOG_INFO("Preprocessing AprilGrid data ...");
	aprilgrid_detector_t detector{6, 6, 0.088, 0.3};
	const auto grids0_path = test_data.grids0_path;
	const auto grids1_path = test_data.grids1_path;
	if (system(("mkdir -p " + grids0_path).c_str()) != 0) {
		FATAL("Failed to create dir [%s]", grids0_path.c_str());
	}
	if (system(("mkdir -p " + grids1_path).c_str()) != 0) {
		FATAL("Failed to create dir [%s]", grids1_path.c_str());
	}

	auto timeline = test_data.data.timeline();
	size_t i = 0;
	for (const auto &ts : timeline.timestamps) {
		if (i++ % 100 == 0) {
			printf(".");
			fflush(stdout);
		}

		const auto kv = timeline.data.equal_range(ts);
		for (auto it = kv.first; it != kv.second; it++) {
			const auto event = it->second;
			if (event.type == CAMERA_EVENT) {
				const auto ts = event.ts;
				const int cam_idx = event.camera_index;
				const cv::Mat image = cv::imread(event.image_path);
				const auto grid_fname = std::to_string(ts) + ".csv";

        std::string grid_file;
				if (cam_idx == 0) {
          grid_file = grids0_path + "/" + grid_fname;
        } else if (cam_idx == 1) {
          grid_file = grids1_path + "/" + grid_fname;
        }

        aprilgrid_t grid;
        if (file_exists(grid_file) == false) {
					grid = detector.detect(event.ts, image);
					grid.save(grid_file);
        } else {
          grid.load(grid_file);
        }

        switch (cam_idx) {
        case 0: test_data.grids0.push_back(grid); break;
        case 1: test_data.grids1.push_back(grid); break;
        }
			}
		}
	}
	printf("\n");

	return test_data;
}

int test_calib_target_origin() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const vec2_t cam_res{752, 480};
  const double hfov = 90.0;

  const mat4_t origin = calib_target_origin(target, cam_res, hfov);
  print_matrix("origin", origin);

  return 0;
}

int test_calib_init_poses() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const mat4s_t poses = calib_init_poses(target);
  const std::string save_path = "/tmp/calib_poses.csv";
  save_poses(save_path, poses);

  return 0;
}


int test_calib_nbv_poses() {
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const mat4s_t poses = calib_nbv_poses(target);
  const std::string save_path = "/tmp/calib_poses.csv";
  save_poses(save_path, poses);

  return 0;
}

int test_nbv_draw() {
	// Setup Camera
	const id_t id = 0;
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
  const camera_params_t cam_params{id, cam_idx, cam_res,
														  	 	 proj_model, dist_model,
														  	 	 proj_params, dist_params};

  // Setup nbv poses
  const calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};
  const mat4s_t poses = calib_init_poses(target);
  const std::string save_path = "/tmp/calib_poses.csv";
  save_poses(save_path, poses);

  for (const auto &T_FC : poses) {
    cv::Mat image(cam_res[1], cam_res[0], CV_8UC3, cv::Scalar(255, 255, 255));
    nbv_draw<pinhole_radtan4_t>(target, cam_params, T_FC, image);
    cv::imshow("image", image);
    cv::waitKey(0);
  }

  return 0;
}

int test_nbv_test_grid() {
	// Setup Camera
	const int img_w = 752;
	const int img_h = 480;
	const int cam_res[2] = {img_w, img_h};
	const double lens_hfov = 90.0;
	const double lens_vfov = 90.0;
  const double fx = pinhole_focal(img_w, lens_hfov);
  const double fy = pinhole_focal(img_h, lens_vfov);
  const double cx = img_w / 2.0;
  const double cy = img_h / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.0001, 0.0001, 0.0001};
	pinhole_radtan4_t camera{cam_res, proj_params, dist_params};

	// NBV test grid
	nbv_test_grid_t test_grid(camera);

	for (size_t i = 0; i < test_grid.nb_points; i++) {
		const vec3_t p = test_grid.object_points[i];
		const vec2_t z = test_grid.keypoints[i];
		printf("r_FFi: [%f, %f, %f] ", p(0), p(1), p(2));
		printf("z: [%f, %f]\n", z(0), z(1));
	}

  return 0;
}

int test_nbv_find() {
  test_data_t test_data = setup_test_data();

  // Calibration target
	calib_target_t target{"aprilgrid", 6, 6, 0.088, 0.3};

  // Camera
  const id_t id = 0;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const std::string proj_model = "pinhole";
  const std::string dist_model = "radtan4";
  const double fx = pinhole_focal(cam_res[0], 98.0);
  const double fy = pinhole_focal(cam_res[1], 73.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  const camera_params_t cam_params{id, cam_idx, cam_res,
														  	 	 proj_model, dist_model,
														  	 	 proj_params, dist_params};

	// Optimize
  aprilgrids_t grids0;
  for (const auto &grid : test_data.grids0) {
    if (grid.detected) {
      grids0.push_back(grid);
    }
    if (grids0.size() == 5) {
      break;
    }
  }

	calib_mono_data_t data{grids0, cam_params};
  calib_mono_solve<pinhole_radtan4_t>(data);

  mat4_t T_FC;
  nbv_find<pinhole_radtan4_t>(target, data, T_FC);
  print_matrix("T_FC", T_FC);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_target_origin);
  MU_ADD_TEST(test_calib_init_poses);
  MU_ADD_TEST(test_calib_nbv_poses);
  MU_ADD_TEST(test_nbv_draw);
  MU_ADD_TEST(test_nbv_test_grid);
  MU_ADD_TEST(test_nbv_find);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
