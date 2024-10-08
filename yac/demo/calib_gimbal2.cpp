#include <Eigen/Core>
#include <ceres/ceres.h>

#include "util/util.hpp"
#include "calib_data.hpp"
#include "calib_params.hpp"

using namespace yac;

/**
 * Form rotation matrix around z axis
 */
template <typename T>
Eigen::Matrix<T, 3, 3> ROTZ(const T theta) {
  Eigen::Matrix<T, 3, 3> C;

  C(0, 0) = cos(theta);
  C(0, 1) = -sin(theta);
  C(0, 2) = T(0.0);

  C(1, 0) = sin(theta);
  C(1, 1) = cos(theta);
  C(1, 2) = T(0.0);

  C(2, 0) = T(0.0);
  C(2, 1) = T(0.0);
  C(2, 2) = T(1.0);

  return C;
}

/**
 * Quaternion to rotation matrix.
 */
template <typename T>
Eigen::Matrix<T, 3, 3> quat2rot(const Eigen::Quaternion<T> &q) {
  const T qw = q.w();
  const T qx = q.x();
  const T qy = q.y();
  const T qz = q.z();

  const T qx2 = qx * qx;
  const T qy2 = qy * qy;
  const T qz2 = qz * qz;
  const T qw2 = qw * qw;

  // Homogeneous form
  Eigen::Matrix<T, 3, 3> C;
  // -- 1st row
  C(0, 0) = qw2 + qx2 - qy2 - qz2;
  C(0, 1) = T(2.0) * (qx * qy - qw * qz);
  C(0, 2) = T(2.0) * (qx * qz + qw * qy);
  // -- 2nd row
  C(1, 0) = T(2.0) * (qx * qy + qw * qz);
  C(1, 1) = qw2 - qx2 + qy2 - qz2;
  C(1, 2) = T(2.0) * (qy * qz - qw * qx);
  // -- 3rd row
  C(2, 0) = T(2.0) * (qx * qz - qw * qy);
  C(2, 1) = T(2.0) * (qy * qz + qw * qx);
  C(2, 2) = qw2 - qx2 - qy2 + qz2;

  return C;
}

/**
 * Initialize transformation vector.
 */
void transform_init(real_t *x) {
  x[0] = 0.0;
  x[1] = 0.0;
  x[2] = 0.0;

  x[3] = 1.0;
  x[4] = 0.0;
  x[5] = 0.0;
  x[6] = 0.0;
}

/**
 * Initialize transformation vector.
 */
void transform_copy(const real_t *x, real_t *x_copy) {
  x_copy[0] = x[0];
  x_copy[1] = x[1];
  x_copy[2] = x[2];

  x_copy[3] = x[3];
  x_copy[4] = x[4];
  x_copy[5] = x[5];
  x_copy[6] = x[6];
}

/**
 * Form transformation matrix.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> transform(const T *params) {
  const Eigen::Quaternion<T> q{params[3], params[4], params[5], params[6]};
  const Eigen::Vector<T, 3> r{params[0], params[1], params[2]};

  Eigen::Matrix<T, 4, 4> transform;
  transform.setIdentity();
  transform.block(0, 0, 3, 3) = quat2rot(q);
  transform.block(0, 3, 3, 1) = r;

  return transform;
}

/**
 * Form transformation matrix.
 */
vecx_t transform_vector(const mat4_t &T) {
  const vec3_t r = tf_trans(T);
  const quat_t q = tf_quat(T);
  vecx_t vec;
  vec.resize(7);
  vec << r.x(), r.y(), r.z(), q.w(), q.x(), q.y(), q.z();
  return vec;
}

/**
 * Perturb pose translation and rotation component.
 */
void pose_perturb(double *pose, const real_t dr, const real_t drot) {
  const auto T = transform(pose);
  const auto T_perturbed = tf_perturb(T, dr, drot);
  const auto pose_vector = transform_vector(T_perturbed);
  for (int i = 0; i < 7; i++) {
    pose[i] = pose_vector[i];
  }
}

/**
 * Copy pose.
 */
void pose_copy(const double *src, double *dst) {
  for (int i = 0; i < 7; i++) {
    dst[i] = src[i];
  }
}

/**
 * Gimbal joint angle to transformation matrix.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> gimbal_joint_transform(const T angle) {
  Eigen::Matrix<T, 4, 4> transform;
  transform.setIdentity();
  transform.block(0, 0, 3, 3) = ROTZ(angle);
  return transform;
}

/**
 * String copy from `src` to `dst`.
 */
size_t string_copy(char *dst, const char *src) {
  dst[0] = '\0';
  memcpy(dst, src, strlen(src));
  dst[strlen(src)] = '\0'; // Null terminate
  return strlen(dst);
}

/**
 * Strip whitespace from string `s`.
 */
char *string_strip(char *s) {
  char *end;

  // Trim leading space
  while (*s == ' ') {
    s++;
  }

  if (*s == 0) { // All spaces?
    return s;
  }

  // Trim trailing space
  end = s + strlen(s) - 1;
  while (end > s && (*end == ' ' || *end == '\n')) {
    end--;
  }

  // Write new null terminator character
  end[1] = '\0';

  return s;
}

/**
 * Strip specific character `c` from string `s`.
 */
char *string_strip_char(char *s, const char c) {
  char *end;

  // Trim leading space
  while (*s == c) {
    s++;
  }

  if (*s == 0) { // All spaces?
    return s;
  }

  // Trim trailing space
  end = s + strlen(s) - 1;
  while (end > s && *end == c) {
    end--;
  }

  // Write new null terminator character
  end[1] = '\0';

  return s;
}

/**
 * Split string `s` by delimiter `d`
 */
char **string_split(char *a_str, const char a_delim, size_t *n) {
  char **result = 0;
  char *tmp = a_str;
  char *last_comma = 0;
  char delim[2];
  delim[0] = a_delim;
  delim[1] = 0;

  /* Count how many elements will be extracted. */
  while (*tmp) {
    if (a_delim == *tmp) {
      (*n)++;
      last_comma = tmp;
    }
    tmp++;
  }

  /* Add space for trailing token. */
  *n += last_comma < (a_str + strlen(a_str) - 1);

  /* Add space for terminating null string so caller
     knows where the list of returned strings ends. */
  (*n)++;
  result = (char **)malloc(sizeof(char *) * *n);

  if (result) {
    size_t idx = 0;
    char *token = strtok(a_str, delim);

    while (token) {
      assert(idx < *n);
      *(result + idx++) = strdup(token);
      token = strtok(0, delim);
    }
    assert(idx == *n - 1);
    *(result + idx) = 0;
  }

  // Return results
  (*n)--;

  return result;
}

/**
 * Skip line
 */
static void parse_skip_line(FILE *fp) {
  assert(fp != NULL);
  const size_t buf_len = 9046;
  char buf[9046] = {0};
  const char *read = fgets(buf, buf_len, fp);
  UNUSED(read);
}

/**
 * Parse integer vector from string line.
 * @returns `0` for success or `-1` for failure
 */
static int parse_vector_line(char *line, const char *type, void *data, int n) {
  assert(line != NULL);
  assert(data != NULL);
  char entry[1024] = {0};
  int index = 0;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == '[' || c == ' ') {
      continue;
    }

    if (c == ',' || c == ']' || c == '\n') {
      if (strcmp(type, "int") == 0) {
        ((int *)data)[index] = strtod(entry, NULL);
      } else if (strcmp(type, "double") == 0) {
        ((double *)data)[index] = strtod(entry, NULL);
      } else {
        FATAL("Invalid type [%s]\n", type);
      }
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  if (index != n) {
    return -1;
  }

  return 0;
}

/**
 * Parse key-value pair from string line
 **/
void parse_key_value(FILE *fp,
                     const char *key,
                     const char *value_type,
                     void *value) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(value_type != NULL);
  assert(value != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = ":";
  char *key_str = strtok(buf, delim);
  char *value_str = strtok(NULL, delim);
  if (key_str == NULL || value_str == NULL) {
    FATAL("Failed to parse [%s]\n", key);
  }
  key_str = string_strip(key_str);
  value_str = string_strip(value_str);

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Typecase value
  if (value_type == NULL) {
    FATAL("Value type not set!\n");
  }

  // Parse value
  if (strcmp(value_type, "int") == 0) {
    *(int *)value = atoi(value_str);
  } else if (strcmp(value_type, "double") == 0) {
    *(double *)value = atof(value_str);
  } else if (strcmp(value_type, "int64_t") == 0) {
    *(int64_t *)value = atol(value_str);
  } else if (strcmp(value_type, "uint64_t") == 0) {
    *(uint64_t *)value = atol(value_str);
  } else if (strcmp(value_type, "string") == 0) {
    value_str = string_strip_char(value_str, '"');
    string_copy((char *)value, value_str);
  } else if (strcmp(value_type, "vec2i") == 0) {
    parse_vector_line(value_str, "int", value, 2);
  } else if (strcmp(value_type, "vec3i") == 0) {
    parse_vector_line(value_str, "int", value, 3);
  } else if (strcmp(value_type, "vec2d") == 0) {
    parse_vector_line(value_str, "double", value, 2);
  } else if (strcmp(value_type, "vec3d") == 0) {
    parse_vector_line(value_str, "double", value, 3);
  } else if (strcmp(value_type, "vec4d") == 0) {
    parse_vector_line(value_str, "double", value, 4);
  } else if (strcmp(value_type, "vec7d") == 0) {
    parse_vector_line(value_str, "double", value, 7);
  } else if (strcmp(value_type, "pose") == 0) {
    parse_vector_line(value_str, "double", value, 7);
  } else {
    FATAL("Invalid value type [%s]\n", value_type);
  }
}

/**
 * CalibData
 */
struct CalibData {
  int num_cams = 0;
  int num_links = 0;
  int num_views = 0;
  int num_joints = 0;

  std::map<int, camera_params_t> cam_params;
  double gimbal_ext[7] = {0};
  double link0_ext[7] = {0};
  double link1_ext[7] = {0};
  double end_ext[7] = {0};
  double cam0_ext[7] = {0};
  double cam1_ext[7] = {0};
  double cam2_ext[7] = {0};
  double cam3_ext[7] = {0};
  double fiducial_pose[7] = {0};

  std::vector<timestamp_t> timestamps;
  double pose[7] = {0};
  std::map<timestamp_t, double> joint0_data;
  std::map<timestamp_t, double> joint1_data;
  std::map<timestamp_t, double> joint2_data;
  std::map<timestamp_t, aprilgrid_t> cam0_grids;
  std::map<timestamp_t, aprilgrid_t> cam1_grids;
  std::map<timestamp_t, aprilgrid_t> cam2_grids;
  std::map<timestamp_t, aprilgrid_t> cam3_grids;
  std::map<timestamp_t, cv::Mat> cam0_images;
  std::map<timestamp_t, cv::Mat> cam1_images;
  std::map<timestamp_t, cv::Mat> cam2_images;
  std::map<timestamp_t, cv::Mat> cam3_images;

  CalibData(const std::string &data_path, const bool format_v2) {
    load_config(data_path);
    load_joints(data_path);
    load_grids(data_path, format_v2);
    load_images(data_path);
    init_fiducial_pose();
  }

  void load_config(const std::string &data_path) {
    // Open config file
    std::string conf_path = data_path + "/calib.config";
    FILE *conf = fopen(conf_path.c_str(), "r");
    if (conf == NULL) {
      FATAL("Failed to open [%s]!\n", conf_path.c_str());
    }

    // Parse general
    parse_key_value(conf, "num_cams", "int", &num_cams);
    parse_key_value(conf, "num_links", "int", &num_links);
    parse_skip_line(conf);

    // Parse camera parameters
    for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
      int cam_res[2] = {0};
      char proj_model[30] = {0};
      char dist_model[30] = {0};
      real_t p[4] = {0};
      real_t d[4] = {0};

      parse_skip_line(conf);
      parse_key_value(conf, "resolution", "vec2i", cam_res);
      parse_key_value(conf, "proj_model", "string", proj_model);
      parse_key_value(conf, "dist_model", "string", dist_model);
      parse_key_value(conf, "proj_params", "vec4d", p);
      parse_key_value(conf, "dist_params", "vec4d", d);
      parse_skip_line(conf);

      const Eigen::Vector4d K{p[0], p[1], p[2], p[3]};
      const Eigen::Vector4d D{d[0], d[1], d[2], d[3]};
      camera_params_t cam(cam_idx, cam_res, "pinhole", "radtan4", K, D);
      cam_params[cam_idx] = cam;
    }

    // Parse extrinsics
    parse_key_value(conf, "gimbal_ext", "pose", gimbal_ext);  // T_C0M0
    parse_key_value(conf, "link0_ext", "pose", link0_ext);    // T_L0M1
    parse_key_value(conf, "link1_ext", "pose", link1_ext);    // T_L1M2
    parse_key_value(conf, "end_ext", "pose", end_ext);        // T_L2C2
    parse_key_value(conf, "cam0_cam1_ext", "pose", cam1_ext); // T_C0C1
    parse_key_value(conf, "cam2_cam3_ext", "pose", cam3_ext); // T_C2C3
    transform_init(cam0_ext);                                 // T_C0C0
    transform_init(cam2_ext);                                 // T_C2C2

    // Clean up
    fclose(conf);
  }

  void load_joints(const std::string &data_path) {
    // Open joint angles file
    std::string joints_path = data_path + "/joint_angles.dat";
    FILE *joints_file = fopen(joints_path.c_str(), "r");
    if (joints_file == NULL) {
      FATAL("Failed to open [%s]!\n", joints_path.c_str());
    }

    // Parse
    parse_key_value(joints_file, "num_views", "int", &num_views);
    parse_key_value(joints_file, "num_joints", "int", &num_joints);
    parse_skip_line(joints_file);
    parse_skip_line(joints_file);

    for (int view_idx = 0; view_idx < num_views; view_idx++) {
      // Get line
      const size_t buf_len = 1024;
      char buf[1024] = {0};
      if (fgets(buf, buf_len, joints_file) == NULL) {
        FATAL("Failed to view parse data!\n");
      }

      // Parse timestamp and joint angles
      size_t n = 0;
      char **s = string_split(buf, ',', &n);
      const timestamp_t ts = strtol(s[0], NULL, 10);
      free(s[0]);

      assert(n == num_joints + 1);
      timestamps.push_back(ts);

      joint0_data[ts] = strtod(s[1], NULL);
      free(s[1]);
      joint1_data[ts] = strtod(s[2], NULL);
      free(s[2]);
      joint2_data[ts] = strtod(s[3], NULL);
      free(s[3]);

      free(s);
    }

    // Clean up
    fclose(joints_file);
  }

  void load_grids(const std::string &data_path, const bool format_v2) {
    // Calibration data preprocessing
    const std::string grid0_path = data_path + "/grid0/cam0";
    const std::string grid1_path = data_path + "/grid0/cam1";
    const std::string grid2_path = data_path + "/grid0/cam2";
    const std::string grid3_path = data_path + "/grid0/cam3";

    std::vector<std::string> grid0_files;
    std::vector<std::string> grid1_files;
    std::vector<std::string> grid2_files;
    std::vector<std::string> grid3_files;
    list_files(grid0_path, grid0_files);
    list_files(grid1_path, grid1_files);
    list_files(grid2_path, grid2_files);
    list_files(grid3_path, grid3_files);

    for (const auto &fname : grid0_files) {
      const std::string grid_path = grid0_path + "/" + fname;
      aprilgrid_t grid{grid_path, format_v2};
      cam0_grids[grid.timestamp] = grid;
    }

    for (const auto &fname : grid1_files) {
      const std::string grid_path = grid1_path + "/" + fname;
      aprilgrid_t grid{grid_path, format_v2};
      cam1_grids[grid.timestamp] = grid;
    }

    for (const auto &fname : grid2_files) {
      const std::string grid_path = grid2_path + "/" + fname;
      aprilgrid_t grid{grid_path, format_v2};
      cam2_grids[grid.timestamp] = grid;
    }

    for (const auto &fname : grid3_files) {
      const std::string grid_path = grid3_path + "/" + fname;
      aprilgrid_t grid{grid_path, format_v2};
      cam3_grids[grid.timestamp] = grid;
    }
  }

  void load_images(const std::string &data_path) {
    const std::string cam0_path = data_path + "/cam0";
    const std::string cam1_path = data_path + "/cam1";
    const std::string cam2_path = data_path + "/cam2";
    const std::string cam3_path = data_path + "/cam3";

    std::vector<std::string> cam0_files;
    std::vector<std::string> cam1_files;
    std::vector<std::string> cam2_files;
    std::vector<std::string> cam3_files;
    list_files(cam0_path, cam0_files);
    list_files(cam1_path, cam1_files);
    list_files(cam2_path, cam2_files);
    list_files(cam3_path, cam3_files);

    for (const auto &fname : cam0_files) {
      const std::string ts_str = fname.substr(0, 19);
      const timestamp_t ts = std::stoull(ts_str);
      const std::string img_path = cam0_path + "/" + fname;
      cam0_images[ts] = cv::imread(img_path);
    }

    for (const auto &fname : cam1_files) {
      const std::string ts_str = fname.substr(0, 19);
      const timestamp_t ts = std::stoull(ts_str);
      const std::string img_path = cam1_path + "/" + fname;
      cam1_images[ts] = cv::imread(img_path);
    }

    for (const auto &fname : cam2_files) {
      const std::string ts_str = fname.substr(0, 19);
      const timestamp_t ts = std::stoull(ts_str);
      const std::string img_path = cam2_path + "/" + fname;
      cam2_images[ts] = cv::imread(img_path);
    }

    for (const auto &fname : cam3_files) {
      const std::string ts_str = fname.substr(0, 19);
      const timestamp_t ts = std::stoull(ts_str);
      const std::string img_path = cam3_path + "/" + fname;
      cam3_images[ts] = cv::imread(img_path);
    }
  }

  void init_fiducial_pose() {
    // Set body pose
    std::vector<double> fid_pos_x;
    std::vector<double> fid_pos_y;
    std::vector<double> fid_pos_z;
    std::vector<double> fid_rot_r;
    std::vector<double> fid_rot_p;
    std::vector<double> fid_rot_y;

    for (const auto ts : timestamps) {
      std::vector<int> tag_ids;
      std::vector<int> corner_idxs;
      vec2s_t kps;
      vec3s_t obj_pts;
      cam0_grids[ts].get_measurements(tag_ids, corner_idxs, kps, obj_pts);
      if (tag_ids.size() < 20) {
        continue;
      }

      mat4_t T_C0F;
      pinhole_radtan4_t cam_geom;
      const int retval = solvepnp(&cam_geom,
                                  cam_params[0].resolution,
                                  cam_params[0].param,
                                  kps,
                                  obj_pts,
                                  T_C0F);
      if (retval != 0) {
        continue;
      }

      // const auto &cam0_grid = cam0_grids[ts];
      // const auto &viz = cam0_grid.draw(cam0_images[ts]);
      // cv::imshow("cam0", viz);
      // cv::waitKey(1);

      const Eigen::Vector3d r_C0F = tf_trans(T_C0F);
      const Eigen::Vector3d rpy_C0F = quat2euler(tf_quat(T_C0F));
      const Eigen::VectorXd fiducial = transform_vector(T_C0F);
      // std::cout << "fiducial pose: " << fiducial.transpose() << std::endl;
      fid_pos_x.push_back(r_C0F.x());
      fid_pos_y.push_back(r_C0F.y());
      fid_pos_z.push_back(r_C0F.z());
      fid_rot_r.push_back(rpy_C0F.x());
      fid_rot_p.push_back(rpy_C0F.y());
      fid_rot_y.push_back(rpy_C0F.z());
    }

    // Median fiducial pose
    const auto pos_x = median(fid_pos_x);
    const auto pos_y = median(fid_pos_y);
    const auto pos_z = median(fid_pos_z);
    const auto rot_r = median(fid_rot_r);
    const auto rot_p = median(fid_rot_p);
    const auto rot_y = median(fid_rot_y);
    const Eigen::Vector3d r_C0F{pos_x, pos_y, pos_z};
    const Eigen::Vector3d rpy_C0F{rot_r, rot_p, rot_y};
    const Eigen::Quaterniond q_C0F{euler321(rpy_C0F)};
    const Eigen::Matrix4d T_C0F = tf(q_C0F, r_C0F);
    const Eigen::VectorXd fiducial = transform_vector(T_C0F);
    for (int i = 0; i < 7; i++) {
      fiducial_pose[i] = fiducial[i];
    }
  }
};

/**
 * Gimbal Reprojection Error.
 */
struct GimbalReprojError {
  Eigen::VectorXd proj_params;
  Eigen::Vector2d z;
  Eigen::Vector3d p_FFi;

  GimbalReprojError(const Eigen::VectorXd &proj_params_,
                    const Eigen::Vector2d &z_,
                    const Eigen::Vector3d &p_FFi_)
      : proj_params{proj_params_}, z{z_}, p_FFi{p_FFi_} {}

  // Create GimbalReprojectError
  static ceres::CostFunction *Create(const Eigen::VectorXd &proj_params_,
                                     const Eigen::Vector2d &z_,
                                     const Eigen::Vector3d &p_FFi_) {
    return new ceres::AutoDiffCostFunction<GimbalReprojError,
                                           2, // Residual size
                                           7, // T_C0M0: Gimbal extrinsic
                                           7, // T_L0M1: link0 extrinsic
                                           7, // T_L1M2: link1 extrinsic
                                           1, // T_M0L0: joint0 angle
                                           1, // T_M1L1: joint1 angle
                                           1, // T_M2L2: joint2 angle
                                           7, // T_L2C2: End-effector extrinsic
                                           7, // T_ECi: Camera extrinsic
                                           7  // T_C0F: Relative pose
                                           >(
        new GimbalReprojError(proj_params_, z_, p_FFi_));
  }

  template <typename T>
  bool operator()(const T *const gimbal_ext,
                  const T *const link0_ext,
                  const T *const link1_ext,
                  const T *const joint0,
                  const T *const joint1,
                  const T *const joint2,
                  const T *const end_ext,
                  const T *const cam_ext,
                  const T *const rel_pose,
                  T *residuals) const {
    // // Map variables
    Eigen::Matrix<T, 4, 4> T_C0M0 = transform(gimbal_ext);
    Eigen::Matrix<T, 4, 4> T_L0M1 = transform(link0_ext);
    Eigen::Matrix<T, 4, 4> T_L1M2 = transform(link1_ext);
    Eigen::Matrix<T, 4, 4> T_M0L0 = gimbal_joint_transform(joint0[0]);
    Eigen::Matrix<T, 4, 4> T_M1L1 = gimbal_joint_transform(joint1[0]);
    Eigen::Matrix<T, 4, 4> T_M2L2 = gimbal_joint_transform(joint2[0]);
    Eigen::Matrix<T, 4, 4> T_L2C2 = transform(end_ext);
    Eigen::Matrix<T, 4, 4> T_C2Ci = transform(cam_ext);
    Eigen::Matrix<T, 4, 4> T_C0F = transform(rel_pose);

    // Transform point in fiducial frame to camera frame
    auto T_C0C2 = T_C0M0 * T_M0L0 * T_L0M1 * T_M1L1 * T_L1M2 * T_M2L2 * T_L2C2;
    auto T_CiF = (T_C0F.inverse() * T_C0C2 * T_C2Ci).inverse();
    const auto p_Ci = (T_CiF * p_FFi.homogeneous()).head(3);

    // Project to image plane
    const T fx = T(proj_params(0));
    const T fy = T(proj_params(1));
    const T cx = T(proj_params(2));
    const T cy = T(proj_params(3));
    Eigen::Vector<T, 2> z_hat;
    z_hat(0) = fx * (p_Ci.x() / p_Ci.z()) + cx;
    z_hat(1) = fy * (p_Ci.y() / p_Ci.z()) + cy;

    // Calculate residual
    residuals[0] = T(z[0]) - z_hat[0];
    residuals[1] = T(z[1]) - z_hat[1];

    return true;
  }
};

/**
 * Gimbal Joint Error.
 */
struct GimbalJointError {
  double joint_angle = 0.0;
  double covar = 0.0;
  double sqrt_info = 0.0;

  GimbalJointError(const double joint_angle_, const double covar_)
      : joint_angle{joint_angle_}, covar{covar_}, sqrt_info{sqrt(1.0 / covar)} {
  }

  static ceres::CostFunction *Create(const double joint_angle_,
                                     const double covar_) {
    return new ceres::AutoDiffCostFunction<GimbalJointError,
                                           1, // Residual size
                                           1  // Joint Angle
                                           >(
        new GimbalJointError(joint_angle_, covar_));
  }

  template <typename T>
  bool operator()(const T *const est_joint_angle, T *residuals) const {
    residuals[0] = sqrt_info * (joint_angle - est_joint_angle[0]);
    return true;
  }
};

/**
 * Inspect calibration
 */
void inspect_calib(const CalibData &calib_data,
                   const std::vector<timestamp_t> &test_timestamps) {
  // Draw settings
  const cv::Scalar color_red(0, 0, 255);
  const cv::Scalar color_green(0, 255, 0);
  const int marker_size = 2;

  // Transforms
  const Eigen::Matrix4d T_C0F = transform(calib_data.fiducial_pose);
  const Eigen::Matrix4d T_C0M0 = transform(calib_data.gimbal_ext);
  const Eigen::Matrix4d T_L0M1 = transform(calib_data.link0_ext);
  const Eigen::Matrix4d T_L1M2 = transform(calib_data.link1_ext);
  const Eigen::Matrix4d T_L2C2 = transform(calib_data.end_ext);
  const Eigen::Matrix4d T_C0C1 = transform(calib_data.cam1_ext);
  const Eigen::Matrix4d T_C2C3 = transform(calib_data.cam3_ext);

  // Draw function
  auto draw = [&](const std::string cam_str,
                  const camera_params_t &cam,
                  const aprilgrid_t &grid,
                  const mat4_t &T_CiF,
                  cv::Mat &frame) {
    // Get measurements
    std::vector<int> tag_ids;
    std::vector<int> corner_idxs;
    vec2s_t kps;
    vec3s_t obj_pts;
    grid.get_measurements(tag_ids, corner_idxs, kps, obj_pts);
    if (tag_ids.size() == 0) {
      return;
    }

    std::vector<double> reproj_errors;
    for (size_t i = 0; i < tag_ids.size(); i++) {
      // Project point
      const auto p_FFi = obj_pts[i];
      const auto p_Ci = (T_CiF * p_FFi.homogeneous()).head(3);
      vec2_t z_hat;
      pinhole_radtan4_project(cam.resolution, cam.param, p_Ci, z_hat);
      reproj_errors.push_back((kps[i] - z_hat).norm());

      // Draw measured corner
      const cv::Point2f p_meas(kps[i].x(), kps[i].y());
      cv::circle(frame, p_meas, marker_size, color_red, -1);

      // Draw predicted corner
      const cv::Point2f p_pred(z_hat.x(), z_hat.y());
      cv::circle(frame, p_pred, marker_size, color_green, -1);
    }

    // Draw camera string
    {
      const cv::Point cxy(5, 15);
      const cv::Scalar text_color(0, 255, 0);
      const int font = cv::FONT_HERSHEY_PLAIN;
      const double font_scale = 1;
      const int thickness = 1;
      cv::putText(frame,
                  cam_str,
                  cxy,
                  font,
                  font_scale,
                  text_color,
                  thickness,
                  cv::LINE_AA);
    }

    // Draw reprojection error
    {
      const cv::Point cxy(5, 30);
      const cv::Scalar text_color(0, 255, 0);
      const int font = cv::FONT_HERSHEY_PLAIN;
      const double font_scale = 1;
      const int thickness = 1;
      const auto error = rmse(reproj_errors);
      const auto text = cv::format("RMS Reproj Error: %.2f [px]", error);
      cv::putText(frame,
                  text,
                  cxy,
                  font,
                  font_scale,
                  text_color,
                  thickness,
                  cv::LINE_AA);
    }
  };

  // Loop through dataset
  for (const auto ts : test_timestamps) {
    const double joint0 = calib_data.joint0_data.at(ts);
    const double joint1 = calib_data.joint1_data.at(ts);
    const double joint2 = calib_data.joint2_data.at(ts);
    const Eigen::Matrix4d T_M0L0 = gimbal_joint_transform(joint0);
    const Eigen::Matrix4d T_M1L1 = gimbal_joint_transform(joint1);
    const Eigen::Matrix4d T_M2L2 = gimbal_joint_transform(joint2);

    auto T_C0C2 = T_C0M0 * T_M0L0 * T_L0M1 * T_M1L1 * T_L1M2 * T_M2L2 * T_L2C2;
    auto T_C1F = (T_C0F.inverse() * T_C0C1).inverse();
    auto T_C2F = (T_C0F.inverse() * T_C0C2).inverse();
    auto T_C3F = (T_C0F.inverse() * T_C0C2 * T_C2C3).inverse();

    // Draw camera0 frame
    const auto &cam0 = calib_data.cam_params.at(0);
    const auto &grid0 = calib_data.cam0_grids.at(ts);
    auto frame0 = calib_data.cam0_images.at(ts).clone();
    draw("Camera 0", cam0, grid0, T_C0F, frame0);

    // Draw camera1 frame
    const auto &cam1 = calib_data.cam_params.at(1);
    const auto &grid1 = calib_data.cam1_grids.at(ts);
    auto frame1 = calib_data.cam1_images.at(ts).clone();
    draw("Camera 1", cam1, grid1, T_C1F, frame1);

    // Draw camera2 frame
    const auto &cam2 = calib_data.cam_params.at(2);
    const auto &grid2 = calib_data.cam2_grids.at(ts);
    auto frame2 = calib_data.cam2_images.at(ts).clone();
    draw("Camera 2", cam2, grid2, T_C2F, frame2);

    // Draw camera3 frame
    const auto &cam3 = calib_data.cam_params.at(3);
    const auto &grid3 = calib_data.cam3_grids.at(ts);
    auto frame3 = calib_data.cam3_images.at(ts).clone();
    draw("Camera 3", cam3, grid3, T_C3F, frame3);

    // Show visualization
    cv::Mat viz_row0;
    cv::Mat viz_row1;
    cv::Mat viz;
    cv::hconcat(frame2, frame3, viz_row0);
    cv::hconcat(frame0, frame1, viz_row1);
    cv::vconcat(viz_row0, viz_row1, viz);
    // cv::imwrite(cv::format("/tmp/viz-%ld.png", ts), viz);
    cv::imshow("Viz", viz);
    if (cv::waitKey(0) == 'q') {
      break;
    }
  }
}

/**
 * Save results
 **/
void save_results(CalibData &calib_data, const std::string &data_path) {
  // Vector to string
  auto vec2str = [](double *vec, const size_t size) {
    Eigen::Map<Eigen::VectorXd> v(vec, size);
    return yac::vec2str(v, true, true);
  };

  // Save camera parameters
  auto save_camera_params = [](FILE *out,
                               const int cam_idx,
                               const camera_params_t *cam) {
    const bool max_digits = (out == stdout) ? false : true;
    const int *cam_res = cam->resolution;
    const char *proj_model = cam->proj_model.c_str();
    const char *dist_model = cam->dist_model.c_str();
    const auto proj_params = yac::vec2str(cam->proj_params(), true, max_digits);
    const auto dist_params = yac::vec2str(cam->dist_params(), true, max_digits);

    fprintf(out, "cam%d:\n", cam_idx);
    fprintf(out, "  resolution: [%d, %d]\n", cam_res[0], cam_res[1]);
    fprintf(out, "  proj_model: \"%s\"\n", proj_model);
    fprintf(out, "  dist_model: \"%s\"\n", dist_model);
    fprintf(out, "  proj_params: %s\n", proj_params.c_str());
    fprintf(out, "  dist_params: %s\n", dist_params.c_str());
    fprintf(out, "\n");
  };

  // Save results
  const std::string results_path = data_path + "/calib_gimbal-results.yaml";
  double *gimbal_ext = calib_data.gimbal_ext;
  double *link0_ext = calib_data.link0_ext;
  double *link1_ext = calib_data.link1_ext;
  double *end_ext = calib_data.end_ext;
  double *cam0_ext = calib_data.cam0_ext;
  double *cam1_ext = calib_data.cam1_ext;
  double *cam2_ext = calib_data.cam2_ext;
  double *cam3_ext = calib_data.cam3_ext;
  double *fiducial_pose = calib_data.fiducial_pose;

  FILE *results = fopen(results_path.c_str(), "w");
  fprintf(results, "num_cams: 4\n");
  fprintf(results, "num_links: 2\n");
  fprintf(results, "\n");
  save_camera_params(results, 0, &calib_data.cam_params.at(0));
  save_camera_params(results, 1, &calib_data.cam_params.at(1));
  save_camera_params(results, 2, &calib_data.cam_params.at(2));
  save_camera_params(results, 3, &calib_data.cam_params.at(3));
  fprintf(results, "gimbal_ext: %s\n", vec2str(gimbal_ext, 7).c_str());
  fprintf(results, "link0_ext: %s\n", vec2str(link0_ext, 7).c_str());
  fprintf(results, "link1_ext: %s\n", vec2str(link1_ext, 7).c_str());
  fprintf(results, "end_ext: %s\n", vec2str(end_ext, 7).c_str());
  fprintf(results, "cam0_ext: %s\n", vec2str(cam0_ext, 7).c_str());
  fprintf(results, "cam1_ext: %s\n", vec2str(cam1_ext, 7).c_str());
  fprintf(results, "cam2_ext: %s\n", vec2str(cam2_ext, 7).c_str());
  fprintf(results, "cam3_ext: %s\n", vec2str(cam3_ext, 7).c_str());
  fprintf(results, "fiducial_pose: %s\n", vec2str(fiducial_pose, 7).c_str());
  fclose(results);
}

int main(int argc, char *argv[]) {
  bool format_v2 = true;
  bool fix_joints = false;
  bool enable_joint_errors = false;

  // Check arguments
  if (argc != 2) {
    printf("usage: %s <data_path>\n", argv[0]);
    return -1;
  }

  // Setup
  const std::string data_path = argv[1];
  CalibData calib_data{data_path, format_v2};

  // Setup problem
  // clang-format off
  ceres::Problem::Options prob_options;
  prob_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem{prob_options};
  ceres::ProductManifold<ceres::EuclideanManifold<3>, ceres::QuaternionManifold> R3xSO3;
  ceres::ProductManifold<ceres::SubsetManifold, ceres::QuaternionManifold> link_manif{ceres::SubsetManifold{3, {0, 1}}, ceres::QuaternionManifold{}};
  std::vector<ceres::ResidualBlockId> res_ids;
  // clang-format on

  problem.AddParameterBlock(calib_data.gimbal_ext, 7, &R3xSO3);
  problem.AddParameterBlock(calib_data.link0_ext, 7, &link_manif);
  problem.AddParameterBlock(calib_data.link1_ext, 7, &link_manif);
  problem.AddParameterBlock(calib_data.end_ext, 7, &R3xSO3);
  problem.AddParameterBlock(calib_data.cam0_ext, 7, &R3xSO3);
  problem.AddParameterBlock(calib_data.cam1_ext, 7, &R3xSO3);
  problem.AddParameterBlock(calib_data.cam2_ext, 7, &R3xSO3);
  problem.AddParameterBlock(calib_data.cam3_ext, 7, &R3xSO3);
  problem.AddParameterBlock(calib_data.fiducial_pose, 7, &R3xSO3);
  // problem.SetParameterBlockConstant(calib_data.gimbal_ext);
  problem.SetParameterBlockConstant(calib_data.cam0_ext);
  problem.SetParameterBlockConstant(calib_data.cam1_ext);
  problem.SetParameterBlockConstant(calib_data.cam2_ext);
  problem.SetParameterBlockConstant(calib_data.cam3_ext);
  problem.SetParameterBlockConstant(calib_data.fiducial_pose);

  // -- Lambda function to add reprojection errors
  auto add_grid = [&](const int cam_idx, const aprilgrid_t &grid) {
    // Get AprilGrid measurments
    const timestamp_t ts = grid.timestamp;
    std::vector<int> tag_ids;
    std::vector<int> corner_idxs;
    vec2s_t kps;
    vec3s_t obj_pts;
    grid.get_measurements(tag_ids, corner_idxs, kps, obj_pts);

    // Undistort keypoints
    vec2s_t kps_undistorted;
    const auto K = calib_data.cam_params[cam_idx].param;
    for (const auto &kp : kps) {
      kps_undistorted.push_back(pinhole_radtan4_undistort(K, kp));
    }

    // Add residual blocks
    const auto proj_params = calib_data.cam_params[cam_idx].proj_params();
    double *cam_ext = nullptr;
    if (cam_idx == 2) {
      cam_ext = calib_data.cam2_ext;
    } else if (cam_idx == 3) {
      cam_ext = calib_data.cam3_ext;
    } else {
      FATAL("Invalid camera index!");
    }

    for (size_t i = 0; i < tag_ids.size(); i++) {
      // Add joint angles
      double *joint0 = &calib_data.joint0_data[ts];
      double *joint1 = &calib_data.joint1_data[ts];
      double *joint2 = &calib_data.joint2_data[ts];
      problem.AddParameterBlock(joint0, 1);
      problem.AddParameterBlock(joint1, 1);
      problem.AddParameterBlock(joint2, 1);
      if (fix_joints) {
        problem.SetParameterBlockConstant(joint0);
        problem.SetParameterBlockConstant(joint1);
        problem.SetParameterBlockConstant(joint2);
      }

      // Add Gimbal Reprojection Error
      const Eigen::Vector2d z = kps_undistorted[i];
      const Eigen::Vector3d p_FFi = obj_pts[i];
      auto reproj_error = GimbalReprojError::Create(proj_params, z, p_FFi);
      std::vector<double *> param_blocks;
      param_blocks.push_back(calib_data.gimbal_ext);
      param_blocks.push_back(calib_data.link0_ext);
      param_blocks.push_back(calib_data.link1_ext);
      param_blocks.push_back(joint0);
      param_blocks.push_back(joint1);
      param_blocks.push_back(joint2);
      param_blocks.push_back(calib_data.end_ext);
      param_blocks.push_back(cam_ext);
      param_blocks.push_back(calib_data.fiducial_pose);
      auto res_id =
          problem.AddResidualBlock(reproj_error, nullptr, param_blocks);
      res_ids.push_back(res_id);

      // Add Gimbal Joint Error
      if (enable_joint_errors) {
        auto joint0_error = GimbalJointError::Create(*joint0, 1.0);
        auto joint1_error = GimbalJointError::Create(*joint1, 1.0);
        auto joint2_error = GimbalJointError::Create(*joint2, 1.0);
        problem.AddResidualBlock(joint0_error, nullptr, {joint0});
        problem.AddResidualBlock(joint1_error, nullptr, {joint1});
        problem.AddResidualBlock(joint2_error, nullptr, {joint2});
      }
    }
  };

  std::vector<timestamp_t> timestamps = calib_data.timestamps;
  std::random_device rd;
  std::mt19937 generator(rd());
  std::shuffle(timestamps.begin(), timestamps.end(), generator);

  const int N = timestamps.size() * 0.8;
  std::vector<timestamp_t> train_timestamps;
  std::vector<timestamp_t> test_timestamps;
  train_timestamps.insert(train_timestamps.end(),
                          timestamps.begin(),
                          timestamps.begin() + N);
  test_timestamps.insert(test_timestamps.end(),
                         timestamps.begin() + N + 1,
                         timestamps.end());

  // -- Add reprojection errors
  // for (const auto ts : train_timestamps) {
  //   add_grid(2, calib_data.cam2_grids[ts]);
  //   add_grid(3, calib_data.cam3_grids[ts]);
  // }
  for (const auto ts : timestamps) {
    add_grid(2, calib_data.cam2_grids[ts]);
    add_grid(3, calib_data.cam3_grids[ts]);
  }

  // -- Solve
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.minimizer_progress_to_stdout = true;
  ceres::Solve(options, &problem, &summary);

  std::vector<double> reproj_errors;
  for (const auto res_id : res_ids) {
    double cost = 0.0;
    double r[2] = {0};
    problem.EvaluateResidualBlock(res_id, false, &cost, r, nullptr);
    reproj_errors.push_back(sqrt(r[0] * r[0] + r[1] * r[1]));
  }
  printf("rmse: %f\n", rmse(reproj_errors));
  printf("mean: %f\n", mean(reproj_errors));

  printf("[after]\n");
  print_vector("gimbal_ext   ", calib_data.gimbal_ext, 7);
  print_vector("link0_ext    ", calib_data.link0_ext, 7);
  print_vector("link1_ext    ", calib_data.link1_ext, 7);
  print_vector("end_ext      ", calib_data.end_ext, 7);
  print_vector("cam0_ext     ", calib_data.cam0_ext, 7);
  print_vector("cam1_ext     ", calib_data.cam1_ext, 7);
  print_vector("cam2_ext     ", calib_data.cam2_ext, 7);
  print_vector("cam3_ext     ", calib_data.cam3_ext, 7);
  print_vector("fiducial_pose", calib_data.fiducial_pose, 7);

  inspect_calib(calib_data, timestamps);
  // inspect_calib(calib_data, test_timestamps);
  save_results(calib_data, data_path);

  return 0;
}
