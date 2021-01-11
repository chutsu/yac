#include "core.hpp"

namespace yac {

/*****************************************************************************
 *                                  DATA
 *****************************************************************************/

int8_t int8(const uint8_t *data, const size_t offset) {
  return (int8_t)(data[offset]);
}

uint8_t uint8(const uint8_t *data, const size_t offset) {
  return (uint8_t)(data[offset]);
}

int16_t int16(const uint8_t *data, const size_t offset) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

uint16_t uint16(const uint8_t *data, const size_t offset) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

int32_t sint32(const uint8_t *data, const size_t offset) {
  return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                   (data[offset + 1] << 8) | (data[offset]));
}

uint32_t uint32(const uint8_t *data, const size_t offset) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                    (data[offset + 1] << 8) | (data[offset]));
}

char *malloc_string(const char *s) {
  char *retval = (char *) malloc(sizeof(char) * strlen(s) + 1);
  strcpy(retval, s);
  return retval;
}

int csv_rows(const char *fp) {
  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return -1;
  }

  // Loop through lines
  int nb_rows = 0;
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      nb_rows++;
    }
  }

  // Cleanup
  fclose(infile);

  return nb_rows;
}

int csv_cols(const char *fp) {
  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return -1;
  }

  // Get line that isn't the header
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  // Parse line to obtain number of elements
  int nb_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < len_max; i++) {
    if (line[i] == ',') {
      found_separator = 1;
      nb_elements++;
    }
  }

  // Cleanup
  fclose(infile);

  return (found_separator) ? nb_elements : -1;
}

char **csv_fields(const char *fp, int *nb_fields) {
  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Get last header line
  char field_line[1024] = {0};
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      break;
    } else {
      strcpy(field_line, line);
    }
  }

  // Parse fields
  *nb_fields = csv_cols(fp);
  char **fields = (char **) malloc(sizeof(char *) * *nb_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    // Ignore # and ' '
    if (c == '#' || c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      // Add field name to fields
      fields[field_idx] = malloc_string(field_name);
      memset(field_name, '\0', sizeof(char) * 100);
      field_idx++;
    } else {
      // Append field name
      field_name[strlen(field_name)] = c;
    }
  }

  // Cleanup
  fclose(infile);

  return fields;
}

real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols) {
  // Obtain number of rows and columns in csv data
  *nb_rows = csv_rows(fp);
  *nb_cols = csv_cols(fp);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  // Initialize memory for csv data
  real_t **data = (real_t **) malloc(sizeof(real_t *) * *nb_rows);
  for (int i = 0; i < *nb_cols; i++) {
    data[i] = (real_t *) malloc(sizeof(real_t) * *nb_cols);
  }

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Loop through data
  char line[1024] = {0};
  size_t len_max = 1024;
  int row_idx = 0;
  int col_idx = 0;

  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[row_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  // Cleanup
  fclose(infile);

  return data;
}

static int *parse_iarray_line(char *line) {
  char entry[1024] = {0};
  int index = 0;
  int *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = (int *) calloc(array_size + 1, sizeof(int));
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

int **load_iarrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = csv_rows(csv_path);
  int **array = (int **) calloc(*nb_arrays, sizeof(int *));

  char line[1024] = {0};
  int frame_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_iarray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

static real_t *parse_darray_line(char *line) {
  char entry[1024] = {0};
  int index = 0;
  real_t *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = (real_t *) calloc(array_size, sizeof(real_t));
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

real_t **load_darrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = csv_rows(csv_path);
  real_t **array = (real_t **) calloc(*nb_arrays, sizeof(real_t *));

  char line[1024] = {0};
  int frame_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_darray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

int csv_rows(const std::string &file_path) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of lines
  int nb_rows = 0;
  std::string line;
  while (std::getline(infile, line)) {
    nb_rows++;
  }

  return nb_rows;
}

int csv_cols(const std::string &file_path) {
  int nb_elements = 1;
  bool found_separator = false;

  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of commas
  std::string line;
  std::getline(infile, line);
  for (size_t i = 0; i < line.length(); i++) {
    if (line[i] == ',') {
      found_separator = true;
      nb_elements++;
    }
  }

  return (found_separator) ? nb_elements : 0;
}

int csv2mat(const std::string &file_path, const bool header, matx_t &data) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of rows and cols
  int nb_rows = csv_rows(file_path);
  int nb_cols = csv_cols(file_path);

  // Skip header line?
  std::string line;
  if (header) {
    std::getline(infile, line);
    nb_rows -= 1;
  }

  // Load data
  int line_no = 0;
  std::vector<real_t> vdata;
  data = zeros(nb_rows, nb_cols);

  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    // Load data row
    std::string element;
    for (int i = 0; i < nb_cols; i++) {
      std::getline(ss, element, ',');
      const real_t value = atof(element.c_str());
      data(line_no, i) = value;
    }

    line_no++;
  }

  return 0;
}

int mat2csv(const std::string &file_path, const matx_t &data) {
  // Open file
  FILE *outfile = fopen(file_path.c_str(), "w");
  if (outfile == nullptr) {
    return -1;
  }

  // Save matrix
  for (int i = 0; i < data.rows(); i++) {
    for (int j = 0; j < data.cols(); j++) {
      fprintf(outfile, "%f", data(i, j));

      if ((j + 1) != data.cols()) {
        fprintf(outfile, ",");
      }
    }
    fprintf(outfile, "\n");
  }

  // Close file
  fclose(outfile);

  return 0;
}

int vec2csv(const std::string &file_path, const std::deque<vec3_t> &data) {
  // Open file
  std::ofstream outfile(file_path);
  if (outfile.good() != true) {
    return -1;
  }

  // Save vector
  for (const auto &v : data) {
    outfile << v(0);
    outfile << ",";
    outfile << v(1);
    outfile << ",";
    outfile << v(2);
    outfile << std::endl;
  }

  // Close file
  outfile.close();
  return 0;
}

int ts2csv(const std::string &file_path, const std::deque<timestamp_t> &data) {
  // Open file
  std::ofstream outfile(file_path);
  if (outfile.good() != true) {
    return -1;
  }

  // Save vector
  for (const auto &ts : data) {
    outfile << ts << std::endl;
  }

  // Close file
  outfile.close();
  return 0;
}

void print_progress(const real_t percentage) {
  const char *PBSTR =
      "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
  const int PBWIDTH = 60;

  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);

  if ((fabs(percentage - 1.0) < 1e-10)) {
    printf("\n");
  }
}

bool all_true(const std::vector<bool> x) {
  for (const auto i : x) {
    if (i == false) {
      return false;
    }
  }

  return true;
}

void save_features(const std::string &path, const vec3s_t &features) {
  FILE *csv = fopen(path.c_str(), "w");
  for (const auto &f : features) {
    fprintf(csv, "%f,%f,%f\n", f(0), f(1), f(2));
  }
  fflush(csv);
  fclose(csv);
}

void save_pose(FILE *csv_file,
               const timestamp_t &ts,
               const quat_t &rot,
               const vec3_t &pos) {
  fprintf(csv_file, "%ld,", ts);
  fprintf(csv_file, "%f,%f,%f,%f,", rot.w(), rot.x(), rot.y(), rot.z());
  fprintf(csv_file, "%f,%f,%f\n", pos(0), pos(1), pos(2));
}

void save_pose(FILE *csv_file, const timestamp_t &ts, const vecx_t &pose) {
  fprintf(csv_file, "%ld,", ts);
  fprintf(csv_file, "%f,%f,%f,%f,", pose(0), pose(1), pose(2), pose(3));
  fprintf(csv_file, "%f,%f,%f\n", pose(4), pose(5), pose(6));
}

void save_poses(const std::string &path,
                const timestamps_t &timestamps,
                const quats_t &orientations,
                const vec3s_t &positions) {
  FILE *csv = fopen(path.c_str(), "w");
  for (size_t i = 0; i < timestamps.size(); i++) {
    const timestamp_t ts = timestamps[i];
    const quat_t rot = orientations[i];
    const vec3_t pos = positions[i];
    save_pose(csv, ts, rot, pos);
  }
  fflush(csv);
  fclose(csv);
}

void save_poses(const std::string &path, const mat4s_t &poses) {
  FILE *csv = fopen(path.c_str(), "w");
  for (const auto &pose : poses) {
    const auto &q = tf_quat(pose);
    const auto &r = tf_trans(pose);

    fprintf(csv, "%f,", q.w());
    fprintf(csv, "%f,", q.x());
    fprintf(csv, "%f,", q.y());
    fprintf(csv, "%f,", q.z());

    fprintf(csv, "%f,", r(0));
    fprintf(csv, "%f,", r(1));
    fprintf(csv, "%f\n", r(2));
  }
  fflush(csv);
  fclose(csv);
}

void save_poses(const std::string &path,
								const timestamps_t &timestamps,
								const mat4s_t &poses) {
  FILE *csv = fopen(path.c_str(), "w");
  for (size_t k = 0; k < timestamps.size(); k++) {
		const auto ts = timestamps[k];
    const auto &q = tf_quat(poses[k]);
    const auto &r = tf_trans(poses[k]);

    fprintf(csv, "%ld,", ts);

    fprintf(csv, "%f,", q.w());
    fprintf(csv, "%f,", q.x());
    fprintf(csv, "%f,", q.y());
    fprintf(csv, "%f,", q.z());

    fprintf(csv, "%f,", r(0));
    fprintf(csv, "%f,", r(1));
    fprintf(csv, "%f\n", r(2));
  }
  fflush(csv);
  fclose(csv);
}

void save_pose(const std::string &path, const mat4_t &pose) {
  FILE *csv = fopen(path.c_str(), "w");
  const auto &q = tf_quat(pose);
  const auto &r = tf_trans(pose);

  fprintf(csv, "%f,", q.w());
  fprintf(csv, "%f,", q.x());
  fprintf(csv, "%f,", q.y());
  fprintf(csv, "%f,", q.z());

  fprintf(csv, "%f,", r(0));
  fprintf(csv, "%f,", r(1));
  fprintf(csv, "%f\n", r(2));

  fflush(csv);
  fclose(csv);
}

void save_pose(const std::string &path,
               const timestamp_t &ts,
               const mat4_t &pose) {
  FILE *csv = fopen(path.c_str(), "w");
  const auto &q = tf_quat(pose);
  const auto &r = tf_trans(pose);

  fprintf(csv, "%ld,", ts);

  fprintf(csv, "%f,", q.w());
  fprintf(csv, "%f,", q.x());
  fprintf(csv, "%f,", q.y());
  fprintf(csv, "%f,", q.z());

  fprintf(csv, "%f,", r(0));
  fprintf(csv, "%f,", r(1));
  fprintf(csv, "%f\n", r(2));

  fflush(csv);
  fclose(csv);
}

void save_imu_data(const std::string &path,
							     const timestamps_t &imu_ts,
								   const vec3s_t &imu_acc,
								   const vec3s_t &imu_gyr) {
  assert(imu_ts.size() == imu_acc.size());
  assert(imu_ts.size() == imu_gyr.size());

  FILE *csv = fopen(path.c_str(), "w");
  for (size_t k = 0; k < imu_ts.size(); k++) {
    fprintf(csv, "%ld,", imu_ts[k]);

    fprintf(csv, "%f,", imu_acc[k].x());
    fprintf(csv, "%f,", imu_acc[k].y());
    fprintf(csv, "%f,", imu_acc[k].z());

    fprintf(csv, "%f,", imu_gyr[k].x());
    fprintf(csv, "%f,", imu_gyr[k].y());
    fprintf(csv, "%f", imu_gyr[k].z());

    fprintf(csv, "\n");
  }
  fflush(csv);
  fclose(csv);
}

mat4_t load_pose(const std::string &fpath) {
  mat4_t T_WF;

  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(fpath.c_str(), "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", fpath.c_str());
  }

  // Create format string
  std::string str_format;
  str_format = "%ld,";              // Timestamp[ns]
  str_format += "%lf,%lf,%lf,%lf,"; // Quaternion
  str_format += "%lf,%lf,%lf";      // Position

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double qw, qx, qy, qz = 0.0;
    double px, py, pz = 0.0;
    int retval =
        fscanf(fp, str_format.c_str(), &ts, &qw, &qx, &qy, &qz, &px, &py, &pz);
    if (retval != 8) {
      FATAL("Failed to parse line in [%s:%d]", fpath.c_str(), i);
    }

    // Just need 1 pose
    quat_t q{qw, qx, qy, qz};
    vec3_t r{px, py, pz};
    return tf(q, r);
  }

  return I(4);
}

void load_poses(const std::string &fpath,
                timestamps_t &timestamps,
                mat4s_t &poses) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(fpath.c_str(), "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", fpath.c_str());
  }

  // Create format string
  std::string str_format;
  str_format = "%ld,";              // Timestamp[ns]
  str_format += "%lf,%lf,%lf,%lf,"; // Quaternion
  str_format += "%lf,%lf,%lf";      // Position

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double qw, qx, qy, qz = 0.0;
    double px, py, pz = 0.0;
    int retval =
        fscanf(fp, str_format.c_str(), &ts, &qw, &qx, &qy, &qz, &px, &py, &pz);
    if (retval != 8) {
      FATAL("Failed to parse line in [%s:%d]", fpath.c_str(), i);
    }

    // Record
    timestamps.push_back(ts);
    quat_t q{qw, qx, qy, qz};
    vec3_t r{px, py, pz};
    poses.push_back(tf(q, r));
  }
}

int check_jacobian(const std::string &jac_name,
                   const matx_t &fdiff,
                   const matx_t &jac,
                   const real_t threshold,
                   const bool print) {
  // Pre-check
  if (jac.size() == 0) {
    LOG_ERROR("Provided analytical jacobian is empty!");
    return false;
  } else if (fdiff.size() == 0) {
    LOG_ERROR("Provided numerical jacobian is empty!");
    return false;
  } else if (fdiff.rows() != jac.rows()) {
    LOG_ERROR("rows(fdiff) != rows(jac)");
    return false;
  } else if (fdiff.cols() != jac.cols()) {
    LOG_ERROR("cols(fdiff) != cols(jac)");
    return false;
  }

  // Check if any of the values are beyond the threshold
  const matx_t delta = (fdiff - jac);
  bool failed = false;
  for (long i = 0; i < delta.rows(); i++) {
    for (long j = 0; j < delta.cols(); j++) {
      if (fabs(delta(i, j)) >= threshold) {
        failed = true;
      }
    }
  }

  // Print result
  int retval = 0;
  if (failed) {
    if (print) {
      LOG_ERROR("Check [%s] failed!\n", jac_name.c_str());
      print_matrix("num diff jac", fdiff);
      print_matrix("analytical jac", jac);
      print_matrix("difference matrix", delta);
      // exit(-1);
    }
    retval = -1;

  } else {
    if (print) {
      printf("Check [%s] passed!\n", jac_name.c_str());
    }
    retval = 0;
  }

  return retval;
}

/******************************************************************************
 *                               FILESYSTEM
 *****************************************************************************/

FILE *file_open(const std::string &path,
                const std::string &mode,
                int *nb_rows) {
  FILE *fp = fopen(path.c_str(), mode.c_str());
  if (fp == NULL) {
    return nullptr;
  }

  if (nb_rows != nullptr) {
    *nb_rows = file_rows(path);
  }

  return fp;
}

void skip_line(FILE *fp) {
  char header[BUFSIZ];
  char *retval = fgets(header, BUFSIZ, fp);
  if (retval == NULL) {
    FATAL("Failed to skip line!");
  }
}

int file_rows(const std::string &file_path) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of lines
  int nb_rows = 0;
  std::string line;
  while (std::getline(infile, line)) {
    nb_rows++;
  }

  return nb_rows;
}

int file_copy(const std::string &src, const std::string &dest) {
  // Open input path
  FILE *src_file = fopen(src.c_str(), "rb");
  if (src_file == NULL) {
    fclose(src_file);
    return -1;
  }

  // Open output path
  FILE *dest_file = fopen(dest.c_str(), "wb");
  if (dest_file == NULL) {
    fclose(src_file);
    fclose(dest_file);
    return -2;
  }

  // BUFSIZE default is 8192 bytes
  // BUFSIZE of 1 means one chareter at time
  char buf[BUFSIZ];
  while (size_t size = fread(buf, 1, BUFSIZ, src_file)) {
    fwrite(buf, 1, size, dest_file);
  }

  // Clean up
  fclose(src_file);
  fclose(dest_file);

  return 0;
}

std::string parse_fext(const std::string &path) {
  return path.substr(path.find_last_of("."));
}

std::string parse_fname(const std::string &path) {
  auto output = path;
  const size_t last_slash_idx = output.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    output.erase(0, last_slash_idx + 1);
  }

  return output;
}

bool file_exists(const std::string &path) {
  FILE *file;

  file = fopen(path.c_str(), "r");
  if (file != NULL) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

bool dir_exists(const std::string &path) {
  DIR *dir = opendir(path.c_str());

  if (dir) {
    closedir(dir);
    return true;
  } else if (ENOENT == errno) {
    return false;
  } else {
    LOG_ERROR("dir_exists() failed! %s", strerror(errno));
    exit(-1);
  }
}

int dir_create(const std::string &path) {
  std::string command = "mkdir -p " + path;
  return system(command.c_str());
}

std::string dir_name(const std::string &path) {
  std::size_t found = path.find_last_of("/\\");
  return path.substr(0, found);
}

std::string strip(const std::string &s, const std::string &target) {
  size_t first = s.find_first_not_of(target);
  if (std::string::npos == first) {
    return s;
  }

  size_t last = s.find_last_not_of(target);
  return s.substr(first, (last - first + 1));
}

std::string strip_end(const std::string &s, const std::string &target) {
  size_t last = s.find_last_not_of(target);
  return s.substr(0, last + 1);
}

int create_dir(const std::string &path) {
  const std::string command = "mkdir -p " + path;
  const int retval = system(command.c_str());
  if (retval == -1) {
    printf("Error creating directory!n");
    return -1;
  }

  return 0;
}

int remove_dir(const std::string &path) {
  DIR *dir = opendir(path.c_str());
  struct dirent *next_file;

  // pre-check
  if (dir == NULL) {
    return -1;
  }

  // remove files in path
  while ((next_file = readdir(dir)) != NULL) {
    remove(std::string(path + "/" + next_file->d_name).c_str());
  }

  // remove dir
  remove(path.c_str());
  closedir(dir);

  return 0;
}

std::string remove_ext(const std::string &path) {
  auto output = path;
  const size_t period_idx = output.rfind('.');
  if (std::string::npos != period_idx) {
    output.erase(period_idx);
  }
  return output;
}

int list_dir(const std::string &path, std::vector<std::string> &results) {
  struct dirent *entry;
  DIR *dp;

  // Check directory
  dp = opendir(path.c_str());
  if (dp == NULL) {
    return -1;
  }

  // List directory
  while ((entry = readdir(dp))) {
    std::string value(entry->d_name);
    if (value != "." && value != "..") {
      results.push_back(value);
    }
  }

  // Clean up
  closedir(dp);
  return 0;
}

std::vector<std::string> path_split(const std::string path) {
  std::string s;
  std::vector<std::string> splits;

  s = "";
  for (size_t i = 0; i < path.length(); i++) {
    if (s != "" && path[i] == '/') {
      splits.push_back(s);
      s = "";
    } else if (path[i] != '/') {
      s += path[i];
    }
  }
  splits.push_back(s);

  return splits;
}

std::string paths_join(const std::string path1, const std::string path2) {
  int dirs_up;
  std::string result;
  std::vector<std::string> splits1;
  std::vector<std::string> splits2;

  // setup
  result = "";
  splits1 = path_split(path1);
  splits2 = path_split(path2);

  // obtain number of directory ups in path 2
  dirs_up = 0;
  for (size_t i = 0; i < splits2.size(); i++) {
    if (splits2[i] == "..") {
      dirs_up++;
    }
  }

  // drop path1 elements as path2 dir ups
  for (int i = 0; i < dirs_up; i++) {
    splits1.pop_back();
  }

  // append path1 to result
  if (path1[0] == '/') {
    result += "/";
  }
  for (size_t i = 0; i < splits1.size(); i++) {
    result += splits1[i];
    result += "/";
  }

  // append path2 to result
  for (size_t i = dirs_up; i < splits2.size(); i++) {
    result += splits2[i];
    result += "/";
  }

  // remove trailing slashes
  for (size_t i = result.length() - 1; i > 0; i--) {
    if (result[i] == '/') {
      result.pop_back();
    } else {
      break;
    }
  }

  return result;
}


/******************************************************************************
 *                               CONFIG
 *****************************************************************************/

config_t::config_t() {}

config_t::config_t(const std::string &file_path_) : file_path{file_path_} {
  if (yaml_load_file(file_path_, root) == 0) {
    ok = true;
  }
}

config_t::~config_t() {}

int yaml_load_file(const std::string file_path, YAML::Node &root) {
  // Pre-check
  if (file_exists(file_path) == false) {
    FATAL("File not found: %s", file_path.c_str());
  }

  // Load and parse file
  try {
    root = YAML::LoadFile(file_path);
  } catch (YAML::ParserException &e) {
    LOG_ERROR("%s", e.what());
    return -1;
  }

  return 0;
}

int yaml_get_node(const config_t &config,
                  const std::string &key,
                  const bool optional,
                  YAML::Node &node) {
  assert(config.ok == true);

  // Recurse down config key
  std::vector<YAML::Node> traversal;
  traversal.push_back(config.root);

  std::istringstream iss(key);
  std::string element;

  while (std::getline(iss, element, '.')) {
    traversal.push_back(traversal.back()[element]);
  }
  node = traversal.back();
  // Note:
  //
  //    yaml_node = yaml_node["some_level_deeper"];
  //
  // YAML::Node is mutable, by doing the above it destroys the parsed yaml
  // tree/graph, to avoid this problem we store the visited YAML::Node into
  // a std::vector and return the last visited YAML::Node

  // Check key
  if (node.IsDefined() == false && optional == false) {
    FATAL("Opps [%s] missing in yaml file [%s]!",
          key.c_str(),
          config.file_path.c_str());
    return -1;
  } else if (node.IsDefined() == false && optional == true) {
    return -1;
  }

  return 0;
}

int yaml_has_key(const config_t &config, const std::string &key) {
  assert(config.ok == true);

  // Recurse down config key
  std::vector<YAML::Node> traversal;
  traversal.push_back(config.root);

  std::istringstream iss(key);
  std::string element;

  while (std::getline(iss, element, '.')) {
    traversal.push_back(traversal.back()[element]);
  }
  auto node = traversal.back();
  // Note:
  //
  //    yaml_node = yaml_node["some_level_deeper"];
  //
  // YAML::Node is mutable, by doing the above it destroys the parsed yaml
  // tree/graph, to avoid this problem we store the visited YAML::Node into
  // a std::vector and return the last visited YAML::Node

  // Check key
  if (node.IsDefined() == false) {
    return 0;
  }

  return 1;
}

int yaml_has_key(const std::string &file_path, const std::string &key) {
  const config_t config{file_path};
  return yaml_has_key(config, key);
}

void yaml_check_matrix_fields(const YAML::Node &node,
                              const std::string &key,
                              size_t &rows,
                              size_t &cols) {
  const std::string targets[3] = {"rows", "cols", "data"};
  for (int i = 0; i < 3; i++) {
    if (!node[targets[i]]) {
      FATAL("Key [%s] is missing for matrix [%s]!",
            targets[i].c_str(),
            key.c_str());
    }
  }
  rows = node["rows"].as<int>();
  cols = node["cols"].as<int>();
}

int parse(const config_t &config,
          const std::string &key,
          vec2_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec2_t>(node, key, optional);
  vec = vec2_t{node[0].as<real_t>(), node[1].as<real_t>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec3_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec3_t>(node, key, optional);
  vec =
      vec3_t{node[0].as<real_t>(), node[1].as<real_t>(), node[2].as<real_t>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec4_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec4_t>(node, key, optional);
  vec = vec4_t{node[0].as<real_t>(),
               node[1].as<real_t>(),
               node[2].as<real_t>(),
               node[3].as<real_t>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vecx_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  const size_t vector_size = yaml_check_vector<vecx_t>(node, key, optional);
  vec = vecx_t::Zero(vector_size, 1);
  for (size_t i = 0; i < node.size(); i++) {
    vec(i) = node[i].as<real_t>();
  }
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          mat2_t &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_matrix<mat2_t>(node, key, optional);
  mat(0, 0) = node["data"][0].as<real_t>();
  mat(0, 1) = node["data"][1].as<real_t>();
  mat(1, 0) = node["data"][2].as<real_t>();
  mat(1, 1) = node["data"][3].as<real_t>();
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          mat3_t &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_matrix<mat3_t>(node, key, optional);
  // -- Col 1
  mat(0, 0) = node["data"][0].as<real_t>();
  mat(0, 1) = node["data"][1].as<real_t>();
  mat(0, 2) = node["data"][2].as<real_t>();
  // -- Col 2
  mat(1, 0) = node["data"][3].as<real_t>();
  mat(1, 1) = node["data"][4].as<real_t>();
  mat(1, 2) = node["data"][5].as<real_t>();
  // -- Col 3
  mat(2, 0) = node["data"][6].as<real_t>();
  mat(2, 1) = node["data"][7].as<real_t>();
  mat(2, 2) = node["data"][8].as<real_t>();
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          mat4_t &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_matrix<mat4_t>(node, key, optional);
  size_t index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      mat(i, j) = node["data"][index].as<real_t>();
      index++;
    }
  }

  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          matx_t &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  size_t rows = 0;
  size_t cols = 0;
  yaml_check_matrix<matx_t>(node, key, optional, rows, cols);

  mat.resize(rows, cols);
  size_t index = 0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      mat(i, j) = node["data"][index].as<real_t>();
      index++;
    }
  }

  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          cv::Mat &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  size_t rows = 0;
  size_t cols = 0;
  yaml_check_matrix<cv::Mat>(node, key, optional, rows, cols);

  mat = cv::Mat(rows, cols, CV_64F);
  size_t index = 0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      mat.at<real_t>(i, j) = node["data"][index].as<real_t>();
      index++;
    }
  }

  return 0;
}

/******************************************************************************
 *                                ALGEBRA
 *****************************************************************************/

int sign(const real_t x) {
  if (fltcmp(x, 0.0) == 0) {
    return 0;
  } else if (x < 0) {
    return -1;
  }
  return 1;
}

int fltcmp(const real_t f1, const real_t f2) {
  if (fabs(f1 - f2) <= 0.0001) {
    return 0;
  } else if (f1 > f2) {
    return 1;
  } else {
    return -1;
  }
}

real_t binomial(const real_t n, const real_t k) {
  if (k == 0 || k == n) {
    return 1.0;
  }

  return binomial(n - 1, k - 1) + binomial(n - 1, k);
}

/******************************************************************************
 *                            LINEAR ALGEBRA
 *****************************************************************************/

void print_shape(const std::string &name, const matx_t &A) {
  std::cout << name << ": " << A.rows() << "x" << A.cols() << std::endl;
}

void print_shape(const std::string &name, const vecx_t &v) {
  std::cout << name << ": " << v.rows() << "x" << v.cols() << std::endl;
}

void print_array(const std::string &name,
                 const real_t *array,
                 const size_t size) {
  std::cout << name << std::endl;
  for (size_t i = 0; i < size; i++) {
    printf("%.4f ", array[i]);
  }
  printf("\b\n");
}

void print_vector(const std::string &name, const vecx_t &v) {
  printf("%s: ", name.c_str());
  for (long i = 0; i < v.size(); i++) {
    printf("%f", v(i));
    if ((i + 1) != v.size()) {
      printf(", ");
    }
  }
  printf("\n");
}

void print_matrix(const std::string &name, const matx_t &m) {
  printf("%s:\n", name.c_str());
  for (long i = 0; i < m.rows(); i++) {
    for (long j = 0; j < m.cols(); j++) {
      printf("%f", m(i, j));
      if ((j + 1) != m.cols()) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void print_quaternion(const std::string &name, const quat_t &q) {
  printf("%s: ", name.c_str());
  printf("w:%f, x:%f, y:%f, z:%f\n", q.w(), q.x(), q.y(), q.z());
}

std::string array2str(const real_t *array, const size_t size) {
  std::stringstream os;
  for (size_t i = 0; i < (size - 1); i++) {
    os << array[i] << " ";
  }
  os << array[size - 1];

  return os.str();
}

void array2vec(const real_t *x, const size_t size, vecx_t y) {
  y.resize(size);
  for (size_t i = 0; i < size; i++) {
    y(i) = x[i];
  }
}

real_t *vec2array(const vecx_t &v) {
  real_t *array = (real_t *) malloc(sizeof(real_t) * v.size());
  for (int i = 0; i < v.size(); i++) {
    array[i] = v(i);
  }
  return array;
}

real_t *mat2array(const matx_t &m) {
  real_t *array = (real_t *) malloc(sizeof(real_t) * m.size());

  int index = 0;
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < m.cols(); j++) {
      array[index] = m(i, j);
      index++;
    }
  }
  return array;
}

real_t *quat2array(const quat_t &q) {
  real_t *array = (real_t *) malloc(sizeof(real_t) * 4);

  array[0] = q.x();
  array[1] = q.y();
  array[2] = q.z();
  array[3] = q.w();

  return array;
}

void vec2array(const vecx_t &v, real_t *out) {
  for (int i = 0; i < v.size(); i++) {
    out[i] = v(i);
  }
}

void mat2array(const matx_t &A, real_t *out) {
  int index = 0;
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      out[index] = A(i, j);
      index++;
    }
  }
}

std::vector<vecx_t> mat2vec(const matx_t &m, bool row_wise) {
  std::vector<vecx_t> vectors;

  if (row_wise) {
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

vec3s_t mat2vec3(const matx_t &m, bool row_wise) {
  vec3s_t vectors;

  if (row_wise) {
    assert(m.cols() == 3);
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    assert(m.rows() == 3);
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

vec2s_t mat2vec2(const matx_t &m, bool row_wise) {
  vec2s_t vectors;

  if (row_wise) {
    assert(m.cols() == 2);
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    assert(m.rows() == 2);
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

matx_t vecs2mat(const vec3s_t &vs) {
  matx_t retval;
  retval.resize(4, vs.size());

  int idx = 0;
  for (const auto &v : vs) {
    const real_t x = v(0);
    const real_t y = v(1);
    const real_t z = v(2);
    retval.block(0, idx, 4, 1) = vec4_t{x, y, z, 1.0};
    idx++;
  }

  return retval;
}

std::string vec2str(const vecx_t &v, bool brackets) {
  std::string str;

  if (brackets) {
    str += "[";
  }

  for (int i = 0; i < v.size(); i++) {
    str += std::to_string(v(i));
    if ((i + 1) != v.size()) {
      str += ", ";
    }
  }

  if (brackets) {
    str += "]";
  }

  return str;
}

std::string arr2str(const real_t *arr, const size_t len, bool brackets) {
  std::string str;

  if (brackets) {
    str += "[";
  }

  for (size_t i = 0; i < len; i++) {
    str += std::to_string(arr[i]);
    if ((i + 1) != len) {
      str += ", ";
    }
  }

  if (brackets) {
    str += "]";
  }

  return str;
}

std::string mat2str(const matx_t &m, const std::string &indent) {
  std::string str;

  for (int i = 0; i < m.rows(); i++) {
    if ((i + 1) != m.rows()) {
      str += indent;
      str += vec2str(m.row(i), false) + ",\n";
    } else {
      str += indent;
      str += vec2str(m.row(i), false);
    }
  }

  return str;
}

vec3_t normalize(const vec3_t &v) { return v / v.norm(); }

real_t cond(const matx_t &A) {
  Eigen::JacobiSVD<matx_t> svd(A);
  const auto max_sigma = svd.singularValues()(0);
  const auto min_sigma = svd.singularValues()(svd.singularValues().size() - 1);
  return max_sigma / min_sigma;
}

matx_t zeros(const int rows, const int cols) {
  return matx_t::Zero(rows, cols);
}

matx_t zeros(const int size) { return matx_t::Zero(size, size); }

matx_t I(const int rows, const int cols) {
  return matx_t::Identity(rows, cols);
}

matx_t I(const int size) { return matx_t::Identity(size, size); }

matx_t ones(const int rows, const int cols) {
  matx_t A{rows, cols};
  A.fill(1.0);
  return A;
}

matx_t ones(const int size) { return ones(size, size); }

matx_t hstack(const matx_t &A, const matx_t &B) {
  matx_t C(A.rows(), A.cols() + B.cols());
  C << A, B;
  return C;
}

matx_t vstack(const matx_t &A, const matx_t &B) {
  matx_t C(A.rows() + B.rows(), A.cols());
  C << A, B;
  return C;
}

matx_t dstack(const matx_t &A, const matx_t &B) {
  matx_t C = zeros(A.rows() + B.rows(), A.cols() + B.cols());
  C.block(0, 0, A.rows(), A.cols()) = A;
  C.block(A.rows(), A.cols(), B.rows(), B.cols()) = B;
  return C;
}

mat3_t skew(const vec3_t &w) {
  mat3_t S;
  // clang-format off
  S << 0.0, -w(2), w(1),
       w(2), 0.0, -w(0),
       -w(1), w(0), 0.0;
  // clang-format on
  return S;
}

mat3_t skewsq(const vec3_t &w) {
  mat3_t SS = (w * w.transpose()) - pow(w.norm(), 2) * I(3);
  return SS;
}

matx_t enforce_psd(const matx_t &A) {
  matx_t A_psd;

  A_psd.resize(A.rows(), A.cols());

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (i == j) {
        A_psd(i, j) = std::fabs(A(i, j));
      } else {
        const real_t x = 0.5 * (A(i, j) + A(j, i));
        A_psd(i, j) = x;
        A_psd(j, i) = x;
      }
    }
  }

  return A_psd;
}

matx_t nullspace(const matx_t &A) {
  Eigen::FullPivLU<matx_t> lu(A);
  matx_t A_null_space = lu.kernel();
  return A_null_space;
}

bool equals(const matx_t &A, const matx_t &B) {
  if (A.rows() != B.rows()) {
    return false;
  }

  if (A.cols() != B.cols()) {
    return false;
  }

  for (long i = 0; i < A.rows(); i++) {
    for (long j = 0; j < A.cols(); j++) {
      if (fltcmp(A(i, j), B(i, j)) != 0) {
        return false;
      }
    }
  }

  return true;
}

void load_matrix(const std::vector<real_t> &x,
                 const int rows,
                 const int cols,
                 matx_t &y) {
  int idx;

  // Setup
  idx = 0;
  y.resize(rows, cols);

  // Load matrix
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      y(j, i) = x[idx];
      idx++;
    }
  }
}

void load_matrix(const matx_t &A, std::vector<real_t> &x) {
  for (int i = 0; i < A.cols(); i++) {
    for (int j = 0; j < A.rows(); j++) {
      x.push_back(A(j, i));
    }
  }
}

matx_t pinv(const matx_t &A, const real_t tol) {
  auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &vals_ = svd.singularValues();
  matx_t vals_inv = zeros(A.cols(), A.rows());

  for (unsigned int i = 0; i < vals_.size(); ++i) {
    if (vals_(i) > tol) {
      vals_inv(i, i) = ((real_t) 1.0) / vals_(i);
    } else {
      vals_inv(i, i) = ((real_t) 0);
    }
  }

  return svd.matrixV() * vals_inv * svd.matrixU().adjoint();
}

long int rank(const matx_t &A) {
  Eigen::FullPivLU<matx_t> lu_decomp(A);
  return lu_decomp.rank();
}

int schurs_complement(matx_t &H, vecx_t &b,
                      const size_t m, const size_t r,
                      const bool precond, const bool debug) {
  assert(m > 0 && r > 0);
  assert(H.isZero() == false);

  // Setup
  const long size = m + r;
  matx_t H_marg = zeros(size, size);
  vecx_t b_marg = zeros(size, 1);

  // Precondition Hmm
  matx_t Hmm = H.block(0, 0, m, m);
  if (precond) {
    Hmm = 0.5 * (Hmm + Hmm.transpose());
  }

  // Pseudo inverse of Hmm via Eigen-decomposition:
  //
  //   A_pinv = V * Lambda_pinv * V_transpose
  //
  // Where Lambda_pinv is formed by **replacing every non-zero diagonal entry
  // by its reciprocal, leaving the zeros in place, and transposing the
  // resulting matrix.**
  const double eps = 1.0e-8;
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
  const matx_t V = eig.eigenvectors();
  const auto eigvals = eig.eigenvalues().array();
  const auto eigvals_inv = (eigvals > eps).select(eigvals.inverse(), 0);
  const matx_t Lambda_inv = vecx_t(eigvals_inv).asDiagonal();
  const matx_t Hmm_inv = V * Lambda_inv * V.transpose();
  // const matx_t Hmm_inv = pinv(Hmm);
  // const matx_t Hmm_inv = Hmm.inverse();
  const double inv_check = ((Hmm * Hmm_inv) - I(m, m)).sum();
  if (fabs(inv_check) > 1e-4) {
    LOG_ERROR("FAILED!: Inverse identity check: %f", inv_check);
    // print_matrix("Hmm", Hmm);
    // print_matrix("bmm", b.segment(0, m));
    // mat2csv("/tmp/H.csv", H);
    // mat2csv("/tmp/Hmm.csv", Hmm);
    // exit(0);
    return -1;
  }

  // Calculate Schur's complement
  const matx_t Hmr = H.block(0, m, m, r);
  const matx_t Hrm = H.block(m, 0, r, m);
  const matx_t Hrr = H.block(m, m, r, r);
  const vecx_t bmm = b.segment(0, m);
  const vecx_t brr = b.segment(m, r);
  H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  b_marg = brr - Hrm * Hmm_inv * bmm;

  if (debug) {
    mat2csv("/tmp/H.csv", H);
    mat2csv("/tmp/Hmm.csv", Hmm);
    mat2csv("/tmp/Hmr.csv", Hmr);
    mat2csv("/tmp/Hrm.csv", Hrm);
    mat2csv("/tmp/Hrr.csv", Hrr);
    mat2csv("/tmp/bmm.csv", bmm);
    mat2csv("/tmp/brr.csv", brr);
    mat2csv("/tmp/H_marg.csv", H_marg);
    mat2csv("/tmp/b_marg.csv", b_marg);
  }

  H = H_marg;
  b = b_marg;
  return 0;
}

/******************************************************************************
 *                                GEOMETRY
 *****************************************************************************/

real_t sinc(const real_t x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    static const real_t c_2 = 1.0 / 6.0;
    static const real_t c_4 = 1.0 / 120.0;
    static const real_t c_6 = 1.0 / 5040.0;
    const real_t x_2 = x * x;
    const real_t x_4 = x_2 * x_2;
    const real_t x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

vec3_t deg2rad(const vec3_t d) { return d * (M_PI / 180.0); }

real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

vec3_t rad2deg(const vec3_t &r) { return r * (180.0 / M_PI); }

real_t wrap180(const real_t euler_angle) {
  return fmod((euler_angle + 180.0), 360.0) - 180.0;
}

real_t wrap360(const real_t euler_angle) {
  if (euler_angle > 0) {
    return fmod(euler_angle, 360.0);
  } else {
    return fmod(euler_angle + 360, 360.0);
  }
}

real_t wrapPi(const real_t r) { return deg2rad(wrap180(rad2deg(r))); }

real_t wrap2Pi(const real_t r) { return deg2rad(wrap360(rad2deg(r))); }

vec2_t circle(const real_t r, const real_t theta) {
  return vec2_t{r * cos(theta), r * sin(theta)};
}

vec3_t sphere(const real_t rho, const real_t theta, const real_t phi) {
  const real_t x = rho * sin(theta) * cos(phi);
  const real_t y = rho * sin(theta) * sin(phi);
  const real_t z = rho * cos(theta);
  return vec3_t{x, y, z};
}

mat4_t lookat(const vec3_t &cam_pos,
              const vec3_t &target,
              const vec3_t &up_axis) {
  // Note: If we were using OpenGL the cam_dir would be the opposite direction,
  // since in OpenGL the camera forward is -z. In robotics however our camera
  // is +z forward.
  const vec3_t cam_dir = (target - cam_pos).normalized();
  const vec3_t cam_right = (up_axis.cross(cam_dir)).normalized();
  const vec3_t cam_up = cam_dir.cross(cam_right);

  // clang-format off
  mat4_t A;
  A << cam_right(0), cam_right(1), cam_right(2), 0.0,
       cam_up(0), cam_up(1), cam_up(2), 0.0,
       cam_dir(0), cam_dir(1), cam_dir(2), 0.0,
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // clang-format off
  mat4_t B;
  B << 1.0, 0.0, 0.0, -cam_pos(0),
       0.0, 1.0, 0.0, -cam_pos(1),
       0.0, 0.0, 1.0, -cam_pos(2),
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  mat4_t T_camera_target = A * B;
  mat4_t T_target_camera = T_camera_target.inverse();
  return T_target_camera;
}

real_t cross_track_error(const vec2_t &p1,
                         const vec2_t &p2,
                         const vec2_t &pos) {
  const real_t x0 = pos(0);
  const real_t y0 = pos(1);

  const real_t x1 = p1(0);
  const real_t y1 = p1(0);

  const real_t x2 = p2(0);
  const real_t y2 = p2(0);

  // calculate perpendicular distance between line (p1, p2) and point (pos)
  const real_t n = ((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
  const real_t d = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));

  return fabs(n) / d;
}

int point_left_right(const vec2_t &a, const vec2_t &b, const vec2_t &c) {
  const real_t a0 = a(0);
  const real_t a1 = a(1);
  const real_t b0 = b(0);
  const real_t b1 = b(1);
  const real_t c0 = c(0);
  const real_t c1 = c(1);
  const real_t x = (b0 - a0) * (c1 - a1) - (b1 - a1) * (c0 - a0);

  if (x > 0) {
    return 1; // left
  } else if (x < 0) {
    return 2; // right
  } else if (x == 0) {
    return 0; // parallel
  }

  return -1;
}

real_t closest_point(const vec2_t &a,
                     const vec2_t &b,
                     const vec2_t &p,
                     vec2_t &closest) {
  // pre-check
  if ((a - b).norm() == 0) {
    closest = a;
    return -1;
  }

  // calculate closest point
  const vec2_t v1 = p - a;
  const vec2_t v2 = b - a;
  const real_t t = v1.dot(v2) / v2.squaredNorm();
  closest = a + t * v2;

  return t;
}

void latlon_offset(real_t lat_ref,
                   real_t lon_ref,
                   real_t offset_N,
                   real_t offset_E,
                   real_t *lat_new,
                   real_t *lon_new) {
  *lat_new = lat_ref + (offset_E / EARTH_RADIUS_M);
  *lon_new = lon_ref + (offset_N / EARTH_RADIUS_M) / cos(deg2rad(lat_ref));
}

void latlon_diff(real_t lat_ref,
                 real_t lon_ref,
                 real_t lat,
                 real_t lon,
                 real_t *dist_N,
                 real_t *dist_E) {
  real_t d_lon = lon - lon_ref;
  real_t d_lat = lat - lat_ref;

  *dist_N = deg2rad(d_lat) * EARTH_RADIUS_M;
  *dist_E = deg2rad(d_lon) * EARTH_RADIUS_M * cos(deg2rad(lat));
}

real_t latlon_dist(real_t lat_ref, real_t lon_ref, real_t lat, real_t lon) {
  real_t dist_N = 0.0;
  real_t dist_E = 0.0;

  latlon_diff(lat_ref, lon_ref, lat, lon, &dist_N, &dist_E);
  real_t dist = sqrt(pow(dist_N, 2) + pow(dist_E, 2));

  return dist;
}

/*****************************************************************************
 *                         DIFFERENTIAL GEOMETRY
 *****************************************************************************/

namespace lie {

mat3_t Exp(const vec3_t &phi) {
  const real_t norm = phi.norm();

  // Small angle approx
  if (norm < 1e-3) {
    return mat3_t{I(3) + skew(phi)};
  }

  // Exponential map from so(3) to SO(3)
  const mat3_t phi_skew = skew(phi);
  mat3_t C = I(3);
  C += (sin(norm) / norm) * phi_skew;
  C += ((1 - cos(norm)) / (norm * norm)) * (phi_skew * phi_skew);

  return C;
}

// vec3_t Log(const mat3_t &C) {
//   const auto phi = acos(C.trace() - 1 / 2);
//   return phi * (C * C.transpose()) / (2 * sin(phi));
// }

mat3_t Jr(const vec3_t &psi) {
  const real_t psi_norm = psi.norm();
  const real_t psi_norm_sq = psi_norm * psi_norm;
  const real_t psi_norm_cube = psi_norm_sq * psi_norm;
  const mat3_t psi_skew = skew(psi);
  const mat3_t psi_skew_sq = psi_skew * psi_skew;

  mat3_t J = I(3);
  J -= ((1 - cos(psi_norm)) / psi_norm_sq) * psi_skew;
  J += (psi_norm - sin(psi_norm)) / (psi_norm_cube) * psi_skew_sq;
  return J;
}

} // namespace lie

/******************************************************************************
 *                               STATISTICS
 *****************************************************************************/

int randi(int ub, int lb) { return rand() % lb + ub; }

real_t randf(const real_t ub, const real_t lb) {
  const real_t f = (real_t) rand() / RAND_MAX;
  return lb + f * (ub - lb);
}

real_t sum(const std::vector<real_t> &x) {
  double sum = 0.0;

  for (const auto &x_i : x) {
    sum += x_i;
  }

  return sum;
}

real_t median(const std::vector<real_t> &v) {
  // sort values
  std::vector<real_t> v_copy = v;
  std::sort(v_copy.begin(), v_copy.end());

  // obtain median
  if (v_copy.size() % 2 == 1) {
    // return middle value
    return v_copy[v_copy.size() / 2];

  } else {
    // grab middle two values and calc mean
    const real_t a = v_copy[v_copy.size() / 2];
    const real_t b = v_copy[(v_copy.size() / 2) - 1];
    return (a + b) / 2.0;
  }
}

real_t mean(const std::vector<real_t> &x) {
  real_t sum = 0.0;
  for (const auto i : x) {
    sum += i;
  }

  const real_t N = x.size();
  return sum / N;
}

vec3_t mean(const vec3s_t &x) {
  vec3_t x_hat{0.0, 0.0, 0.0};

  for (const auto &v : x) {
    x_hat += v;
  }
  x_hat *= 1.0f / x.size();

  return x_hat;
}

real_t var(const std::vector<real_t> &x) {
  const double mu = mean(x);
  const double N = x.size();

  double sum = 0.0;
  for (const auto x_i : x) {
    sum += pow(x_i - mu, 2);
  }

  return sum / (N - 1.0);
}

vec3_t var(const vec3s_t &vecs) {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;

  for (const vec3_t &v : vecs) {
    x.push_back(v(0));
    y.push_back(v(1));
    z.push_back(v(2));
  }

  return vec3_t{var(x), var(y), var(z)};
}

real_t stddev(const std::vector<real_t> &x) {
  return sqrt(var(x));
}

real_t rmse(const std::vector<real_t> &residuals) {
  real_t sse = 0.0;
  for (const auto r : residuals) {
    sse += r * r;
  }

  real_t n = residuals.size();
  real_t mse = sse / n;
  return sqrt(mse);
}

real_t shannon_entropy(const matx_t &covar) {
  const real_t n = covar.rows();
  const real_t covar_det = covar.determinant();
  const real_t entropy = 0.5 * log(pow(2 * M_PI * exp(1), n) * covar_det);
  return entropy;
}

vec3_t mvn(std::default_random_engine &engine,
           const vec3_t &mu,
           const vec3_t &stdev) {
  std::normal_distribution<real_t> normal_x(mu(0), stdev(0));
  std::normal_distribution<real_t> normal_y(mu(1), stdev(1));
  std::normal_distribution<real_t> normal_z(mu(2), stdev(2));
  return vec3_t{normal_x(engine), normal_y(engine), normal_z(engine)};
}

real_t gauss_normal() {
  static real_t V1, V2, S;
  static int phase = 0;
  real_t X;

  if (phase == 0) {
    do {
      real_t U1 = (real_t) rand() / RAND_MAX;
      real_t U2 = (real_t) rand() / RAND_MAX;

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    } while (S >= 1 || S == 0);

    X = V1 * sqrt(-2 * log(S) / S);
  } else {
    X = V2 * sqrt(-2 * log(S) / S);
  }

  phase = 1 - phase;
  return X;
}

/*****************************************************************************
 *                               TRANSFORM
 *****************************************************************************/

mat4_t tf(const double *params) {
  const vec3_t r{params[0], params[1], params[2]};
  const quat_t q{params[6], params[3], params[4], params[5]};
  return tf(q, r);
}

mat4_t tf(const vecx_t &params) {
  assert(params.size() == 7);
  const vec3_t r{params[0], params[1], params[2]};
  const quat_t q{params[6], params[3], params[4], params[5]};
  return tf(q, r);
}

mat4_t tf(const mat3_t &C, const vec3_t &r) {
  mat4_t T = I(4);
  T.block(0, 0, 3, 3) = C;
  T.block(0, 3, 3, 1) = r;
  return T;
}

mat4_t tf(const quat_t &q, const vec3_t &r) {
  return tf(q.toRotationMatrix(), r);
}

mat4_t tf_perturb_rot(const mat4_t &T, real_t step_size, const int i) {
  const mat3_t C = tf_rot(T);
  const vec3_t r = tf_trans(T);

  mat3_t C_diff;
  if (i == -1) {
    C_diff = rvec2rot(ones(3, 1) * step_size, 1e-8) * C;
  } else {
    const mat3_t drvec = I(3) * step_size;
    C_diff = rvec2rot(drvec.col(i), 1e-8) * C;
  }
  return tf(C_diff, r);
}

mat4_t tf_perturb_trans(const mat4_t &T, const real_t step_size, const int i) {
  const mat3_t C = tf_rot(T);
  const vec3_t r = tf_trans(T);

  vec3_t r_diff;
  if (i == -1) {
    r_diff = r + ones(3, 1) * step_size;
  } else {
    const mat3_t dr = I(3) * step_size;
    r_diff = r + dr.col(i);
  }
  return tf(C, r_diff);
}

vec3_t tf_point(const mat4_t &T, const vec3_t &p) {
  return (T * p.homogeneous()).head(3);
}

mat3_t rotx(const real_t theta) {
  mat3_t R;

  // clang-format off
  R << 1.0, 0.0, 0.0,
       0.0, cos(theta), sin(theta),
       0.0, -sin(theta), cos(theta);
  // clang-format on

  return R;
}

mat3_t roty(const real_t theta) {
  mat3_t R;

  // clang-format off
  R << cos(theta), 0.0, -sin(theta),
       0.0, 1.0, 0.0,
       sin(theta), 0.0, cos(theta);
  // clang-format on

  return R;
}

mat3_t rotz(const real_t theta) {
  mat3_t R;

  // clang-format off
  R << cos(theta), sin(theta), 0.0,
       -sin(theta), cos(theta), 0.0,
       0.0, 0.0, 1.0;
  // clang-format on

  return R;
}

mat3_t euler123(const vec3_t &euler) {
  // i.e. XYZ rotation sequence (body to world)
  const real_t phi = euler(0);
  const real_t theta = euler(1);
  const real_t psi = euler(2);

  const real_t R11 = cos(psi) * cos(theta);
  const real_t R21 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const real_t R31 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);

  const real_t R12 = sin(psi) * cos(theta);
  const real_t R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const real_t R32 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);

  const real_t R13 = -sin(theta);
  const real_t R23 = cos(theta) * sin(phi);
  const real_t R33 = cos(theta) * cos(phi);

  mat3_t R;
  // clang-format off
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

mat3_t euler321(const vec3_t &euler) {
  // i.e. ZYX rotation sequence (world to body)
  const real_t phi = euler(0);
  const real_t theta = euler(1);
  const real_t psi = euler(2);

  const real_t R11 = cos(psi) * cos(theta);
  const real_t R21 = sin(psi) * cos(theta);
  const real_t R31 = -sin(theta);

  const real_t R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const real_t R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const real_t R32 = cos(theta) * sin(phi);

  const real_t R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  const real_t R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  const real_t R33 = cos(theta) * cos(phi);

  mat3_t R;
  // clang-format off
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

quat_t euler2quat(const vec3_t &euler) {
  const real_t phi = euler(1);
  const real_t theta = euler(2);
  const real_t psi = euler(3);

  const real_t c_phi = cos(phi / 2.0);
  const real_t c_theta = cos(theta / 2.0);
  const real_t c_psi = cos(psi / 2.0);
  const real_t s_phi = sin(phi / 2.0);
  const real_t s_theta = sin(theta / 2.0);
  const real_t s_psi = sin(psi / 2.0);

  const real_t qx = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
  const real_t qy = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
  const real_t qz = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
  const real_t qw = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;

  const real_t mag = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  return quat_t{qw / mag, qx / mag, qy / mag, qz / mag};
}

mat3_t vecs2rot(const vec3_t &a_B, const vec3_t &g) {
  // Create Quaternion from two vectors
  const real_t cos_theta = a_B.normalized().transpose() * g.normalized();
  const real_t half_cos = sqrt(0.5 * (1.0 + cos_theta));
  const real_t half_sin = sqrt(0.5 * (1.0 - cos_theta));
  const vec3_t w = a_B.cross(g).normalized();

  const real_t qw = half_cos;
  const real_t qx = half_sin * w(0);
  const real_t qy = half_sin * w(1);
  const real_t qz = half_sin * w(2);

  // Convert Quaternion to rotation matrix
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

  const real_t R11 = qw2 + qx2 - qy2 - qz2;
  const real_t R12 = 2 * (qx * qy - qw * qz);
  const real_t R13 = 2 * (qx * qz + qw * qy);

  const real_t R21 = 2 * (qx * qy + qw * qz);
  const real_t R22 = qw2 - qx2 + qy2 - qz2;
  const real_t R23 = 2 * (qy * qz - qw * qx);

  const real_t R31 = 2 * (qx * qz - qw * qy);
  const real_t R32 = 2 * (qy * qz + qw * qx);
  const real_t R33 = qw2 - qx2 - qy2 + qz2;

  mat3_t R;
  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;
  return R;
}

mat3_t rvec2rot(const vec3_t &rvec, const real_t eps) {
  // Magnitude of rvec
  const real_t theta = sqrt(rvec.transpose() * rvec);
  // ^ basically norm(rvec), but faster

  // Check if rotation is too small
  if (theta < eps) {
    // clang-format off
    mat3_t R;
    R << 1, -rvec(2), rvec(1),
         rvec(2), 1, -rvec(0),
         -rvec(1), rvec(0), 1;
    return R;
    // clang-format on
  }

  // Convert rvec to rotation matrix
  const vec3_t rvec_normalized = rvec / theta;
  const real_t x = rvec_normalized(0);
  const real_t y = rvec_normalized(1);
  const real_t z = rvec_normalized(2);

  const real_t c = cos(theta);
  const real_t s = sin(theta);
  const real_t C = 1 - c;

  const real_t xs = x * s;
  const real_t ys = y * s;
  const real_t zs = z * s;

  const real_t xC = x * C;
  const real_t yC = y * C;
  const real_t zC = z * C;

  const real_t xyC = x * yC;
  const real_t yzC = y * zC;
  const real_t zxC = z * xC;

  // clang-format off
  mat3_t R;
  R << x * xC + c, xyC - zs, zxC + ys,
       xyC + zs, y * yC + c, yzC - xs,
       zxC - ys, yzC + xs, z * zC + c;
  return R;
  // clang-format on
}

vec3_t quat2euler(const quat_t &q) {
  const real_t qw = q.w();
  const real_t qx = q.x();
  const real_t qy = q.y();
  const real_t qz = q.z();

  const real_t qw2 = qw * qw;
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;

  const real_t t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const real_t t2 = asin(2 * (qy * qw - qx * qz));
  const real_t t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  return vec3_t{t1, t2, t3};
}

mat3_t quat2rot(const quat_t &q) {
  const real_t qw = q.w();
  const real_t qx = q.x();
  const real_t qy = q.y();
  const real_t qz = q.z();

  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

  // Homogeneous form
  mat3_t C;
  // -- 1st row
  C(0, 0) = qw2 + qx2 - qy2 - qz2;
  C(0, 1) = 2 * (qx * qy - qw * qz);
  C(0, 2) = 2 * (qx * qz + qw * qy);
  // -- 2nd row
  C(1, 3) = 2 * (qx * qy + qw * qz);
  C(1, 4) = qw2 - qx2 + qy2 - qz2;
  C(1, 5) = 2 * (qy * qz - qw * qx);
  // -- 3rd row
  C(2, 6) = 2 * (qx * qz - qw * qy);
  C(2, 7) = 2 * (qy * qz + qw * qx);
  C(2, 8) = qw2 - qx2 - qy2 + qz2;

  return C;
}

quat_t quat_delta(const vec3_t &dalpha) {
  const real_t half_norm = 0.5 * dalpha.norm();
  const vec3_t vector = sinc(half_norm) * 0.5 * dalpha;
  const real_t scalar = cos(half_norm);
  return quat_t{scalar, vector(0), vector(1), vector(2)};
}

mat4_t quat_lmul(const quat_t &q) {
  const real_t qw = q.w();
  const real_t qx = q.x();
  const real_t qy = q.y();
  const real_t qz = q.z();
  mat4_t lmul;
  // clang-format off
  lmul << qw, -qx, -qy, -qz,
          qx,  qw, -qz,  qy,
          qy,  qz,  qw, -qx,
          qz, -qy,  qx,  qw;
  // clang-format on
  return lmul;
}

mat3_t quat_lmul_xyz(const quat_t &q) {
  mat4_t Q = quat_lmul(q);
  return Q.bottomRightCorner<3, 3>();
}

mat4_t quat_rmul(const quat_t &q) {
  const real_t qw = q.w();
  const real_t qx = q.x();
  const real_t qy = q.y();
  const real_t qz = q.z();
  mat4_t lmul;
  // clang-format off
  lmul << qw, -qx, -qy, -qz,
          qx,  qw,  qz, -qy,
          qy, -qz,  qw,  qx,
          qz,  qy, -qx,  qw;
  // clang-format on
  return lmul;
}

mat3_t quat_rmul_xyz(const quat_t &q) {
  mat4_t Q = quat_rmul(q);
  return Q.bottomRightCorner<3, 3>();
}

mat3_t quat_mat_xyz(const mat4_t &Q) {
  return Q.bottomRightCorner<3, 3>();
}

mat3_t add_noise(const mat3_t &rot, const real_t n) {
  const vec3_t rpy_n{randf(-n, n), randf(-n, n), randf(-n, n)};
  const vec3_t rpy = quat2euler(quat_t{rot}) + deg2rad(rpy_n);
  return euler321(rpy);
}

vec3_t add_noise(const vec3_t &pos, const real_t n) {
  const vec3_t pos_n{randf(-n, n), randf(-n, n), randf(-n, n)};
  return pos + pos_n;
}

mat4_t add_noise(const mat4_t &pose, const real_t pos_n, const real_t rot_n) {
  vec3_t pos = add_noise(tf_trans(pose), pos_n);
  mat3_t rot = add_noise(tf_rot(pose), rot_n);
  return tf(rot, pos);
}

void imu_init_attitude(const vec3s_t w_m,
                       const vec3s_t a_m,
                       mat3_t &C_WS,
                       const size_t buffer_size) {
  // Sample IMU measurements
  vec3_t sum_angular_vel = vec3_t::Zero();
  vec3_t sum_linear_acc = vec3_t::Zero();
  for (size_t i = 0; i < buffer_size; i++) {
    sum_angular_vel += w_m[i];
    sum_linear_acc += a_m[i];
  }

  // Initialize the initial orientation, so that the estimation
  // is consistent with the inertial frame.
  const vec3_t mean_accel = sum_linear_acc / buffer_size;
  const vec3_t gravity{0.0, 0.0, -9.81};
  C_WS = vecs2rot(mean_accel, -gravity);

  // // Extract roll, pitch and set yaw to 0
  // const quat_t q_WS = quat_t(C_WS);
  // const vec3_t rpy = quat2euler(q_WS);
  // const real_t roll = rpy(0);
  // const real_t pitch = rpy(1);
  // const real_t yaw = 0.0;
  // C_WS = euler321(vec3_t{roll, pitch, yaw});
}

/*****************************************************************************
 *                                TIME
 *****************************************************************************/

void timestamp_print(const timestamp_t &ts, const std::string &prefix) {
  if (prefix != "") {
    printf("%s: " "%" PRIu64 "\n", prefix.c_str(), ts);
  } else {
    printf("%" PRIu64 "\n", ts);
  }
}

real_t ts2sec(const timestamp_t &ts) { return ts * 1.0e-9; }

real_t ns2sec(const int64_t ns) { return ns * 1.0e-9; }

struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

float toc(struct timespec *tic) {
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

float mtoc(struct timespec *tic) { return toc(tic) * 1000.0; }

real_t time_now() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((real_t) t.tv_sec + ((real_t) t.tv_usec) / 1000000.0);
}

/*****************************************************************************
 *                                NETWORKING
 *****************************************************************************/

int ip_port_info(const int sockfd, char *ip, int *port) {
  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  if (getpeername(sockfd, (struct sockaddr *) &addr, &len) != 0) {
    return -1;
  }

  // Deal with both IPv4 and IPv6:
  char ipstr[INET6_ADDRSTRLEN];

  if (addr.ss_family == AF_INET) {
    // IPV4
    struct sockaddr_in *s = (struct sockaddr_in *) &addr;
    *port = ntohs(s->sin_port);
    inet_ntop(AF_INET, &s->sin_addr, ipstr, sizeof(ipstr));
  } else {
    // IPV6
    struct sockaddr_in6 *s = (struct sockaddr_in6 *) &addr;
    *port = ntohs(s->sin6_port);
    inet_ntop(AF_INET6, &s->sin6_addr, ipstr, sizeof(ipstr));
  }
  strcpy(ip, ipstr);

  return 0;
}

int ip_port_info(const int sockfd, std::string &ip, int &port) {
  char ipstr[INET6_ADDRSTRLEN];
  const int retval = ip_port_info(sockfd, ipstr, &port);
  ip = std::string{ipstr};
  return retval;
}

tcp_server_t::tcp_server_t(int port_) : port{port_} {}

tcp_client_t::tcp_client_t(const std::string &server_ip_, int server_port_)
    : server_ip{server_ip_}, server_port{server_port_} {}

int tcp_server_config(tcp_server_t &server) {
  // Create socket
  server.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (server.sockfd == -1) {
    LOG_ERROR("Socket creation failed...");
    return -1;
  }

  // Socket options
  const int en = 1;
  const size_t int_sz = sizeof(int);
  if (setsockopt(server.sockfd, SOL_SOCKET, SO_REUSEADDR, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEADDR) failed");
  }
  if (setsockopt(server.sockfd, SOL_SOCKET, SO_REUSEPORT, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEPORT) failed");
  }

  // Assign IP, PORT
  struct sockaddr_in sockaddr;
  bzero(&sockaddr, sizeof(sockaddr));
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  sockaddr.sin_port = htons(server.port);

  // Bind newly created socket to given IP
  int retval =
      bind(server.sockfd, (struct sockaddr *) &sockaddr, sizeof(sockaddr));
  if (retval != 0) {
    LOG_ERROR("Socket bind failed: %s", strerror(errno));
    return -1;
  }

  return 0;
}

int tcp_server_loop(tcp_server_t &server) {
  // Server is ready to listen
  if ((listen(server.sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  }

  // Accept the data packet from client and verification
  std::map<int, pthread_t *> threads;
  int thread_id = 0;

  DEBUG("Server ready!");
  while (true) {
    // Accept incomming connections
    struct sockaddr_in sockaddr;
    socklen_t len = sizeof(sockaddr);
    int connfd = accept(server.sockfd, (struct sockaddr *) &sockaddr, &len);
    if (connfd < 0) {
      LOG_ERROR("Server acccept failed!");
      return -1;
    } else {
      server.conns.push_back(connfd);
    }

    // Fork off a thread to handle the connection
    pthread_t thread;
    pthread_create(&thread, nullptr, server.conn_thread, (void *) &server);
    threads.insert({thread_id, &thread});
    thread_id++;
  }
  DEBUG("Server shutting down ...");

  return 0;
}

int tcp_client_config(tcp_client_t &client) {
  // Create socket
  client.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (client.sockfd == -1) {
    LOG_ERROR("Socket creation failed!");
    return -1;
  }

  // Assign IP, PORT
  struct sockaddr_in server;
  size_t server_size = sizeof(server);
  bzero(&server, server_size);
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(client.server_ip.c_str());
  server.sin_port = htons(client.server_port);

  // Connect to server
  if (connect(client.sockfd, (struct sockaddr *) &server, server_size) != 0) {
    LOG_ERROR("Failed to connect to server!");
    return -1;
  }
  DEBUG("Connected to the server!");

  return 0;
}

int tcp_client_loop(tcp_client_t &client) {
  while (true) {
    if (client.loop_cb) {
      int retval = client.loop_cb(client);
      switch (retval) {
      case -1: return -1;
      case 1: break;
      }
    }
  }

  return 0;
}

/*****************************************************************************
 *                             INTERPOLATION
 ****************************************************************************/

quat_t slerp(const quat_t &q_start, const quat_t &q_end, const real_t alpha) {
  vec4_t q0{q_start.coeffs().data()};
  vec4_t q1{q_end.coeffs().data()};

  // Only unit quaternions are valid rotations.
  // Normalize to avoid undefined behavior.
  q0.normalize();
  q1.normalize();

  // Compute the cosine of the angle between the two vectors.
  real_t dot = q0.dot(q1);

  // If the dot product is negative, slerp won't take
  // the shorter path. Note that q1 and -q1 are equivalent when
  // the negation is applied to all four components. Fix by
  // reversing one quaternion.
  if (dot < 0.0f) {
    q1 = -q1;
    dot = -dot;
  }

  const real_t DOT_THRESHOLD = 0.9995;
  if (dot > DOT_THRESHOLD) {
    // If the inputs are too close for comfort, linearly interpolate
    // and normalize the result.
    vec4_t result = q0 + alpha * (q1 - q0);
    result.normalize();
    return quat_t{result(3), result(0), result(1), result(2)};
  }

  // Since dot is in range [0, DOT_THRESHOLD], acos is safe
  const real_t theta_0 = acos(dot);     // theta_0 = angle between input vectors
  const real_t theta = theta_0 * alpha; // theta = angle between q0 and result
  const real_t sin_theta = sin(theta);  // compute this value only once
  const real_t sin_theta_0 = sin(theta_0); // compute this value only once

  // == sin(theta_0 - theta) / sin(theta_0)
  const real_t s0 = cos(theta) - dot * sin_theta / sin_theta_0;
  const real_t s1 = sin_theta / sin_theta_0;

  const vec4_t result = (s0 * q0) + (s1 * q1);
  return quat_t{result(3), result(0), result(1), result(2)};
}

mat4_t interp_pose(const mat4_t &p0, const mat4_t &p1, const real_t alpha) {
  // Decompose start pose
  const vec3_t trans0 = tf_trans(p0);
  const quat_t quat0{tf_rot(p0)};

  // Decompose end pose
  const vec3_t trans1 = tf_trans(p1);
  const quat_t quat1{tf_rot(p1)};

  // Interpolate translation and rotation
  const auto trans_interp = lerp(trans0, trans1, alpha);
  const auto quat_interp = quat1.slerp(alpha, quat0);
  // const auto quat_interp = slerp(quat0, quat1, alpha);

  return tf(quat_interp, trans_interp);
}

void interp_poses(const timestamps_t &timestamps,
                  const mat4s_t &poses,
                  const timestamps_t &interp_ts,
                  mat4s_t &interped_poses,
                  const real_t threshold) {
  assert(timestamps.size() > 0);
  assert(timestamps.size() == poses.size());
  assert(interp_ts.size() > 0);
  assert(timestamps[0] < interp_ts[0]);

  // Interpolation variables
  timestamp_t ts_start = 0;
  timestamp_t ts_end = 0;
  mat4_t pose0 = I(4);
  mat4_t pose1 = I(4);

  size_t interp_idx = 0;
  for (size_t i = 0; i < timestamps.size(); i++) {
    const timestamp_t ts = timestamps[i];
    const mat4_t T = poses[i];

    const real_t diff = (ts - interp_ts[interp_idx]) * 1e-9;
    if (diff < threshold) {
      // Set interpolation start point
      ts_start = ts;
      pose0 = T;

    } else if (diff > threshold) {
      // Set interpolation end point
      ts_end = ts;
      pose1 = T;

      // Calculate alpha
      const real_t numerator = (interp_ts[interp_idx] - ts_start) * 1e-9;
      const real_t denominator = (ts_end - ts_start) * 1e-9;
      const real_t alpha = numerator / denominator;

      // Interpoate translation and rotation and add to results
      interped_poses.push_back(interp_pose(pose0, pose1, alpha));
      interp_idx++;

      // Shift interpolation current end point to start point
      ts_start = ts_end;
      pose0 = pose1;

      // Reset interpolation end point
      ts_end = 0;
      pose1 = I(4);
    }

    // Check if we're done
    if (interp_idx == interp_ts.size()) {
      break;
    }
  }
}

void closest_poses(const timestamps_t &timestamps,
                   const mat4s_t &poses,
                   const timestamps_t &target_ts,
                   mat4s_t &result) {
  assert(timestamps.size() > 0);
  assert(timestamps.size() == poses.size());
  assert(target_ts.size() > 0);
  assert(timestamps[0] < target_ts[0]);

  // Variables
  const timestamp_t ts = timestamps[0];
  real_t diff_closest = fabs((ts - target_ts[0]) * 1e-9);
  mat4_t pose_closest = poses[0];

  size_t target_idx = 0;
  for (size_t i = 1; i < timestamps.size(); i++) {
    const timestamp_t ts = timestamps[i];
    const mat4_t pose = poses[i];

    // Find closest pose
    const real_t diff = fabs((ts - target_ts[target_idx]) * 1e-9);
    if (diff < diff_closest) {
      // Update closest pose
      pose_closest = pose;
      diff_closest = diff;

    } else if (diff > diff_closest) {
      // Add to results
      result.push_back(pose_closest);
      target_idx++;

      // Initialize closest pose with current ts and pose
      diff_closest = fabs((ts - target_ts[target_idx]) * 1e-9);
      pose_closest = pose;
    }

    // Check if we're done
    if (target_idx == target_ts.size()) {
      break;
    }
  }
}


std::deque<timestamp_t> lerp_timestamps(const std::deque<timestamp_t> &t0,
                                        const std::deque<timestamp_t> &t1) {
  // Determine whether t0 or t1 has a higher rate?
  // Then create interpolation timestamps
  timestamp_t ts_start = 0;
  timestamp_t ts_end = 0;
  std::deque<timestamp_t> base_timestamps;

  if (t0.size() > t1.size()) {
    ts_start = t1.front();
    ts_end = t1.back();
    base_timestamps = t0;
  } else {
    ts_start = t0.front();
    ts_end = t0.back();
    base_timestamps = t1;
  }

  // Form interpolation timestamps
  std::deque<timestamp_t> lerp_ts;
  for (const auto ts : base_timestamps) {
    if (ts >= ts_start && ts <= ts_end) {
      lerp_ts.push_back(ts);
    }
  }

  return lerp_ts;
}

void lerp_data(const std::deque<timestamp_t> &lerp_ts,
               std::deque<timestamp_t> &target_ts,
               std::deque<vec3_t> &target_data,
               const bool keep_old) {
  std::deque<timestamp_t> result_ts;
  std::deque<vec3_t> result_data;

  timestamp_t t0 = target_ts.front();
  timestamp_t t1 = 0;
  vec3_t v0 = target_data.front();
  vec3_t v1 = zeros(3, 1);

  // Loop through target signal
  size_t lerp_idx = 0;
  for (size_t i = 1; i < target_ts.size(); i++) {
    const timestamp_t ts = target_ts[i];
    const vec3_t data = target_data[i];

    // Interpolate
    const bool do_interp = ((ts - lerp_ts[lerp_idx]) * 1e-9) > 0;
    if (do_interp) {
      t1 = ts;
      v1 = data;

      // Loop through interpolation points
      while (lerp_idx < lerp_ts.size()) {
        // Check if interp point is beyond interp end point
        if (t1 < lerp_ts[lerp_idx]) {
          break;
        }

        // Calculate interpolation parameter alpha
        const real_t num = (lerp_ts[lerp_idx] - t0) * 1e-9;
        const real_t den = (t1 - t0) * 1e-9;
        const real_t alpha = num / den;

        // Lerp and add to results
        result_data.push_back(lerp(v0, v1, alpha));
        result_ts.push_back(lerp_ts[lerp_idx]);
        lerp_idx++;
      }

      // Shift interpolation end point to start point
      t0 = t1;
      v0 = v1;

      // Reset interpolation end point
      t1 = 0;
      v1 = zeros(3, 1);
    }

    // Add end point into results, since we are retaining the old data.
    if (keep_old) {
      result_ts.push_back(ts);
      result_data.push_back(data);
    }
  }

  target_ts = result_ts;
  target_data = result_data;
}

mat4_t lerp_pose(const timestamp_t &t0,
                 const mat4_t &pose0,
                 const timestamp_t &t1,
                 const mat4_t &pose1,
                 const timestamp_t &t_lerp) {
  // Calculate alpha
  const double numerator = (t_lerp - t0) * 1e-9;
  const double denominator = (t1 - t0) * 1e-9;
  const double alpha = numerator / denominator;

  // Decompose start pose
  const vec3_t trans0 = tf_trans(pose0);
  const quat_t quat0{tf_rot(pose0)};

  // Decompose end pose
  const vec3_t trans1 = tf_trans(pose1);
  const quat_t quat1{tf_rot(pose1)};

  // Interpolate translation and rotation
  const auto trans_interp = lerp(trans0, trans1, alpha);
  const auto quat_interp = quat1.slerp(alpha, quat0);

  return tf(quat_interp, trans_interp);
}

static void align_front(const std::deque<timestamp_t> &reference,
                        std::deque<timestamp_t> &target,
                        std::deque<vec3_t> &data) {
  const auto front = reference.front();
  while (true) {
    if (target.front() < front) {
      target.pop_front();
      data.pop_front();
    } else {
      break;
    }
  }
}

static void align_back(const std::deque<timestamp_t> &reference,
                       std::deque<timestamp_t> &target,
                       std::deque<vec3_t> &data) {
  const auto back = reference.back();
  while (true) {
    if (target.back() > back) {
      target.pop_back();
      data.pop_back();
    } else {
      break;
    }
  }
}

void lerp_data(std::deque<timestamp_t> &ts0,
               std::deque<vec3_t> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<vec3_t> &vs1) {
  // Create interpolate timestamps
  auto lerp_ts = lerp_timestamps(ts0, ts1);

  // Interpolate
  if (ts0.size() > ts1.size()) {
    lerp_data(lerp_ts, ts1, vs1);
  } else {
    lerp_data(lerp_ts, ts0, vs0);
  }

  // Chop the front and back so both timestamps and data are sync-ed
  align_front(lerp_ts, ts1, vs1);
  align_front(lerp_ts, ts0, vs0);
  align_back(lerp_ts, ts1, vs1);
  align_back(lerp_ts, ts0, vs0);
}

ctraj_t::ctraj_t(const timestamps_t &timestamps,
                 const vec3s_t &positions,
                 const quats_t &orientations)
    : timestamps{timestamps}, positions{positions}, orientations{orientations},
      ts_s_start{ts2sec(timestamps.front())},
      ts_s_end{ts2sec(timestamps.back())}, ts_s_gap{ts_s_end - ts_s_start} {
  assert(timestamps.size() == positions.size());
  assert(timestamps.size() == orientations.size());
  assert(timestamps.size() > 4);
  ctraj_init(*this);
}

inline static real_t ts_normalize(const ctraj_t &ctraj, const timestamp_t ts) {
  const real_t ts_s_k = ts2sec(ts);
  const real_t ts_s_start = ctraj.ts_s_start;
  const real_t ts_s_end = ctraj.ts_s_end;
  return (ts_s_k - ts_s_start) / (ts_s_end - ts_s_start);
}

void ctraj_init(ctraj_t &ctraj) {
  assert(ctraj.timestamps.size() == ctraj.positions.size());
  assert(ctraj.timestamps.size() == ctraj.orientations.size());
  assert(ctraj.timestamps.size() > (3 + 1));

  // Create knots
  const size_t nb_knots = ctraj.timestamps.size();
  row_vector_t knots{nb_knots};
  for (size_t i = 0; i < ctraj.timestamps.size(); i++) {
    knots(i) = ts_normalize(ctraj, ctraj.timestamps[i]);
  }

  // Prep position data
  matx_t pos{3, nb_knots};
  for (size_t i = 0; i < nb_knots; i++) {
    pos.block<3, 1>(0, i) = ctraj.positions[i];
  }

  // Prep orientation data
  matx_t rvec{3, nb_knots};
  angle_axis_t aa{ctraj.orientations[0]};
  rvec.block<3, 1>(0, 0) = aa.angle() * aa.axis();

  for (size_t i = 1; i < nb_knots; i++) {
    const angle_axis_t aa{ctraj.orientations[i]};
    const vec3_t rvec_k = aa.angle() * aa.axis();
    const vec3_t rvec_km1 = rvec.block<3, 1>(0, i - 1);

    // Calculate delta from rvec_km1 to rvec_k
    vec3_t delta = rvec_k - rvec_km1;
    while (delta.squaredNorm() > (M_PI * M_PI)) {
      delta -= 2 * M_PI * delta.normalized();
    }

    // Add new rotation vector
    rvec.block<3, 1>(0, i) = rvec_km1 + delta;
  }

  // Create splines
  const int spline_degree = 3;
  ctraj.pos_spline = SPLINE3D(pos, knots, spline_degree);
  ctraj.rvec_spline = SPLINE3D(rvec, knots, spline_degree);
}

mat4_t ctraj_get_pose(const ctraj_t &ctraj, const timestamp_t ts) {
  const real_t u = ts_normalize(ctraj, ts);

  // Translation
  const vec3_t r = ctraj.pos_spline(u);

  // Orientation
  const vec3_t rvec = ctraj.rvec_spline(u);
  if (rvec.norm() < 1e-12) { // Check angle is not zero
    return tf(I(3), r);
  }
  const angle_axis_t aa{rvec.norm(), rvec.normalized()};
  const quat_t q{aa};

  return tf(q, r);
}

vec3_t ctraj_get_velocity(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const real_t u = ts_normalize(ctraj, ts);
  const real_t scale = (1 / ctraj.ts_s_gap);

  return ctraj.pos_spline.derivatives(u, 1).col(1) * scale;
}

vec3_t ctraj_get_acceleration(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const real_t u = ts_normalize(ctraj, ts);
  const real_t scale = pow(1 / ctraj.ts_s_gap, 2);

  return ctraj.pos_spline.derivatives(u, 2).col(2) * scale;
}

vec3_t ctraj_get_angular_velocity(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const real_t u = ts_normalize(ctraj, ts);
  const real_t scale = 1 / ctraj.ts_s_gap;

  const auto rvec_spline_deriv = ctraj.rvec_spline.derivatives(u, 1);
  const vec3_t rvec = rvec_spline_deriv.col(0);
  const vec3_t rvec_deriv = rvec_spline_deriv.col(1) * scale;

  // Check magnitude of the rotation vector
  const real_t rvec_norm = rvec.norm();
  if (rvec_norm < 1e-12) {
    return vec3_t{rvec_deriv};
  }

  // Calculate angular velocity
  const mat3_t axis_skew = skew(rvec.normalized());
  vec3_t w =
      (I(3) + axis_skew * (1.0 - cos(rvec_norm)) / rvec_norm +
       axis_skew * axis_skew * (rvec_norm - sin(rvec_norm)) / rvec_norm) *
      rvec_deriv;

  return w;
}

int ctraj_save(const ctraj_t &ctraj, const std::string &save_path) {
  // Setup output file
  std::ofstream file{save_path};
  if (file.good() != true) {
    LOG_ERROR("Failed to open file for output!");
    return -1;
  }

  // Output trajectory timestamps, positions and orientations as csv
  for (size_t i = 0; i < ctraj.timestamps.size(); i++) {
    file << ctraj.timestamps[i] << ",";
    file << ctraj.positions[i](0) << ",";
    file << ctraj.positions[i](1) << ",";
    file << ctraj.positions[i](2) << ",";
    file << ctraj.orientations[i].w() << ",";
    file << ctraj.orientations[i].x() << ",";
    file << ctraj.orientations[i].y() << ",";
    file << ctraj.orientations[i].z() << std::endl;
  }

  // Close file
  file.close();
  return 0;
}

void sim_imu_reset(sim_imu_t &imu) {
  imu.started = false;
  imu.b_g = zeros(3, 1);
  imu.b_a = zeros(3, 1);
  imu.ts_prev = 0;
}

void sim_imu_measurement(sim_imu_t &imu,
                         std::default_random_engine &rndeng,
                         const timestamp_t &ts,
                         const mat4_t &T_WS_W,
                         const vec3_t &w_WS_W,
                         const vec3_t &a_WS_W,
                         vec3_t &a_WS_S,
                         vec3_t &w_WS_S) {
  // Delta time according to sample rate
  real_t dt = 1.0 / imu.rate;

  // Check consistency of time increments:
  if (imu.started == false) {
    if (fabs(ts2sec(ts - imu.ts_prev) - dt) < (dt / 2.0)) {
      FATAL("Inconsisten sample rate with parameter setting: %f < %f",
            fabs(real_t(ts - imu.ts_prev) * 1.0e-9 - dt),
            dt / 2.0);
    }
  }

  // IMU initialised?
  if (imu.started == false) {
    // Stationary properties of an Ornstein-Uhlenbeck process
    imu.b_g = mvn(rndeng) * imu.sigma_gw_c * sqrt(imu.tau_g / 2.0);
    imu.b_a = mvn(rndeng) * imu.sigma_aw_c * sqrt(imu.tau_a / 2.0);
    imu.started = true;

  } else {
    // Propagate biases (slow moving signal)
    const vec3_t w_g = mvn(rndeng); // Gyro white noise
    imu.b_g += -imu.b_g / imu.tau_g * dt + w_g * imu.sigma_gw_c * sqrt(dt);
    const vec3_t w_a = mvn(rndeng); // Accel white noise
    imu.b_a += -imu.b_a / imu.tau_a * dt + w_a * imu.sigma_aw_c * sqrt(dt);
  }

  // Compute gyro measurement
  const mat3_t C_SW = tf_rot(T_WS_W).transpose();
  const vec3_t w_g = mvn(rndeng); // Gyro white noise
  w_WS_S = C_SW * w_WS_W + imu.b_g + w_g * imu.sigma_g_c * sqrt(dt);

  // Compute accel measurement
  const vec3_t g{0.0, 0.0, imu.g}; // Gravity vector
  const vec3_t w_a = mvn(rndeng);   // Accel white noise
  a_WS_S = C_SW * (a_WS_W + g) + imu.b_a + w_a * imu.sigma_a_c * sqrt(dt);

  imu.ts_prev = ts;
}

/*****************************************************************************
 *                                 CONTROL
 *****************************************************************************/

pid_t::pid_t() {}

pid_t::pid_t(const real_t k_p_, const real_t k_i_, const real_t k_d_)
    : k_p{k_p_}, k_i{k_i_}, k_d{k_d_} {}

pid_t::~pid_t() {}

std::ostream &operator<<(std::ostream &os, const pid_t &pid) {
  os << "error_prev:" << pid.error_prev << std::endl;
  os << "error_sum:" << pid.error_sum << std::endl;
  os << "error_p:" << pid.error_p << std::endl;
  os << "error_i:" << pid.error_i << std::endl;
  os << "error_d:" << pid.error_d << std::endl;
  os << "k_p:" << pid.k_p << std::endl;
  os << "k_i:" << pid.k_i << std::endl;
  os << "k_d:" << pid.k_d << std::endl;
  return os;
}

real_t pid_update(pid_t &p,
                  const real_t setpoint,
                  const real_t actual,
                  const real_t dt) {
  // Calculate errors
  const real_t error = setpoint - actual;
  p.error_sum += error * dt;

  // Calculate output
  p.error_p = p.k_p * error;
  p.error_i = p.k_i * p.error_sum;
  p.error_d = p.k_d * (error - p.error_prev) / dt;
  const real_t output = p.error_p + p.error_i + p.error_d;

  p.error_prev = error;
  return output;
}

real_t pid_update(pid_t &p, const real_t error, const real_t dt) {
  return pid_update(p, error, 0.0, dt);
}

void pid_reset(pid_t &p) {
  p.error_prev = 0.0;
  p.error_sum = 0.0;

  // p.error_p = 0.0;
  // p.error_i = 0.0;
  // p.error_d = 0.0;
}

carrot_ctrl_t::carrot_ctrl_t() {}

carrot_ctrl_t::~carrot_ctrl_t() {}

int carrot_ctrl_configure(carrot_ctrl_t &cc,
                          const vec3s_t &waypoints,
                          const real_t look_ahead_dist) {
  if (waypoints.size() <= (size_t) 2) {
    LOG_ERROR("Too few waypoints!");
    return -1;
  }

  cc.waypoints = waypoints;
  cc.wp_start = cc.waypoints[0];
  cc.wp_end = cc.waypoints[1];
  cc.wp_index = 1;
  cc.look_ahead_dist = look_ahead_dist;

  return 0;
}

int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
                              const vec3_t &pos,
                              vec3_t &result) {
  // Calculate closest point
  const vec3_t v1 = pos - cc.wp_start;
  const vec3_t v2 = cc.wp_end - cc.wp_start;
  const real_t t = v1.dot(v2) / v2.squaredNorm();
  result = cc.wp_start + t * v2;

  return t;
}

int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
                             const vec3_t &pos,
                             vec3_t &result) {
  vec3_t closest_pt;
  int t = carrot_ctrl_closest_point(cc, pos, closest_pt);

  if (t == -1) {
    // Closest point is before wp_start
    result = cc.wp_start;

  } else if (t == 0) {
    // Closest point is between wp_start wp_end
    const vec3_t u = cc.wp_end - cc.wp_start;
    const vec3_t v = u / u.norm();
    result = closest_pt + cc.look_ahead_dist * v;

  } else if (t == 1) {
    // Closest point is after wp_end
    result = cc.wp_end;
  }

  return t;
}

int carrot_ctrl_update(carrot_ctrl_t &cc,
                       const vec3_t &pos,
                       vec3_t &carrot_pt) {
  // Calculate new carot point
  int status = carrot_ctrl_carrot_point(cc, pos, carrot_pt);

  // Check if there are more waypoints
  if ((cc.wp_index + 1) == cc.waypoints.size()) {
    return 1;
  }

  // Update waypoints
  if (status == 1) {
    cc.wp_index++;
    cc.wp_start = cc.wp_end;
    cc.wp_end = cc.waypoints[cc.wp_index];
  }

  return 0;
}

/*****************************************************************************
 *                                  MODEL
 ****************************************************************************/

mat4_t dh_transform(const real_t theta,
                    const real_t d,
                    const real_t a,
                    const real_t alpha) {
  // clang-format off
  mat4_t T;
  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
       sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
       0.0, sin(alpha), cos(alpha), d,
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return T;
}

gimbal_model_t::gimbal_model_t() {}

gimbal_model_t::gimbal_model_t(const vec6_t &tau_s,
                               const vec6_t &tau_d,
                               const real_t Lambda1,
                               const vec3_t w1,
                               const real_t Lambda2,
                               const vec3_t w2,
                               const real_t theta1_offset,
                               const real_t theta2_offset)
    : tau_s{tau_s}, tau_d{tau_d}, Lambda1{Lambda1}, w1{w1}, Lambda2{Lambda2},
      w2{w2}, theta1_offset{theta1_offset}, theta2_offset{theta2_offset} {}

gimbal_model_t::~gimbal_model_t() {}

void gimbal_model_set_attitude(gimbal_model_t &model,
                               const real_t roll_,
                               const real_t pitch_) {
  model.Lambda1 = roll_;
  model.Lambda2 = pitch_;
}

vec2_t gimbal_model_get_joint_angles(const gimbal_model_t &model) {
  return vec2_t{model.Lambda1, model.Lambda2};
}

mat4_t gimbal_model_T_BS(const gimbal_model_t &model) {
  mat4_t T_sb = zeros(4, 4);
  T_sb.block(0, 0, 3, 3) = euler321(model.tau_s.tail(3));
  T_sb.block(0, 3, 3, 1) = model.tau_s.head(3);
  T_sb(3, 3) = 1.0;

  return T_sb;
}

mat4_t gimbal_model_T_EB(const gimbal_model_t &model) {
  const real_t theta1 = model.Lambda1 + model.theta1_offset;
  const real_t d1 = model.w1[0];
  const real_t a1 = model.w1[1];
  const real_t alpha1 = model.w1[2];

  const real_t theta2 = model.Lambda2 + model.theta2_offset;
  const real_t d2 = model.w2[0];
  const real_t a2 = model.w2[1];
  const real_t alpha2 = model.w2[2];

  const mat4_t T_1b = dh_transform(theta1, d1, a1, alpha1).inverse();
  const mat4_t T_e1 = dh_transform(theta2, d2, a2, alpha2).inverse();
  const mat4_t T_EB = T_e1 * T_1b;

  return T_EB;
}

mat4_t gimbal_model_T_DE(const gimbal_model_t &model) {
  mat4_t T_DE = zeros(4, 4);
  T_DE.block(0, 0, 3, 3) = euler321(model.tau_d.tail(3));
  T_DE.block(0, 3, 3, 1) = model.tau_d.head(3);
  T_DE(3, 3) = 1.0;

  return T_DE;
}

mat4_t gimbal_model_T_DS(const gimbal_model_t &model) {
  const auto T_DE = gimbal_model_T_DE(model);
  const auto T_EB = gimbal_model_T_EB(model);
  const auto T_BS = gimbal_model_T_BS(model);
  return T_DE * T_EB * T_BS;
}

mat4_t gimbal_model_T_DS(gimbal_model_t &model, const vec2_t &theta) {
  gimbal_model_set_attitude(model, theta(0), theta(1));
  const auto T_DE = gimbal_model_T_DE(model);
  const auto T_EB = gimbal_model_T_EB(model);
  const auto T_BS = gimbal_model_T_BS(model);
  return T_DE * T_EB * T_BS;
}

std::ostream &operator<<(std::ostream &os, const gimbal_model_t &model) {
  os << "tau_s: " << model.tau_s.transpose() << std::endl;
  os << "tau_d: " << model.tau_d.transpose() << std::endl;
  os << "w1: " << model.w1.transpose() << std::endl;
  os << "w2: " << model.w2.transpose() << std::endl;
  os << "Lambda1: " << model.Lambda1 << std::endl;
  os << "Lambda2: " << model.Lambda2 << std::endl;
  os << "theta1_offset: " << model.theta1_offset << std::endl;
  os << "theta2_offset: " << model.theta2_offset << std::endl;
  return os;
}

void circle_trajectory(const real_t r,
                       const real_t v,
                       real_t *w,
                       real_t *time) {
  const real_t dist = 2 * M_PI * r;
  *time = dist / v;
  *w = (2 * M_PI) / *time;
}

int mav_model_update(mav_model_t &model,
                     const vec4_t &motor_inputs,
                     const real_t dt) {
  const real_t ph = model.attitude(0);
  const real_t th = model.attitude(1);
  const real_t ps = model.attitude(2);

  const real_t p = model.angular_velocity(0);
  const real_t q = model.angular_velocity(1);
  const real_t r = model.angular_velocity(2);

  const real_t x = model.position(0);
  const real_t y = model.position(1);
  const real_t z = model.position(2);

  const real_t vx = model.linear_velocity(0);
  const real_t vy = model.linear_velocity(1);
  const real_t vz = model.linear_velocity(2);

  const real_t Ix = model.Ix;
  const real_t Iy = model.Iy;
  const real_t Iz = model.Iz;

  const real_t kr = model.kr;
  const real_t kt = model.kt;

  const real_t m = model.m;
  const real_t g = model.g;

  // convert motor inputs to angular p, q, r and total thrust
  // clang-format off
  mat4_t A;
  A << 1.0, 1.0, 1.0, 1.0,
       0.0, -model.l, 0.0, model.l,
       -model.l, 0.0, model.l, 0.0,
       -model.d, model.d, -model.d, model.d;
  // clang-format on
  const vec4_t tau = A * motor_inputs;
  const real_t tauf = tau(0);
  const real_t taup = tau(1);
  const real_t tauq = tau(2);
  const real_t taur = tau(3);

  // update
  // clang-format off
  model.attitude(0) = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
  model.attitude(1) = th + (q * cos(ph) - r * sin(ph)) * dt;
  model.attitude(2) = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;
  model.angular_velocity(0) = p + (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
  model.angular_velocity(1) = q + (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
  model.angular_velocity(2) = r + (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;
  model.position(0) = x + vx * dt;
  model.position(1) = y + vy * dt;
  model.position(2) = z + vz * dt;
  model.linear_velocity(0) = vx + ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
  model.linear_velocity(1) = vy + ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
  model.linear_velocity(2) = vz + (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;
  // clang-format on

  // constrain yaw to be [-180, 180]
  // if (model.attitude(2) > M_PI) {
  //   model.attitude(2) -= 2 * M_PI;
  // } else if (model.attitude(2) < -M_PI) {
  //   model.attitude(2) += 2 * M_PI;
  // }

  return 0;
}

/*****************************************************************************
 *                                  CV
 ****************************************************************************/

bool is_equal(const cv::Mat &m1, const cv::Mat &m2) {
  // Pre-check
  if (m1.empty() && m2.empty()) {
    return true;
  }

  // Check dimensions
  if (m1.cols != m2.cols) {
    return false;
  } else if (m1.rows != m2.rows) {
    return false;
  } else if (m1.dims != m2.dims) {
    return false;
  }

  // Check matrix elements
  cv::Mat diff;
  cv::compare(m1, m2, diff, cv::CMP_NE);

  return cv::countNonZero(diff) ? false : true;
}

void convert(const cv::Mat &x, matx_t &y) {
  y.resize(x.rows, x.cols);

  for (int i = 0; i < x.rows; i++) {
    for (int j = 0; j < x.cols; j++) {
      y(i, j) = x.at<real_t>(i, j);
    }
  }
}

void convert(const matx_t &x, cv::Mat &y) {
  y = cv::Mat(x.rows(), x.cols(), cv::DataType<real_t>::type);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      y.at<real_t>(i, j) = x(i, j);
    }
  }
}

matx_t convert(const cv::Mat &x) {
  matx_t y;
  convert(x, y);
  return y;
}

cv::Mat convert(const matx_t &x) {
  cv::Mat y;
  convert(x, y);
  return y;
}

std::vector<cv::KeyPoint>
sort_keypoints(const std::vector<cv::KeyPoint> keypoints, const size_t limit) {
  if (keypoints.size() == 0) {
    return std::vector<cv::KeyPoint>();
  }

  // Obtain vector responses
  std::vector<int> responses;
  for (size_t i = 0; i < keypoints.size(); i++) {
    responses.push_back(keypoints[i].response);
  }

  // Sort responses
  std::vector<int> index(responses.size());
  std::iota(std::begin(index), std::end(index), 0);
  cv::sortIdx(responses, index, CV_SORT_DESCENDING);

  // Form sorted keypoints
  std::vector<cv::KeyPoint> keypoints_sorted;
  for (size_t i = 0; i < keypoints.size(); i++) {
    keypoints_sorted.push_back(keypoints[index[i]]);
    if (keypoints_sorted.size() == limit) {
      break;
    }
  }

  return keypoints_sorted;
}

cv::Mat gray2rgb(const cv::Mat &image) {
  const int image_height = image.rows;
  const int image_width = image.cols;
  cv::Mat out_image(image_height, image_width, CV_8UC3);

  if (image.channels() == 1) {
    cv::cvtColor(image, out_image, CV_GRAY2RGB);
  } else {
    return image.clone();
  }

  return out_image;
}

cv::Mat rgb2gray(const cv::Mat &image) {
  cv::Mat image_gray;

  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  } else {
    return image.clone();
  }

  return image_gray;
}

cv::Mat roi(const cv::Mat &image,
            const int width,
            const int height,
            const real_t cx,
            const real_t cy) {
  const real_t x = cx - width / 2.0;
  const real_t y = cy - height / 2.0;
  cv::Rect roi(x, y, width, height);
  return image(roi);
}

bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
                                  const cv::KeyPoint &kp2) {
  // Keypoint with higher response will be at the beginning of the vector
  return kp1.response > kp2.response;
}

real_t reprojection_error(const vec2s_t &measured, const vec2s_t &projected) {
  assert(measured.size() == projected.size());

  real_t sse = 0.0;
  const size_t nb_keypoints = measured.size();
  for (size_t i = 0; i < nb_keypoints; i++) {
    sse += pow((measured[i] - projected[i]).norm(), 2);
  }
  const real_t rmse = sqrt(sse / nb_keypoints);

  return rmse;
}

real_t reprojection_error(const std::vector<cv::Point2f> &measured,
                          const std::vector<cv::Point2f> &projected) {
  assert(measured.size() == projected.size());

  real_t sse = 0.0;
  const size_t nb_keypoints = measured.size();
  for (size_t i = 0; i < nb_keypoints; i++) {
    sse += pow(cv::norm(measured[i] - projected[i]), 2);
  }
  const real_t rmse = sqrt(sse / nb_keypoints);

  return rmse;
}

real_t reprojection_error(const std::vector<vec2_t> &errors) {
  double sse = 0.0;
  for (const auto &e : errors) {
    sse += pow(e.norm(), 2);
  }
  const double mse = sse / errors.size();

  return sqrt(mse);
}

matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::Point2f> points,
                    const int patch_width) {
  matx_t mask = ones(image_height, image_width);

  // Create a mask around each point
  for (const auto &p : points) {
    // Skip if pixel is out of image bounds
    const real_t px = static_cast<int>(p.x);
    const real_t py = static_cast<int>(p.y);
    if (px >= image_width || px <= 0) {
      continue;
    } else if (py >= image_height || py <= 0) {
      continue;
    }

    // Calculate patch top left corner, patch width and height
    vec2_t top_left{px - patch_width, py - patch_width};
    vec2_t top_right{px + patch_width, py - patch_width};
    vec2_t btm_left{px - patch_width, py + patch_width};
    vec2_t btm_right{px + patch_width, py + patch_width};
    std::vector<vec2_t *> corners{&top_left, &top_right, &btm_left, &btm_right};
    for (auto corner : corners) {
      // Check corner in x-axis
      if ((*corner)(0) < 0) {
        (*corner)(0) = 0;
      } else if ((*corner)(0) > image_width) {
        (*corner)(0) = image_width;
      }

      // Check corner in y-axis
      if ((*corner)(1) < 0) {
        (*corner)(1) = 0;
      } else if ((*corner)(1) > image_height) {
        (*corner)(1) = image_height;
      }
    }

    // Create mask around pixel
    const int row = top_left(1);
    const int col = top_left(0);
    int width = top_right(0) - top_left(0) + 1;
    int height = btm_left(1) - top_left(1) + 1;
    width = (col + width) > image_width ? width - 1 : width;
    height = (row + height) > image_height ? height - 1 : height;

    // std::cout << "---" << std::endl;
    // std::cout << image_width << std::endl;
    // std::cout << image_height << std::endl;
    // std::cout << row << std::endl;
    // std::cout << col << std::endl;
    // std::cout << width << std::endl;
    // std::cout << height << std::endl;
    // std::cout << "---" << std::endl;

    mask.block(row, col, height, width) = zeros(height, width);
  }

  return mask;
}

matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::KeyPoint> keypoints,
                    const int patch_width) {
  std::vector<cv::Point2f> points;
  for (const auto &kp : keypoints) {
    points.emplace_back(kp.pt);
  }

  return feature_mask(image_width, image_height, points, patch_width);
}

cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::Point2f> points,
                            const int patch_width) {
  auto mask = feature_mask(image_width, image_height, points, patch_width);

  cv::Mat mask_cv;
  cv::eigen2cv(mask, mask_cv);
  mask_cv.convertTo(mask_cv, CV_8UC1);

  return mask_cv;
}

cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::KeyPoint> keypoints,
                            const int patch_width) {
  auto mask = feature_mask(image_width, image_height, keypoints, patch_width);

  cv::Mat mask_cv;
  cv::eigen2cv(mask, mask_cv);
  mask_cv.convertTo(mask_cv, CV_8UC1);

  return mask_cv;
}

cv::Mat radtan_undistort_image(const mat3_t &K,
                               const vecx_t &D,
                               const cv::Mat &image) {
  cv::Mat image_ud;
  cv::Mat K_ud = convert(K).clone();
  cv::undistort(image, image_ud, convert(K), convert(D), K_ud);
  return image_ud;
}

cv::Mat equi_undistort_image(const mat3_t &K,
                             const vecx_t &D,
                             const cv::Mat &image,
                             const real_t balance,
                             cv::Mat &Knew) {
  // Estimate new camera matrix first
  const cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(convert(K),
                                                          convert(D),
                                                          image.size(),
                                                          R,
                                                          Knew,
                                                          balance);

  // Undistort image
  cv::Mat image_ud;
  cv::fisheye::undistortImage(image, image_ud, convert(K), convert(D), Knew);

  return image_ud;
}

void illum_invar_transform(cv::Mat &image,
                           const real_t lambda_1,
                           const real_t lambda_2,
                           const real_t lambda_3) {
  // The following method is adapted from:
  // Illumination Invariant Imaging: Applications in Robust Vision-based
  // Localisation, Mapping and Classification for Autonomous Vehicles
  // Maddern et al (2014)

  // clang-format off
  real_t alpha = (lambda_1 * lambda_3 - lambda_1 * lambda_2) /
                 (lambda_2 * lambda_3 - lambda_1 * lambda_2);
  // clang-format on

  std::vector<cv::Mat> channels(3);
  split(image, channels);
  channels[0].convertTo(channels[0], CV_32F);
  channels[1].convertTo(channels[1], CV_32F);
  channels[2].convertTo(channels[2], CV_32F);

  channels[0].row(0).setTo(cv::Scalar(1));
  channels[1].row(0).setTo(cv::Scalar(1));
  channels[2].row(0).setTo(cv::Scalar(1));

  cv::Mat log_ch_1, log_ch_2, log_ch_3;
  cv::log(channels[0] / 255.0, log_ch_1);
  cv::log(channels[1] / 255.0, log_ch_2);
  cv::log(channels[2] / 255.0, log_ch_3);

  image = 0.5 + log_ch_2 - alpha * log_ch_3 - (1 - alpha) * log_ch_1;
  image.setTo(0, image < 0);
  image.setTo(1, image > 1);
  cv::normalize(image, image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}

cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status) {
  // Draw tracks
  for (size_t i = 0; i < status.size(); i++) {
    // Check if point was lost
    if (status[i] == 0) {
      continue;
    }

    // Draw circle and line
    cv::circle(img_cur, p0[i], 1, cv::Scalar(0, 255, 0), -1);
    cv::circle(img_cur, p1[i], 1, cv::Scalar(0, 255, 0), -1);
    cv::line(img_cur, p0[i], p1[i], cv::Scalar(0, 255, 0));
  }

  return img_cur;
}

cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < status.size(); i++) {
    if (status[i]) {
      cv::Point2f p0 = k0[i];
      cv::Point2f p1 = k1[i];

      // Point 1
      p1.y += img0.rows;

      // Draw circle and line
      cv::circle(match_img, p0, 2, cv::Scalar(0, 255, 0), -1);
      cv::circle(match_img, p1, 2, cv::Scalar(0, 255, 0), -1);
      cv::line(match_img, p0, p1, cv::Scalar(0, 255, 0));
    }
  }

  return match_img;
}

cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < matches.size(); i++) {
    const int k0_idx = matches[i].queryIdx;
    const int k1_idx = matches[i].trainIdx;
    cv::KeyPoint p0 = k0[k0_idx];
    cv::KeyPoint p1 = k1[k1_idx];

    // Point 1
    p1.pt.y += img0.rows;

    // Draw circle and line
    cv::circle(match_img, p0.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(match_img, p1.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::line(match_img, p0.pt, p1.pt, cv::Scalar(0, 255, 0));
  }

  return match_img;
}

cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::Point2f> features) {
  cv::Mat out_image = image.clone();

  // Draw corners
  for (auto p : features) {
    cv::circle(out_image, p, 2, cv::Scalar(0, 255, 0), -1);
  }

  // Draw vertical lines
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;

  for (int x = dx; x < image_width; x += dx) {
    const cv::Point start(x, 0);
    const cv::Point end(x, image_height);
    const cv::Scalar color(0, 0, 255);
    cv::line(out_image, start, end, color, 2);
  }

  // Draw horizontal lines
  for (int y = dy; y < image_height; y += dy) {
    const cv::Point start(0, y);
    const cv::Point end(image_width, y);
    const cv::Scalar color(0, 0, 255);
    cv::line(out_image, start, end, color, 2);
  }

  return out_image;
}

cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::KeyPoint> features) {
  std::vector<cv::Point2f> points;
  for (const auto &f : features) {
    points.emplace_back(f.pt);
  }

  return draw_grid_features(image, grid_rows, grid_cols, points);
}

std::vector<cv::Point2f> grid_fast(const cv::Mat &image,
                                   const int max_keypoints,
                                   const int grid_rows,
                                   const int grid_cols,
                                   const real_t threshold,
                                   const bool nonmax_suppression) {
  // Prepare input image - make sure it is grayscale
  cv::Mat image_gray = rgb2gray(image);

  // Calculate number of grid cells and max corners per cell
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;
  const int nb_cells = grid_rows * grid_cols;
  const size_t max_per_cell = std::ceil((float) max_keypoints / (float) nb_cells);

  // Detect corners in each grid cell
  std::vector<cv::Point2f> keypoints_all;

  for (int x = 0; x < image_width; x += dx) {
    for (int y = 0; y < image_height; y += dy) {
      // Make sure roi width and height are not out of bounds
      const real_t w = (x + dx > image_width) ? image_width - x : dx;
      const real_t h = (y + dy > image_height) ? image_height - y : dy;

      // Detect corners in grid cell
      cv::Rect roi = cv::Rect(x, y, w, h);
      std::vector<cv::KeyPoint> keypoints;
      cv::FAST(image_gray(roi), keypoints, threshold, nonmax_suppression);

      // Sort by keypoint response
      keypoints = sort_keypoints(keypoints);

      // Adjust keypoint's position according to the offset limit to max
      // corners per cell
      size_t cell_counter = 0;
      for (auto &kp : keypoints) {
        kp.pt.x += x;
        kp.pt.y += y;

        keypoints_all.push_back(kp.pt);
        cell_counter++;
        if (cell_counter >= max_per_cell) {
          break;
        }
      }
    }
  }

  return keypoints_all;
}

std::vector<cv::Point2f> grid_good(const cv::Mat &image,
                                   const int max_keypoints,
                                   const int grid_rows,
                                   const int grid_cols,
                                   const real_t quality_level,
                                   const real_t min_distance,
                                   const cv::Mat mask,
                                   const int block_size,
                                   const bool use_harris_detector,
                                   const real_t k) {
  // Prepare input image - make sure it is grayscale
  cv::Mat image_gray = rgb2gray(image);

  // Calculate number of grid cells and max corners per cell
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;
  const int nb_cells = grid_rows * grid_cols;
  const size_t max_per_cell = std::ceil((float) max_keypoints / (float) nb_cells);

  // Detect corners in each grid cell
  std::vector<cv::Point2f> keypoints_all;

  for (int x = 0; x < image_width; x += dx) {
    for (int y = 0; y < image_height; y += dy) {
      // Make sure roi width and height are not out of bounds
      const real_t w = (x + dx > image_width) ? image_width - x : dx;
      const real_t h = (y + dy > image_height) ? image_height - y : dy;

      // Detect keypoints in grid cell
      const cv::Rect roi = cv::Rect(x, y, w, h);
      const cv::Mat sub_mask = (mask.rows == 0) ? cv::Mat() : mask(roi);
      std::vector<cv::Point2f> keypoints;
      cv::goodFeaturesToTrack(image_gray(roi),
                              keypoints,
                              max_keypoints,
                              quality_level,
                              min_distance,
                              sub_mask,
                              block_size,
                              use_harris_detector,
                              k);

      // Adjust keypoint's position according to the offset limit to max
      // keypoints per cell
      size_t cell_counter = 0;
      for (auto &kp : keypoints) {
        kp.x += x;
        kp.y += y;

        keypoints_all.push_back(kp);
        cell_counter++;
        if (cell_counter >= max_per_cell) {
          break;
        }
      }
    }
  }

  return keypoints_all;
}

std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4) {
  os << "k1: " << radtan4.k1() << std::endl;
  os << "k2: " << radtan4.k2() << std::endl;
  os << "p1: " << radtan4.p1() << std::endl;
  os << "p2: " << radtan4.p2() << std::endl;
  return os;
}

std::ostream &operator<<(std::ostream &os, const equi4_t &equi4) {
  os << "k1: " << equi4.k1() << std::endl;
  os << "k2: " << equi4.k2() << std::endl;
  os << "k3: " << equi4.k3() << std::endl;
  os << "k4: " << equi4.k4() << std::endl;
  return os;
}

real_t pinhole_focal(const int image_size, const real_t fov) {
  return ((image_size / 2.0) / tan(deg2rad(fov) / 2.0));
}

mat3_t pinhole_K(const real_t fx,
                 const real_t fy,
                 const real_t cx,
                 const real_t cy) {
  mat3_t K = zeros(3, 3);
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;
  K(2, 2) = 1.0;
  return K;
}

mat3_t pinhole_K(const vec4_t &params) {
  return pinhole_K(params(0), params(1), params(2), params(3));
}

mat3_t pinhole_K(const int img_w,
                 const int img_h,
                 const real_t lens_hfov,
                 const real_t lens_vfov) {
  const real_t fx = pinhole_focal(img_w, lens_hfov);
  const real_t fy = pinhole_focal(img_h, lens_vfov);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;

  mat3_t K = zeros(3, 3);
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;
  K(2, 2) = 1.0;
  return K;
}

/*****************************************************************************
 *                               SIMULATION
 *****************************************************************************/

void vio_sim_data_t::add(const int sensor_id,
                         const timestamp_t &ts,
                         const vec3_t &accel,
                         const vec3_t &gyro) {
  sim_event_t event(sensor_id, ts, accel, gyro);
  timeline.insert({ts, event});
}

void vio_sim_data_t::add(const int sensor_id,
                         const timestamp_t &ts,
                         const vec2s_t &keypoints,
                         const std::vector<size_t> &feature_idxs) {
  sim_event_t event{sensor_id, ts, keypoints, feature_idxs};
  timeline.insert({event.ts, event});
}

void vio_sim_data_t::save(const std::string &dir) {
  if (create_dir(dir) == -1) {
    FATAL("Failed to create dir [%s]!", dir.c_str());
  }

  auto features_csv_path = dir + "/features.csv";
  auto cam0_obs_csv_path = dir + "/cam0_observations.csv";
  auto cam0_kps_csv_path = dir + "/cam0_keypoints.csv";
  auto cam0_pose_csv_path = dir + "/cam0_pose.csv";
  auto imu_csv_path = dir + "/imu.csv";
  auto imu_pose_csv_path = dir + "/imu_pose.csv";
  auto imu_vel_csv_path = dir + "/imu_vel.csv";

  FILE *features_csv = fopen(features_csv_path.c_str(), "w");
  FILE *cam0_obs_csv = fopen(cam0_obs_csv_path.c_str(), "w");
  FILE *cam0_kps_csv = fopen(cam0_kps_csv_path.c_str(), "w");
  FILE *cam0_pose_csv = fopen(cam0_pose_csv_path.c_str(), "w");
  FILE *imu_csv = fopen(imu_csv_path.c_str(), "w");
  FILE *imu_pose_csv = fopen(imu_pose_csv_path.c_str(), "w");
  FILE *imu_vel_csv = fopen(imu_vel_csv_path.c_str(), "w");

  // Save features
  for (const auto &f : features) {
    fprintf(features_csv, "%f,%f,%f\n", f(0), f(1), f(2));
  }
  fclose(features_csv);

  // Save observations
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto ts = cam_ts[k];
    fprintf(cam0_obs_csv, "%" PRIu64 ",", ts);
    fprintf(cam0_obs_csv, "%zu,", observations[k].size());

    for (size_t i = 0; i < observations[k].size(); i++) {
      const auto obs = observations[k][i];
      fprintf(cam0_obs_csv, "%zu", obs);
      if ((i + 1) != observations[k].size()) {
        fprintf(cam0_obs_csv, ",");
      } else {
        fprintf(cam0_obs_csv, "\n");
      }
    }
  }
  fclose(cam0_obs_csv);

  // Save keypoints
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto ts = cam_ts[k];
    fprintf(cam0_kps_csv, "%" PRIu64 ",", ts);
    fprintf(cam0_kps_csv, "%zu,", keypoints[k].size());

    for (size_t i = 0; i < keypoints[k].size(); i++) {
      const auto kp= keypoints[k][i];
      fprintf(cam0_kps_csv, "%f,%f", kp(0), kp(1));
      if ((i + 1) != keypoints[k].size()) {
        fprintf(cam0_kps_csv, ",");
      } else {
        fprintf(cam0_kps_csv, "\n");
      }
    }
  }
  fclose(cam0_kps_csv);

  // Save camera poses
  for (size_t k = 0; k < cam_ts.size(); k++) {
    const auto ts = cam_ts[k];
    const auto q = cam_rot_gnd[k];
    const auto r = cam_pos_gnd[k];
    save_pose(cam0_pose_csv, ts, q, r);
  }
  fclose(cam0_pose_csv);

  // Save imu data
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto a = imu_acc[k];
    const auto w = imu_gyr[k];
    fprintf(imu_csv, "%" PRIu64 ",", ts);
    fprintf(imu_csv, "%f,%f,%f,", a(0), a(1), a(2));
    fprintf(imu_csv, "%f,%f,%f\n", w(0), w(1), w(2));
  }
  fclose(imu_csv);

  // Save imu poses
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto q = imu_rot[k];
    const auto r = imu_pos[k];
    fprintf(imu_pose_csv, "%" PRIu64 ",", ts);
    fprintf(imu_pose_csv, "%f,%f,%f,%f,", q.w(), q.x(), q.y(), q.z());
    fprintf(imu_pose_csv, "%f,%f,%f\n", r(0), r(1), r(2));
  }
  fclose(imu_pose_csv);

  // Save imu velocities
  for (size_t k = 0; k < imu_ts.size(); k++) {
    const auto ts = imu_ts[k];
    const auto vel = imu_vel[k];
    fprintf(imu_vel_csv, "%" PRIu64 ",", ts);
    fprintf(imu_vel_csv, "%f,%f,%f\n", vel(0), vel(1), vel(2));
  }
  fclose(imu_vel_csv);
}

static vec3s_t create_3d_features(const real_t *x_bounds,
                                  const real_t *y_bounds,
                                  const real_t *z_bounds,
                                  const size_t nb_features) {
  vec3s_t features;
  for (size_t i = 0; i < nb_features; i++) {
    real_t x = randf(x_bounds[0], x_bounds[1]);
    real_t y = randf(y_bounds[0], y_bounds[1]);
    real_t z = randf(z_bounds[0], z_bounds[1]);
    features.emplace_back(x, y, z);
  }
  return features;
}

static vec3s_t create_3d_features_perimeter(const vec3_t &origin,
                                            const vec3_t &dim,
                                            const size_t nb_features) {
  // Dimension of the outskirt
  const real_t w = dim(0);
  const real_t l = dim(1);
  const real_t h = dim(2);

  // Features per side
  size_t nb_fps = nb_features / 4.0;
  vec3s_t features;

  // Features in the east side
  {
    const real_t x_bounds[2] = {origin(0) - w, origin(0) + w};
    const real_t y_bounds[2] = {origin(1) + l, origin(1) + l};
    const real_t z_bounds[2] = {origin(2) - h, origin(2) + h};
    auto f = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);
    features.reserve(features.size() + std::distance(f.begin(), f.end()));
    features.insert(features.end(), f.begin(), f.end());
  }

  // Features in the north side
  {
    const real_t x_bounds[2] = {origin(0) + w, origin(0) + w};
    const real_t y_bounds[2] = {origin(1) - l, origin(1) + l};
    const real_t z_bounds[2] = {origin(2) - h, origin(2) + h};
    auto f = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);
    features.reserve(features.size() + std::distance(f.begin(), f.end()));
    features.insert(features.end(), f.begin(), f.end());
  }

  // Features in the west side
  {
    const real_t x_bounds[2] = {origin(0) - w, origin(0) + w};
    const real_t y_bounds[2] = {origin(1) - l, origin(1) - l};
    const real_t z_bounds[2] = {origin(2) - h, origin(2) + h};
    auto f = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);
    features.reserve(features.size() + std::distance(f.begin(), f.end()));
    features.insert(features.end(), f.begin(), f.end());
  }

  // Features in the south side
  {
    const real_t x_bounds[2] = {origin(0) - w, origin(0) - w};
    const real_t y_bounds[2] = {origin(1) - l, origin(1) + l};
    const real_t z_bounds[2] = {origin(2) - h, origin(2) + h};
    auto f = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);
    features.reserve(features.size() + std::distance(f.begin(), f.end()));
    features.insert(features.end(), f.begin(), f.end());
  }

  return features;
}

void sim_circle_trajectory(const real_t circle_r, vio_sim_data_t &sim_data) {
  const real_t circle_dist = 2 * M_PI * circle_r;
  const real_t time_taken = circle_dist / sim_data.sensor_velocity;
  const real_t f = 1.0 / time_taken;
  const real_t w = -2.0 * M_PI * f;
  const real_t w2 = w * w;

  // Create features
  const vec3_t origin{0.0, 0.0, 0.0};
  const vec3_t dim{5.0, 5.0, 5.0};
  const size_t nb_features = 100;
  sim_data.features = create_3d_features_perimeter(origin, dim, nb_features);

  // Create camera
  const int res[2] = {640, 480};
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(res[0], lens_hfov);
  const real_t fy = pinhole_focal(res[1], lens_vfov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const pinhole_radtan4_t camera{res, proj_params, dist_params};

  // Simulate camera
  {
    const int cam_id = 0;
    const real_t cam_rate = 30.0;
    const real_t dt = 1.0 / cam_rate;
    const real_t t_end = time_taken;
    real_t t = 0.0;
    real_t theta = deg2rad(180.0);
    real_t yaw = 0.0;

    while (t <= t_end) {
      const real_t x = circle_r * cos(theta);
      const real_t y = circle_r * sin(theta);
      const real_t z = 0.0;
      const vec3_t r_WC{x, y, z};
      const vec3_t rpy{deg2rad(-90.0), 0.0, yaw};
      const mat3_t C_WC = euler321(rpy);
      const mat4_t T_WC = tf(C_WC, r_WC);

      sim_data.cam_ts.push_back(t * 1e9);

      sim_data.cam_pos_gnd.push_back(r_WC);
      sim_data.cam_rot_gnd.emplace_back(C_WC);
      sim_data.cam_poses_gnd.push_back(tf(C_WC, r_WC));

      const mat4_t T_WC_n = add_noise(T_WC, 0.1, 1.0);
      sim_data.cam_pos.push_back(tf_trans(T_WC_n));
      sim_data.cam_rot.push_back(tf_quat(T_WC_n));
      sim_data.cam_poses.push_back(T_WC_n);

      // Check which features in the scene is observable by camera
      const mat4_t T_CW = T_WC.inverse();
      size_t feature_idx = 0;
      std::vector<size_t> frame_obs;
      vec2s_t frame_kps;

      for (const vec3_t &p_W : sim_data.features) {
        vec2_t z;
        const vec3_t p_C = tf_point(T_CW, p_W);
        if (camera.project(p_C, z) == 0) {
          frame_obs.push_back(feature_idx);
          frame_kps.push_back(z);
        }
        feature_idx++;
      }
      sim_data.observations.push_back(frame_obs);
      sim_data.keypoints.push_back(frame_kps);
      sim_data.add(cam_id, t * 1e9, frame_kps, frame_obs);

      // Update
      t += dt;
      theta += w * dt; // -ve to go from 180 to -180 in CW fashion
      yaw += w * dt;   // -ve to yaw the camera CW fashion
    }
  }

  // Simulate imu
  {
    const int imu_id = 0;
    const real_t dt = 1.0 / sim_data.imu_rate;
    const real_t t_end = time_taken;
    real_t t = 0.0;
    real_t theta = deg2rad(180.0);
    real_t yaw = deg2rad(90.0);

    std::default_random_engine rndeng;
    sim_imu_t imu;
    imu.rate = sim_data.imu_rate;
    imu.tau_a = 3600;
    imu.tau_g = 3600;
    // imu.sigma_g_c = 0.0;
    // imu.sigma_a_c = 0.0;
    // imu.sigma_gw_c = 0.0;
    // imu.sigma_aw_c = 0.0;
    imu.sigma_g_c = 0.005;
    imu.sigma_a_c = 0.025;
    imu.sigma_gw_c = 1e-05;
    imu.sigma_aw_c = 0.001;
    imu.g = 9.81;

    while (t <= t_end) {
      // Form sensor pose
      // -- Orientation
      const vec3_t rpy{0.0, 0.0, wrapPi(yaw)};
      const mat3_t C_WS_W = euler321(rpy);
      // -- Position
      const real_t rx = circle_r * cos(theta);
      const real_t ry = circle_r * sin(theta);
      const real_t rz = 0.0;
      const vec3_t r_WS_W{rx, ry, rz};
      // -- Pose
      const mat4_t T_WS_W = tf(C_WS_W, r_WS_W);

      // Form sensor velocity
      const real_t vx = -circle_r * w * sin(theta);
      const real_t vy = circle_r * w * cos(theta);
      const real_t vz = 0.0;
      sim_data.imu_vel.emplace_back(vx, vy, vz);

      // Form sensor angular velocity
      const timestamp_t ts_k = t * 1e9;
      const vec3_t w_WS_W{0.0, 0.0, w};

      // Form Sensor acceleration
      const real_t ax = -circle_r * w2 * cos(theta);
      const real_t ay = -circle_r * w2 * sin(theta);
      const real_t az = 0.0;
      const vec3_t a_WS_W{ax, ay, az};

      // Simulate imu measurements
      vec3_t a_WS_S;
      vec3_t w_WS_S;
      sim_imu_measurement(imu,
                          rndeng,
                          ts_k,
                          T_WS_W,
                          w_WS_W,
                          a_WS_W,
                          a_WS_S,
                          w_WS_S);
      sim_data.imu_ts.push_back(ts_k);
      sim_data.imu_acc.push_back(a_WS_S);
      sim_data.imu_gyr.push_back(w_WS_S);
      sim_data.imu_pos.push_back(tf_trans(T_WS_W));
      sim_data.imu_poses.push_back(T_WS_W);
      sim_data.imu_rot.push_back(tf_quat(T_WS_W));
      sim_data.add(imu_id, ts_k, a_WS_S, w_WS_S);

      // Update
      t += dt;
      theta += w * dt;  // -ve to go from 180 to -180 in CW fashion
      yaw += w * dt;    // -ve to go from 180 to -180 in CW fashion
    }
  }
}


} // namespace yac
