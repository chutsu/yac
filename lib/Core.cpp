#include "Core.hpp"

namespace yac {

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

int list_files(const std::string &path, std::vector<std::string> &files) {
  // Check dir
  if (dir_exists(path) == false) {
    LOG_ERROR("Dir [%s] does not exist!", path.c_str());
    return -1;
  }

  // Get files
  if (list_dir(path, files) != 0) {
    LOG_ERROR("Failed to traverse dir [%s]!", path.c_str());
    return -1;
  }
  std::sort(files.begin(), files.end());

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

config_t::config_t(const std::string &file_path_) : file_path{file_path_} {
  if (yaml_load_file(file_path_, root) == 0) {
    ok = true;
  }
}

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
  vec = vec2_t{node[0].as<double>(), node[1].as<double>()};
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
      vec3_t{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
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
  vec = vec4_t{node[0].as<double>(),
               node[1].as<double>(),
               node[2].as<double>(),
               node[3].as<double>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec5_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec5_t>(node, key, optional);
  vec << node[0].as<double>(), node[1].as<double>(), node[2].as<double>(),
      node[3].as<double>(), node[4].as<double>();
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec6_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec6_t>(node, key, optional);
  vec << node[0].as<double>(), node[1].as<double>(), node[2].as<double>(),
      node[3].as<double>(), node[4].as<double>(), node[5].as<double>();
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec7_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec7_t>(node, key, optional);
  vec << node[0].as<double>(), node[1].as<double>(), node[2].as<double>(),
      node[3].as<double>(), node[4].as<double>(), node[5].as<double>(),
      node[6].as<double>();
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
    vec(i) = node[i].as<double>();
  }
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec2i_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec2i_t>(node, key, optional);
  vec = vec2i_t{node[0].as<int>(), node[1].as<int>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec3i_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec3i_t>(node, key, optional);
  vec = vec3i_t{node[0].as<int>(), node[1].as<int>(), node[2].as<int>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec4i_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec4i_t>(node, key, optional);
  vec = vec4i_t{node[0].as<int>(),
                node[1].as<int>(),
                node[2].as<int>(),
                node[3].as<int>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vecxi_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  const size_t vector_size = yaml_check_vector<vecxi_t>(node, key, optional);
  vec = vecxi_t::Zero(vector_size, 1);
  for (size_t i = 0; i < node.size(); i++) {
    vec(i) = node[i].as<int>();
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
  mat(0, 0) = node["data"][0].as<double>();
  mat(0, 1) = node["data"][1].as<double>();
  mat(1, 0) = node["data"][2].as<double>();
  mat(1, 1) = node["data"][3].as<double>();
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
  mat(0, 0) = node["data"][0].as<double>();
  mat(0, 1) = node["data"][1].as<double>();
  mat(0, 2) = node["data"][2].as<double>();
  // -- Col 2
  mat(1, 0) = node["data"][3].as<double>();
  mat(1, 1) = node["data"][4].as<double>();
  mat(1, 2) = node["data"][5].as<double>();
  // -- Col 3
  mat(2, 0) = node["data"][6].as<double>();
  mat(2, 1) = node["data"][7].as<double>();
  mat(2, 2) = node["data"][8].as<double>();
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
      mat(i, j) = node["data"][index].as<double>();
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
      mat(i, j) = node["data"][index].as<double>();
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
      mat.at<double>(i, j) = node["data"][index].as<double>();
      index++;
    }
  }

  return 0;
}

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
  char *retval = (char *)malloc(sizeof(char) * strlen(s) + 1);
  strcpy(retval, s);
  return retval;
}

std::string replace(const std::string &in,
                    const std::string &from,
                    const std::string &to) {
  return std::regex_replace(in, std::regex(from), to);
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
  char **fields = (char **)malloc(sizeof(char *) * *nb_fields);
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

double **csv_data(const char *fp, int *nb_rows, int *nb_cols) {
  // Obtain number of rows and columns in csv data
  *nb_rows = csv_rows(fp);
  *nb_cols = csv_cols(fp);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  // Initialize memory for csv data
  double **data = (double **)malloc(sizeof(double *) * *nb_rows);
  for (int i = 0; i < *nb_cols; i++) {
    data[i] = (double *)malloc(sizeof(double) * *nb_cols);
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
        data = (int *)calloc(array_size + 1, sizeof(int));
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
  int **array = (int **)calloc(*nb_arrays, sizeof(int *));

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

static double *parse_darray_line(char *line) {
  char entry[1024] = {0};
  int index = 0;
  double *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = (double *)calloc(array_size, sizeof(double));
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

double **load_darrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = csv_rows(csv_path);
  double **array = (double **)calloc(*nb_arrays, sizeof(double *));

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
  int num_rows = csv_rows(file_path);
  int num_cols = csv_cols(file_path);
  if (num_cols == 0) {
    return -2;
  }

  // Skip header line?
  std::string line;
  if (header) {
    std::getline(infile, line);
    num_rows -= 1;
  }

  // Load data
  int line_no = 0;
  std::vector<double> vdata;
  data = zeros(num_rows, num_cols);

  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    // Load data row
    std::string element;
    for (int i = 0; i < num_cols; i++) {
      std::getline(ss, element, ',');
      const double value = atof(element.c_str());
      data(line_no, i) = value;
    }

    line_no++;
  }

  return 0;
}

int csv2vec(const std::string &file_path, const bool header, vecx_t &data) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of rows and cols
  int num_rows = csv_rows(file_path);
  if (num_rows == 0) {
    return -2;
  }

  // Skip header line?
  std::string line;
  if (header) {
    std::getline(infile, line);
    num_rows -= 1;
  }

  // Load data
  int line_no = 0;
  std::vector<double> vdata;
  data = zeros(num_rows, 1);

  while (std::getline(infile, line)) {
    const double value = atof(line.c_str());
    data(line_no) = value;
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

void print_progress(const double percentage, const std::string &prefix) {
  const char *PBSTR =
      "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
  const int PBWIDTH = 60;

  int val = (int)(percentage * 100);
  int lpad = (int)(percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%s[%.*s%*s]%3d%%", prefix.c_str(), lpad, PBSTR, rpad, "", val);
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

void save_data(const std::string &save_path,
               const timestamps_t &ts,
               const vec3s_t &y) {
  std::ofstream file{save_path};
  if (file.good() != true) {
    printf("Failed to open file for output!");
    exit(-1);
  }

  for (size_t i = 0; i < ts.size(); i++) {
    file << ts[i] << ",";
    file << y[i](0) << ",";
    file << y[i](1) << ",";
    file << y[i](2) << std::endl;
  }

  file.close();
}

void save_data(const std::string &save_path,
               const timestamps_t &ts,
               const quats_t &y) {
  std::ofstream file{save_path};
  if (file.good() != true) {
    printf("Failed to open file for output!");
    exit(-1);
  }

  for (size_t i = 0; i < ts.size(); i++) {
    file << ts[i] << ",";
    file << y[i].w() << ",";
    file << y[i].x() << ",";
    file << y[i].y() << ",";
    file << y[i].z() << std::endl;
  }

  file.close();
}

void save_features(const std::string &path, const vec3s_t &features) {
  FILE *csv = fopen(path.c_str(), "w");
  for (const auto &f : features) {
    fprintf(csv, "%f,%f,%f\n", f(0), f(1), f(2));
  }
  fflush(csv);
  fclose(csv);
}

void save_poses(const std::string &path,
                const timestamps_t &timestamps,
                const mat4s_t &poses) {
  FILE *csv = fopen(path.c_str(), "w");
  fprintf(csv, "#ts,rx,ry,rz,qx,qy,qz,qw\n");

  for (size_t k = 0; k < timestamps.size(); k++) {
    const auto ts = timestamps[k];
    const auto &r = tf_trans(poses[k]);
    const auto &q = tf_quat(poses[k]);
    fprintf(csv, "%ld,", ts);
    fprintf(csv, "%f,", r.x());
    fprintf(csv, "%f,", r.y());
    fprintf(csv, "%f,", r.z());
    fprintf(csv, "%f,", q.x());
    fprintf(csv, "%f,", q.y());
    fprintf(csv, "%f,", q.z());
    fprintf(csv, "%f\n", q.w());
  }

  fflush(csv);
  fclose(csv);
}

void save_extrinsics(const std::string &path, const mat4_t &extrinsics) {
  FILE *csv = fopen(path.c_str(), "w");
  fprintf(csv, "#rx,ry,rz,qx,qy,qz,qw\n");

  const auto &q = tf_quat(extrinsics);
  const auto &r = tf_trans(extrinsics);
  fprintf(csv, "%f,", r.x());
  fprintf(csv, "%f,", r.y());
  fprintf(csv, "%f,", r.z());
  fprintf(csv, "%f,", q.x());
  fprintf(csv, "%f,", q.y());
  fprintf(csv, "%f,", q.z());
  fprintf(csv, "%f\n", q.w());

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
  str_format = "%ld,";             // Timestamp[ns]
  str_format += "%lf,%lf,%lf,";    // Position
  str_format += "%lf,%lf,%lf,%lf"; // Quaternion

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
    double rx, ry, rz = 0.0;
    int retval =
        fscanf(fp, str_format.c_str(), &ts, &rx, &ry, &rz, &qx, &qy, &qz, &qw);
    if (retval != 8) {
      FATAL("Failed to parse line in [%s:%d]", fpath.c_str(), i);
    }

    // Just need 1 pose
    vec3_t r{rx, ry, rz};
    quat_t q{qw, qx, qy, qz};
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
  str_format = "%ld,";             // Timestamp[ns]
  str_format += "%lf,%lf,%lf,";    // Position
  str_format += "%lf,%lf,%lf,%lf"; // Quaternion

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double rx, ry, rz = 0.0;
    double qw, qx, qy, qz = 0.0;
    int retval =
        fscanf(fp, str_format.c_str(), &ts, &rx, &ry, &rz, &qx, &qy, &qz, &qw);
    if (retval != 8) {
      FATAL("Failed to parse line in [%s:%d]", fpath.c_str(), i);
    }

    // Record
    timestamps.push_back(ts);
    vec3_t r{rx, ry, rz};
    quat_t q{qw, qx, qy, qz};
    poses.push_back(tf(q, r));
  }
}

/******************************************************************************
 *                                ALGEBRA
 *****************************************************************************/

int sign(const double x) {
  if (fltcmp(x, 0.0) == 0) {
    return 0;
  } else if (x < 0) {
    return -1;
  }
  return 1;
}

int fltcmp(const double f1, const double f2) {
  if (fabs(f1 - f2) <= 0.0001) {
    return 0;
  } else if (f1 > f2) {
    return 1;
  } else {
    return -1;
  }
}

double binomial(const double n, const double k) {
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
                 const double *array,
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
    // printf("%f", v(i));
    printf("%e", v(i));
    if ((i + 1) != v.size()) {
      printf(", ");
    }
  }
  printf("\n");
}

void print_vector(const std::string &name, const double *v, const int N) {
  Eigen::VectorXd vec;
  vec.resize(N);
  for (int i = 0; i < N; i++) {
    vec(i) = v[i];
  }

  print_vector(name, vec);
}

void print_matrix(const std::string &name,
                  const matx_t &m,
                  const std::string &indent) {
  printf("%s:\n", name.c_str());
  for (long i = 0; i < m.rows(); i++) {
    printf("%s", indent.c_str());
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

std::string array2str(const double *array, const size_t size) {
  std::stringstream os;
  for (size_t i = 0; i < (size - 1); i++) {
    os << array[i] << " ";
  }
  os << array[size - 1];

  return os.str();
}

void array2vec(const double *x, const size_t size, vecx_t y) {
  y.resize(size);
  for (size_t i = 0; i < size; i++) {
    y(i) = x[i];
  }
}

double *vec2array(const vecx_t &v) {
  double *array = (double *)malloc(sizeof(double) * v.size());
  for (int i = 0; i < v.size(); i++) {
    array[i] = v(i);
  }
  return array;
}

double *mat2array(const matx_t &m) {
  double *array = (double *)malloc(sizeof(double) * m.size());

  int index = 0;
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < m.cols(); j++) {
      array[index] = m(i, j);
      index++;
    }
  }
  return array;
}

double *quat2array(const quat_t &q) {
  double *array = (double *)malloc(sizeof(double) * 4);

  array[0] = q.x();
  array[1] = q.y();
  array[2] = q.z();
  array[3] = q.w();

  return array;
}

void vec2array(const vecx_t &v, double *out) {
  for (int i = 0; i < v.size(); i++) {
    out[i] = v(i);
  }
}

void mat2array(const matx_t &A, double *out) {
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
    const double x = v(0);
    const double y = v(1);
    const double z = v(2);
    retval.block(0, idx, 4, 1) = vec4_t{x, y, z, 1.0};
    idx++;
  }

  return retval;
}

std::string vec2str(const vecx_t &v,
                    const bool brackets,
                    const bool max_digits) {
  std::ostringstream ss;

  if (max_digits) {
    typedef std::numeric_limits<double> numeric_limits;
    ss << std::setprecision(numeric_limits::max_digits10);
  }

  if (brackets) {
    ss << "[";
  }

  for (int i = 0; i < v.size(); i++) {
    ss << v(i);
    if ((i + 1) != v.size()) {
      ss << ", ";
    }
  }

  if (brackets) {
    ss << "]";
  }

  return ss.str();
}

std::string arr2str(const double *arr, const size_t len, bool brackets) {
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

std::string mat2str(const matx_t &m,
                    const std::string &indent,
                    const bool max_digits) {
  std::string str;

  for (int i = 0; i < m.rows(); i++) {
    if ((i + 1) != m.rows()) {
      str += indent;
      str += vec2str(m.row(i), false, max_digits) + ",\n";
    } else {
      str += indent;
      str += vec2str(m.row(i), false, max_digits);
    }
  }

  return str;
}

vec3_t normalize(const vec3_t &v) { return v / v.norm(); }

double cond(const matx_t &A) {
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
        const double x = 0.5 * (A(i, j) + A(j, i));
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

void load_matrix(const std::vector<double> &x,
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

void load_matrix(const matx_t &A, std::vector<double> &x) {
  for (int i = 0; i < A.cols(); i++) {
    for (int j = 0; j < A.rows(); j++) {
      x.push_back(A(j, i));
    }
  }
}

matx_t pinv(const matx_t &A, const double tol) {
  auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &vals_ = svd.singularValues();
  matx_t vals_inv = zeros(A.cols(), A.rows());

  for (unsigned int i = 0; i < vals_.size(); ++i) {
    if (vals_(i) > tol) {
      vals_inv(i, i) = ((double)1.0) / vals_(i);
    } else {
      vals_inv(i, i) = ((double)0);
    }
  }

  return svd.matrixV() * vals_inv * svd.matrixU().adjoint();
}

long int rank(const matx_t &A) {
  Eigen::FullPivLU<matx_t> lu_decomp(A);
  return lu_decomp.rank();
}

bool full_rank(const matx_t &A) {
  if (rank(A) != A.rows() || rank(A) != A.cols()) {
    return false;
  }

  return true;
}

int schurs_complement(
    matx_t &H, vecx_t &b, const size_t m, const size_t r, const bool precond) {
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

  H = H_marg;
  b = b_marg;
  return 0;
}

mat4_t oplus(const quat_t &q) {
  // clang-format off
  mat4_t Q;
  Q(0,0) =  q.w(); Q(0,1) =  q.z(); Q(0,2) = -q.y(); Q(0,3) =  q.x();
  Q(1,0) = -q.z(); Q(1,1) =  q.w(); Q(1,2) =  q.x(); Q(1,3) =  q.y();
  Q(2,0) =  q.y(); Q(2,1) = -q.x(); Q(2,2) =  q.w(); Q(2,3) =  q.z();
  Q(3,0) = -q.x(); Q(3,1) = -q.y(); Q(3,2) = -q.z(); Q(3,3) =  q.w();
  // clang-format on
  return Q;
}

mat_t<6, 7, row_major_t> lift_pose_jacobian(const mat4_t pose) {
  Eigen::Matrix<double, 3, 4> Jq_pinv;
  Jq_pinv.bottomRightCorner<3, 1>().setZero();
  Jq_pinv.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 2.0;

  const quat_t q = tf_quat(pose);
  Eigen::Matrix4d Qplus = oplus(q.inverse());

  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
  J_lift.setZero();
  J_lift.topLeftCorner<3, 3>().setIdentity();
  J_lift.bottomRightCorner<3, 4>() = Jq_pinv * Qplus;

  return J_lift;
}

/******************************************************************************
 *                                GEOMETRY
 *****************************************************************************/

double sinc(const double x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    static const double c_2 = 1.0 / 6.0;
    static const double c_4 = 1.0 / 120.0;
    static const double c_6 = 1.0 / 5040.0;
    const double x_2 = x * x;
    const double x_4 = x_2 * x_2;
    const double x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

double deg2rad(const double d) { return d * (M_PI / 180.0); }

vec3_t deg2rad(const vec3_t d) { return d * (M_PI / 180.0); }

double rad2deg(const double r) { return r * (180.0 / M_PI); }

vec3_t rad2deg(const vec3_t &r) { return r * (180.0 / M_PI); }

double wrap180(const double euler_angle) {
  return fmod((euler_angle + 180.0), 360.0) - 180.0;
}

double wrap360(const double euler_angle) {
  if (euler_angle > 0) {
    return fmod(euler_angle, 360.0);
  } else {
    return fmod(euler_angle + 360, 360.0);
  }
}

double wrapPi(const double r) { return deg2rad(wrap180(rad2deg(r))); }

double wrap2Pi(const double r) { return deg2rad(wrap360(rad2deg(r))); }

vec2_t circle(const double r, const double theta) {
  return vec2_t{r * cos(theta), r * sin(theta)};
}

vec3_t sphere(const double rho, const double theta, const double phi) {
  const double x = rho * sin(theta) * cos(phi);
  const double y = rho * sin(theta) * sin(phi);
  const double z = rho * cos(theta);
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

double cross_track_error(const vec2_t &p1,
                         const vec2_t &p2,
                         const vec2_t &pos) {
  const double x0 = pos(0);
  const double y0 = pos(1);

  const double x1 = p1(0);
  const double y1 = p1(0);

  const double x2 = p2(0);
  const double y2 = p2(0);

  // calculate perpendicular distance between line (p1, p2) and point (pos)
  const double n = ((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
  const double d = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));

  return fabs(n) / d;
}

int point_left_right(const vec2_t &a, const vec2_t &b, const vec2_t &c) {
  const double a0 = a(0);
  const double a1 = a(1);
  const double b0 = b(0);
  const double b1 = b(1);
  const double c0 = c(0);
  const double c1 = c(1);
  const double x = (b0 - a0) * (c1 - a1) - (b1 - a1) * (c0 - a0);

  if (x > 0) {
    return 1; // left
  } else if (x < 0) {
    return 2; // right
  } else if (x == 0) {
    return 0; // parallel
  }

  return -1;
}

double closest_point(const vec2_t &a,
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
  const double t = v1.dot(v2) / v2.squaredNorm();
  closest = a + t * v2;

  return t;
}

void fit_circle(const vec2s_t &points, double &cx, double &cy, double &radius) {
  assert(points.size() > 3);

  // Parametric circle equation
  // (x - cx)^2 + (y - cy)^2 = r^2

  // Expand and rewrite the circle equation
  // (x^2 - 2x * cx + cx^2) + (y^2 - 2y * cy + cy^2) = r^2
  // -2x * cx + cx^2 - 2y * cy + cy^2 = r^2 - x^2 - y^2
  // (-2x * cx + cx^2) - (2y * cy + cy^2) - r^2 = -(x^2 + y^2)
  // (-2x * cx) + (-2y * cy) + (-r^2  + cx^2 + cy^2) = -(x^2 + y^2)

  // Matrix form: Ax = b
  // Let
  //   A = [-2x -2y 1]
  //   x = [cx, cy, -r^2 + cx^2 + cy^2]'
  //   b = [-(x^2 + y^2)]'
  // [-2x -2y 1] [cx cy -r^2+cx^2+cy^2]' = [-(x^2 + y^2)]'

  // Form A matrix and vector b
  int nb_points = points.size();
  matx_t A;
  vecx_t b;
  A.resize(nb_points, 3);
  b.resize(nb_points, 1);

  for (int i = 0; i < nb_points; i++) {
    const vec2_t p = points[i];
    A(i, 0) = -2.0 * p.x();
    A(i, 1) = -2.0 * p.y();
    A(i, 2) = 1.0;
    b(i) = -(p.x() * p.x() + p.y() * p.y());
  }

  // Solve Ax = b
  Eigen::JacobiSVD<matx_t> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  vecx_t x = svd.solve(b);

  // Results
  cx = x(0);
  cy = x(1);
  radius = sqrt((cx * cx) + (cy * cy) - x(2));
}

vec2s_t intersect_circles(const double cx0,
                          const double cy0,
                          const double r0,
                          const double cx1,
                          const double cy1,
                          const double r1) {
  vec2s_t ipts;

  // Check if circles are separate
  double d = sqrt(pow(cx0 - cx1, 2) + pow(cy0 - cy1, 2));
  if (d > r0 + r1) {
    return ipts;
  }

  // Check if one circle is contained within the other
  if (d < fabs(r0 - r1)) {
    return ipts;
  }

  // Check if circles intersect only at a single point
  double a = (pow(r0, 2) - pow(r1, 2) + pow(d, 2)) / (2.0 * d);
  double h = sqrt(pow(r0, 2) - pow(a, 2));
  double x3 = cx0 + a * (cx1 - cx0) / d;
  double y3 = cy0 + a * (cy1 - cy0) / d;
  if (h < 1e-10) {
    ipts.emplace_back(x3, y3);
    return ipts;
  }

  // Circles interset at two points
  ipts.emplace_back(x3 + h * (cy1 - cy0) / d, y3 - h * (cx1 - cx0) / d);
  ipts.emplace_back(x3 - h * (cy1 - cy0) / d, y3 + h * (cx1 - cx0) / d);
  return ipts;
}

void latlon_offset(double lat_ref,
                   double lon_ref,
                   double offset_N,
                   double offset_E,
                   double *lat_new,
                   double *lon_new) {
  *lat_new = lat_ref + (offset_E / EARTH_RADIUS_M);
  *lon_new = lon_ref + (offset_N / EARTH_RADIUS_M) / cos(deg2rad(lat_ref));
}

void latlon_diff(double lat_ref,
                 double lon_ref,
                 double lat,
                 double lon,
                 double *dist_N,
                 double *dist_E) {
  double d_lon = lon - lon_ref;
  double d_lat = lat - lat_ref;

  *dist_N = deg2rad(d_lat) * EARTH_RADIUS_M;
  *dist_E = deg2rad(d_lon) * EARTH_RADIUS_M * cos(deg2rad(lat));
}

double latlon_dist(double lat_ref, double lon_ref, double lat, double lon) {
  double dist_N = 0.0;
  double dist_E = 0.0;

  latlon_diff(lat_ref, lon_ref, lat, lon, &dist_N, &dist_E);
  double dist = sqrt(pow(dist_N, 2) + pow(dist_E, 2));

  return dist;
}

/******************************************************************************
 *                               STATISTICS
 *****************************************************************************/

int randi(int ub, int lb) { return rand() % lb + ub; }

double randf(const double ub, const double lb) {
  const double f = (double)rand() / RAND_MAX;
  return lb + f * (ub - lb);
}

double sum(const std::vector<double> &x) {
  double sum = 0.0;

  for (const auto &x_i : x) {
    sum += x_i;
  }

  return sum;
}

double median(const std::vector<double> &v) {
  // sort values
  std::vector<double> v_copy = v;
  std::sort(v_copy.begin(), v_copy.end());

  // obtain median
  if (v_copy.size() % 2 == 1) {
    // return middle value
    return v_copy[v_copy.size() / 2];

  } else {
    // grab middle two values and calc mean
    const double a = v_copy[v_copy.size() / 2];
    const double b = v_copy[(v_copy.size() / 2) - 1];
    return (a + b) / 2.0;
  }
}

double max(const std::vector<double> &x) {
  double max_value = x[0];
  for (const auto x_i : x) {
    if (x_i > max_value) {
      max_value = x_i;
    }
  }
  return max_value;
}

double mean(const std::vector<double> &x) {
  double sum = 0.0;
  for (const auto i : x) {
    sum += i;
  }

  const double N = x.size();
  return sum / N;
}

vec2_t mean(const vec2s_t &x) {
  vec2_t x_hat{0.0, 0.0};

  for (const auto &v : x) {
    x_hat += v;
  }
  x_hat *= 1.0f / x.size();

  return x_hat;
}

vec3_t mean(const vec3s_t &x) {
  vec3_t x_hat{0.0, 0.0, 0.0};

  for (const auto &v : x) {
    x_hat += v;
  }
  x_hat *= 1.0f / x.size();

  return x_hat;
}

double var(const std::vector<double> &x) {
  const double mu = mean(x);
  const double N = x.size();

  double sum = 0.0;
  for (const auto x_i : x) {
    sum += pow(x_i - mu, 2);
  }

  return sum / (N - 1.0);
}

vec2_t var(const vec2s_t &vecs) {
  std::vector<double> x;
  std::vector<double> y;

  for (const vec2_t &v : vecs) {
    x.push_back(v(0));
    y.push_back(v(1));
  }

  return vec2_t{var(x), var(y)};
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

double stddev(const std::vector<double> &x) { return sqrt(var(x)); }

vec2_t stddev(const vec2s_t &v) {
  const vec2_t v_var = var(v);
  return vec2_t{sqrt(v_var.x()), sqrt(v_var.y())};
}

vec3_t stddev(const vec3s_t &v) {
  const vec3_t v_var = var(v);
  return vec3_t{sqrt(v_var.x()), sqrt(v_var.y()), sqrt(v_var.z())};
}

double rmse(const std::vector<double> &residuals) {
  double sse = 0.0;
  for (const auto r : residuals) {
    sse += r * r;
  }

  double n = residuals.size();
  double mse = sse / n;
  return sqrt(mse);
}

vec2_t rmse(const vec2s_t &vecs) {
  std::vector<double> x;
  std::vector<double> y;

  for (const auto &v : vecs) {
    x.push_back(v.x());
    y.push_back(v.y());
  }

  return vec2_t{rmse(x), rmse(y)};
}

vec3_t rmse(const vec3s_t &vecs) {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;

  for (const auto &v : vecs) {
    x.push_back(v.x());
    y.push_back(v.y());
    z.push_back(v.z());
  }

  return vec3_t{rmse(x), rmse(y), rmse(z)};
}

vec3_t mvn(std::default_random_engine &engine,
           const vec3_t &mu,
           const vec3_t &stdev) {
  std::normal_distribution<double> normal_x(mu(0), stdev(0));
  std::normal_distribution<double> normal_y(mu(1), stdev(1));
  std::normal_distribution<double> normal_z(mu(2), stdev(2));
  return vec3_t{normal_x(engine), normal_y(engine), normal_z(engine)};
}

double gauss_normal() {
  static double V1, V2, S;
  static int phase = 0;
  double X;

  if (phase == 0) {
    do {
      double U1 = (double)rand() / RAND_MAX;
      double U2 = (double)rand() / RAND_MAX;

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

mat4_t tf_inv(const mat4_t &T) {
  const vec3_t r = tf_trans(T);
  const mat3_t C = tf_rot(T);
  return tf(C.transpose(), -C.transpose() * r);
}

vecx_t tf_vec(const mat4_t &T) {
  const vec3_t r = tf_trans(T);
  const quat_t q = tf_quat(T);
  vecx_t vec;
  vec.resize(7);
  vec << r.x(), r.y(), r.z(), q.x(), q.y(), q.z(), q.w();
  return vec;
}

mat4_t tf_perturb_rot(const mat4_t &T, double step_size, const int i) {
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

mat4_t tf_perturb_trans(const mat4_t &T, const double step_size, const int i) {
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

mat4_t tf_perturb(const mat4_t &T, const double dr, const double drot) {
  const double dx = randf(dr, -dr);
  const double dy = randf(dr, -dr);
  const double dz = randf(dr, -dr);
  const double drotx = randf(drot, -drot);
  const double droty = randf(drot, -drot);
  const double drotz = randf(drot, -drot);

  auto T_perturbed = tf_perturb_trans(T, dx, 0);
  T_perturbed = tf_perturb_trans(T_perturbed, dy, 1);
  T_perturbed = tf_perturb_trans(T_perturbed, dz, 2);

  T_perturbed = tf_perturb_rot(T_perturbed, drotx, 0);
  T_perturbed = tf_perturb_rot(T_perturbed, droty, 1);
  T_perturbed = tf_perturb_rot(T_perturbed, drotz, 2);

  return T_perturbed;
}

vec3_t tf_point(const mat4_t &T, const vec3_t &p) {
  return (T * p.homogeneous()).head(3);
}

/**
 * Inverse Quaternion `q`.
 */
static void quat_inv(const double q[4], double q_inv[4]) {
  q_inv[0] = q[0];
  q_inv[1] = -q[1];
  q_inv[2] = -q[2];
  q_inv[3] = -q[3];
}

/**
 * Quaternion left-multiply `p` with `q`, results are outputted to `r`.
 */
static void quat_lmul(const double p[4], const double q[4], double r[4]) {
  assert(p != NULL);
  assert(q != NULL);
  assert(r != NULL);

  const double pw = p[0];
  const double px = p[1];
  const double py = p[2];
  const double pz = p[3];

  r[0] = pw * q[0] - px * q[1] - py * q[2] - pz * q[3];
  r[1] = px * q[0] + pw * q[1] - pz * q[2] + py * q[3];
  r[2] = py * q[0] + pz * q[1] + pw * q[2] - px * q[3];
  r[3] = pz * q[0] - py * q[1] + px * q[2] + pw * q[3];
}

/**
 * Quaternion multiply `p` with `q`, results are outputted to `r`.
 */
static void quat_mul(const double p[4], const double q[4], double r[4]) {
  assert(p != NULL);
  assert(q != NULL);
  assert(r != NULL);
  quat_lmul(p, q, r);
}

void pose_diff(const double pose0[7],
               const double pose1[7],
               double *dr,
               double *drot) {
  assert(pose0 != NULL);
  assert(pose1 != NULL);

  // dr
  Eigen::Vector3d dpos;
  dpos(0) = pose0[0] - pose1[0];
  dpos(1) = pose0[1] - pose1[1];
  dpos(2) = pose0[2] - pose1[2];
  *dr = dpos.norm();

  // dq = quat_mul(quat_inv(q_meas), q_est);
  const double *q0 = pose0 + 3;
  const double *q1 = pose1 + 3;
  double q0_inv[4] = {0};
  double dq[4] = {0};
  quat_inv(q0, q0_inv);
  quat_mul(q0_inv, q1, dq);

  // dtheta = 2 * dq;
  Eigen::Vector3d dtheta;
  dtheta(0) = 2.0 * dq[1];
  dtheta(1) = 2.0 * dq[2];
  dtheta(2) = 2.0 * dq[3];
  *drot = dtheta.norm();
}

mat3_t rotx(const double theta) {
  mat3_t R;

  // clang-format off
  R << 1.0, 0.0, 0.0,
       0.0, cos(theta), sin(theta),
       0.0, -sin(theta), cos(theta);
  // clang-format on

  return R;
}

mat3_t roty(const double theta) {
  mat3_t R;

  // clang-format off
  R << cos(theta), 0.0, -sin(theta),
       0.0, 1.0, 0.0,
       sin(theta), 0.0, cos(theta);
  // clang-format on

  return R;
}

mat3_t rotz(const double theta) {
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
  const double phi = euler(0);
  const double theta = euler(1);
  const double psi = euler(2);

  const double R11 = cos(psi) * cos(theta);
  const double R21 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const double R31 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);

  const double R12 = sin(psi) * cos(theta);
  const double R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const double R32 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);

  const double R13 = -sin(theta);
  const double R23 = cos(theta) * sin(phi);
  const double R33 = cos(theta) * cos(phi);

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
  const double phi = euler.x();
  const double theta = euler.y();
  const double psi = euler.z();

  const double R11 = cos(psi) * cos(theta);
  const double R21 = sin(psi) * cos(theta);
  const double R31 = -sin(theta);

  const double R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const double R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const double R32 = cos(theta) * sin(phi);

  const double R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  const double R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  const double R33 = cos(theta) * cos(phi);

  mat3_t R;
  // clang-format off
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

quat_t euler2quat(const vec3_t &euler) {
  const double phi = euler.x();
  const double theta = euler.y();
  const double psi = euler.z();

  const double c_phi = cos(phi / 2.0);
  const double c_theta = cos(theta / 2.0);
  const double c_psi = cos(psi / 2.0);
  const double s_phi = sin(phi / 2.0);
  const double s_theta = sin(theta / 2.0);
  const double s_psi = sin(psi / 2.0);

  const double qx = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
  const double qy = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
  const double qz = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
  const double qw = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;

  const double mag = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  return quat_t{qw / mag, qx / mag, qy / mag, qz / mag};
}

mat3_t vecs2rot(const vec3_t &a_B, const vec3_t &g) {
  // Create Quaternion from two vectors
  const double cos_theta = a_B.normalized().transpose() * g.normalized();
  const double half_cos = sqrt(0.5 * (1.0 + cos_theta));
  const double half_sin = sqrt(0.5 * (1.0 - cos_theta));
  const vec3_t w = a_B.cross(g).normalized();

  const double qw = half_cos;
  const double qx = half_sin * w(0);
  const double qy = half_sin * w(1);
  const double qz = half_sin * w(2);

  // Convert Quaternion to rotation matrix
  const double qx2 = qx * qx;
  const double qy2 = qy * qy;
  const double qz2 = qz * qz;
  const double qw2 = qw * qw;

  const double R11 = qw2 + qx2 - qy2 - qz2;
  const double R12 = 2 * (qx * qy - qw * qz);
  const double R13 = 2 * (qx * qz + qw * qy);

  const double R21 = 2 * (qx * qy + qw * qz);
  const double R22 = qw2 - qx2 + qy2 - qz2;
  const double R23 = 2 * (qy * qz - qw * qx);

  const double R31 = 2 * (qx * qz - qw * qy);
  const double R32 = 2 * (qy * qz + qw * qx);
  const double R33 = qw2 - qx2 - qy2 + qz2;

  mat3_t R;
  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;
  return R;
}

mat3_t rvec2rot(const vec3_t &rvec, const double eps) {
  // Magnitude of rvec
  const double theta = sqrt(rvec.transpose() * rvec);
  // ^ basically norm(rvec), but faster

  // Check if rotation is too small
  if (theta < eps) {
    // clang-format off
    mat3_t R;
    R << 1.0, -rvec.z(), rvec.y(),
         rvec.z(), 1.0, -rvec.x(),
         -rvec.y(), rvec.x(), 1.0;
    return R;
    // clang-format on
  }

  // Convert rvec to rotation matrix
  const vec3_t rvec_normalized = rvec / theta;
  const double x = rvec_normalized(0);
  const double y = rvec_normalized(1);
  const double z = rvec_normalized(2);

  const double c = cos(theta);
  const double s = sin(theta);
  const double C = 1.0 - c;

  const double xs = x * s;
  const double ys = y * s;
  const double zs = z * s;

  const double xC = x * C;
  const double yC = y * C;
  const double zC = z * C;

  const double xyC = x * yC;
  const double yzC = y * zC;
  const double zxC = z * xC;

  // clang-format off
  mat3_t R;
  R << x * xC + c, xyC - zs, zxC + ys,
       xyC + zs, y * yC + c, yzC - xs,
       zxC - ys, yzC + xs, z * zC + c;
  return R;
  // clang-format on
}

vec3_t quat2euler(const quat_t &q) {
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();

  const double qw2 = qw * qw;
  const double qx2 = qx * qx;
  const double qy2 = qy * qy;
  const double qz2 = qz * qz;

  const double t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const double t2 = asin(2 * (qy * qw - qx * qz));
  const double t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  return vec3_t{t1, t2, t3};
}

mat3_t quat2rot(const quat_t &q) {
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();

  const double qx2 = qx * qx;
  const double qy2 = qy * qy;
  const double qz2 = qz * qz;
  const double qw2 = qw * qw;

  // Homogeneous form
  mat3_t C;
  // -- 1st row
  C(0, 0) = qw2 + qx2 - qy2 - qz2;
  C(0, 1) = 2 * (qx * qy - qw * qz);
  C(0, 2) = 2 * (qx * qz + qw * qy);
  // -- 2nd row
  C(1, 0) = 2 * (qx * qy + qw * qz);
  C(1, 1) = qw2 - qx2 + qy2 - qz2;
  C(1, 2) = 2 * (qy * qz - qw * qx);
  // -- 3rd row
  C(2, 0) = 2 * (qx * qz - qw * qy);
  C(2, 1) = 2 * (qy * qz + qw * qx);
  C(2, 2) = qw2 - qx2 - qy2 + qz2;

  return C;
}

quat_t quat_delta(const vec3_t &dalpha) {
  const double half_norm = 0.5 * dalpha.norm();
  const vec3_t vector = sinc(half_norm) * 0.5 * dalpha;
  const double scalar = cos(half_norm);
  return quat_t{scalar, vector(0), vector(1), vector(2)};
}

mat4_t quat_left(const quat_t &q) {
  // clang-format off
  mat4_t Q_left;
  Q_left <<
    q.w(), -q.x(), -q.y(), -q.z(),
    q.x(), q.w(), -q.z(), q.y(),
    q.y(), q.z(), q.w(), -q.x(),
    q.z(), -q.y(), q.x(), q.w();
  // clang-format on

  return Q_left;
}

mat4_t quat_right(const quat_t &q) {
  // clang-format off
  mat4_t Q_right;
  Q_right <<
    q.w(), -q.x(), -q.y(), -q.z(),
    q.x(), q.w(), q.z(), -q.y(),
    q.y(), -q.z(), q.w(), q.x(),
    q.z(), q.y(), -q.x(), q.w();
  // clang-format on

  return Q_right;
}

mat4_t quat_lmul(const quat_t &q) {
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();
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
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();
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

mat3_t quat_mat_xyz(const mat4_t &Q) { return Q.bottomRightCorner<3, 3>(); }

Eigen::Quaterniond
quat_average(const std::vector<Eigen::Quaterniond> &quaternions) {
  if (quaternions.empty()) {
    throw std::invalid_argument("Quaternion list is empty!");
  }

  // Accumulate the outer products of each quaternion
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
  for (const auto &q : quaternions) {
    Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
    A += q_vec * q_vec.transpose();
  }

  // Normalize the matrix
  A /= static_cast<double>(quaternions.size());

  // Perform the Eigen decomposition of the covariance matrix
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigenSolver(A);
  if (eigenSolver.info() != Eigen::Success) {
    throw std::runtime_error("Eigen decomposition failed");
  }

  // Extract the eigenvector corresponding to the largest eigenvalue
  Eigen::Vector4d average_quat_vec = eigenSolver.eigenvectors().col(3);
  // ^The last column corresponds to the largest eigenvalue

  // Convert the eigenvector to a quaternion
  Eigen::Quaterniond average_quaternion(average_quat_vec(0),
                                        average_quat_vec(1),
                                        average_quat_vec(2),
                                        average_quat_vec(3));

  // Normalize the resulting quaternion
  return average_quaternion.normalized();
}

mat3_t add_noise(const mat3_t &rot, const double n) {
  const vec3_t rpy_n{randf(-n, n), randf(-n, n), randf(-n, n)};
  const vec3_t rpy = quat2euler(quat_t{rot}) + deg2rad(rpy_n);
  return euler321(rpy);
}

vec3_t add_noise(const vec3_t &pos, const double n) {
  const vec3_t pos_n{randf(-n, n), randf(-n, n), randf(-n, n)};
  return pos + pos_n;
}

mat4_t add_noise(const mat4_t &pose, const double pos_n, const double rot_n) {
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

  // Extract roll, pitch and set yaw to 0
  const quat_t q_WS = quat_t(C_WS);
  const vec3_t rpy = quat2euler(q_WS);
  const double roll = rpy(0);
  const double pitch = rpy(1);
  const double yaw = 0.0;
  C_WS = euler321(vec3_t{roll, pitch, yaw});
}

/*****************************************************************************
 *                                TIME
 *****************************************************************************/

void timestamp_print(const timestamp_t &ts, const std::string &prefix) {
  if (prefix != "") {
    printf("%s: "
           "%" PRIu64 "\n",
           prefix.c_str(),
           ts);
  } else {
    printf("%" PRIu64 "\n", ts);
  }
}

timestamp_t sec2ts(const double sec) { return sec * 1.0e9; }

double ts2sec(const timestamp_t &ts) { return ts * 1.0e-9; }

void tsdecomp(const timestamp_t &ts, long int &sec, long int &nsec) {
  sec = ts * 1e-9;
  nsec = ts - (long int)(sec * 1e9);
}

timestamp_t tsform(const long int sec, const long int &nsec) {
  return (long int)(sec * 1e9) + nsec;
}

double ns2sec(const int64_t ns) { return ns * 1.0e-9; }

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

double time_now() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec) / 1000000.0);
}

/*****************************************************************************
 *                             INTERPOLATION
 ****************************************************************************/

quat_t slerp(const quat_t &q_start, const quat_t &q_end, const double alpha) {
  vec4_t q0{q_start.coeffs().data()};
  vec4_t q1{q_end.coeffs().data()};

  // Only unit quaternions are valid rotations.
  // Normalize to avoid undefined behavior.
  q0.normalize();
  q1.normalize();

  // Compute the cosine of the angle between the two vectors.
  double dot = q0.dot(q1);

  // If the dot product is negative, slerp won't take
  // the shorter path. Note that q1 and -q1 are equivalent when
  // the negation is applied to all four components. Fix by
  // reversing one quaternion.
  if (dot < 0.0f) {
    q1 = -q1;
    dot = -dot;
  }

  const double DOT_THRESHOLD = 0.9995;
  if (dot > DOT_THRESHOLD) {
    // If the inputs are too close for comfort, linearly interpolate
    // and normalize the result.
    vec4_t result = q0 + alpha * (q1 - q0);
    result.normalize();
    return quat_t{result(3), result(0), result(1), result(2)};
  }

  // Since dot is in range [0, DOT_THRESHOLD], acos is safe
  const double theta_0 = acos(dot);     // theta_0 = angle between input vectors
  const double theta = theta_0 * alpha; // theta = angle between q0 and result
  const double sin_theta = sin(theta);  // compute this value only once
  const double sin_theta_0 = sin(theta_0); // compute this value only once

  // == sin(theta_0 - theta) / sin(theta_0)
  const double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
  const double s1 = sin_theta / sin_theta_0;

  const vec4_t result = (s0 * q0) + (s1 * q1);
  return quat_t{result(3), result(0), result(1), result(2)};
}

mat4_t interp_pose(const mat4_t &p0, const mat4_t &p1, const double alpha) {
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
                  const double threshold) {
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

    const double diff = (ts - interp_ts[interp_idx]) * 1e-9;
    if (diff < threshold) {
      // Set interpolation start point
      ts_start = ts;
      pose0 = T;

    } else if (diff > threshold) {
      // Set interpolation end point
      ts_end = ts;
      pose1 = T;

      // Calculate alpha
      const double numerator = (interp_ts[interp_idx] - ts_start) * 1e-9;
      const double denominator = (ts_end - ts_start) * 1e-9;
      const double alpha = numerator / denominator;

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
  double diff_closest = fabs((ts - target_ts[0]) * 1e-9);
  mat4_t pose_closest = poses[0];

  size_t target_idx = 0;
  for (size_t i = 1; i < timestamps.size(); i++) {
    const timestamp_t ts = timestamps[i];
    const mat4_t pose = poses[i];

    // Find closest pose
    const double diff = fabs((ts - target_ts[target_idx]) * 1e-9);
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
        const double num = (lerp_ts[lerp_idx] - t0) * 1e-9;
        const double den = (t1 - t0) * 1e-9;
        const double alpha = num / den;

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
  double dt = 1.0 / imu.rate;

  // Check consistency of time increments:
  if (imu.started == false) {
    if (fabs(ts2sec(ts - imu.ts_prev) - dt) < (dt / 2.0)) {
      FATAL("Inconsisten sample rate with parameter setting: %f < %f",
            fabs(double(ts - imu.ts_prev) * 1.0e-9 - dt),
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
  // w_WS_S = C_SW * w_WS_W + imu.b_g + w_g * imu.sigma_g_c * sqrt(dt);
  UNUSED(w_g);
  w_WS_S = C_SW * w_WS_W;

  // Compute accel measurement
  const vec3_t g{0.0, 0.0, imu.g}; // Gravity vector
  const vec3_t w_a = mvn(rndeng);  // Accel white noise
  // a_WS_S = C_SW * (a_WS_W + g) + imu.b_a + w_a * imu.sigma_a_c * sqrt(dt);
  UNUSED(w_a);
  a_WS_S = C_SW * (a_WS_W + g);

  imu.ts_prev = ts;
}

/*****************************************************************************
 *                                OPENCV
 ****************************************************************************/

cv::Mat gray2rgb(const cv::Mat &image) {
  const int image_height = image.rows;
  const int image_width = image.cols;
  cv::Mat out_image(image_height, image_width, CV_8UC3);

  if (image.channels() == 1) {
    cv::cvtColor(image, out_image, cv::COLOR_GRAY2RGB);
  } else {
    return image.clone();
  }

  return out_image;
}

cv::Mat rgb2gray(const cv::Mat &image) {
  cv::Mat image_gray;

  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  } else {
    return image.clone();
  }

  return image_gray;
}

} // namespace yac
