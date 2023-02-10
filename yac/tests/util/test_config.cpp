#include "../munit.hpp"
#include "util/config.hpp"

namespace yac {

#ifndef TEST_PATH
  #define TEST_PATH "."
#endif

#define TEST_CONFIG TEST_PATH "/test_data/core/config/config.yaml"

int test_config_constructor() {
  config_t config{TEST_CONFIG};

  MU_CHECK(config.ok == true);
  MU_CHECK(config.file_path == TEST_CONFIG);

  return 0;
}

int test_config_parse_primitive() {
  config_t config{TEST_CONFIG};

  // INTEGER
  int i = 0;
  parse(config, "int", i);
  MU_CHECK(1 == i);

  // FLOAT
  float f = 0.0f;
  parse(config, "float", f);
  MU_CHECK(fltcmp(2.2, f) == 0);

  // real_t
  real_t d = 0.0;
  parse(config, "double", d);
  MU_CHECK(fltcmp(3.3, d) == 0);

  // STRING
  std::string s;
  parse(config, "string", s);
  MU_CHECK("hello world!" == s);

  return 0;
}

int test_config_parse_array() {
  config_t config{TEST_CONFIG};

  // BOOL ARRAY
  std::vector<bool> b_array;
  parse(config, "bool_array", b_array);
  MU_CHECK(b_array[0]);
  MU_CHECK(b_array[1] == false);
  MU_CHECK(b_array[2]);
  MU_CHECK(b_array[3] == false);

  // INTEGER
  std::vector<int> i_array;
  parse(config, "int_array", i_array);
  for (int i = 0; i < 4; i++) {
    MU_CHECK(i_array[i] == i + 1);
  }

  // FLOAT
  std::vector<float> f_array;
  parse(config, "float_array", f_array);
  for (int i = 0; i < 4; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, f_array[i]);
  }

  // real_t
  std::vector<real_t> d_array;
  parse(config, "double_array", d_array);
  for (int i = 0; i < 4; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, d_array[i]);
  }

  // STRING
  std::vector<std::string> s_array;
  parse(config, "string_array", s_array);
  MU_CHECK(s_array[0] == "1.1");
  MU_CHECK(s_array[1] == "2.2");
  MU_CHECK(s_array[2] == "3.3");
  MU_CHECK(s_array[3] == "4.4");

  return 0;
}

int test_config_parse_vector() {
  config_t config{TEST_CONFIG};

  // VECTOR 2
  vec2_t vec2;
  parse(config, "vector2", vec2);
  std::cout << vec2.transpose() << std::endl;
  MU_CHECK_FLOAT(1.1, vec2(0));
  MU_CHECK_FLOAT(2.2, vec2(1));

  // VECTOR 3
  vec3_t vec3;
  parse(config, "vector3", vec3);
  std::cout << vec3.transpose() << std::endl;
  MU_CHECK_FLOAT(1.1, vec3(0));
  MU_CHECK_FLOAT(2.2, vec3(1));
  MU_CHECK_FLOAT(3.3, vec3(2));

  // VECTOR 4
  vec4_t vec4;
  parse(config, "vector4", vec4);
  std::cout << vec4.transpose() << std::endl;
  MU_CHECK_FLOAT(1.1, vec4(0));
  MU_CHECK_FLOAT(2.2, vec4(1));
  MU_CHECK_FLOAT(3.3, vec4(2));
  MU_CHECK_FLOAT(4.4, vec4(3));

  // VECTOR X
  vecx_t vecx;
  parse(config, "vector", vecx);
  std::cout << vecx.transpose() << std::endl;
  for (int i = 0; i < 9; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, vecx(i));
  }

  return 0;
}

int test_config_parse_matrix() {
  config_t config{TEST_CONFIG};

  // MATRIX 2
  mat2_t mat2;
  parse(config, "matrix2", mat2);
  std::cout << mat2 << std::endl;

  MU_CHECK_FLOAT(1.1, mat2(0, 0));
  MU_CHECK_FLOAT(2.2, mat2(0, 1));
  MU_CHECK_FLOAT(3.3, mat2(1, 0));
  MU_CHECK_FLOAT(4.4, mat2(1, 1));

  // MATRIX 3
  mat3_t mat3;
  parse(config, "matrix3", mat3);
  std::cout << mat3 << std::endl;

  int index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, mat3(i, j));
      index++;
    }
  }

  // MATRIX 4
  mat4_t mat4;
  parse(config, "matrix4", mat4);
  std::cout << mat4 << std::endl;

  index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, mat4(i, j));
      index++;
    }
  }

  // MATRIX X
  matx_t matx;
  parse(config, "matrix", matx);
  std::cout << matx << std::endl;

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, matx(i, j));
      index++;
    }
  }

  // CV MATRIX
  cv::Mat cvmat;
  parse(config, "matrix", cvmat);
  std::cout << cvmat << std::endl;

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, cvmat.at<real_t>(i, j));
      index++;
    }
  }

  return 0;
}

int test_config_parser_full_example() {
  config_t config{TEST_CONFIG};

  // Primitives
  bool b = false;
  int i = 0;
  float f = 0.0f;
  double d = 0.0;

  parse(config, "bool", b);
  parse(config, "int", i);
  parse(config, "float", f);
  parse(config, "double", d);

  // Array
  std::string s;
  std::vector<bool> b_array;
  std::vector<int> i_array;
  std::vector<float> f_array;
  std::vector<double> d_array;
  std::vector<std::string> s_array;

  parse(config, "string", s);
  parse(config, "bool_array", b_array);
  parse(config, "int_array", i_array);
  parse(config, "float_array", f_array);
  parse(config, "double_array", d_array);
  parse(config, "string_array", s_array);

  // Vectors
  vec2_t vec2;
  vec3_t vec3;
  vec4_t vec4;
  vecx_t vecx;

  parse(config, "vector2", vec2);
  parse(config, "vector3", vec3);
  parse(config, "vector4", vec4);
  parse(config, "vector", vecx);

  // Matrices
  mat2_t mat2;
  mat3_t mat3;
  mat4_t mat4;
  matx_t matx;

  parse(config, "matrix2", mat2);
  parse(config, "matrix3", mat3);
  parse(config, "matrix4", mat4);
  parse(config, "matrix", matx);

  cv::Mat cvmat;
  parse(config, "matrix", cvmat);
  parse(config, "non_existant_key", cvmat, true);

  std::cout << "bool: " << b << std::endl;
  std::cout << "int: " << i << std::endl;
  std::cout << "float: " << f << std::endl;
  std::cout << "double: " << d << std::endl;
  std::cout << "string: " << s << std::endl;
  std::cout << std::endl;

  std::cout << "vector2: " << vec2.transpose() << std::endl;
  std::cout << "vector3: " << vec3.transpose() << std::endl;
  std::cout << "vector4: " << vec4.transpose() << std::endl;
  std::cout << "vector: " << vecx.transpose() << std::endl;
  std::cout << std::endl;

  std::cout << "matrix2: \n" << mat2 << std::endl;
  std::cout << "matrix3: \n" << mat3 << std::endl;
  std::cout << "matrix4: \n" << mat4 << std::endl;
  std::cout << "matrix: \n" << matx << std::endl;
  std::cout << "cvmatrix: \n" << cvmat << std::endl;
  std::cout << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_config_constructor);
  MU_ADD_TEST(test_config_parse_primitive);
  MU_ADD_TEST(test_config_parse_array);
  MU_ADD_TEST(test_config_parse_vector);
  MU_ADD_TEST(test_config_parse_matrix);
  MU_ADD_TEST(test_config_parser_full_example);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
