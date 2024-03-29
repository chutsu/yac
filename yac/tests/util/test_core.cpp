#include <unistd.h>

#include "../munit.hpp"
#include "util/core.hpp"
#include "util/fs.hpp"
#include "util/data.hpp"

namespace yac {

/******************************************************************************
 * ALGEBRA
 *****************************************************************************/

int test_sign() {
  MU_CHECK(sign(1.0) == 1);
  MU_CHECK(fltcmp(sign(0.0), 0.0) == 0);
  MU_CHECK(sign(-1.0) == -1);
  return 0;
}

int test_fltcmp() {
  MU_CHECK(fltcmp(1.0, 1.0) == 0);
  MU_CHECK(fltcmp(1.0, 0.9999) == 1);
  MU_CHECK(fltcmp(1.0, 0.0) == 1);
  MU_CHECK(fltcmp(0.0, 1.0) == -1);
  return 0;
}

int test_linspace() {
  const auto range = linspace(0.0, 5.0, 10);

  for (const auto &el : range) {
    std::cout << el << std::endl;
  }

  MU_CHECK(fltcmp(range.front(), 0.0) == 0);
  MU_CHECK(fltcmp(range.back(), 5.0) == 0);
  MU_CHECK(range.size() == 10);

  return 0;
}

int test_linspace_timestamps() {
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  const timestamps_t range = linspace(ts_start, ts_end, 10);

  for (const auto &el : range) {
    std::cout << el << std::endl;
  }

  MU_CHECK(range.front() == 0);
  MU_CHECK(range.back() == 5e9);
  MU_CHECK(range.size() == 10);

  return 0;
}

/******************************************************************************
 * LINEAR ALGEBRA
 *****************************************************************************/

int test_zeros() {
  matx_t A = zeros(2, 2);

  MU_CHECK(A.rows() == 2);
  MU_CHECK(A.cols() == 2);

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      MU_CHECK_FLOAT(0.0, A(i, j));
    }
  }

  return 0;
}

int test_I() {
  matx_t A = I(2, 2);

  MU_CHECK(A.rows() == 2);
  MU_CHECK(A.cols() == 2);

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (i != j) {
        MU_CHECK_FLOAT(0.0, A(i, j));
      } else {
        MU_CHECK_FLOAT(1.0, A(i, j));
      }
    }
  }

  return 0;
}

int test_ones() {
  matx_t A = ones(2, 2);

  MU_CHECK(A.rows() == 2);
  MU_CHECK(A.cols() == 2);
  std::cout << A << std::endl;

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      MU_CHECK_FLOAT(1.0, A(i, j));
    }
  }

  return 0;
}

int test_hstack() {
  mat2_t A;
  A.fill(1);

  mat2_t B;
  B.fill(2);

  matx_t C = hstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK(C.rows() == 2);
  MU_CHECK(C.cols() == 4);
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(0, 2, 2, 2).isApprox(B));

  return 0;
}

int test_vstack() {
  mat2_t A;
  A.fill(1);

  mat2_t B;
  B.fill(2);

  matx_t C = vstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK(C.rows() == 4);
  MU_CHECK(C.cols() == 2);
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(2, 0, 2, 2).isApprox(B));

  return 0;
}

int test_dstack() {
  mat2_t A;
  A.fill(1);

  mat2_t B;
  B.fill(2);

  matx_t C = dstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK(C.rows() == 4);
  MU_CHECK(C.cols() == 4);
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(0, 2, 2, 2).isApprox(zeros(2, 2)));
  MU_CHECK(C.block(2, 2, 2, 2).isApprox(B));
  MU_CHECK(C.block(2, 0, 2, 2).isApprox(zeros(2, 2)));

  return 0;
}

int test_skew() {
  vec3_t v{1.0, 2.0, 3.0};
  mat3_t X = skew(v);

  mat3_t X_expected;
  // clang-format off
  X_expected << 0.0, -3.0, 2.0,
                3.0, 0.0, -1.0,
                -2.0, 1.0, 0.0;
  // clang-format on

  MU_CHECK(X_expected.isApprox(X));

  return 0;
}

int test_skewsq() { return 0; }

int test_enforce_psd() {
  mat3_t A;
  // clang-format off
  A << 0.0, -3.0, 2.0,
       3.0, 0.0, -1.0,
       -2.0, 1.0, 0.0;
  // clang-format on

  mat3_t B = enforce_psd(A);
  for (int i = 0; i < B.rows(); i++) {
    for (int j = 0; j < B.cols(); j++) {
      MU_CHECK(B(i, j) >= 0);
    }
  }

  return 0;
}

int test_nullspace() {
  mat3_t A;
  // clang-format off
  A << 1.0, 2.0, 3.0,
       1.0, 2.0, 3.0,
       1.0, 2.0, 3.0;
  // clang-format on

  matx_t B = nullspace(A);
  std::cout << B << std::endl;
  std::cout << A * B << std::endl;

  matx_t C = A * B;
  for (int i = 0; i < C.rows(); i++) {
    for (int j = 0; j < C.cols(); j++) {
      MU_CHECK_FLOAT(0.0, C(i, j));
    }
  }

  return 0;
}

real_t covar_recover(const long i,
                     const long l,
                     const sp_mat_t &L,
                     const vecx_t &diag,
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

    for (Eigen::SparseVector<double>::InnerIterator it(L_row); it; ++it) {
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
  // Decompose H using Cholesky decomposition (LLT)
  const Eigen::LLT<matx_t> llt(H);
  if (llt.info() != Eigen::Success) {
    FATAL("Failed to decompose H using cholesky decomposition!");
  }
  const matx_t L = llt.matrixL();

  // // Decompose H using QR decomposition
  // const Eigen::HouseholderQR<matx_t> qr(H);
  // const matx_t L = qr.matrixQR().triangularView<Eigen::Lower>();

  // Pre-calculate diagonal inverses
  vecx_t diag(L.rows());
  size_t nb_rows = L.rows();
  for (size_t i = 0; i < nb_rows; i++) {
    if (L(i, i) < 1e-8) {
      FATAL("L(%zu, %zu): %f < 1e-8!", i, i, L(i, i));
    }
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

int test_covar_recover() {
  profiler_t profile;

  matx_t H;
  mat2csv("/tmp/H.csv", H);

  // matx_t covar;
  // mat2csv("/tmp/covar.csv", covar);

  // matx_t H_diag = H.diagonal().asDiagonal();
  // H = H + 1e-5 * H_diag;

  profile.start("H_inv");
  matx_t covar_est = pinv(H);
  // matx_t covar_est = H.inverse();
  profile.print("H_inv");

  // Recover
  mat_indicies_t indicies;
  for (int i = H.rows() - 16; i < H.rows(); i++) {
    // for (int j = H.rows() - 16; j < H.rows(); j++) {
    indicies.emplace_back(i, i);
    // }
  }

  profile.start("covar_recover");
  mat_hash_t results = covar_recover(H, indicies);
  profile.print("covar_recover");

  // for (const auto &ij : indicies) {
  //   const long i = ij.first;
  //   const long j = ij.second;
  //   if (i == j) {
  //     printf("(%ld, %ld) ", i, j);
  //     printf("recovered: %e\n", results[i][j]);
  //   }
  // }

  // print_vector("H_cam_diag", H.diagonal().tail(16));

  for (const auto &ij : indicies) {
    const long i = ij.first;
    const long j = ij.second;
    printf("(%ld, %ld) ", i, j);
    MU_CHECK(std::isnan(results[i][j]) == false);
    // printf("result: %f\n", results[i][j]);
    printf("result: %f recovered: %f\n", results[i][j], covar_est(i, j));
    // MU_CHECK(fltcmp(results[i][j], covar_est(i, j)) == 0);
  }

  return 0;
}

/******************************************************************************
 * GEOMETRY
 *****************************************************************************/

int test_deg2rad_rad2deg() {
  real_t d_deg;
  real_t d_rad;

  d_deg = 10;
  d_rad = deg2rad(d_deg);
  MU_CHECK_FLOAT(d_deg, rad2deg(d_rad));

  return 0;
}

int test_wrap180() {
  real_t retval;

  // normal cases
  retval = wrap180(90.0);
  MU_CHECK_FLOAT(90.0, retval);

  retval = wrap180(180.0);
  MU_CHECK_FLOAT(-180.0, retval);

  retval = wrap180(270.0);
  MU_CHECK_FLOAT(-90.0, retval);

  retval = wrap180(360.0);
  MU_CHECK_FLOAT(0.0, retval);

  // edge cases
  retval = wrap180(-180.0);
  MU_CHECK_FLOAT(-180.0, retval);

  retval = wrap180(-90.0);
  MU_CHECK_FLOAT(-90.0, retval);

  retval = wrap180(450.0);
  MU_CHECK_FLOAT(90.0, retval);

  return 0;
}

int test_wrap360() {
  real_t retval;

  // normal cases
  retval = wrap360(90.0);
  MU_CHECK_FLOAT(90.0, retval);

  retval = wrap360(180.0);
  MU_CHECK_FLOAT(180.0, retval);

  retval = wrap360(270.0);
  MU_CHECK_FLOAT(270.0, retval);

  retval = wrap360(360.0);
  MU_CHECK_FLOAT(0.0, retval);

  retval = wrap360(450.0);
  MU_CHECK_FLOAT(90.0, retval);

  // edge cases
  retval = wrap360(-180.0);
  MU_CHECK_FLOAT(180.0, retval);

  retval = wrap360(-90.0);
  MU_CHECK_FLOAT(270.0, retval);

  return 0;
}

int test_cross_track_error() {
  vec2_t pos, p1, p2;

  pos << 2, 2;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_FLOAT(0.0, cross_track_error(p1, p2, pos));

  pos << 2, 3;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK(0.0 < cross_track_error(p1, p2, pos));

  return 0;
}

int test_point_left_right() {
  vec2_t pos, p1, p2;

  pos << 2, 3;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK(point_left_right(p1, p2, pos) == 1);

  pos << 2, 1;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK(point_left_right(p1, p2, pos) == 2);

  pos << 2, 2;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK(point_left_right(p1, p2, pos) == 0);

  pos << 2, 1;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK(point_left_right(p1, p2, pos) == 1);

  pos << 2, 3;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK(point_left_right(p1, p2, pos) == 2);

  pos << 2, 2;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK(point_left_right(p1, p2, pos) == 0);

  return 0;
}

int test_closest_point() {
  int retval;
  vec2_t p1, p2, p3, closest;

  // setup
  p1 << 0, 0;
  p2 << 5, 0;

  // point middle of point a, b
  p3 << 2, 2;
  retval = closest_point(p1, p2, p3, closest);
  MU_CHECK(retval == 0);
  MU_CHECK_FLOAT(2.0, closest(0));
  MU_CHECK_FLOAT(0.0, closest(1));

  // // point before of point a
  // p3 << -1, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(1, retval);
  // MU_CHECK_FLOAT(-1.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));
  //
  // // point after point b
  // p3 << 6, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(2, retval);
  // MU_CHECK_FLOAT(6.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));
  //
  // // if point 1 and 2 are same
  // p1 << 0, 0;
  // p2 << 0, 0;
  // p3 << 0, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(-1, retval);
  // MU_CHECK_FLOAT(0.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));

  return 0;
}

int test_fit_circle() {
  int nb_points = 1000;
  int nb_tests = 10;

  for (int i = 0; i < nb_tests; i++) {
    vec2s_t points;
    double gnd_cx = randf(-5.0, 5.0);
    double gnd_cy = randf(-5.0, 5.0);
    double gnd_radius = randf(0.1, 5.0);

    double theta = 0.0;
    double dtheta = 2.0 * M_PI / (double)nb_points;
    while (theta <= 2.0 * M_PI) {
      const double x = gnd_radius * cos(theta) + gnd_cx;
      const double y = gnd_radius * sin(theta) + gnd_cy;
      points.emplace_back(x, y);
      theta += dtheta;
    }

    double est_cx = 0.0;
    double est_cy = 0.0;
    double est_radius = 0.0;
    fit_circle(points, est_cx, est_cy, est_radius);

    // printf("true_cx: %f, est_cx: %f\n", gnd_cx, est_cx);
    // printf("true_cy: %f, est_cy: %f\n", gnd_cy, est_cy);
    // printf("true_radius: %f, est_radius: %f\n", gnd_radius, est_radius);

    MU_CHECK(fabs(est_cx - gnd_cx) < 1e-4);
    MU_CHECK(fabs(est_cy - gnd_cy) < 1e-4);
    MU_CHECK(fabs(est_radius - gnd_radius) < 1e-4);
  }

  return 0;
}

int test_latlon_offset() {
  // UWaterloo 110 yards Canadian Football field from one end to another
  real_t lat = 43.474357;
  real_t lon = -80.550415;

  real_t offset_N = 44.1938;
  real_t offset_E = 90.2336;

  real_t lat_new = 0.0;
  real_t lon_new = 0.0;

  // calculate football field GPS coordinates
  latlon_offset(lat, lon, offset_N, offset_E, &lat_new, &lon_new);
  std::cout << "lat new: " << lat_new << std::endl;
  std::cout << "lon new: " << lon_new << std::endl;

  // gps coordinates should be close to (43.474754, -80.549298)
  MU_CHECK_NEAR(43.474754, lat_new, 0.0015);
  MU_CHECK_NEAR(-80.549298, lon_new, 0.0015);

  return 0;
}

int test_latlon_diff() {
  // UWaterloo 110 yards Canadian Football field from one end to another
  real_t lat_ref = 43.474357;
  real_t lon_ref = -80.550415;
  real_t lat = 43.474754;
  real_t lon = -80.549298;

  real_t dist_N = 0.0;
  real_t dist_E = 0.0;

  // calculate football field distance
  latlon_diff(lat_ref, lon_ref, lat, lon, &dist_N, &dist_E);
  real_t dist = sqrt(pow(dist_N, 2) + pow(dist_E, 2));
  std::cout << "distance north: " << dist_N << std::endl;
  std::cout << "distance east: " << dist_E << std::endl;

  // 110 yards is approx 100 meters
  MU_CHECK_NEAR(100, dist, 1.0);

  return 0;
}

int test_latlon_dist() {
  // UWaterloo 110 yards Canadian Football field from one end to another
  real_t lat_ref = 43.474357;
  real_t lon_ref = -80.550415;
  real_t lat = 43.474754;
  real_t lon = -80.549298;

  // calculate football field distance
  real_t dist = latlon_dist(lat_ref, lon_ref, lat, lon);
  std::cout << "distance: " << dist << std::endl;

  // 110 yards is approx 100 meters
  MU_CHECK_NEAR(100, dist, 1.0);

  return 0;
}

/******************************************************************************
 * STATISTICS
 *****************************************************************************/

int test_median() {
  std::vector<real_t> v;

  v.push_back(6);
  v.push_back(3);
  v.push_back(4);
  v.push_back(1);
  v.push_back(5);
  v.push_back(8);

  MU_CHECK_FLOAT(4.5, median(v));

  v.push_back(9);
  MU_CHECK_FLOAT(5.0, median(v));

  return 0;
}

int test_mvn() {
  std::default_random_engine engine;
  const int nb_tests = 10000; // number of experiments

  matx_t results{3, nb_tests};
  for (int i = 0; i < nb_tests; i++) {
    results.block<3, 1>(0, i) = mvn(engine);
  }
  mat2csv("/tmp/mvn.csv", results);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_mvn.m /tmp/mvn.csv");
  }

  return 0;
}

int test_gauss_normal() {
  const int nb_tests = 10000; // number of experiments

  vecx_t results{nb_tests};
  for (int i = 0; i < nb_tests; i++) {
    results(i) = gauss_normal();
  }
  mat2csv("/tmp/gauss_normal.csv", results);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_gauss_normal.m /tmp/gauss_normal.csv");
  }

  return 0;
}

/******************************************************************************
 * TRANSFORM
 *****************************************************************************/

int test_tf_rot() {
  mat4_t T_WS = I(4);
  T_WS.block<3, 3>(0, 0) = 1.0 * ones(3);
  T_WS.block<3, 1>(0, 3) = 2.0 * ones(3, 1);

  MU_CHECK((1.0 * ones(3) - tf_rot(T_WS)).norm() < 1e-5);

  return 0;
}

int test_tf_trans() {
  mat4_t T_WS = I(4);
  T_WS.block<3, 3>(0, 0) = 1.0 * ones(3);
  T_WS.block<3, 1>(0, 3) = 2.0 * ones(3, 1);

  MU_CHECK((2.0 * ones(3, 1) - tf_trans(T_WS)).norm() < 1e-5);

  return 0;
}

/******************************************************************************
 * TIME
 *****************************************************************************/

int test_ts2sec() {
  timestamp_t ts = 10 * 1e9;
  MU_CHECK(fltcmp(ts2sec(ts), 10.0) == 0);
  return 0;
}

int test_ns2sec() {
  timestamp_t ts = 10 * 1e9;
  MU_CHECK(fltcmp(ns2sec(ts), 10.0) == 0);
  return 0;
}

int test_tic_toc() {
  struct timespec start = tic();
  usleep(10 * 1000);
  MU_CHECK(toc(&start) < 0.012);
  MU_CHECK(toc(&start) > 0.009);
  MU_CHECK(mtoc(&start) < 11.0);
  MU_CHECK(mtoc(&start) > 9.0);

  return 0;
}

/******************************************************************************
 * INTERPOLATION
 *****************************************************************************/

#define CAM0_CSV_PATH "/tmp/test_measurement-cam0.csv"
#define ACCEL0_CSV_PATH "/tmp/test_measurement-accel.csv"
#define GYRO0_CSV_PATH "/tmp/test_measurement-gyro.csv"

void load_camera_data(const std::string &data_path,
                      std::deque<timestamp_t> &camera_ts,
                      const bool skip_header = false) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int line_no = 0; line_no < nb_rows; line_no++) {
    // Skip first line
    if (line_no == 0 && skip_header) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    int retval = fscanf(fp, "%" SCNu64, &ts);
    if (retval != 1) {
      FATAL("Failed to parse line [%d]", line_no);
    }

    camera_ts.push_back(ts);
  }
  fclose(fp);
}

void load_accel_data(const std::string &data_path,
                     std::deque<timestamp_t> &accel_ts,
                     std::deque<vec3_t> &accel_data,
                     const bool skip_header = false) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int line_no = 0; line_no < nb_rows; line_no++) {
    // Skip first line
    if (line_no == 0 && skip_header) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double a_x, a_y, a_z = 0.0;
    int retval = fscanf(fp, "%" SCNu64 ",%lf,%lf,%lf", &ts, &a_x, &a_y, &a_z);
    if (retval != 4) {
      FATAL("Failed to parse line [%d]", line_no);
    }

    accel_ts.push_back(ts);
    accel_data.emplace_back(a_x, a_y, a_z);
  }
  fclose(fp);
}

void load_gyro_data(const std::string &data_path,
                    std::deque<timestamp_t> &gyro_ts,
                    std::deque<vec3_t> &gyro_data,
                    const bool skip_header = false) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int line_no = 0; line_no < nb_rows; line_no++) {
    // Skip first line
    if (line_no == 0 && skip_header) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double w_x, w_y, w_z = 0.0;
    int retval = fscanf(fp, "%" SCNu64 ",%lf,%lf,%lf", &ts, &w_x, &w_y, &w_z);
    if (retval != 4) {
      FATAL("Failed to parse line [%d]", line_no);
    }

    gyro_ts.push_back(ts);
    gyro_data.emplace_back(w_x, w_y, w_z);
  }
  fclose(fp);
}

void save_interpolated_cam0_data(const std::deque<timestamp_t> cam0_ts) {
  std::string out_path = "/tmp/lerp_data-cam0_ts.csv";
  if (ts2csv(out_path, cam0_ts) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }
}

void save_interpolated_gyro_data(const std::deque<timestamp_t> gyro_ts,
                                 const std::deque<vec3_t> gyro_data) {
  std::string out_path = "/tmp/lerp_data-gyro_ts.csv";
  if (ts2csv(out_path, gyro_ts) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }

  out_path = "/tmp/lerp_data-gyro_data.csv";
  if (vec2csv(out_path, gyro_data) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }
}

void save_interpolated_accel_data(const std::deque<timestamp_t> accel_ts,
                                  const std::deque<vec3_t> accel_data) {
  std::string out_path = "/tmp/lerp_data-accel_ts.csv";
  if (ts2csv(out_path, accel_ts) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }

  out_path = "/tmp/lerp_data-accel_data.csv";
  if (vec2csv(out_path, accel_data) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }
}

struct test_data_t {
  std::deque<timestamp_t> cam0_ts;

  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel_data;

  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro_data;

  test_data_t() {
    OCTAVE_SCRIPT("scripts/measurement/sim_measurements.m");
    load_camera_data(CAM0_CSV_PATH, cam0_ts);
    load_gyro_data(GYRO0_CSV_PATH, gyro_ts, gyro_data);
    load_accel_data(ACCEL0_CSV_PATH, accel_ts, accel_data);
  }

  void flatten(std::deque<std::string> &buf_seq,
               std::deque<timestamp_t> &buf_ts,
               std::deque<vec3_t> &buf_data) {
    size_t accel_idx = 0;
    size_t gyro_idx = 0;

    const auto add_gyro = [&]() {
      buf_seq.push_back("G");
      buf_ts.push_back(gyro_ts[gyro_idx]);
      buf_data.push_back(gyro_data[gyro_idx]);
      gyro_idx++;
    };

    const auto add_accel = [&]() {
      buf_seq.push_back("A");
      buf_ts.push_back(accel_ts[accel_idx]);
      buf_data.push_back(accel_data[accel_idx]);
      accel_idx++;
    };

    while (true) {
      if (accel_idx >= accel_ts.size() && gyro_idx >= gyro_ts.size()) {
        break;
      } else if (accel_idx >= accel_ts.size()) {
        add_gyro();
      } else if (gyro_idx >= gyro_ts.size()) {
        add_accel();
      } else if (accel_ts[accel_idx] < gyro_ts[gyro_idx]) {
        add_accel();
      } else if (accel_ts[accel_idx] > gyro_ts[gyro_idx]) {
        add_gyro();
      } else if (accel_ts[accel_idx] == gyro_ts[gyro_idx]) {
        add_gyro();
        add_accel();
      }
    }

    // Make sure the timestamps are not going backwards at any point
    timestamp_t ts_prev = buf_ts.at(0);
    for (size_t i = 1; i < buf_ts.size(); i++) {
      const timestamp_t ts_now = buf_ts.at(i);
      if (ts_now < ts_prev) {
        printf("ERROR! [ts_now < ts_prev] timestamps are not correct!");
      }
      ts_prev = ts_now;
    }
  }
};

int test_lerp() {
  const vec2_t a{0.0, 5.0};
  const vec2_t b{5.0, 0.0};
  const vec2_t result = lerp(a, b, 0.8);
  std::cout << result << std::endl;

  return 0;
}

int test_slerp() {
  for (int i = 0; i < 1000; i++) {
    const real_t roll_start = randf(-1.0, 1.0);
    const real_t pitch_start = randf(-1.0, 1.0);
    const real_t yaw_start = randf(-1.0, 1.0);
    const vec3_t rpy_start{roll_start, pitch_start, yaw_start};

    const real_t roll_end = randf(-1.0, 1.0);
    const real_t pitch_end = randf(-1.0, 1.0);
    const real_t yaw_end = randf(-1.0, 1.0);
    const vec3_t rpy_end{roll_end, pitch_end, yaw_end};

    const real_t alpha = randf(0.0, 1.0);
    const auto q0 = quat_t{euler321(rpy_start)};
    const auto q1 = quat_t{euler321(rpy_end)};
    const auto expect = q0.slerp(alpha, q1);
    const auto actual = slerp(q0, q1, alpha);

    MU_CHECK_FLOAT(expect.w(), actual.w());
    MU_CHECK_FLOAT(expect.x(), actual.x());
    MU_CHECK_FLOAT(expect.y(), actual.y());
    MU_CHECK_FLOAT(expect.z(), actual.z());
  }

  return 0;
}

int test_interp_pose() {
  const vec3_t trans_start{0.0, 0.0, 0.0};
  const vec3_t trans_end{1.0, 2.0, 3.0};
  const vec3_t rpy_start{0.0, 0.0, 0.0};
  const vec3_t rpy_end{deg2rad(10.0), deg2rad(0.0), deg2rad(0.0)};

  const auto pose_start = tf(euler321(rpy_start), trans_start);
  const auto pose_end = tf(euler321(rpy_end), trans_end);
  const auto pose_interp = interp_pose(pose_start, pose_end, 0.5);

  std::cout << "pose_start:\n" << pose_start << std::endl << std::endl;
  std::cout << "pose_end:\n" << pose_end << std::endl << std::endl;
  std::cout << "pose_interp:\n" << pose_interp << std::endl << std::endl;

  MU_CHECK((tf_trans(pose_interp) - vec3_t{0.5, 1.0, 1.5}).norm() - 1e-5);

  return 0;
}

int test_interp_poses() {
  // Create timestamps
  timestamps_t timestamps;
  timestamps.push_back(1500000000000000000);
  timestamps.push_back(1500000000200000000);
  timestamps.push_back(1500000000400000000);
  timestamps.push_back(1500000000600000000);
  timestamps.push_back(1500000000800000000);
  timestamps.push_back(1500000001000000000);
  timestamps.push_back(1500000001200000000);
  timestamps.push_back(1500000001400000000);
  timestamps.push_back(1500000001600000000);
  timestamps.push_back(1500000001800000000);

  // Create poses
  mat4s_t poses;
  const vec3_t trans_start{0.0, 0.0, 0.0};
  const vec3_t trans_end{1.0, 2.0, 3.0};
  const vec3_t rpy_start{0.0, 0.0, 0.0};
  const vec3_t rpy_end{deg2rad(90.0), deg2rad(90.0), deg2rad(90.0)};

  const size_t nb_timestamps = timestamps.size();
  const real_t step = 1.0 / nb_timestamps;
  vec3s_t trans_interp_gnd;
  for (size_t i = 0; i < nb_timestamps; i++) {
    const auto trans_interp = lerp(trans_start, trans_end, step * i);
    const auto rpy_interp = lerp(rpy_start, rpy_end, step * i);
    const auto rot = euler321(rpy_interp);
    const auto T = tf(rot, trans_interp);

    poses.push_back(T);
  }

  // Create interpolate points in time
  timestamps_t interp_ts;
  // interp_ts.push_back(1500000000100000000);
  // interp_ts.push_back(1500000000300000000);
  // interp_ts.push_back(1500000000500000000);
  // interp_ts.push_back(1500000000700000000);
  // interp_ts.push_back(1500000000900000000);
  interp_ts.push_back(1500000001100000000);
  // interp_ts.push_back(1500000001300000000);
  // interp_ts.push_back(1500000001500000000);
  // interp_ts.push_back(1500000001700000000);

  // Interpolate poses
  mat4s_t interped_poses;
  interp_poses(timestamps, poses, interp_ts, interped_poses);
  // MU_CHECK(interped_poses.size() == poses.size());

  return 0;
}

int test_closest_poses() {
  // Create timestamps
  timestamps_t timestamps;
  timestamps.push_back(1500000000000000000);
  timestamps.push_back(1500000000200000000);
  timestamps.push_back(1500000000400000000);
  timestamps.push_back(1500000000600000000);
  timestamps.push_back(1500000000800000000);
  timestamps.push_back(1500000001000000000);
  timestamps.push_back(1500000001200000000);
  timestamps.push_back(1500000001400000000);
  timestamps.push_back(1500000001600000000);
  timestamps.push_back(1500000001800000000);

  // Create poses
  mat4s_t poses;
  const vec3_t trans_start{0.0, 0.0, 0.0};
  const vec3_t trans_end{1.0, 2.0, 3.0};
  const vec3_t rpy_start{0.0, 0.0, 0.0};
  const vec3_t rpy_end{deg2rad(1.0), deg2rad(2.0), deg2rad(3.0)};

  const size_t nb_timestamps = timestamps.size();
  const real_t step = 1.0 / nb_timestamps;
  vec3s_t trans_interp_gnd;
  for (size_t i = 0; i < nb_timestamps; i++) {
    const auto trans_interp = lerp(trans_start, trans_end, step * i);
    const auto rpy_interp = lerp(rpy_start, rpy_end, step * i);
    const auto rot = euler321(rpy_interp);
    const auto T = tf(rot, trans_interp);
    poses.push_back(T);
  }

  // Create interpolate points in time
  timestamps_t target_ts;
  target_ts.push_back(1500000000100000000);
  // target_ts.push_back(1500000000300000000);
  // target_ts.push_back(1500000000500000000);
  target_ts.push_back(1500000000700000000);
  // target_ts.push_back(1500000000900000000);
  // target_ts.push_back(1500000001100000000);
  // target_ts.push_back(1500000001300000000);
  // target_ts.push_back(1500000001500000000);
  // target_ts.push_back(1500000001700000000);

  // Interpolate poses
  mat4s_t result;
  closest_poses(timestamps, poses, target_ts, result);

  printf("Poses:\n");
  printf("------------------------------------------------\n");
  int index = 0;
  for (const auto &tf : poses) {
    printf("index[%d]\ttimestamp[%ld]\n", index, timestamps[index]);
    print_quaternion("rot:", quat_t{tf_rot(tf)});
    print_vector("trans: ", tf_trans(tf));
    printf("\n");
    index++;
  }

  printf("Result:\n");
  printf("------------------------------------------------\n");
  index = 0;
  for (const auto &tf : result) {
    printf("index[%d]\ttimestamp[%ld]\n", index, target_ts[index]);
    print_quaternion("rot:", quat_t{tf_rot(tf)});
    print_vector("trans: ", tf_trans(tf));
    printf("\n");
    index++;
  }

  return 0;
}

int test_intersection() {
  std::vector<int> v1 = {1, 2, 3, 4, 5};
  std::vector<int> v2 = {2, 4, 6};
  std::vector<int> v3 = {1, 4};
  std::list<std::vector<int>> data = {v1, v2, v3};

  auto result = intersection(data);
  MU_CHECK(result.size() == 1);
  MU_CHECK(result.find(4) != result.end());

  return 0;
}

int test_lerp_timestamps() {
  test_data_t td;

  const auto lerp_ts = lerp_timestamps(td.gyro_ts, td.accel_ts);
  MU_CHECK(lerp_ts.size() > td.accel_ts.size());
  MU_CHECK(lerp_ts.front() >= td.accel_ts.front());
  MU_CHECK(lerp_ts.back() <= td.accel_ts.back());

  return 0;
}

int test_lerp_data() {
  test_data_t td;
  const auto lerp_ts = lerp_timestamps(td.accel_ts, td.gyro_ts);

  lerp_data(lerp_ts, td.accel_ts, td.accel_data);
  MU_CHECK(lerp_ts.size() == td.accel_ts.size());
  MU_CHECK(lerp_ts.size() == td.accel_data.size());
  MU_CHECK(lerp_ts.front() == td.accel_ts.front());
  MU_CHECK(lerp_ts.back() == td.accel_ts.back());
  for (size_t i = 0; i < td.accel_ts.size(); i++) {
    MU_CHECK(lerp_ts[i] == td.accel_ts[i]);
  }

  // Save interpolated data
  save_interpolated_gyro_data(td.gyro_ts, td.gyro_data);
  save_interpolated_accel_data(td.accel_ts, td.accel_data);

  // Plot data
  const bool debug = false;
  // const bool debug = true;
  if (debug) {
    OCTAVE_SCRIPT("scripts/measurement/plot_lerp.m "
                  "/tmp/lerp_data-gyro_ts.csv "
                  "/tmp/lerp_data-gyro_data.csv "
                  "/tmp/lerp_data-accel_ts.csv "
                  "/tmp/lerp_data-accel_data.csv " GYRO0_CSV_PATH
                  " " ACCEL0_CSV_PATH);
  }

  return 0;
}

int test_lerp_data2() {
  test_data_t td;

  const auto lerp_ts = lerp_timestamps(td.gyro_ts, td.accel_ts);
  lerp_data(td.accel_ts, td.accel_data, td.gyro_ts, td.gyro_data);

  MU_CHECK(td.gyro_ts.front() == lerp_ts.front());
  MU_CHECK(td.gyro_ts.back() == lerp_ts.back());
  MU_CHECK(td.accel_ts.front() == lerp_ts.front());
  MU_CHECK(td.accel_ts.back() == lerp_ts.back());
  MU_CHECK(td.gyro_ts.front() == td.accel_ts.front());
  MU_CHECK(td.gyro_ts.back() == td.accel_ts.back());
  MU_CHECK(td.gyro_ts.size() == td.accel_ts.size());

  // Save interpolated data
  save_interpolated_gyro_data(td.gyro_ts, td.gyro_data);
  save_interpolated_accel_data(td.accel_ts, td.accel_data);

  // Plot data
  const bool debug = false;
  // const bool debug = true;
  if (debug) {
    OCTAVE_SCRIPT("scripts/measurement/plot_lerp.m "
                  "/tmp/lerp_data-gyro_ts.csv "
                  "/tmp/lerp_data-gyro_data.csv "
                  "/tmp/lerp_data-accel_ts.csv "
                  "/tmp/lerp_data-accel_data.csv " GYRO0_CSV_PATH
                  " " ACCEL0_CSV_PATH);
  }

  return 0;
}

int test_lerp_data3() {
  // Test data
  std::deque<std::string> buf_seq;
  std::deque<timestamp_t> buf_ts;
  std::deque<vec3_t> buf_data;
  test_data_t td;
  td.flatten(buf_seq, buf_ts, buf_data);

  // Lerp data
  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro_data;
  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel_data;

  timestamp_t t0 = 0;
  vec3_t d0;
  timestamp_t t1 = 0;
  vec3_t d1;
  bool t0_set = false;

  std::deque<timestamp_t> lerp_ts;
  std::deque<vec3_t> lerp_data;

  while (buf_ts.size()) {
    // Timestamp
    const timestamp_t ts = buf_ts.front();
    buf_ts.pop_front();

    // Sequence
    const std::string seq = buf_seq.front();
    buf_seq.pop_front();

    // Data
    const vec3_t data = buf_data.front();
    buf_data.pop_front();

    if (t0_set == false && seq == "A") {
      t0 = ts;
      d0 = data;
      t0_set = true;

    } else if (t0_set && seq == "A") {
      // std::cout << std::endl;
      t1 = ts;
      d1 = data;

      while (lerp_ts.size()) {
        const timestamp_t lts = lerp_ts.front();
        const vec3_t ldata = lerp_data.front();
        const real_t dt = static_cast<real_t>(t1 - t0) * 1e-9;
        const real_t alpha = static_cast<real_t>(lts - t0) * 1e-9 / dt;

        accel_ts.push_back(lts);
        accel_data.push_back(lerp(d0, d1, alpha));

        gyro_ts.push_back(lts);
        gyro_data.push_back(ldata);

        lerp_ts.pop_front();
        lerp_data.pop_front();
      }

      t0 = t1;
      d0 = d1;

    } else if (t0_set && ts >= t0 && seq == "G") {
      lerp_ts.push_back(ts);
      lerp_data.push_back(data);
    }
  }

  std::cout << "gyro_ts size: " << gyro_ts.size() << std::endl;
  std::cout << "accel_ts size: " << accel_ts.size() << std::endl;
  save_interpolated_gyro_data(gyro_ts, gyro_data);
  save_interpolated_accel_data(accel_ts, accel_data);

  // Plot data
  const bool debug = false;
  // const bool debug = true;
  if (debug) {
    OCTAVE_SCRIPT("scripts/measurement/plot_lerp.m "
                  "/tmp/lerp_data-gyro_ts.csv "
                  "/tmp/lerp_data-gyro_data.csv "
                  "/tmp/lerp_data-accel_ts.csv "
                  "/tmp/lerp_data-accel_data.csv " GYRO0_CSV_PATH
                  " " ACCEL0_CSV_PATH);
  }
  return 0;
}

/******************************************************************************
 * SPLINE
 *****************************************************************************/

void generate_trajectory(timestamps_t &timestamps,
                         vec3s_t &positions,
                         quats_t &orientations) {
  timestamp_t ts_k = 0;
  const timestamp_t ts_end = 5.0 * 1e9;
  const real_t f = 100.0;
  const timestamp_t dt = (1 / f) * 1e9;

  while (ts_k <= ts_end) {
    // Time
    timestamps.push_back(ts_k);

    // Position
    const real_t ts_s_k = ts2sec(ts_k);
    positions.emplace_back(sin(2 * M_PI * 2 * ts_s_k) + 1.0,
                           sin(2 * M_PI * 2 * ts_s_k) + 2.0,
                           0.0);

    // Orientation
    const vec3_t rpy(sin(2 * M_PI * 2 * ts_s_k),
                     sin(2 * M_PI * 2 * ts_s_k + M_PI / 4),
                     sin(2 * M_PI * 2 * ts_s_k + M_PI / 2));
    orientations.emplace_back(euler321(rpy));

    // Update
    ts_k += dt;
  }
}

int test_ctraj() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  const auto ts_start = timestamps.front();
  const auto ts_end = timestamps.back();
  ctraj_t ctraj(ts_start, ts_end, positions, orientations);
  save_data("/tmp/positions.csv", timestamps, positions);
  save_data("/tmp/orientations.csv", timestamps, orientations);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_data.m "
                  "/tmp/pos_data.csv "
                  "/tmp/att_data.csv ");
  }

  return 0;
}

int test_ctraj_sandbox() {
  const size_t nb_knots = 5;
  row_vector_t knots{nb_knots};
  knots[0] = 0.0;
  knots[1] = 0.5;
  knots[2] = 1.0;
  knots[3] = 1.5;
  knots[4] = 2.0;

  row_vector_t pos{nb_knots};
  pos[0] = 0.0;
  pos[1] = 0.2;
  pos[2] = 0.4;
  pos[3] = 0.6;
  pos[4] = 0.8;

  row_vector_t derivs{2};
  derivs << 0.0, 0.0;

  row_vector_t deriv_indices{2};
  deriv_indices << 0, pos.size() - 1;

  // clang-format off
  auto spline = Eigen::SplineFitting<Spline1D>::InterpolateWithDerivatives(pos, derivs, deriv_indices, 3, knots);
  // auto spline = Eigen::SplineFitting<Spline1D>::InterpolateWithDerivatives(pos, derivs, deriv_indices, 3);
  // auto spline = Eigen::SplineFitting<Spline1D>::Interpolate(pos, 3, knots);
  // clang-format on

  // for (int i = 0; i < nb_knots; i++) {
  //   std::cout << spline(knots(i)) << std::endl;
  // }
  std::cout << spline(0.0) << std::endl;
  std::cout << spline(0.5) << std::endl;
  std::cout << spline(1.0) << std::endl;

  return 0;
}

int test_ctraj_get_pose() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps.front(), timestamps.back(), positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);
  save_data("/tmp/att_data.csv", timestamps, orientations);

  {
    timestamps_t t;
    vec3s_t r;
    quats_t q;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = timestamps.back();
    const real_t f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);

      const auto T_WS = ctraj_get_pose(ctraj, ts_k);
      r.push_back(tf_trans(T_WS));
      q.push_back(tf_quat(T_WS));

      ts_k += dt;
    }

    save_data("/tmp/pos_interp.csv", t, r);
    save_data("/tmp/att_interp.csv", t, q);
  }

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_pose.m "
                  "/tmp/pos_data.csv "
                  "/tmp/att_data.csv "
                  "/tmp/pos_interp.csv "
                  "/tmp/att_interp.csv ");
  }

  return 0;
}

int test_ctraj_get_velocity() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps.front(), timestamps.back(), positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);

  {
    timestamps_t t;
    vec3s_t v;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = timestamps.back();
    const real_t f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);
      v.push_back(ctraj_get_velocity(ctraj, ts_k));
      ts_k += dt;
    }

    save_data("/tmp/vel_interp.csv", t, v);
  }

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_velocity.m "
                  "/tmp/pos.csv "
                  "/tmp/vel_interp.csv ");
  }

  return 0;
}

int test_ctraj_get_acceleration() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps.front(), timestamps.back(), positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);
  save_data("/tmp/att_data.csv", timestamps, orientations);

  {
    timestamps_t t;
    vec3s_t a;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = timestamps.back();
    const real_t f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);
      a.push_back(ctraj_get_acceleration(ctraj, ts_k));
      ts_k += dt;
    }

    save_data("/tmp/acc_interp.csv", t, a);
  }

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_acceleration.m "
                  "/tmp/pos_data.csv "
                  "/tmp/acc_interp.csv ");
  }

  return 0;
}

int test_ctraj_get_angular_velocity() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps.front(), timestamps.back(), positions, orientations);
  save_data("/tmp/att_data.csv", timestamps, orientations);

  // Setup
  timestamps_t t_hist;
  quats_t q_hist;
  quats_t q_prop_hist;
  vec3s_t w_hist;

  timestamp_t ts_k = 0;
  const timestamp_t ts_end = timestamps.back();
  const real_t f = 1000.0;
  const timestamp_t dt = (1 / f) * 1e9;

  // Initialize first attitude
  auto T_WB = ctraj_get_pose(ctraj, 0.0);
  mat3_t C_WB = tf_rot(T_WB);

  // Interpolate pose, angular velocity
  while (ts_k <= ts_end) {
    t_hist.push_back(ts_k);

    // Attitude at time k
    T_WB = ctraj_get_pose(ctraj, ts_k);
    const auto q_WB_k = tf_quat(T_WB);
    q_hist.push_back(q_WB_k);

    // Angular velocity at time k
    const auto w_WB_k = ctraj_get_angular_velocity(ctraj, ts_k);
    w_hist.push_back(w_WB_k);

    // Propagate angular velocity to obtain attitude at time k
    const mat3_t C_BW = tf_rot(T_WB).inverse();
    C_WB = C_WB * lie::Exp(C_BW * w_WB_k * ts2sec(dt));
    q_prop_hist.emplace_back(quat_t{C_WB});

    ts_k += dt;
  }
  save_data("/tmp/att_interp.csv", t_hist, q_hist);
  save_data("/tmp/att_prop.csv", t_hist, q_prop_hist);
  save_data("/tmp/avel_interp.csv", t_hist, w_hist);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_angular_velocity.m "
                  "/tmp/att_data.csv "
                  "/tmp/att_interp.csv "
                  "/tmp/att_prop.csv "
                  "/tmp/avel_interp.csv ");
  }

  return 0;
}

int test_sim_imu_measurement() {
  // Setup imu sim
  sim_imu_t imu;
  imu.rate = 400;
  imu.tau_a = 3600;
  imu.tau_g = 3600;
  imu.sigma_g_c = 0.00275;
  imu.sigma_a_c = 0.0250;
  imu.sigma_gw_c = 1.65e-05;
  imu.sigma_aw_c = 0.000441;
  imu.g = 9.81007;

  // Generate trajectory
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;
  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps.front(), timestamps.back(), positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);
  save_data("/tmp/att_data.csv", timestamps, orientations);

  // Simulate IMU measurements
  std::default_random_engine rndeng;
  timestamps_t imu_ts;
  vec3s_t imu_accel;
  vec3s_t imu_gyro;
  vec3s_t pos_prop;
  vec3s_t vel_prop;
  quats_t att_prop;

  timestamp_t ts_k = 0;
  const timestamp_t ts_end = timestamps.back();
  const timestamp_t dt = (1 / imu.rate) * 1e9;

  // -- Initialize position, velocity and attidue
  auto T_WS = ctraj_get_pose(ctraj, 0.0);
  vec3_t r_WS = tf_trans(T_WS);
  mat3_t C_WS = tf_rot(T_WS);
  vec3_t v_WS = ctraj_get_velocity(ctraj, 0.0);

  // -- Simulate imu measurements
  while (ts_k <= ts_end) {
    const auto T_WS_W = ctraj_get_pose(ctraj, ts_k);
    const auto w_WS_W = ctraj_get_angular_velocity(ctraj, ts_k);
    const auto a_WS_W = ctraj_get_acceleration(ctraj, ts_k);
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

    // Propagate simulated IMU measurements
    const real_t dt_s = ts2sec(dt);
    const real_t dt_s_sq = dt_s * dt_s;
    const vec3_t g{0.0, 0.0, -imu.g};
    // -- Position at time k
    const vec3_t b_a = ones(3, 1) * imu.b_a;
    const vec3_t n_a = ones(3, 1) * imu.sigma_a_c;
    r_WS += v_WS * dt_s;
    r_WS += 0.5 * g * dt_s_sq;
    r_WS += 0.5 * C_WS * (a_WS_S - b_a - n_a) * dt_s_sq;
    // -- velocity at time k
    v_WS += C_WS * (a_WS_S - b_a - n_a) * dt_s + g * dt_s;
    // -- Attitude at time k
    const vec3_t b_g = ones(3, 1) * imu.b_g;
    const vec3_t n_g = ones(3, 1) * imu.sigma_g_c;
    C_WS = C_WS * lie::Exp((w_WS_S - b_g - n_g) * ts2sec(dt));

    // Reocord IMU measurments
    pos_prop.push_back(r_WS);
    vel_prop.push_back(v_WS);
    att_prop.emplace_back(quat_t{C_WS});
    imu_ts.push_back(ts_k);
    imu_accel.push_back(a_WS_S);
    imu_gyro.push_back(w_WS_S);

    ts_k += dt;
  }
  save_data("/tmp/att_prop.csv", imu_ts, att_prop);
  save_data("/tmp/pos_prop.csv", imu_ts, pos_prop);
  save_data("/tmp/imu_accel.csv", imu_ts, imu_accel);
  save_data("/tmp/imu_gyro.csv", imu_ts, imu_gyro);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_imu_measurements.m "
                  "/tmp/pos_data.csv "
                  "/tmp/pos_prop.csv "
                  "/tmp/att_data.csv "
                  "/tmp/att_prop.csv "
                  "/tmp/imu_accel.csv "
                  "/tmp/imu_gyro.csv ");
  }

  return 0;
}

// /*****************************************************************************
//  * CONTROL
//  *****************************************************************************/
//
// int test_pid_construct() {
//   pid_t p;
//
//   MU_CHECK_FLOAT(0.0, p.error_prev);
//   MU_CHECK_FLOAT(0.0, p.error_sum);
//
//   MU_CHECK_FLOAT(0.0, p.error_p);
//   MU_CHECK_FLOAT(0.0, p.error_i);
//   MU_CHECK_FLOAT(0.0, p.error_d);
//
//   MU_CHECK_FLOAT(0.0, p.k_p);
//   MU_CHECK_FLOAT(0.0, p.k_i);
//   MU_CHECK_FLOAT(0.0, p.k_d);
//
//   return 0;
// }
//
// int test_pid_setup() {
//   pid_t p(1.0, 2.0, 3.0);
//
//   MU_CHECK_FLOAT(0.0, p.error_prev);
//   MU_CHECK_FLOAT(0.0, p.error_sum);
//
//   MU_CHECK_FLOAT(0.0, p.error_p);
//   MU_CHECK_FLOAT(0.0, p.error_i);
//   MU_CHECK_FLOAT(0.0, p.error_d);
//
//   MU_CHECK_FLOAT(1.0, p.k_p);
//   MU_CHECK_FLOAT(2.0, p.k_i);
//   MU_CHECK_FLOAT(3.0, p.k_d);
//
//   return 0;
// }
//
// int test_pid_update() {
//   pid_t p(1.0, 2.0, 3.0);
//
//   // test and assert
//   real_t output = pid_update(p, 10.0, 0.0, 0.1);
//   std::cout << p << std::endl;
//   std::cout << output << std::endl;
//
//   // MU_CHECK_FLOAT(1.0, p.error_sum);
//   // MU_CHECK_FLOAT(10.0, p.error_p);
//   // MU_CHECK_FLOAT(2.0, p.error_i);
//   // MU_CHECK_FLOAT(300.0, p.error_d);
//   // MU_CHECK_FLOAT(10.0, p.error_prev);
//   // MU_CHECK_FLOAT(111.0, output);
//
//   return 0;
// }
//
// int test_pid_reset() {
//   pid_t p;
//
//   p.error_prev = 0.1;
//   p.error_sum = 0.2;
//
//   p.error_p = 0.3;
//   p.error_i = 0.4;
//   p.error_d = 0.5;
//
//   pid_reset(p);
//
//   MU_CHECK_FLOAT(0.0, p.error_prev);
//   MU_CHECK_FLOAT(0.0, p.error_sum);
//
//   MU_CHECK_FLOAT(0.3, p.error_p);
//   MU_CHECK_FLOAT(0.4, p.error_i);
//   MU_CHECK_FLOAT(0.5, p.error_d);
//
//   return 0;
// }
//
// int test_carrot_ctrl_constructor() {
//   carrot_ctrl_t cc;
//
//   MU_CHECK(cc.wp_start.isApprox(vec3_t::Zero()));
//   MU_CHECK(cc.wp_end.isApprox(vec3_t::Zero()));
//   MU_CHECK(cc.wp_index == 0);
//   MU_CHECK_FLOAT(0.0, cc.look_ahead_dist);
//
//   return 0;
// }
//
// int test_carrot_ctrl_configure() {
//   carrot_ctrl_t cc;
//
//   vec3s_t waypoints;
//   waypoints.emplace_back(0.0, 0.0, 0.0);
//   waypoints.emplace_back(1.0, 1.0, 0.0);
//   waypoints.emplace_back(2.0, 2.0, 0.0);
//   waypoints.emplace_back(3.0, 3.0, 0.0);
//   carrot_ctrl_configure(cc, waypoints, 0.1);
//
//   MU_CHECK(cc.wp_start.isApprox(vec3_t::Zero()));
//   MU_CHECK(cc.wp_end.isApprox(vec3_t{1.0, 1.0, 0.0}));
//   MU_CHECK(cc.wp_index == 1);
//   MU_CHECK_FLOAT(0.1, cc.look_ahead_dist);
//
//   return 0;
// }
//
// int test_carrot_ctrl_closest_point() {
//   carrot_ctrl_t cc;
//
//   vec3s_t wps;
//   wps.emplace_back(0.0, 0.0, 0.0);
//   wps.emplace_back(1.0, 1.0, 0.0);
//   wps.emplace_back(2.0, 2.0, 0.0);
//   wps.emplace_back(3.0, 3.0, 0.0);
//   carrot_ctrl_configure(cc, wps, 0.1);
//
//   MU_CHECK(cc.wp_start.isApprox(vec3_t::Zero()));
//   MU_CHECK(cc.wp_end.isApprox(vec3_t{1.0, 1.0, 0.0}));
//   MU_CHECK(cc.wp_index == 1);
//   MU_CHECK_FLOAT(0.1, cc.look_ahead_dist);
//
//   // Test before waypoint start
//   vec3_t pos0{-1.0, -1.0, 0.0};
//   vec3_t res0;
//   int s0 = carrot_ctrl_closest_point(cc, pos0, res0);
//   MU_CHECK(res0.isApprox(vec3_t{-1.0, -1.0, 0.0}));
//   MU_CHECK(s0 == -1);
//
//   // Test between waypoint start and end
//   vec3_t pos1{0.5, 0.5, 0.0};
//   vec3_t res1;
//   int s1 = carrot_ctrl_closest_point(cc, pos1, res1);
//   MU_CHECK(res1.isApprox(vec3_t{0.5, 0.5, 0.0}));
//   MU_CHECK(s1 == 0);
//
//   // Test after waypoint end
//   vec3_t pos2{1.5, 1.5, 0.0};
//   vec3_t res2;
//   int s2 = carrot_ctrl_closest_point(cc, pos2, res2);
//   MU_CHECK(res2.isApprox(vec3_t{1.5, 1.5, 0.0}));
//   MU_CHECK(s2 == 1);
//
//   return 0;
// }
//
// int test_carrot_ctrl_carrot_point() {
//   carrot_ctrl_t cc;
//
//   vec3s_t wps;
//   wps.emplace_back(0.0, 0.0, 0.0);
//   wps.emplace_back(1.0, 0.0, 0.0);
//   wps.emplace_back(2.0, 0.0, 0.0);
//   wps.emplace_back(3.0, 0.0, 0.0);
//   carrot_ctrl_configure(cc, wps, 0.1);
//
//   MU_CHECK(cc.wp_start.isApprox(vec3_t::Zero()));
//   MU_CHECK(cc.wp_end.isApprox(vec3_t{1.0, 0.0, 0.0}));
//   MU_CHECK(cc.wp_index == 1);
//   MU_CHECK_FLOAT(0.1, cc.look_ahead_dist);
//
//   // Test before waypoint start
//   vec3_t pos0{-1.0, 0.0, 0.0};
//   vec3_t res0;
//   int s0 = carrot_ctrl_carrot_point(cc, pos0, res0);
//   MU_CHECK(res0.isApprox(vec3_t{0.0, 0.0, 0.0}));
//   MU_CHECK(s0 == -1);
//
//   // Test between waypoint start and end
//   vec3_t pos1{0.5, 0.0, 0.0};
//   vec3_t res1;
//   int s1 = carrot_ctrl_carrot_point(cc, pos1, res1);
//   MU_CHECK(res1.isApprox(vec3_t{0.6, 0.0, 0.0}));
//   MU_CHECK(s1 == 0);
//
//   // Test after waypoint end
//   vec3_t pos2{1.5, 0.0, 0.0};
//   vec3_t res2;
//   int s2 = carrot_ctrl_carrot_point(cc, pos2, res2);
//   MU_CHECK(res2.isApprox(vec3_t{1.0, 0.0, 0.0}));
//   MU_CHECK(s2 == 1);
//
//   return 0;
// }
//
// int test_carrot_ctrl_update() {
//   carrot_ctrl_t cc;
//
//   vec3s_t wps;
//   wps.emplace_back(0.0, 0.0, 0.0);
//   wps.emplace_back(1.0, 0.0, 0.0);
//   wps.emplace_back(2.0, 0.0, 0.0);
//   wps.emplace_back(3.0, 0.0, 0.0);
//   carrot_ctrl_configure(cc, wps, 0.1);
//
//   MU_CHECK(cc.wp_start.isApprox(vec3_t::Zero()));
//   MU_CHECK(cc.wp_end.isApprox(vec3_t{1.0, 0.0, 0.0}));
//   MU_CHECK(cc.wp_index == 1);
//   MU_CHECK_FLOAT(0.1, cc.look_ahead_dist);
//
//   // Test before waypoint start
//   vec3_t pos0{-1.0, 0.0, 0.0};
//   vec3_t res0;
//   int s0 = carrot_ctrl_update(cc, pos0, res0);
//   MU_CHECK(res0.isApprox(vec3_t{0.0, 0.0, 0.0}));
//   MU_CHECK(s0 == 0);
//
//   // Test between waypoint start and end
//   vec3_t pos1{0.5, 0.0, 0.0};
//   vec3_t res1;
//   int s1 = carrot_ctrl_update(cc, pos1, res1);
//   MU_CHECK(res1.isApprox(vec3_t{0.6, 0.0, 0.0}));
//   MU_CHECK(s1 == 0);
//
//   // Test after waypoint end
//   vec3_t pos2{1.5, 0.0, 0.0};
//   vec3_t res2;
//   int s2 = carrot_ctrl_update(cc, pos2, res2);
//   MU_CHECK(res2.isApprox(vec3_t{1.0, 0.0, 0.0}));
//   MU_CHECK(s2 == 0);
//   MU_CHECK(cc.wp_index == 2);
//   MU_CHECK(cc.wp_start.isApprox(vec3_t{1.0, 0.0, 0.0}));
//   MU_CHECK(cc.wp_end.isApprox(vec3_t{2.0, 0.0, 0.0}));
//
//   return 0;
// }

/*****************************************************************************
 * Measurements
 ****************************************************************************/

int test_imu_meas() {
  imu_meas_t imu_meas;

  MU_CHECK(imu_meas.ts == 0);
  MU_CHECK(equals(imu_meas.accel, zeros(3, 1)));
  MU_CHECK(equals(imu_meas.gyro, zeros(3, 1)));

  return 0;
}

int test_imu_data() {
  imu_data_t imu_data;

  MU_CHECK(imu_data.timestamps.size() == 0);
  MU_CHECK(imu_data.accel.size() == 0);
  MU_CHECK(imu_data.gyro.size() == 0);

  return 0;
}

int test_imu_data_add() {
  imu_data_t imu_data;

  imu_data.add(0, {1, 2, 3}, {4, 5, 6});
  MU_CHECK(imu_data.timestamps.size() == 1);
  MU_CHECK(imu_data.accel.size() == 1);
  MU_CHECK(imu_data.gyro.size() == 1);

  return 0;
}

int test_imu_data_size() {
  imu_data_t imu_data;

  imu_data.add(0, {1, 2, 3}, {4, 5, 6});
  imu_data.add(0, {1, 2, 3}, {4, 5, 6});
  imu_data.add(0, {1, 2, 3}, {4, 5, 6});
  imu_data.add(0, {1, 2, 3}, {4, 5, 6});
  imu_data.add(0, {1, 2, 3}, {4, 5, 6});
  MU_CHECK(imu_data.size() == 5);

  return 0;
}

int test_imu_data_last_ts() {
  imu_data_t imu_data;

  imu_data.add(0, {1, 2, 3}, {4, 5, 6});
  imu_data.add(1, {1, 2, 3}, {4, 5, 6});
  imu_data.add(2, {1, 2, 3}, {4, 5, 6});
  imu_data.add(3, {1, 2, 3}, {4, 5, 6});
  imu_data.add(4, {1, 2, 3}, {4, 5, 6});
  MU_CHECK(imu_data.last_ts() == 4);

  return 0;
}

int test_imu_data_clear() {
  imu_data_t imu_data;

  imu_data.add(0, {1, 2, 3}, {4, 5, 6});
  imu_data.add(1, {1, 2, 3}, {4, 5, 6});
  imu_data.add(2, {1, 2, 3}, {4, 5, 6});
  imu_data.add(3, {1, 2, 3}, {4, 5, 6});
  imu_data.add(4, {1, 2, 3}, {4, 5, 6});
  imu_data.clear();
  MU_CHECK(imu_data.size() == 0);

  return 0;
}

// /*****************************************************************************
//  * MODEL
//  ****************************************************************************/
//
// int test_two_wheel_constructor() {
//   two_wheel_t model;
//
//   MU_CHECK(model.r_G.isApprox(vec3_t::Zero()));
//   MU_CHECK(model.rpy_G.isApprox(vec3_t::Zero()));
//   MU_CHECK(model.w_B.isApprox(vec3_t::Zero()));
//   MU_CHECK(model.v_B.isApprox(vec3_t::Zero()));
//   MU_CHECK(model.a_B.isApprox(vec3_t::Zero()));
//
//   return 0;
// }
//
// int test_two_wheel_update() {
//   // Setup output file
//   std::ofstream output_file("/tmp/twowheel.dat");
//   if (output_file.good() == false) {
//     LOG_ERROR("Failed to open file for output!");
//     return -1;
//   }
//
//   // Write output file header
//   const std::string header = "t,x,y,z,vx,vy,vz,ax,ay,az,roll,pitch,yaw";
//   output_file << header << std::endl;
//
//   // Setup model
//   real_t t_end = 10.0;
//   const real_t dt = 0.1;
//
//   real_t wz_B = 0.0;
//   const real_t circle_radius = 10.0;
//   const real_t circle_velocity = 1.0;
//   circle_trajectory(circle_radius, circle_velocity, &wz_B, &t_end);
//   two_wheel_t model;
//   model.v_B(0) = circle_velocity;
//   model.w_B(2) = wz_B;
//
//   // Record initial model state
//   output_file << 0.0 << ",";
//   output_file << model.r_G(0) << ",";
//   output_file << model.r_G(1) << ",";
//   output_file << model.r_G(2) << ",";
//   output_file << model.v_G(0) << ",";
//   output_file << model.v_G(1) << ",";
//   output_file << model.v_G(2) << ",";
//   output_file << model.a_G(0) << ",";
//   output_file << model.a_G(1) << ",";
//   output_file << model.a_G(2) << ",";
//   output_file << model.rpy_G(0) << ",";
//   output_file << model.rpy_G(1) << ",";
//   output_file << model.rpy_G(2) << std::endl;
//
//   // Simulate model motion
//   for (real_t t = 0.0; t < t_end; t += dt) {
//     // Update
//     model.update(dt);
//
//     // Record model state
//     output_file << t << ",";
//     output_file << model.r_G(0) << ",";
//     output_file << model.r_G(1) << ",";
//     output_file << model.r_G(2) << ",";
//     output_file << model.v_G(0) << ",";
//     output_file << model.v_G(1) << ",";
//     output_file << model.v_G(2) << ",";
//     output_file << model.a_G(0) << ",";
//     output_file << model.a_G(1) << ",";
//     output_file << model.a_G(2) << ",";
//     output_file << model.rpy_G(0) << ",";
//     output_file << model.rpy_G(1) << ",";
//     output_file << model.rpy_G(2) << std::endl;
//   }
//   // PYTHON_SCRIPT("scripts/plot_twowheel.py /tmp/twowheel.dat")
//
//   return 0;
// }

// int setup_output_files(std::ofstream &gnd_file,
//                        std::ofstream &mea_file,
//                        std::ofstream &est_file) {
//   // Ground truth file
//   const std::string gnd_file_path = "/tmp/mav_gnd.dat";
//   gnd_file.open(gnd_file_path);
//   if (gnd_file.good() == false) {
//     LOG_ERROR("Failed to open ground truth file for recording [%s]",
//               gnd_file_path.c_str());
//     return -1;
//   }
//
//   // Measurement file
//   const std::string mea_file_path = "/tmp/mav_mea.dat";
//   mea_file.open(mea_file_path);
//   if (mea_file.good() == false) {
//     LOG_ERROR("Failed to open measurement file for recording [%s]",
//               mea_file_path.c_str());
//     return -1;
//   }
//
//   // Estimate file
//   const std::string est_file_path = "/tmp/mav_est.dat";
//   est_file.open(est_file_path);
//   if (est_file.good() == false) {
//     LOG_ERROR("Failed to open estimate file for recording [%s]",
//               est_file_path.c_str());
//     return -1;
//   }
//
//   // Write header
//   const std::string gnd_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
//   gnd_file << gnd_header << std::endl;
//   const std::string mea_header = "t,ax_B,ay_B,az_B,wx_B,wy_B,wz_B";
//   mea_file << mea_header << std::endl;
//   const std::string est_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
//   est_file << est_header << std::endl;
//
//   return 0;
// }

// void record_timestep(const real_t t,
//                      const mav_model_t &mav,
//                      std::ofstream &gnd_file,
//                      std::ofstream &mea_file,
//                      std::ofstream &est_file) {
//   // Record mav ground truth
//   // -- Time
//   gnd_file << t << ",";
//   // -- Position
//   gnd_file << mav.position(0) << ",";
//   gnd_file << mav.position(1) << ",";
//   gnd_file << mav.position(2) << ",";
//   // -- Velocity
//   gnd_file << mav.linear_velocity(0) << ",";
//   gnd_file << mav.linear_velocity(1) << ",";
//   gnd_file << mav.linear_velocity(2) << ",";
//   // -- Attitude
//   gnd_file << mav.attitude(0) << ",";
//   gnd_file << mav.attitude(1) << ",";
//   gnd_file << mav.attitude(2) << std::endl;
//
//   // // Record mav body acceleration and angular velocity
//   // // -- Time
//   // mea_file << t << ",";
//   // // -- Body acceleration
//   // mea_file << mav.a_B(0) << ",";
//   // mea_file << mav.a_B(1) << ",";
//   // mea_file << mav.a_B(2) << ",";
//   // // -- Body angular velocity
//   // mea_file << mav.w_B(0) << ",";
//   // mea_file << mav.w_B(1) << ",";
//   // mea_file << mav.w_B(2) << std::endl;
// }
//
// int test_mav_model_constructor() {
//   mav_model_t mav;
//   return 0;
// }
//
// int test_mav_model_update() {
//   // Setup output files
//   std::ofstream gnd_file;
//   std::ofstream mea_file;
//   std::ofstream est_file;
//   int retval = setup_output_files(gnd_file, mea_file, est_file);
//   if (retval != 0) {
//     LOG_ERROR("Failed to setup output files!");
//     return -1;
//   }
//
//   // Setup mav model
//   mav_model_t mav;
//
//   // Record initial mav state
//   record_timestep(0.0, mav, gnd_file, mea_file, est_file);
//
//   // Set mav desired position
//   const vec3_t pos_desired{1.0, 0.0, 5.0};
//   mav.position = pos_desired;
//
//   // Simulate
//   const real_t dt = 0.01;
//   // for (real_t t = 0.0; t <= 30.0; t += dt) {
//   for (real_t t = 0.0; t <= 30.0; t += dt) {
//     // Update mav model
//     const vec4_t motor_inputs = zeros(4, 1);
//     mav_model_update(mav, motor_inputs, dt);
//
//     // Record
//     record_timestep(t, mav, gnd_file, mea_file, est_file);
//   }
//   gnd_file.close();
//   mea_file.close();
//   est_file.close();
//
//   return 0;
// }

// int test_sim_circle_trajectory() {
//   vio_sim_data_t sim_data;
//   sim_circle_trajectory(4.0, sim_data);
//   sim_data.save("/tmp/sim_data");
//
//   // Debug
//   const bool debug = true;
//   // const bool debug = false;
//   if (debug) {
//     OCTAVE_SCRIPT("scripts/core/plot_sim_data.m");
//   }
//
//   return 0;
// }

void test_suite() {
  // Algebra
  MU_ADD_TEST(test_sign);
  MU_ADD_TEST(test_linspace);
  MU_ADD_TEST(test_linspace_timestamps);

  // Linear algebra
  MU_ADD_TEST(test_zeros);
  MU_ADD_TEST(test_I);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_hstack);
  MU_ADD_TEST(test_vstack);
  MU_ADD_TEST(test_dstack);
  MU_ADD_TEST(test_skew);
  MU_ADD_TEST(test_skewsq);
  MU_ADD_TEST(test_enforce_psd);
  MU_ADD_TEST(test_nullspace);
  // MU_ADD_TEST(test_covar_recover);

  // Geometry
  MU_ADD_TEST(test_deg2rad_rad2deg);
  MU_ADD_TEST(test_wrap180);
  MU_ADD_TEST(test_wrap360);
  MU_ADD_TEST(test_cross_track_error);
  MU_ADD_TEST(test_point_left_right);
  MU_ADD_TEST(test_closest_point);
  MU_ADD_TEST(test_fit_circle);
  MU_ADD_TEST(test_latlon_offset);
  MU_ADD_TEST(test_latlon_diff);
  MU_ADD_TEST(test_latlon_dist);

  // Statistics
  MU_ADD_TEST(test_median);
  MU_ADD_TEST(test_mvn);
  MU_ADD_TEST(test_gauss_normal);

  // Transform
  MU_ADD_TEST(test_tf_rot);
  MU_ADD_TEST(test_tf_trans);

  // Time
  MU_ADD_TEST(test_ts2sec);
  MU_ADD_TEST(test_ns2sec);
  MU_ADD_TEST(test_tic_toc);

  // Interpolation
  MU_ADD_TEST(test_lerp);
  MU_ADD_TEST(test_slerp);
  MU_ADD_TEST(test_interp_pose);
  MU_ADD_TEST(test_interp_poses);
  MU_ADD_TEST(test_closest_poses);
  MU_ADD_TEST(test_intersection);
  // MU_ADD_TEST(test_lerp_timestamps);
  // MU_ADD_TEST(test_lerp_data);
  // MU_ADD_TEST(test_lerp_data2);
  // MU_ADD_TEST(test_lerp_data3);
  MU_ADD_TEST(test_ctraj);
  MU_ADD_TEST(test_ctraj_sandbox);
  MU_ADD_TEST(test_ctraj_get_pose);
  MU_ADD_TEST(test_ctraj_get_velocity);
  MU_ADD_TEST(test_ctraj_get_acceleration);
  MU_ADD_TEST(test_ctraj_get_angular_velocity);
  MU_ADD_TEST(test_sim_imu_measurement);

  // Control
  // MU_ADD_TEST(test_pid_construct);
  // MU_ADD_TEST(test_pid_setup);
  // MU_ADD_TEST(test_pid_update);
  // MU_ADD_TEST(test_pid_reset);
  // MU_ADD_TEST(test_carrot_ctrl_constructor);
  // MU_ADD_TEST(test_carrot_ctrl_configure);
  // MU_ADD_TEST(test_carrot_ctrl_closest_point);
  // MU_ADD_TEST(test_carrot_ctrl_carrot_point);
  // MU_ADD_TEST(test_carrot_ctrl_update);

  // Measurements
  MU_ADD_TEST(test_imu_meas);
  MU_ADD_TEST(test_imu_data);
  MU_ADD_TEST(test_imu_data_add);
  MU_ADD_TEST(test_imu_data_size);
  MU_ADD_TEST(test_imu_data_last_ts);
  MU_ADD_TEST(test_imu_data_clear);

  // // Model
  // MU_ADD_TEST(test_two_wheel_constructor);
  // MU_ADD_TEST(test_two_wheel_update);
  // MU_ADD_TEST(test_mav_model_constructor);
  // MU_ADD_TEST(test_mav_model_update);

  // Simulation
  // MU_ADD_TEST(test_sim_circle_trajectory);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
