#include "core.hpp"

namespace yac {


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

bool full_rank(const matx_t &A) {
  if (rank(A) != A.rows() || rank(A) != A.cols()) {
    return false;
  }

  return true;
}

int schurs_complement(matx_t &H, vecx_t &b,
                      const size_t m, const size_t r,
                      const bool precond) {
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
  radius= sqrt((cx * cx) + (cy * cy) - x(2));
}

vec2s_t intersect_circles(const double cx0, const double cy0, const double r0,
                          const double cx1, const double cy1, const double r1) {
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

  // Extract roll, pitch and set yaw to 0
  const quat_t q_WS = quat_t(C_WS);
  const vec3_t rpy = quat2euler(q_WS);
  const real_t roll = rpy(0);
  const real_t pitch = rpy(1);
  const real_t yaw = 0.0;
  C_WS = euler321(vec3_t{roll, pitch, yaw});
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

timestamp_t sec2ts(const real_t sec) { return sec * 1.0e9; }

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
    const vec3_t v = ctraj_get_velocity(ctraj, ctraj.timestamps[i]);

    file << ctraj.timestamps[i] << ",";
    file << ctraj.positions[i](0) << ",";
    file << ctraj.positions[i](1) << ",";
    file << ctraj.positions[i](2) << ",";
    file << ctraj.orientations[i].w() << ",";
    file << ctraj.orientations[i].x() << ",";
    file << ctraj.orientations[i].y() << ",";
    file << ctraj.orientations[i].z() << ",";
    file << v.x() << ",";
    file << v.y() << ",";
    file << v.z() << std::endl;
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

// /*****************************************************************************
//  *                                 CONTROL
//  *****************************************************************************/
//
// pid_t::pid_t() {}
//
// pid_t::pid_t(const real_t k_p_, const real_t k_i_, const real_t k_d_)
//     : k_p{k_p_}, k_i{k_i_}, k_d{k_d_} {}
//
// pid_t::~pid_t() {}
//
// std::ostream &operator<<(std::ostream &os, const pid_t &pid) {
//   os << "error_prev:" << pid.error_prev << std::endl;
//   os << "error_sum:" << pid.error_sum << std::endl;
//   os << "error_p:" << pid.error_p << std::endl;
//   os << "error_i:" << pid.error_i << std::endl;
//   os << "error_d:" << pid.error_d << std::endl;
//   os << "k_p:" << pid.k_p << std::endl;
//   os << "k_i:" << pid.k_i << std::endl;
//   os << "k_d:" << pid.k_d << std::endl;
//   return os;
// }
//
// real_t pid_update(pid_t &p,
//                   const real_t setpoint,
//                   const real_t actual,
//                   const real_t dt) {
//   // Calculate errors
//   const real_t error = setpoint - actual;
//   p.error_sum += error * dt;
//
//   // Calculate output
//   p.error_p = p.k_p * error;
//   p.error_i = p.k_i * p.error_sum;
//   p.error_d = p.k_d * (error - p.error_prev) / dt;
//   const real_t output = p.error_p + p.error_i + p.error_d;
//
//   p.error_prev = error;
//   return output;
// }
//
// real_t pid_update(pid_t &p, const real_t error, const real_t dt) {
//   return pid_update(p, error, 0.0, dt);
// }
//
// void pid_reset(pid_t &p) {
//   p.error_prev = 0.0;
//   p.error_sum = 0.0;
//
//   // p.error_p = 0.0;
//   // p.error_i = 0.0;
//   // p.error_d = 0.0;
// }
//
// carrot_ctrl_t::carrot_ctrl_t() {}
//
// carrot_ctrl_t::~carrot_ctrl_t() {}
//
// int carrot_ctrl_configure(carrot_ctrl_t &cc,
//                           const vec3s_t &waypoints,
//                           const real_t look_ahead_dist) {
//   if (waypoints.size() <= (size_t) 2) {
//     LOG_ERROR("Too few waypoints!");
//     return -1;
//   }
//
//   cc.waypoints = waypoints;
//   cc.wp_start = cc.waypoints[0];
//   cc.wp_end = cc.waypoints[1];
//   cc.wp_index = 1;
//   cc.look_ahead_dist = look_ahead_dist;
//
//   return 0;
// }
//
// int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
//                               const vec3_t &pos,
//                               vec3_t &result) {
//   // Calculate closest point
//   const vec3_t v1 = pos - cc.wp_start;
//   const vec3_t v2 = cc.wp_end - cc.wp_start;
//   const real_t t = v1.dot(v2) / v2.squaredNorm();
//   result = cc.wp_start + t * v2;
//
//   return t;
// }
//
// int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
//                              const vec3_t &pos,
//                              vec3_t &result) {
//   vec3_t closest_pt;
//   int t = carrot_ctrl_closest_point(cc, pos, closest_pt);
//
//   if (t == -1) {
//     // Closest point is before wp_start
//     result = cc.wp_start;
//
//   } else if (t == 0) {
//     // Closest point is between wp_start wp_end
//     const vec3_t u = cc.wp_end - cc.wp_start;
//     const vec3_t v = u / u.norm();
//     result = closest_pt + cc.look_ahead_dist * v;
//
//   } else if (t == 1) {
//     // Closest point is after wp_end
//     result = cc.wp_end;
//   }
//
//   return t;
// }
//
// int carrot_ctrl_update(carrot_ctrl_t &cc,
//                        const vec3_t &pos,
//                        vec3_t &carrot_pt) {
//   // Calculate new carot point
//   int status = carrot_ctrl_carrot_point(cc, pos, carrot_pt);
//
//   // Check if there are more waypoints
//   if ((cc.wp_index + 1) == cc.waypoints.size()) {
//     return 1;
//   }
//
//   // Update waypoints
//   if (status == 1) {
//     cc.wp_index++;
//     cc.wp_start = cc.wp_end;
//     cc.wp_end = cc.waypoints[cc.wp_index];
//   }
//
//   return 0;
// }

// /*****************************************************************************
//  *                                  MODEL
//  ****************************************************************************/
//
// mat4_t dh_transform(const real_t theta,
//                     const real_t d,
//                     const real_t a,
//                     const real_t alpha) {
//   // clang-format off
//   mat4_t T;
//   T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
//        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
//        0.0, sin(alpha), cos(alpha), d,
//        0.0, 0.0, 0.0, 1.0;
//   // clang-format on
//
//   return T;
// }
//
// gimbal_model_t::gimbal_model_t() {}
//
// gimbal_model_t::gimbal_model_t(const vec6_t &tau_s,
//                                const vec6_t &tau_d,
//                                const real_t Lambda1,
//                                const vec3_t w1,
//                                const real_t Lambda2,
//                                const vec3_t w2,
//                                const real_t theta1_offset,
//                                const real_t theta2_offset)
//     : tau_s{tau_s}, tau_d{tau_d}, Lambda1{Lambda1}, w1{w1}, Lambda2{Lambda2},
//       w2{w2}, theta1_offset{theta1_offset}, theta2_offset{theta2_offset} {}
//
// gimbal_model_t::~gimbal_model_t() {}
//
// void gimbal_model_set_attitude(gimbal_model_t &model,
//                                const real_t roll_,
//                                const real_t pitch_) {
//   model.Lambda1 = roll_;
//   model.Lambda2 = pitch_;
// }
//
// vec2_t gimbal_model_get_joint_angles(const gimbal_model_t &model) {
//   return vec2_t{model.Lambda1, model.Lambda2};
// }
//
// mat4_t gimbal_model_T_BS(const gimbal_model_t &model) {
//   mat4_t T_sb = zeros(4, 4);
//   T_sb.block(0, 0, 3, 3) = euler321(model.tau_s.tail(3));
//   T_sb.block(0, 3, 3, 1) = model.tau_s.head(3);
//   T_sb(3, 3) = 1.0;
//
//   return T_sb;
// }
//
// mat4_t gimbal_model_T_EB(const gimbal_model_t &model) {
//   const real_t theta1 = model.Lambda1 + model.theta1_offset;
//   const real_t d1 = model.w1[0];
//   const real_t a1 = model.w1[1];
//   const real_t alpha1 = model.w1[2];
//
//   const real_t theta2 = model.Lambda2 + model.theta2_offset;
//   const real_t d2 = model.w2[0];
//   const real_t a2 = model.w2[1];
//   const real_t alpha2 = model.w2[2];
//
//   const mat4_t T_1b = dh_transform(theta1, d1, a1, alpha1).inverse();
//   const mat4_t T_e1 = dh_transform(theta2, d2, a2, alpha2).inverse();
//   const mat4_t T_EB = T_e1 * T_1b;
//
//   return T_EB;
// }
//
// mat4_t gimbal_model_T_DE(const gimbal_model_t &model) {
//   mat4_t T_DE = zeros(4, 4);
//   T_DE.block(0, 0, 3, 3) = euler321(model.tau_d.tail(3));
//   T_DE.block(0, 3, 3, 1) = model.tau_d.head(3);
//   T_DE(3, 3) = 1.0;
//
//   return T_DE;
// }
//
// mat4_t gimbal_model_T_DS(const gimbal_model_t &model) {
//   const auto T_DE = gimbal_model_T_DE(model);
//   const auto T_EB = gimbal_model_T_EB(model);
//   const auto T_BS = gimbal_model_T_BS(model);
//   return T_DE * T_EB * T_BS;
// }
//
// mat4_t gimbal_model_T_DS(gimbal_model_t &model, const vec2_t &theta) {
//   gimbal_model_set_attitude(model, theta(0), theta(1));
//   const auto T_DE = gimbal_model_T_DE(model);
//   const auto T_EB = gimbal_model_T_EB(model);
//   const auto T_BS = gimbal_model_T_BS(model);
//   return T_DE * T_EB * T_BS;
// }
//
// std::ostream &operator<<(std::ostream &os, const gimbal_model_t &model) {
//   os << "tau_s: " << model.tau_s.transpose() << std::endl;
//   os << "tau_d: " << model.tau_d.transpose() << std::endl;
//   os << "w1: " << model.w1.transpose() << std::endl;
//   os << "w2: " << model.w2.transpose() << std::endl;
//   os << "Lambda1: " << model.Lambda1 << std::endl;
//   os << "Lambda2: " << model.Lambda2 << std::endl;
//   os << "theta1_offset: " << model.theta1_offset << std::endl;
//   os << "theta2_offset: " << model.theta2_offset << std::endl;
//   return os;
// }
//
// void circle_trajectory(const real_t r,
//                        const real_t v,
//                        real_t *w,
//                        real_t *time) {
//   const real_t dist = 2 * M_PI * r;
//   *time = dist / v;
//   *w = (2 * M_PI) / *time;
// }
//
// int mav_model_update(mav_model_t &model,
//                      const vec4_t &motor_inputs,
//                      const real_t dt) {
//   const real_t ph = model.attitude(0);
//   const real_t th = model.attitude(1);
//   const real_t ps = model.attitude(2);
//
//   const real_t p = model.angular_velocity(0);
//   const real_t q = model.angular_velocity(1);
//   const real_t r = model.angular_velocity(2);
//
//   const real_t x = model.position(0);
//   const real_t y = model.position(1);
//   const real_t z = model.position(2);
//
//   const real_t vx = model.linear_velocity(0);
//   const real_t vy = model.linear_velocity(1);
//   const real_t vz = model.linear_velocity(2);
//
//   const real_t Ix = model.Ix;
//   const real_t Iy = model.Iy;
//   const real_t Iz = model.Iz;
//
//   const real_t kr = model.kr;
//   const real_t kt = model.kt;
//
//   const real_t m = model.m;
//   const real_t g = model.g;
//
//   // convert motor inputs to angular p, q, r and total thrust
//   // clang-format off
//   mat4_t A;
//   A << 1.0, 1.0, 1.0, 1.0,
//        0.0, -model.l, 0.0, model.l,
//        -model.l, 0.0, model.l, 0.0,
//        -model.d, model.d, -model.d, model.d;
//   // clang-format on
//   const vec4_t tau = A * motor_inputs;
//   const real_t tauf = tau(0);
//   const real_t taup = tau(1);
//   const real_t tauq = tau(2);
//   const real_t taur = tau(3);
//
//   // update
//   // clang-format off
//   model.attitude(0) = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
//   model.attitude(1) = th + (q * cos(ph) - r * sin(ph)) * dt;
//   model.attitude(2) = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;
//   model.angular_velocity(0) = p + (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
//   model.angular_velocity(1) = q + (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
//   model.angular_velocity(2) = r + (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;
//   model.position(0) = x + vx * dt;
//   model.position(1) = y + vy * dt;
//   model.position(2) = z + vz * dt;
//   model.linear_velocity(0) = vx + ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
//   model.linear_velocity(1) = vy + ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
//   model.linear_velocity(2) = vz + (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;
//   // clang-format on
//
//   // constrain yaw to be [-180, 180]
//   // if (model.attitude(2) > M_PI) {
//   //   model.attitude(2) -= 2 * M_PI;
//   // } else if (model.attitude(2) < -M_PI) {
//   //   model.attitude(2) += 2 * M_PI;
//   // }
//
//   return 0;
// }

} // namespace yac
