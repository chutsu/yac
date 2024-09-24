#include <gtest/gtest.h>

#include "InertialError.hpp"
#include "PoseLocalParameterization.hpp"

namespace yac {

// TEST(InertialError, evaluate) {
//   // Setup imu simulation  data
//   ImuSimData sim_data;
//
//   // Setup imu parameters
//   ImuParams imu_params;
//   imu_params.noise_acc = 0.08;   // accelerometer measurement noise stddev.
//   imu_params.noise_gyr = 0.004;  // gyroscope measurement noise stddev.
//   imu_params.noise_ba = 0.00004; // accelerometer bias random work noise
//   stddev. imu_params.noise_bg = 2.0e-6;  // gyroscope bias random work noise
//   stddev.
//
//   // Setup imu buffer
//   const int start_index = 0;
//   const int end_index = 20;
//   const ImuBuffer imu_buffer = sim_data.formImuBuffer(start_index,
//   end_index);
//
//   // Pose i
//   const timestamp_t ts_i = sim_data.timestamps[start_index];
//   const mat4_t T_WS_i = sim_data.poses[ts_i];
//   vecx_t pose_i = tf_vec(T_WS_i);
//
//   // Pose j
//   const timestamp_t ts_j = sim_data.timestamps[end_index - 1];
//   const mat4_t T_WS_j = sim_data.poses[ts_j];
//   vecx_t pose_j = tf_vec(T_WS_j);
//
//   // Speed and bias i
//   const vec3_t vel_i = sim_data.vel[ts_i];
//   const vec3_t ba_i{0.0, 0.0, 0.0};
//   const vec3_t bg_i{0.0, 0.0, 0.0};
//   Eigen::Vector<double, 9> sb_i;
//   sb_i.segment<3>(0) = vel_i;
//   sb_i.segment<3>(3) = ba_i;
//   sb_i.segment<3>(6) = bg_i;
//
//   // Speed and bias j
//   const vec3_t vel_j = sim_data.vel[ts_j];
//   const vec3_t ba_j{0.0, 0.0, 0.0};
//   const vec3_t bg_j{0.0, 0.0, 0.0};
//   Eigen::Vector<double, 9> sb_j;
//   sb_j.segment<3>(0) = vel_j;
//   sb_j.segment<3>(3) = ba_j;
//   sb_j.segment<3>(6) = bg_j;
//
//   // Form Inertial Error
//   auto residual_block = InertialError::create(imu_params,
//                                               imu_buffer,
//                                               pose_i.data(),
//                                               sb_i.data(),
//                                               pose_j.data(),
//                                               sb_j.data());
//
//   // Check residual size and parameter block sizes
//   auto block_sizes = residual_block->parameter_block_sizes();
//   ASSERT_EQ(residual_block->num_residuals(), 15);
//   ASSERT_EQ(block_sizes[0], 7);
//   ASSERT_EQ(block_sizes[1], 9);
//   ASSERT_EQ(block_sizes[2], 7);
//   ASSERT_EQ(block_sizes[3], 9);
//
//   // Check param pointers
//   auto param_ptrs = residual_block->getParamPtrs();
//   ASSERT_EQ(param_ptrs[0], pose_i.data());
//   ASSERT_EQ(param_ptrs[1], sb_i.data());
//   ASSERT_EQ(param_ptrs[2], pose_j.data());
//   ASSERT_EQ(param_ptrs[3], sb_j.data());
//
//   // Check Jacobians
//   ASSERT_TRUE(residual_block->checkJacobian(0, "J_pose_i"));
//   ASSERT_TRUE(residual_block->checkJacobian(1, "J_speed_biases_i"));
//   ASSERT_TRUE(residual_block->checkJacobian(0, "J_pose_j"));
//   ASSERT_TRUE(residual_block->checkJacobian(1, "J_speed_biases_j"));
// }

// class ImuState {
// private:
//   timestamp_t ts_;
//   vecx_t pose_;
//   vecx_t speed_biases_;
//
// public:
//   ImuState(const timestamp_t ts, const vecx_t &pose, const vec3_t &vel)
//       : ts_{ts}, pose_{pose} {
//     speed_biases_.resize(9);
//     speed_biases_.segment<3>(0) = vel;
//     speed_biases_.segment<3>(3) = vec3_t{0.0, 0.0, 0.0};
//     speed_biases_.segment<3>(6) = vec3_t{0.0, 0.0, 0.0};
//   }
//
//   /** Create shared_ptr */
//   static std::shared_ptr<ImuState> create(const timestamp_t ts,
//                                           const vecx_t &pose,
//                                           const vec3_t &vel) {
//     return std::make_shared<ImuState>(ts, pose, vel);
//   }
//
//   /** Return timestamp */
//   timestamp_t getTimestamp() const { return ts_; }
//
//   /** Return pose */
//   vecx_t getPose() const { return pose_; }
//
//   /** Return speed and biases */
//   vecx_t getSpeedBiases() const { return speed_biases_; }
//
//   /** Return pose pointer */
//   double *getPosePtr() { return pose_.data(); }
//
//   /** Return speed biases pointer */
//   double *getSpeedBiasesPtr() { return speed_biases_.data(); }
// };

// TEST(InertialError, propagation) {
//   ImuSimData sim;
//
//   // Setup imu parameters
//   ImuParams imu_params;
//   imu_params.noise_acc = 0.08;   // accelerometer measurement noise stddev.
//   imu_params.noise_gyr = 0.004;  // gyroscope measurement noise stddev.
//   imu_params.noise_ba = 0.00004; // accelerometer bias random work noise stdev.
//   imu_params.noise_bg = 2.0e-6;  // gyroscope bias random work noise stddev.
//                                  //
//   // const int N = sim.getNumMeasurements();
//   const int start_index = 0;
//   const int end_index = 100;
//   const ImuBuffer imu_buffer = sim.formImuBuffer(start_index, end_index);
//
//   const timestamp_t ts_i = sim.timestamps[start_index];
//   const vecx_t pose_i_gnd = tf_vec(sim.poses[ts_i]);
//   const vec3_t vel_i_gnd = sim.vel[ts_i];
//   auto state_i = ImuState::create(ts_i, pose_i_gnd, vel_i_gnd);
//
//   const timestamp_t ts_j = sim.timestamps[start_index];
//   const vecx_t pose_j_gnd = tf_vec(sim.poses[ts_j]);
//   const vec3_t vel_j_gnd = sim.vel[ts_j];
//   auto state_j = ImuState::create(ts_i, pose_i_gnd, vel_j_gnd);
//
//   double *pose_i = state_i->getPosePtr();
//   double *sb_i = state_i->getSpeedBiasesPtr();
//   double *pose_j = state_j->getPosePtr();
//   double *sb_j = state_j->getSpeedBiasesPtr();
//   auto resblock =
//       InertialError::create(imu_params, imu_buffer, pose_i, sb_i, pose_j, sb_j);
//
//
//   const quat_t dq = resblock->getRelativeRotation();
//   const vec3_t dr = resblock->getRelativePosition();
//   const mat4_t dT = tf(dq, dr);
//
//   const mat4_t pose_j_est = sim.poses[ts_i] * dT;
//
//   print_matrix("pose_j_est", pose_j_est);
//   print_matrix("pose_j_gnd", sim.poses[ts_j]);
//
// }

// TEST(InertialError, solve) {
//   // Setup imu simulation  data
//   ImuSimData sim;
//
//   // Setup imu parameters
//   ImuParams imu_params;
//   imu_params.noise_acc = 0.08;   // accelerometer measurement noise stddev.
//   imu_params.noise_gyr = 0.004;  // gyroscope measurement noise stddev.
//   imu_params.noise_ba = 0.00004; // accelerometer bias random work noise
//   stdev. imu_params.noise_bg = 2.0e-6;  // gyroscope bias random work noise
//   stddev.
//
//   // Setup problem
//   std::vector<timestamp_t> state_timestamps;
//   std::map<timestamp_t, std::shared_ptr<ImuState>> states_gnd;
//   std::map<timestamp_t, std::shared_ptr<ImuState>> states_init;
//   std::map<timestamp_t, std::shared_ptr<ImuState>> states_est;
//   std::vector<std::shared_ptr<ResidualBlock>> resblocks;
//
//   ceres::Problem::Options prob_options;
//   prob_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
//   prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
//   prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
//   ceres::Problem problem{prob_options};
//   PoseLocalParameterization pose_plus;
//
//   const int N = sim.getNumMeasurements();
//   int step_size = 20;
//   int start_index = 0;
//   int end_index = step_size;
//   const double dr = 1.0;
//   const double drot = 0.5;
//
//   // Pose and velocity at timestamp i
//   const timestamp_t ts_i = sim.timestamps[start_index];
//   const vecx_t pose_i_gnd = tf_vec(sim.poses[ts_i]);
//   const vecx_t pose_i_est = tf_vec(tf_perturb(sim.poses[ts_i], dr, drot));
//   const vec3_t vel_i = sim.vel[ts_i];
//   if (states_gnd.count(ts_i) == 0) {
//     state_timestamps.push_back(ts_i);
//     states_gnd[ts_i] = ImuState::create(ts_i, pose_i_gnd, vel_i);
//     states_init[ts_i] = ImuState::create(ts_i, pose_i_est, vel_i);
//     states_est[ts_i] = ImuState::create(ts_i, pose_i_est, vel_i);
//   }
//
//   int count = 0;
//   while (end_index < N) {
//     // Form IMU buffer
//     const ImuBuffer imu_buffer = sim.formImuBuffer(start_index, end_index);
//
//     // Pose and velocity at timestamp j
//     const timestamp_t ts_j = sim.timestamps[end_index];
//     const vecx_t pose_j_gnd = tf_vec(sim.poses[ts_j]);
//     const vecx_t pose_j_est = tf_vec(tf_perturb(sim.poses[ts_j], dr, drot));
//     const vec3_t vel_j = sim.vel[ts_j];
//     if (states_gnd.count(ts_j) == 0) {
//       state_timestamps.push_back(ts_j);
//       states_gnd[ts_j] = ImuState::create(ts_j, pose_j_gnd, vel_j);
//       states_init[ts_j] = ImuState::create(ts_j, pose_j_est, vel_j);
//       states_est[ts_j] = ImuState::create(ts_j, pose_j_est, vel_j);
//     }
//
//     // Form Inertial Error
//     double *pose_i = states_est[ts_i]->getPosePtr();
//     double *sb_i = states_est[ts_i]->getSpeedBiasesPtr();
//     double *pose_j = states_est[ts_j]->getPosePtr();
//     double *sb_j = states_est[ts_j]->getSpeedBiasesPtr();
//     auto resblock = InertialError::create(imu_params,
//                                           imu_buffer,
//                                           pose_i,
//                                           sb_i,
//                                           pose_j,
//                                           sb_j);
//     resblocks.push_back(resblock);
//
//     // Add to problem
//     problem.AddParameterBlock(pose_i, 7);
//     problem.AddParameterBlock(pose_j, 7);
//     problem.AddParameterBlock(sb_i, 9);
//     problem.AddParameterBlock(sb_j, 9);
//     problem.SetManifold(pose_i, &pose_plus);
//     problem.SetManifold(pose_j, &pose_plus);
//     problem.AddResidualBlock(resblock.get(), nullptr,
//     resblock->getParamPtrs());
//
//     // Update indicies
//     start_index = end_index;
//     end_index += step_size;
//     count++;
//   }
//
//   // Solver options
//   ceres::Solver::Options options;
//   options.minimizer_progress_to_stdout = true;
//
//   // Solve
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   std::cout << summary.FullReport() << std::endl << std::endl;
//
//   FILE *fp = fopen("/tmp/imu_solve.csv", "w");
//   for (int k = 0; k < states_gnd.size(); k++) {
//     const timestamp_t ts = state_timestamps[k];
//     const vec3_t pos_gnd = states_gnd[ts]->getPose().segment<3>(0);
//     const vec3_t pos_init = states_init[ts]->getPose().segment<3>(0);
//     const vec3_t pos_est = states_est[ts]->getPose().segment<3>(0);
//
//     fprintf(fp, "%ld,", ts);
//     fprintf(fp, "%f,%f,%f,", pos_gnd.x(), pos_gnd.y(), pos_gnd.z());
//     fprintf(fp, "%f,%f,%f,", pos_init.x(), pos_init.y(), pos_init.z());
//     fprintf(fp, "%f,%f,%f\n", pos_est.x(), pos_est.y(), pos_est.z());
//   }
//   fclose(fp);
// }

} // namespace yac
