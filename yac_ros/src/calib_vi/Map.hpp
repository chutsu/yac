#ifndef YAC_MAP_HPP
#define YAC_MAP_HPP

#include <memory>
#include <unordered_map>

#include <ceres/ceres.h>

#include "common/Time.hpp"
#include "common/Parameters.hpp"
#include "common/Measurements.hpp"
#include "error/ErrorInterface.hpp"
#include "param/PoseLocalParameterization.hpp"
// #include "ceres_utils.hpp"

namespace yac {

/**
 * Map keeps track of how parameter blocks are connected to residual blocks.
 *
 * In essence, it encapsulates the ceres::Problem. This way, we can easily
 * manipulate the optimisation problem. You could argue why not use cere's
 * internal mechanisms to do that. We found that our implementation was
 * faster...
 */
class Map {
public:
  // AUTOCAL_DEFINE_EXCEPTION(Exception, std::runtime_error)

  // Estimate outputs
  bool save_estimates_ = true;
  std::string save_dir_ = "/tmp/calib";
  size_t frame_index_ = 0;
  std::map<std::string, FILE *> outfiles_;

  // Parameter blocks
  // std::shared_ptr<ParameterBlock> time_delay_block_;
  // std::shared_ptr<ParameterBlock> fiducial_block_;
  // std::map<int, std::shared_ptr<ParameterBlock>> camera_blocks_;
  // std::map<int, std::shared_ptr<ParameterBlock>> sensor_camera_blocks_;
  // std::map<uint64_t, std::shared_ptr<ParameterBlock>> sensor_pose_blocks_;
  // std::map<uint64_t, std::shared_ptr<ParameterBlock>> speed_bias_blocks_;

  Map();
  virtual ~Map();

  // definitions
  /// Struct to store some infos about a residual.
  struct ResidualBlockSpec {
    ResidualBlockSpec()
        : residualBlockId(0), lossFunctionPtr(0),
          errorInterfacePtr(std::shared_ptr<ErrorInterface>()) {}

    /// Constructor
    /// @param[in] residualBlockId ID of residual block.
    /// @param[in] lossFunctionPtr The m-estimator.
    /// @param[in] errorInterfacePtr The pointer to the error interface of the
    /// respective residual block.
    ResidualBlockSpec(::ceres::ResidualBlockId residualBlockId,
                      ::ceres::LossFunction *lossFunctionPtr,
                      std::shared_ptr<ErrorInterface> errorInterfacePtr)
        : residualBlockId(residualBlockId), lossFunctionPtr(lossFunctionPtr),
          errorInterfacePtr(errorInterfacePtr) {}

    ::ceres::ResidualBlockId residualBlockId; ///< ID of residual block.
    ::ceres::LossFunction *lossFunctionPtr;   ///< The m-estimator.
    std::shared_ptr<ErrorInterface> errorInterfacePtr; ///< The pointer to the
    /// error interface of the
    /// respective residual
    /// block.
  };

  // clang-format off
  typedef std::pair<uint64_t, std::shared_ptr<ParameterBlock>> ParameterBlockSpec;
  typedef std::vector<ResidualBlockSpec> ResidualBlockCollection;
  typedef std::vector<ParameterBlockSpec> ParameterBlockCollection;
  // clang-format on

  /// Map parameterization
  // clang-format off
  enum Parameterization {
    HomogeneousPoint, ///< Use HomogeneousPointLocalParameterization.
    Pose6d, ///< Use PoseLocalParameterization.
    Pose3d, ///< Use PoseLocalParameterization3d (orientation / varying).
    Pose4d, ///< Use PoseLocalParameterization4d (position and yaw / varying).
    Pose2d, ///< Use PoseLocalParameterization2d (roll / pitch / varying).
    Trivial ///< No local parameterisation.
  };
  // clang-format on

  /**
   * Check whether a certain parameter block is part of the map.
   *
   * @param parameterBlockId ID of parameter block to find.
   * @return True if parameter block is part of map.
   */
  bool parameterBlockExists(uint64_t parameterBlockId) const;

  /// @name Print info
  /// @{

  /**
   * Log information on a parameter block.
   */
  void printParameterBlockInfo(uint64_t parameterBlockId) const;

  /**
   * Log information on a residual block.
   */
  void printResidualBlockInfo(::ceres::ResidualBlockId residualBlockId) const;

  /// @}

  // For quality assessment
  /**
   * Obtain the Hessian block for a specific parameter block.
   *
   * @param[in] parameterBlockId Parameter block ID of interest.
   * @param[out] H the output Hessian block.
   */
  void getLhs(uint64_t parameterBlockId, Eigen::MatrixXd &H);

  /**
   * Add a parameter block to the map.
   *
   * @param parameterBlock Parameter block to insert.
   * @param parameterization Parameterization to tell how to do
   * the local parameterisation.
   * @param group Schur elimination group -- currently unused.
   * @return True if successful.
   */
  bool addParameterBlock(
      std::shared_ptr<ParameterBlock> parameterBlock,
      int parameterization = Parameterization::Trivial,
      const int group = -1);

  /**
   * Remove a parameter block from the map.
   *
   * @param parameterBlockId ID of block to remove.
   * @return True if successful.
   */
  bool removeParameterBlock(uint64_t parameterBlockId);

  /**
   * Remove a parameter block from the map.
   *
   * @param parameterBlock Pointer to the block to remove.
   * @return True if successful.
   */
  bool removeParameterBlock(
      std::shared_ptr<ParameterBlock> parameterBlock);

  /**
   * Adds a residual block.
   *
   * @param cost_function The error term to be used.
   * @param loss_function Use an m-estimator? NULL, if not needed.
   * @param parameterBlockPtrs A vector that contains all the parameter blocks
   * the error term relates to.
   * @return Residual block id
   */
  ::ceres::ResidualBlockId
  addResidualBlock(std::shared_ptr<::ceres::CostFunction> cost_function,
                   ::ceres::LossFunction *loss_function,
                   std::vector<std::shared_ptr<ParameterBlock>>
                       &parameterBlockPtrs);

  /**
   * Replace the parameters connected to a residual block ID.
   *
   * @param residualBlockId The ID of the residual block the parameter
   * blocks of which are to be to be replaced.
   * @param parameterBlockPtrs A vector containing the parameter blocks to
   * be replaced.
   */
  void resetResidualBlock(
      ::ceres::ResidualBlockId residualBlockId,
      std::vector<std::shared_ptr<ParameterBlock>>
          &parameterBlockPtrs);

  /**
   * Add a residual block. See respective ceres docu. If more are needed,
   * see other interface.
   *
   * @param cost_function The error term to be used.
   * @param loss_function Use an m-estimator? NULL, if not needed.
   * @param x0 The first parameter block.
   * @param x1 The second parameter block (if existent).
   * @param x2 The third parameter block (if existent).
   * @param x3 The 4th parameter block (if existent).
   * @param x4 The 5th parameter block (if existent).
   * @param x5 The 6th parameter block (if existent).
   * @param x6 The 7th parameter block (if existent).
   * @param x7 The 8th parameter block (if existent).
   * @param x8 The 9th parameter block (if existent).
   * @param x9 The 10th parameter block (if existent).
   *
   * @return The residual block ID, i.e. what cost_function points to.
   */
  ::ceres::ResidualBlockId
  addResidualBlock(std::shared_ptr<::ceres::CostFunction> cost_function,
                   ::ceres::LossFunction *loss_function,
                   std::shared_ptr<ParameterBlock> x0,
                   std::shared_ptr<ParameterBlock> x1 =
                       std::shared_ptr<ParameterBlock>(),
                   std::shared_ptr<ParameterBlock> x2 =
                       std::shared_ptr<ParameterBlock>(),
                   std::shared_ptr<ParameterBlock> x3 =
                       std::shared_ptr<ParameterBlock>(),
                   std::shared_ptr<ParameterBlock> x4 =
                       std::shared_ptr<ParameterBlock>(),
                   std::shared_ptr<ParameterBlock> x5 =
                       std::shared_ptr<ParameterBlock>(),
                   std::shared_ptr<ParameterBlock> x6 =
                       std::shared_ptr<ParameterBlock>(),
                   std::shared_ptr<ParameterBlock> x7 =
                       std::shared_ptr<ParameterBlock>(),
                   std::shared_ptr<ParameterBlock> x8 =
                       std::shared_ptr<ParameterBlock>(),
                   std::shared_ptr<ParameterBlock> x9 =
                       std::shared_ptr<ParameterBlock>());

  /**
   * Remove a residual block
   *
   * @param id The residual block ID of the residual block to be removed.
   * @return True on success.
   */
  bool removeResidualBlock(::ceres::ResidualBlockId id);

  /// @}

  /// @name Set constant/variable/local parameterization
  /// @{

  /**
   * Do not optimise a certain parameter block.
   *
   * @param parameterBlockId The parameter block ID of the parameter block to
   * set fixed.
   * @return True on success.
   */
  bool setParameterBlockConstant(uint64_t parameterBlockId);

  /**
   * Optimise a certain parameter block (this is the default).
   *
   * @param parameterBlockId The parameter block ID of the parameter block to
   * set fixed.
   * @return True on success.
   */
  bool setParameterBlockVariable(uint64_t parameterBlockId);

  /**
   * Do not optimise a certain parameter block.
   *
   * @param[in] parameterBlock Pointer to the parameter block that should be
   * constant.
   * @return True on success.
   */
  bool setParameterBlockConstant(
      std::shared_ptr<ParameterBlock> parameterBlock) {
    return setParameterBlockConstant(parameterBlock->id());
  }

  /**
   * Optimise a certain parameter block (this is the default).
   *
   * @param[in] parameterBlock Pointer to the parameter block that should be
   * optimised.
   * @return True on success.
   */
  bool setParameterBlockVariable(
      std::shared_ptr<ParameterBlock> parameterBlock) {
    return setParameterBlockVariable(parameterBlock->id());
  }

  /**
   * Reset the (local) parameterisation of a parameter block.
   *
   * @param[in] parameterBlockId The ID of the parameter block in question.
   * @param[in] parameterization Parameterization to tell how to
   * do the local parameterisation.
   * @return True on success.
   */
  bool resetParameterization(uint64_t parameterBlockId, int parameterization);

  /**
   * Set the (local) parameterisation of a parameter block.
   * @param[in] parameterBlockId The ID of the parameter block in question.
   * @param[in] local_parameterization Give it an actual local parameterisation
   * object.
   * @return True on success.
   */
  bool
  setParameterization(uint64_t parameterBlockId,
                      ::ceres::LocalParameterization *local_parameterization);

  /**
   * Set the (local) parameterisation of a parameter block.
   *
   * @param[in] parameterBlock The pointer to the parameter block in question.
   * @param[in] local_parameterization Give it an actual local parameterisation
   * object.
   * @return True on success.
   */
  bool setParameterization(
      std::shared_ptr<ParameterBlock> parameterBlock,
      ::ceres::LocalParameterization *local_parameterization) {
    return setParameterization(parameterBlock->id(), local_parameterization);
  }

  /// @}

  /// @name Getters
  /// @{

  /// Get a shared pointer to a parameter block.
  std::shared_ptr<ParameterBlock>
  parameterBlockPtr(uint64_t parameterBlockId); // get a vertex

  /// Get a shared pointer to a parameter block.
  std::shared_ptr<const ParameterBlock>
  parameterBlockPtr(uint64_t parameterBlockId) const; // get a vertex

  /// Get a shared pointer to an error term.
  std::shared_ptr<ErrorInterface>
  errorInterfacePtr(::ceres::ResidualBlockId residualBlockId); // get a vertex

  /// Get a shared pointer to an error term.
  std::shared_ptr<const ErrorInterface> errorInterfacePtr(
      ::ceres::ResidualBlockId residualBlockId) const; // get a vertex

  /// Get the residual blocks of a parameter block.
  /// @param[in] parameterBlockId The ID of the parameter block in question.
  /// @return Infos about all the residual blocks connected.
  ResidualBlockCollection residuals(uint64_t parameterBlockId) const;

  /// Get the parameters of a residual block.
  /// @param[in] residualBlockId The ID of the residual block in question.
  /// @return Infos about all the parameter blocks connected.
  ParameterBlockCollection parameters(::ceres::ResidualBlockId residualBlockId)
      const; // get the parameter blocks connected

  size_t numberOfParameterBlocks() const;

  size_t numberOfParameterBlocks();

  /// @}

  // Jacobian checker
  /**
   * Check a Jacobian with numeric differences.
   * @warning Checks the minimal version only.
   * @param[in] residualBlockId The ID of the residual block to be checked.
   * @param[in] relTol Relative numeric tolerance.
   * @return True if correct.
   */
  bool isJacobianCorrect(::ceres::ResidualBlockId residualBlockId,
                         double relTol = 1e-6) const;

  // access to the map as such
  /// \brief The actual map from Id to parameter block pointer.
  typedef std::unordered_map<uint64_t,
                             std::shared_ptr<ParameterBlock>>
      Id2ParameterBlock_Map;

  /// \brief The actual map from Id to residual block specs.
  typedef std::unordered_map<::ceres::ResidualBlockId, ResidualBlockSpec>
      ResidualBlockId2ResidualBlockSpec_Map;

  /// Get map connecting parameter block IDs to parameter blocks
  const Id2ParameterBlock_Map &id2parameterBlockMap() const {
    return id2ParameterBlock_Map_;
  }
  /// Get the actual map from Id to residual block specs.
  const ResidualBlockId2ResidualBlockSpec_Map &
  residualBlockId2ResidualBlockSpecMap() const {
    return residualBlockId2ResidualBlockSpec_Map_;
  }

  // these are public for convenient manipulation
  /// \brief Ceres options
  ::ceres::Solver::Options options;

  /// \brief Ceres optimization summary
  ::ceres::Solver::Summary summary;

  /// Solve the optimization problem.
  void solve() { Solve(options, problem_.get(), &summary); }

  /// \brief count the inserted residual blocks.
  uint64_t residualCounter_;

  // member variables related to optimization
  /// \brief The ceres problem
  std::shared_ptr<::ceres::Problem> problem_;

  // the actual maps
  /// \brief Go from Id to residual block pointer.
  typedef std::unordered_multimap<uint64_t, ResidualBlockSpec>
      Id2ResidualBlock_Multimap;

  /// \brief Go from residual block id to its parameter blocks.
  typedef std::unordered_map<::ceres::ResidualBlockId, ParameterBlockCollection>
      ResidualBlockId2ParameterBlockCollection_Map;

  /// \brief The map connecting parameter block ID's and parameter blocks
  Id2ParameterBlock_Map id2ParameterBlock_Map_;

  /// \brief Go from residual ID to specs.
  ResidualBlockId2ResidualBlockSpec_Map residualBlockId2ResidualBlockSpec_Map_;

  /// \brief Go from Id to residual block pointer.
  Id2ResidualBlock_Multimap id2ResidualBlock_Multimap_;

  /// \brief Go from residual block id to its parameter blocks.
  ResidualBlockId2ParameterBlockCollection_Map
      residualBlockId2ParameterBlockCollection_Map_;

  /// \brief Store parameterisation locally.
  // HomogeneousPointLocalParameterization
  //     homogeneousPointLocalParameterization_;

  /// \brief Store parameterisation locally.
  OKVISPoseLocalParameterization poseLocalParameterization_;

  // /// \brief Store parameterisation locally.
  // PoseLocalParameterization2d poseLocalParameterization2d_;
  //
  // /// \brief Store parameterisation locally.
  // PoseLocalParameterization3d poseLocalParameterization3d_;
  //
  // /// \brief Store parameterisation locally.
  // PoseLocalParameterization4d poseLocalParameterization4d_;
};

} // namespace yac
#endif /* YAC_MAP_HPP */
