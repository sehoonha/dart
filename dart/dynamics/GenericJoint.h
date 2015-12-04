/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_DYNAMICS_GENERICJOINT_H_
#define DART_DYNAMICS_GENERICJOINT_H_

#include <type_traits>

#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

template <class ConfigSpaceType>
class GenericJoint : public Joint
{
public:

  static const int NumDofs = ConfigSpaceType::NumDofs;

  using Point          = typename ConfigSpaceType::Point;
  using EuclideanPoint = typename ConfigSpaceType::EuclideanPoint;
  using Vector         = typename ConfigSpaceType::Vector;
  using JacobianMatrix = typename ConfigSpaceType::JacobianMatrix;
  using Matrix         = typename ConfigSpaceType::Matrix;

  struct UniqueProperties
  {
    /// Lower limit of position
    EuclideanPoint mPositionLowerLimits;

    /// Upper limit of position
    EuclideanPoint mPositionUpperLimits;

    /// Initial positions
    EuclideanPoint mInitialPositions;

    /// Min value allowed.
    Vector mVelocityLowerLimits;

    /// Max value allowed.
    Vector mVelocityUpperLimits;

    /// Initial velocities
    Vector mInitialVelocities;

    /// Min value allowed.
    Vector mAccelerationLowerLimits;

    /// upper limit of generalized acceleration
    Vector mAccelerationUpperLimits;

    /// Min value allowed.
    Vector mForceLowerLimits;

    /// Max value allowed.
    Vector mForceUpperLimits;

    /// Joint spring stiffness
    Vector mSpringStiffnesses;

    /// Rest joint position for joint spring
    EuclideanPoint mRestPositions;

    /// Joint damping coefficient
    Vector mDampingCoefficients;

    /// Joint Coulomb friction
    Vector mFrictions;

    /// True if the name of the corresponding DOF is not allowed to be
    /// overwritten
    std::array<bool, NumDofs> mPreserveDofNames;

    /// The name of the DegreesOfFreedom for this Joint
    std::array<std::string, NumDofs> mDofNames;

    /// Default constructor
    UniqueProperties(
      const EuclideanPoint& positionLowerLimits = -EuclideanPoint::Constant(std::numeric_limits<double>::infinity()),
      const EuclideanPoint& positionUpperLimits = EuclideanPoint::Constant(std::numeric_limits<double>::infinity()),
      const Vector& velocityLowerLimits = -Vector::Constant(std::numeric_limits<double>::infinity()),
      const Vector& velocityUpperLimits = Vector::Constant(std::numeric_limits<double>::infinity()),
      const Vector& accelerationLowerLimits = -Vector::Constant(std::numeric_limits<double>::infinity()),
      const Vector& accelerationUpperLimits = Vector::Constant(std::numeric_limits<double>::infinity()),
      const Vector& forceLowerLimits = -Vector::Constant(std::numeric_limits<double>::infinity()),
      const Vector& forceUpperLimits = Vector::Constant(std::numeric_limits<double>::infinity()),
      const Vector& springStiffness = Vector::Zero(),
      const EuclideanPoint& restPosition = EuclideanPoint::Zero(),
      const Vector& dampingCoefficient = Vector::Zero(),
      const Vector& coulombFrictions = Vector::Zero());
    // TODO(MXG): In version 6.0, we should add mInitialPositions and
    // mInitialVelocities to the constructor arguments. For now we must wait in
    // order to avoid breaking the API.

    /// Copy constructor
    // Note: we only need this because VS2013 lacks full support for std::array
    // Once std::array is properly supported, this should be removed.
    UniqueProperties(const UniqueProperties& other);

    virtual ~UniqueProperties() = default;

  public:
    // To get byte-aligned Eigen vectors
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct Properties : Joint::Properties, UniqueProperties
  {
    Properties(
        const Joint::Properties& jointProperties = Joint::Properties(),
        const UniqueProperties& templatedProperties = UniqueProperties());

    virtual ~Properties();

  public:
    // To get byte-aligned Eigen vectors
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using GenericJointProperties = Properties;

  GenericJoint(const GenericJoint<ConfigSpaceType>&) = delete;

  /// Destructor
  virtual ~GenericJoint();

  /// Set the Properties of this GenericJoint
  void setProperties(const Properties& properties);

  /// Set the Properties of this GenericJoint
  void setProperties(const UniqueProperties& properties);

  /// Get the Properties of this GenericJoint
  Properties getGenericJointProperties() const;

  /// Copy the Properties of another GenericJoint
  void copy(const GenericJoint<ConfigSpaceType>& otherJoint);

  /// Copy the Properties of another GenericJoint
  void copy(const GenericJoint<ConfigSpaceType>* otherJoint);

  /// Same as copy(const GenericJoint&)
  GenericJoint<ConfigSpaceType>& operator=(
      const GenericJoint<ConfigSpaceType>& otherJoint);

  //----------------------------------------------------------------------------
  /// \{ \name Interface for generalized coordinates
  //----------------------------------------------------------------------------

  // Documentation inherited
  size_t getIndexInSkeleton(size_t index) const override;

  // Documentation inherited
  size_t getIndexInTree(size_t index) const override;

  // Documentation inherited
  DegreeOfFreedom* getDof(size_t index) override;

  // Documentation inherited
  const DegreeOfFreedom* getDof(size_t index) const override;

  // Documentation inherited
  const std::string& setDofName(size_t index,
                                const std::string& name,
                                bool preserveName = true) override;

  // Documentation inherited
  void preserveDofName(size_t index, bool preserve) override;

  // Documentation inherited
  bool isDofNamePreserved(size_t index) const override;

  // Documentation inherited
  const std::string& getDofName(size_t index) const override;

  // Documentation inherited
  size_t getNumDofs() const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Fixed-size mutators and accessors
  //----------------------------------------------------------------------------

  // Note: The fixed-size versions of these functions exist to make it easier
  // to comply with the auto-updating design. Use these functions to avoid
  // accessing mPosition directly, that way it is easier to ensure that the
  // auto-updating design assumptions are being satisfied when reviewing the
  // code.

  /// Fixed-size version of setPositions()
  void setPositionsStatic(const Vector& positions);

  /// Fixed-size version of getPositions()
  const Vector& getPositionsStatic() const;

  /// Fixed-size version of setVelocities()
  void setVelocitiesStatic(const Vector& velocities);

  /// Fixed-size version of getVelocities()
  const Vector& getVelocitiesStatic() const;

  /// Fixed-size version of setAccelerations()
  void setAccelerationsStatic(const Vector& accels);

  /// Fixed-size version of getAccelerations()
  const Vector& getAccelerationsStatic() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setCommand(size_t index, double command) override;

  // Documentation inherited
  double getCommand(size_t index) const override;

  // Documentation inherited
  void setCommands(const Eigen::VectorXd& commands) override;

  // Documentation inherited
  Eigen::VectorXd getCommands() const override;

  // Documentation inherited
  void resetCommands() override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Position
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setPosition(size_t index, double position) override;

  // Documentation inherited
  double getPosition(size_t index) const override;

  // Documentation inherited
  void setPositions(const Eigen::VectorXd& positions) override;

  // Documentation inherited
  Eigen::VectorXd getPositions() const override;

  // Documentation inherited
  void setPositionLowerLimit(size_t index, double position) override;

  // Documentation inherited
  double getPositionLowerLimit(size_t index) const override;

  // Documentation inherited
  void setPositionUpperLimit(size_t index, double position) override;

  // Documentation inherited
  double getPositionUpperLimit(size_t index) const override;

  // Documentation inherited
  bool hasPositionLimit(size_t index) const override;

  // Documentation inherited
  void resetPosition(size_t index) override;

  // Documentation inherited
  void resetPositions() override;

  // Documentation inherited
  void setInitialPosition(size_t index, double initial) override;

  // Documentation inherited
  double getInitialPosition(size_t index) const override;

  // Documentation inherited
  void setInitialPositions(const Eigen::VectorXd& initial) override;

  // Documentation inherited
  Eigen::VectorXd getInitialPositions() const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setVelocity(size_t index, double velocity) override;

  // Documentation inherited
  double getVelocity(size_t index) const override;

  // Documentation inherited
  void setVelocities(const Eigen::VectorXd& velocities) override;

  // Documentation inherited
  Eigen::VectorXd getVelocities() const override;

  // Documentation inherited
  void setVelocityLowerLimit(size_t index, double velocity) override;

  // Documentation inherited
  double getVelocityLowerLimit(size_t index) const override;

  // Documentation inherited
  void setVelocityUpperLimit(size_t index, double velocity) override;

  // Documentation inherited
  double getVelocityUpperLimit(size_t index) const override;

  // Documentation inherited
  void resetVelocity(size_t index) override;

  // Documentation inherited
  void resetVelocities() override;

  // Documentation inherited
  void setInitialVelocity(size_t index, double initial) override;

  // Documentation inherited
  double getInitialVelocity(size_t index) const override;

  // Documentation inherited
  void setInitialVelocities(const Eigen::VectorXd& initial) override;

  // Documentation inherited
  Eigen::VectorXd getInitialVelocities() const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Acceleration
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setAcceleration(size_t index, double acceleration) override;

  // Documentation inherited
  double getAcceleration(size_t index) const override;

  // Documentation inherited
  void setAccelerations(const Eigen::VectorXd& accelerations) override;

  // Documentation inherited
  Eigen::VectorXd getAccelerations() const override;

  // Documentation inherited
  void setAccelerationLowerLimit(size_t index, double acceleration) override;

  // Documentation inherited
  double getAccelerationLowerLimit(size_t index) const override;

  // Documentation inherited
  void setAccelerationUpperLimit(size_t index, double acceleration) override;

  // Documentation inherited
  double getAccelerationUpperLimit(size_t index) const override;

  // Documentation inherited
  void resetAccelerations() override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Force
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setForce(size_t index, double force) override;

  // Documentation inherited
  double getForce(size_t index) override;

  // Documentation inherited
  void setForces(const Eigen::VectorXd& forces) override;

  // Documentation inherited
  Eigen::VectorXd getForces() const override;

  // Documentation inherited
  void setForceLowerLimit(size_t index, double force) override;

  // Documentation inherited
  double getForceLowerLimit(size_t index) const override;

  // Documentation inherited
  void setForceUpperLimit(size_t index, double force) override;

  // Documentation inherited
  double getForceUpperLimit(size_t index) const override;

  // Documentation inherited
  void resetForces() override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity change
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setVelocityChange(size_t index, double velocityChange) override;

  // Documentation inherited
  double getVelocityChange(size_t index) const override;

  // Documentation inherited
  void resetVelocityChanges() override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Constraint impulse
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setConstraintImpulse(size_t index, double impulse) override;

  // Documentation inherited
  double getConstraintImpulse(size_t index) const override;

  // Documentation inherited
  void resetConstraintImpulses() override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Integration and finite difference
  //----------------------------------------------------------------------------

  // Documentation inherited
  void integratePositions(double dt) override;

  // Documentation inherited
  void integrateVelocities(double dt) override;

  // Documentation inherited
  Eigen::VectorXd getPositionDifferences(
      const Eigen::VectorXd& q2, const Eigen::VectorXd& q1) const override;

  /// Fixed-size version of getPositionDifferences()
  virtual Vector getPositionDifferencesStatic(
      const Vector& q2, const Vector& q1) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Passive forces - spring, viscous friction, Coulomb friction
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setSpringStiffness(size_t index, double k) override;

  // Documentation inherited
  double getSpringStiffness(size_t index) const override;

  // Documentation inherited
  void setRestPosition(size_t index, double q0) override;

  // Documentation inherited
  double getRestPosition(size_t index) const override;

  // Documentation inherited
  void setDampingCoefficient(size_t index, double coeff) override;

  // Documentation inherited
  double getDampingCoefficient(size_t index) const override;

  // Documentation inherited
  void setCoulombFriction(size_t index, double friction) override;

  // Documentation inherited
  double getCoulombFriction(size_t index) const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Energy
  //----------------------------------------------------------------------------

  // Documentation inherited
  double getPotentialEnergy() const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobians
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian getLocalJacobian() const override;

  /// Fixed-size version of getLocalJacobian()
  const JacobianMatrix& getLocalJacobianStatic() const;

  // Documentation inherited
  const math::Jacobian getLocalJacobian(
      const Eigen::VectorXd& positions) const override;

  /// Fixed-size version of getLocalJacobian(positions)
  virtual const JacobianMatrix getLocalJacobianStatic(
      const Vector& positions) const = 0;

  // Documentation inherited
  const math::Jacobian getLocalJacobianTimeDeriv() const override;

  /// Fixed-size version of getLocalJacobianTimeDeriv()
  const JacobianMatrix& getLocalJacobianTimeDerivStatic() const;

  /// \}

  /// Get whether this joint contains genCoord
  /// \param[in] Generalized coordinate to see
  /// \return True if this joint contains genCoord
//  bool contains(const GenCoord* genCoord) const;

  /// Get local index of the dof at this joint; if the dof is not presented at
  /// this joint, return -1
//  int getGenCoordLocalIndex(int dofSkelIndex) const;

  // Documentation inherited
  Eigen::Vector6d getBodyConstraintWrench() const override;

protected:

  GenericJoint(const Properties& properties);

  // Documentation inherited
  void registerDofs() override;

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  // Documentation inherited
  void updateLocalSpatialVelocity() const override;

  // Documentation inherited
  void updateLocalSpatialAcceleration() const override;

  // Documentation inherited
  void updateLocalPrimaryAcceleration() const override;

  // Documentation inherited
  void addVelocityTo(Eigen::Vector6d& vel) override;

  // Documentation inherited
  void setPartialAccelerationTo(
      Eigen::Vector6d& partialAcceleration,
      const Eigen::Vector6d& childVelocity) override;
  // TODO(JS): Rename with more informative name

  // Documentation inherited
  void addAccelerationTo(Eigen::Vector6d& acc) override;

  // Documentation inherited
  void addVelocityChangeTo(Eigen::Vector6d& velocityChange) override;

  /// Get the inverse of the projected articulated inertia
  const Matrix& getInvProjArtInertia() const;

  /// Get the inverse of projected articulated inertia for implicit joint
  /// damping and spring forces
  const Matrix& getInvProjArtInertiaImplicit() const;

  // Documentation inherited
  void addChildArtInertiaTo(
      Eigen::Matrix6d& parentArtInertia,
      const Eigen::Matrix6d& childArtInertia) override;

  // Documentation inherited
  void addChildArtInertiaImplicitTo(
      Eigen::Matrix6d& parentArtInertiaImplicit,
      const Eigen::Matrix6d& childArtInertiaImplicit) override;

  // Documentation inherited
  void updateInvProjArtInertia(const Eigen::Matrix6d& artInertia) override;

  // Documentation inherited
  void updateInvProjArtInertiaImplicit(
      const Eigen::Matrix6d& artInertia,
      double timeStep) override;

  // Documentation inherited
  void addChildBiasForceTo(
      Eigen::Vector6d& parentBiasForce,
      const Eigen::Matrix6d& childArtInertia,
      const Eigen::Vector6d& childBiasForce,
      const Eigen::Vector6d& childPartialAcc) override;

  // Documentation inherited
  void addChildBiasImpulseTo(
      Eigen::Vector6d& parentBiasImpulse,
      const Eigen::Matrix6d& childArtInertia,
      const Eigen::Vector6d& childBiasImpulse) override;

  // Documentation inherited
  void updateTotalForce(const Eigen::Vector6d& bodyForce,
                        double timeStep) override;

  // Documentation inherited
  void updateTotalImpulse(const Eigen::Vector6d& bodyImpulse) override;

  // Documentation inherited
  void resetTotalImpulses() override;

  // Documentation inherited
  void updateAcceleration(const Eigen::Matrix6d& artInertia,
                                  const Eigen::Vector6d& spatialAcc) override;

  // Documentation inherited
  void updateVelocityChange(
      const Eigen::Matrix6d& artInertia,
      const Eigen::Vector6d& velocityChange) override;

  // Documentation inherited
  void updateForceID(const Eigen::Vector6d& bodyForce,
                             double timeStep,
                             bool withDampingForces,
                             bool withSpringForces) override;

  // Documentation inherited
  void updateForceFD(const Eigen::Vector6d& bodyForce,
                             double timeStep,
                             bool withDampingForcese,
                             bool withSpringForces) override;

  // Documentation inherited
  void updateImpulseID(const Eigen::Vector6d& bodyImpulse) override;

  // Documentation inherited
  void updateImpulseFD(const Eigen::Vector6d& bodyImpulse) override;

  // Documentation inherited
  void updateConstrainedTerms(double timeStep) override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Recursive algorithm routines for equations of motion
  //----------------------------------------------------------------------------

  // Documentation inherited
  void addChildBiasForceForInvMassMatrix(
      Eigen::Vector6d& parentBiasForce,
      const Eigen::Matrix6d& childArtInertia,
      const Eigen::Vector6d& childBiasForce) override;

  // Documentation inherited
  void addChildBiasForceForInvAugMassMatrix(
      Eigen::Vector6d& parentBiasForce,
      const Eigen::Matrix6d& childArtInertia,
      const Eigen::Vector6d& childBiasForce) override;

  // Documentation inherited
  void updateTotalForceForInvMassMatrix(
      const Eigen::Vector6d& bodyForce) override;

  // Documentation inherited
  void getInvMassMatrixSegment(Eigen::MatrixXd& invMassMat,
                               const size_t col,
                               const Eigen::Matrix6d& artInertia,
                               const Eigen::Vector6d& spatialAcc) override;

  // Documentation inherited
  void getInvAugMassMatrixSegment(Eigen::MatrixXd& invMassMat,
                                  const size_t col,
                                  const Eigen::Matrix6d& artInertia,
                                  const Eigen::Vector6d& spatialAcc) override;

  // Documentation inherited
  void addInvMassMatrixSegmentTo(Eigen::Vector6d& acc) override;

  // Documentation inherited
  Eigen::VectorXd getSpatialToGeneralized(
      const Eigen::Vector6d& spatial) override;

  /// \}

protected:

  /// Properties of this GenericJoint
  typename GenericJoint<ConfigSpaceType>::UniqueProperties mGenericP;
  // TODO(JS): Rename

  /// Array of DegreeOfFreedom objects
  std::array<DegreeOfFreedom*, NumDofs> mDofs;

  /// Command
  Vector mCommands;

  //----------------------------------------------------------------------------
  // Configuration
  //----------------------------------------------------------------------------

  /// Position
  Vector mPositions;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Vector mPositionDeriv;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  /// Generalized velocity
  Vector mVelocities;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Vector mVelocitiesDeriv;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  /// Generalized acceleration
  Vector mAccelerations;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Vector mAccelerationsDeriv;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  /// Generalized force
  Vector mForces;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Vector mForcesDeriv;

  //----------------------------------------------------------------------------
  // Impulse
  //----------------------------------------------------------------------------

  /// Change of generalized velocity
  Vector mVelocityChanges;

  /// Generalized impulse
  Vector mImpulses;

  /// Generalized constraint impulse
  Vector mConstraintImpulses;

  //----------------------------------------------------------------------------
  // For recursive dynamics algorithms
  //----------------------------------------------------------------------------

  /// Spatial Jacobian expressed in the child body frame
  ///
  /// Do not use directly! Use getLocalJacobianStatic() to access this quantity
  mutable JacobianMatrix mJacobian;

  /// Time derivative of spatial Jacobian expressed in the child body frame
  ///
  /// Do not use directly! Use getLocalJacobianTimeDerivStatic() to access this
  /// quantity
  mutable JacobianMatrix mJacobianDeriv;

  /// Inverse of projected articulated inertia
  ///
  /// Do not use directly! Use getInvProjArtInertia() to get this quantity
  mutable Matrix mInvProjArtInertia;

  /// Inverse of projected articulated inertia for implicit joint damping and
  /// spring forces
  ///
  /// Do not use directly! Use getInvProjArtInertiaImplicit() to access this
  /// quantity
  mutable Matrix mInvProjArtInertiaImplicit;

  /// Total force projected on joint space
  Vector mTotalForce;

  /// Total impluse projected on joint space
  Vector mTotalImpulse;

  //----------------------------------------------------------------------------
  // For equations of motion
  //----------------------------------------------------------------------------

  ///
  Vector mInvM_a;

  ///
  Vector mInvMassMatrixSegment;

private:
  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  void addChildArtInertiaToDynamic(
      Eigen::Matrix6d& parentArtInertia,
      const Eigen::Matrix6d& childArtInertia);

  void addChildArtInertiaToKinematic(
      Eigen::Matrix6d& parentArtInertia,
      const Eigen::Matrix6d& childArtInertia);

  void addChildArtInertiaImplicitToDynamic(
      Eigen::Matrix6d& parentArtInertia,
      const Eigen::Matrix6d& childArtInertia);

  void addChildArtInertiaImplicitToKinematic(
      Eigen::Matrix6d& parentArtInertia,
      const Eigen::Matrix6d& childArtInertia);

  void updateInvProjArtInertiaDynamic(
      const Eigen::Matrix6d& artInertia);

  void updateInvProjArtInertiaKinematic(
      const Eigen::Matrix6d& artInertia);

  void updateInvProjArtInertiaImplicitDynamic(
      const Eigen::Matrix6d& artInertia, double timeStep);

  void updateInvProjArtInertiaImplicitKinematic(
      const Eigen::Matrix6d& artInertia, double timeStep);

  void addChildBiasForceToDynamic(
      Eigen::Vector6d& parentBiasForce,
      const Eigen::Matrix6d& childArtInertia,
      const Eigen::Vector6d& childBiasForce,
      const Eigen::Vector6d& childPartialAcc);

  void addChildBiasForceToKinematic(
      Eigen::Vector6d& parentBiasForce,
      const Eigen::Matrix6d& childArtInertia,
      const Eigen::Vector6d& childBiasForce,
      const Eigen::Vector6d& childPartialAcc);

  void addChildBiasImpulseToDynamic(
      Eigen::Vector6d& parentBiasImpulse,
      const Eigen::Matrix6d& childArtInertia,
      const Eigen::Vector6d& childBiasImpulse);

  void addChildBiasImpulseToKinematic(
      Eigen::Vector6d& parentBiasImpulse,
      const Eigen::Matrix6d& childArtInertia,
      const Eigen::Vector6d& childBiasImpulse);

  void updateTotalForceDynamic(const Eigen::Vector6d& bodyForce,
                               double timeStep);

  void updateTotalForceKinematic(const Eigen::Vector6d& bodyForce,
                                 double timeStep);

  void updateTotalImpulseDynamic(
      const Eigen::Vector6d& bodyImpulse);

  void updateTotalImpulseKinematic(
      const Eigen::Vector6d& bodyImpulse);

  void updateAccelerationDynamic(
      const Eigen::Matrix6d& artInertia,
        const Eigen::Vector6d& spatialAcc);

  void updateAccelerationKinematic(
        const Eigen::Matrix6d& artInertia,
        const Eigen::Vector6d& spatialAcc);

  void updateVelocityChangeDynamic(
        const Eigen::Matrix6d& artInertia,
        const Eigen::Vector6d& velocityChange);

  void updateVelocityChangeKinematic(
        const Eigen::Matrix6d& artInertia,
        const Eigen::Vector6d& velocityChange);

  void updateConstrainedTermsDynamic(double timeStep);

  void updateConstrainedTermsKinematic(double timeStep);

  /// \}
};

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/GenericJoint.h"

#endif // DART_DYNAMICS_GENERICJOINT_H_
