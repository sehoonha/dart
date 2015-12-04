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

#include "dart/config.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/ConfigurationSpace.h"

namespace dart {
namespace dynamics {

#ifndef DART_DYNAMICS_DETAIL_TEMPLATEDJOINT_H_
#define DART_DYNAMICS_DETAIL_TEMPLATEDJOINT_H_

#define GENERICJOINT_REPORT_DIM_MISMATCH( func, arg )                  \
  {                                                                    \
    dterr << "[GenericJoint::" #func "] Mismatch beteween size of "    \
          << #arg " [" << arg .size() << "] and the number of "        \
          << "DOFs [" << getNumDofs() << "] for Joint named ["         \
          << getName() << "].\n";                                      \
    assert(false);                                                     \
  }

#define GENERICJOINT_REPORT_OUT_OF_RANGE( func, index )                \
  {                                                                    \
    dterr << "[GenericJoint::" << #func << "] The index [" << index    \
          << "] is out of range for Joint named [" << getName()        \
          << "] which has " << getNumDofs() << " DOFs.\n";             \
    assert(false);                                                     \
  }

#define GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR( func )                      \
  {                                                                           \
    dterr << "[GenericJoint::" # func "] Unsupported actuator type ("         \
          << mJointP.mActuatorType << ") for Joint [" << getName() << "].\n"; \
    assert(false);                                                            \
  }

//==============================================================================
template <class ConfigSpaceType>
GenericJoint<ConfigSpaceType>::UniqueProperties::UniqueProperties(
    const EuclideanPoint& positionLowerLimits,
    const EuclideanPoint& positionUpperLimits,
    const Vector& velocityLowerLimits,
    const Vector& velocityUpperLimits,
    const Vector& accelerationLowerLimits,
    const Vector& accelerationUpperLimits,
    const Vector& forceLowerLimits,
    const Vector& forceUpperLimits,
    const Vector& springStiffness,
    const EuclideanPoint& restPosition,
    const Vector& dampingCoefficient,
    const Vector& coulombFrictions)
  : mPositionLowerLimits(positionLowerLimits),
    mPositionUpperLimits(positionUpperLimits),
    mInitialPositions(EuclideanPoint::Zero()),
    mVelocityLowerLimits(velocityLowerLimits),
    mVelocityUpperLimits(velocityUpperLimits),
    mInitialVelocities(Vector::Zero()),
    mAccelerationLowerLimits(accelerationLowerLimits),
    mAccelerationUpperLimits(accelerationUpperLimits),
    mForceLowerLimits(forceLowerLimits),
    mForceUpperLimits(forceUpperLimits),
    mSpringStiffnesses(springStiffness),
    mRestPositions(restPosition),
    mDampingCoefficients(dampingCoefficient),
    mFrictions(coulombFrictions)
{
  for (size_t i = 0; i < NumDofs; ++i)
  {
    mPreserveDofNames[i] = false;
    mDofNames[i] = std::string();
  }
}

//==============================================================================
template <class ConfigSpaceType>
GenericJoint<ConfigSpaceType>::UniqueProperties::UniqueProperties(
    const UniqueProperties& other)
  : mPositionLowerLimits(other.mPositionLowerLimits),
    mPositionUpperLimits(other.mPositionUpperLimits),
    mInitialPositions(other.mInitialPositions),
    mVelocityLowerLimits(other.mVelocityLowerLimits),
    mVelocityUpperLimits(other.mVelocityUpperLimits),
    mInitialVelocities(other.mInitialVelocities),
    mAccelerationLowerLimits(other.mAccelerationLowerLimits),
    mAccelerationUpperLimits(other.mAccelerationUpperLimits),
    mForceLowerLimits(other.mForceLowerLimits),
    mForceUpperLimits(other.mForceUpperLimits),
    mSpringStiffnesses(other.mSpringStiffnesses),
    mRestPositions(other.mRestPositions),
    mDampingCoefficients(other.mDampingCoefficients),
    mFrictions(other.mFrictions)
{
  for (size_t i = 0; i < NumDofs; ++i)
  {
    mPreserveDofNames[i] = other.mPreserveDofNames[i];
    mDofNames[i] = other.mDofNames[i];
  }
}

//==============================================================================
template <class ConfigSpaceType>
GenericJoint<ConfigSpaceType>::Properties::Properties(
    const Joint::Properties& jointProperties,
    const UniqueProperties& genericProperties)
  : Joint::Properties(jointProperties),
    UniqueProperties(genericProperties)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceType>
GenericJoint<ConfigSpaceType>::Properties::~Properties()
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceType>
GenericJoint<ConfigSpaceType>::~GenericJoint()
{
  for (size_t i = 0; i < NumDofs; ++i)
    delete mDofs[i];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setProperties(const Properties& properties)
{
  Joint::setProperties(static_cast<const Joint::Properties&>(properties));
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setProperties(
    const UniqueProperties& properties)
{
  for (size_t i = 0; i < NumDofs; ++i)
  {
    setDofName(i, properties.mDofNames[i], properties.mPreserveDofNames[i]);
    setPositionLowerLimit    (i, properties.mPositionLowerLimits[i]    );
    setPositionUpperLimit    (i, properties.mPositionUpperLimits[i]    );
    setInitialPosition       (i, properties.mInitialPositions[i]       );
    setVelocityLowerLimit    (i, properties.mVelocityLowerLimits[i]    );
    setVelocityUpperLimit    (i, properties.mVelocityUpperLimits[i]    );
    setInitialVelocity       (i, properties.mInitialVelocities[i]      );
    setAccelerationLowerLimit(i, properties.mAccelerationLowerLimits[i]);
    setAccelerationUpperLimit(i, properties.mAccelerationUpperLimits[i]);
    setForceLowerLimit       (i, properties.mForceLowerLimits[i]       );
    setForceUpperLimit       (i, properties.mForceUpperLimits[i]       );
    setSpringStiffness       (i, properties.mSpringStiffnesses[i]      );
    setRestPosition          (i, properties.mRestPositions[i]          );
    setDampingCoefficient    (i, properties.mDampingCoefficients[i]    );
    setCoulombFriction       (i, properties.mFrictions[i]              );
  }
}

//==============================================================================
template <class ConfigSpaceType>
typename GenericJoint<ConfigSpaceType>::Properties
GenericJoint<ConfigSpaceType>::getGenericJointProperties() const
{
  return GenericJoint<ConfigSpaceType>::Properties(mJointP, mGenericP);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::copy(
    const GenericJoint<ConfigSpaceType>& other)
{
  if (this == &other)
    return;

  setProperties(other.getGenericJointProperties());
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::copy(
    const GenericJoint<ConfigSpaceType>* other)
{
  if (nullptr == other)
    return;

  copy(*other);
}

//==============================================================================
template <class ConfigSpaceType>
GenericJoint<ConfigSpaceType>&
GenericJoint<ConfigSpaceType>::operator=(
    const GenericJoint<ConfigSpaceType>& other)
{
  copy(other);
  return *this;
}

//==============================================================================
template <class ConfigSpaceType>
size_t
GenericJoint<ConfigSpaceType>::getIndexInSkeleton(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getIndexInSkeleton, index);
    return 0;
  }

  return mDofs[index]->mIndexInSkeleton;
}

//==============================================================================
template <class ConfigSpaceType>
size_t GenericJoint<ConfigSpaceType>::getIndexInTree(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getIndexInTree, index);
    return 0;
  }

  return mDofs[index]->mIndexInTree;
}


//==============================================================================
template <class ConfigSpaceType>
DegreeOfFreedom* GenericJoint<ConfigSpaceType>::getDof(size_t index)
{
  if (index < NumDofs)
    return mDofs[index];

  GENERICJOINT_REPORT_OUT_OF_RANGE(getDof, index);

  return nullptr;
}

//==============================================================================
template <class ConfigSpaceType>
const DegreeOfFreedom* GenericJoint<ConfigSpaceType>::getDof(size_t index) const
{
  if (index < NumDofs)
    return mDofs[index];

  GENERICJOINT_REPORT_OUT_OF_RANGE(getDof, index);

  return nullptr;
}

//==============================================================================
template <class ConfigSpaceType>
const std::string& GenericJoint<ConfigSpaceType>::setDofName(
    size_t index,
    const std::string& name,
    bool preserveName)
{
  if (NumDofs <= index)
  {
    dterr << "[GenericJoint::setDofName] Attempting to set the name of DOF "
          << "index " << index << ", which is out of bounds for the Joint ["
          << getName() << "]. We will set the name of DOF index 0 instead.\n";
    assert(false);
    index = 0;
  }

  preserveDofName(index, preserveName);

  std::string& dofName = mGenericP.mDofNames[index];

  if (name == dofName)
    return dofName;

  const SkeletonPtr& skel
      = mChildBodyNode ? mChildBodyNode->getSkeleton() : nullptr;
  if (skel)
    dofName = skel->mNameMgrForDofs.changeObjectName(mDofs[index], name);
  else
    dofName = name;

  return dofName;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::preserveDofName(size_t index, bool preserve)
{
  if (NumDofs <= index)
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(preserveDofName, index);
    return;
  }

  mGenericP.mPreserveDofNames[index] = preserve;
}

//==============================================================================
template <class ConfigSpaceType>
bool GenericJoint<ConfigSpaceType>::isDofNamePreserved(size_t _index) const
{
  if(NumDofs <= _index)
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(isDofNamePreserved, _index);
    _index = 0;
  }

  return mGenericP.mPreserveDofNames[_index];
}

//==============================================================================
template <class ConfigSpaceType>
const std::string& GenericJoint<ConfigSpaceType>::getDofName(size_t index) const
{
  if(NumDofs <= index)
  {
    dterr << "[GenericJoint::getDofName] Requested name of DOF index ["
          << index << "] in Joint [" << getName() << "], but that is out of "
          << "bounds (max " << NumDofs - 1 << "). Returning name of DOF 0.\n";
    assert(false);
    return mGenericP.mDofNames[0];
  }

  return mGenericP.mDofNames[index];
}

//==============================================================================
template <class ConfigSpaceType>
size_t GenericJoint<ConfigSpaceType>::getNumDofs() const
{
  return NumDofs;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setPositionsStatic(const Vector& positions)
{
  if (mPositions == positions)
    return;

  mPositions = positions;
  notifyPositionUpdate();
}

//==============================================================================
template <class ConfigSpaceType>
const typename GenericJoint<ConfigSpaceType>::Vector&
GenericJoint<ConfigSpaceType>::getPositionsStatic() const
{
  return mPositions;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setVelocitiesStatic(
    const Vector& velocities)
{
  if (mVelocities == velocities)
    return;

  mVelocities = velocities;
  notifyVelocityUpdate();
}

//==============================================================================
template <class ConfigSpaceType>
const typename GenericJoint<ConfigSpaceType>::Vector&
GenericJoint<ConfigSpaceType>::getVelocitiesStatic() const
{
  return mVelocities;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setAccelerationsStatic(const Vector& accels)
{
  if (mAccelerations == accels)
    return;

  mAccelerations = accels;
  notifyAccelerationUpdate();
}

//==============================================================================
template <class ConfigSpaceType>
const typename GenericJoint<ConfigSpaceType>::Vector&
GenericJoint<ConfigSpaceType>::getAccelerationsStatic() const
{
  return mAccelerations;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setCommand(size_t index, double command)
{
  if (index >= getNumDofs())
    GENERICJOINT_REPORT_OUT_OF_RANGE(setCommand, index);

  switch (mJointP.mActuatorType)
  {
    case FORCE:
      command = math::clip(command, mGenericP.mForceLowerLimits[index],
                                    mGenericP.mForceUpperLimits[index]);
      mCommands[index] = command;
      assert(!math::isNan(mCommands));
      assert(!math::isInf(mCommands));
      break;
    case PASSIVE:
      if (0.0 != command)
      {
        dtwarn << "[GenericJoint::setCommand] Attempting to set a non-zero ("
               << command << ") command for a PASSIVE joint [" << getName()
               << "].\n";
      }
      mCommands[index] = command;
      break;
    case SERVO:
      command = math::clip(command, mGenericP.mVelocityLowerLimits[index],
                                    mGenericP.mVelocityUpperLimits[index]);
      mCommands[index] = command;
      break;
    case ACCELERATION:
      command = math::clip(command, mGenericP.mAccelerationLowerLimits[index],
                                    mGenericP.mAccelerationUpperLimits[index]);
      mCommands[index] = command;
      break;
    case VELOCITY:
      command = math::clip(command, mGenericP.mVelocityLowerLimits[index],
                                    mGenericP.mVelocityUpperLimits[index]);
      mCommands[index] = command;
      // TODO: This possibly makes the acceleration to exceed the limits.
      break;
    case LOCKED:
      if (0.0 != command)
      {
        dtwarn << "[GenericJoint::setCommand] Attempting to set a non-zero ("
               << command << ") command for a LOCKED joint [" << getName()
               << "].\n";
      }
      mCommands[index] = command;
      break;
    default:
      assert(false);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getCommand(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getCommand, index);
    return 0.0;
  }

  return mCommands[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setCommands(
    const Eigen::VectorXd& commands)
{
  if (static_cast<size_t>(commands.size()) != getNumDofs())
  {
    GENERICJOINT_REPORT_DIM_MISMATCH(setCommands, commands);
    return;
  }

  switch (mJointP.mActuatorType)
  {
    case FORCE:
      mCommands =  math::clip(commands, mGenericP.mForceLowerLimits,
                                        mGenericP.mForceUpperLimits);
      assert(!math::isNan(mCommands));
      assert(!math::isInf(mCommands));
      break;
    case PASSIVE:
      if (Vector::Zero() != commands)
      {
        dtwarn << "[GenericJoint::setCommands] Attempting to set a non-zero ("
               << commands.transpose() << ") command for a PASSIVE joint ["
               << getName() << "].\n";
      }
      mCommands = commands;
      break;
    case SERVO:
      mCommands = math::clip(commands,
                             mGenericP.mVelocityLowerLimits,
                             mGenericP.mVelocityUpperLimits);
      break;
    case ACCELERATION:
      mCommands = math::clip(commands,
                             mGenericP.mAccelerationLowerLimits,
                             mGenericP.mAccelerationUpperLimits);
      break;
    case VELOCITY:
      mCommands = math::clip(commands,
                             mGenericP.mVelocityLowerLimits,
                             mGenericP.mVelocityUpperLimits);
      // TODO: This possibly makes the acceleration to exceed the limits.
      break;
    case LOCKED:
      if (Vector::Zero() != commands)
      {
        dtwarn << "[GenericJoint::setCommands] Attempting to set a non-zero ("
               << commands.transpose() << ") command for a LOCKED joint ["
               << getName() << "].\n";
      }
      mCommands = commands;
      break;
    default:
      assert(false);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getCommands() const
{
  return mCommands;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetCommands()
{
  mCommands.setZero();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setPosition(size_t index, double position)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setPosition, index);
    return;
  }

  if (mPositions[index] == position)
    return;
  // TODO(JS): Above code should be changed something like:
//  if (ConfigSpaceType::getEuclideanPoint(mPositions, index) == position)
//    return;

  // Note: It would not make much sense to use setPositionsStatic() here
  mPositions[index] = position;
  notifyPositionUpdate();
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getPosition(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getPosition, index);
    return 0.0;
  }

  return getPositionsStatic()[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setPositions(
    const Eigen::VectorXd& positions)
{
  if (static_cast<size_t>(positions.size()) != getNumDofs())
  {
    GENERICJOINT_REPORT_DIM_MISMATCH(setPositions, positions);
    return;
  }

  setPositionsStatic(positions);
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getPositions() const
{
  return getPositionsStatic();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setPositionLowerLimit(size_t index,
                                                          double position)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setPositionLowerLimit, index);
    return;
  }

  mGenericP.mPositionLowerLimits[index] = position;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getPositionLowerLimit(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getPositionLowerLimit, index);
    return 0.0;
  }

  return mGenericP.mPositionLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setPositionUpperLimit(size_t index,
                                                          double position)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setPositionUpperLimit, index);
    return;
  }

  mGenericP.mPositionUpperLimits[index] = position;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getPositionUpperLimit(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getPositionUpperLimit, index);
    return 0.0;
  }

  return mGenericP.mPositionUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceType>
bool GenericJoint<ConfigSpaceType>::hasPositionLimit(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(hasPositionLimit, index);
    return true;
  }

  return std::isfinite(mGenericP.mPositionLowerLimits[index])
      || std::isfinite(mGenericP.mPositionUpperLimits[index]);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetPosition(size_t index)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(resetPosition, index);
    return;
  }

  setPosition(index, mGenericP.mInitialPositions[index]);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetPositions()
{
  setPositionsStatic(mGenericP.mInitialPositions);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setInitialPosition(size_t index,
                                                       double initial)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setInitialPosition, index);
    return;
  }

  mGenericP.mInitialPositions[index] = initial;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getInitialPosition(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getInitialPosition, index);
    return 0.0;
  }

  return mGenericP.mInitialPositions[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setInitialPositions(
    const Eigen::VectorXd& initial)
{
  if ( static_cast<size_t>(initial.size()) != getNumDofs() )
  {
    GENERICJOINT_REPORT_DIM_MISMATCH(setInitialPositions, initial);
    return;
  }

  mGenericP.mInitialPositions = initial;
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getInitialPositions() const
{
  return mGenericP.mInitialPositions;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setVelocity(size_t index, double velocity)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setVelocity, index);
    return;
  }

  if (mVelocities[index] == velocity)
    return;

  // Note: It would not make much sense to use setVelocitiesStatic() here
  mVelocities[index] = velocity;
  notifyVelocityUpdate();

#if DART_MAJOR_MINOR_VERSION_AT_MOST(5,2)
  if (mJointP.mActuatorType == VELOCITY)
    mCommands[index] = velocity;
#endif

  assert(!math::isNan(mCommands));
  assert(!math::isInf(mCommands));
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getVelocity(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getVelocity, index);
    return 0.0;
  }

  return getVelocitiesStatic()[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setVelocities(
    const Eigen::VectorXd& velocities)
{
  if (static_cast<size_t>(velocities.size()) != getNumDofs())
  {
    GENERICJOINT_REPORT_DIM_MISMATCH(setVelocities, velocities);
    return;
  }

  setVelocitiesStatic(velocities);

#if DART_MAJOR_MINOR_VERSION_AT_MOST(5,2)
  if (mJointP.mActuatorType == VELOCITY)
    mCommands = getVelocitiesStatic();
#endif

  assert(!math::isNan(mCommands));
  assert(!math::isInf(mCommands));
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getVelocities() const
{
  return getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setVelocityLowerLimit(size_t index,
                                                          double velocity)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setVelocityLowerLimit, index);
    return;
  }

  mGenericP.mVelocityLowerLimits[index] = velocity;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getVelocityLowerLimit(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getVelocityLowerLimit, index);
    return 0.0;
  }

  return mGenericP.mVelocityLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setVelocityUpperLimit(size_t index,
                                                          double velocity)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setVelocityUpperLimit, index);
    return;
  }

  mGenericP.mVelocityUpperLimits[index] = velocity;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getVelocityUpperLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getVelocityUpperLimit, index);
    return 0.0;
  }

  return mGenericP.mVelocityUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetVelocity(size_t index)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(resetVelocity, index);
    return;
  }

  setVelocity(index, mGenericP.mInitialVelocities[index]);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetVelocities()
{
  setVelocitiesStatic(mGenericP.mInitialVelocities);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setInitialVelocity(size_t index,
                                                       double initial)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setInitialVelocity, index);
    return;
  }

  mGenericP.mInitialVelocities[index] = initial;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getInitialVelocity(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getInitialVelocity, index);
    return 0.0;
  }

  return mGenericP.mInitialVelocities[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setInitialVelocities(
    const Eigen::VectorXd& initial)
{
  if ( static_cast<size_t>(initial.size()) != getNumDofs() )
  {
    GENERICJOINT_REPORT_DIM_MISMATCH( setInitialVelocities, initial );
    return;
  }

  mGenericP.mInitialVelocities = initial;
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getInitialVelocities() const
{
  return mGenericP.mInitialVelocities;
}


//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setAcceleration(size_t index,
                                                    double acceleration)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE( setAcceleration, index );
    return;
  }

  if (mAccelerations[index] == acceleration)
    return;

  // Note: It would not make much sense to use setAccelerationsStatic() here
  mAccelerations[index] = acceleration;
  notifyAccelerationUpdate();

#if DART_MAJOR_MINOR_VERSION_AT_MOST(5,2)
  if (mJointP.mActuatorType == ACCELERATION)
    mCommands[index] = acceleration;
#endif

  assert(!math::isNan(mCommands));
  assert(!math::isInf(mCommands));
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getAcceleration(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getAcceleration, index);
    return 0.0;
  }

  return getAccelerationsStatic()[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setAccelerations(
    const Eigen::VectorXd& accelerations)
{
  if (static_cast<size_t>(accelerations.size()) != getNumDofs())
  {
    GENERICJOINT_REPORT_DIM_MISMATCH( setAccelerations, accelerations );
    return;
  }

  setAccelerationsStatic(accelerations);

#if DART_MAJOR_MINOR_VERSION_AT_MOST(5,2)
  if (mJointP.mActuatorType == ACCELERATION)
    mCommands = getAccelerationsStatic();
#endif

  assert(!math::isNan(mCommands));
  assert(!math::isInf(mCommands));
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getAccelerations() const
{
  return getAccelerationsStatic();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setAccelerationLowerLimit(
    size_t index, double acceleration)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setAccelerationLowerLimit, index);
    return;
  }

  mGenericP.mAccelerationLowerLimits[index] = acceleration;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getAccelerationLowerLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getAccelerationLowerLimit, index);
    return 0.0;
  }

  return mGenericP.mAccelerationLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setAccelerationUpperLimit(
    size_t index, double acceleration)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setAccelerationUpperLimit, index)
    return;
  }

  mGenericP.mAccelerationUpperLimits[index] = acceleration;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getAccelerationUpperLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getAccelerationUpperLimit, index);
    return 0.0;
  }

  return mGenericP.mAccelerationUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetAccelerations()
{
  setAccelerationsStatic(Vector::Zero());
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setForce(size_t index, double force)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setForce, index);
    return;
  }

  mForces[index] = force;

#if DART_MAJOR_MINOR_VERSION_AT_MOST(5,2)
  if (mJointP.mActuatorType == FORCE)
    mCommands[index] = force;
#endif

  assert(!math::isNan(mForces));
  assert(!math::isInf(mForces));
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getForce(size_t index)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getForce, index);
    return 0.0;
  }

  return mForces[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setForces(const Eigen::VectorXd& forces)
{
  if (static_cast<size_t>(forces.size()) != getNumDofs())
  {
    GENERICJOINT_REPORT_DIM_MISMATCH(setForces, forces);
    return;
  }

  mForces = forces;

#if DART_MAJOR_MINOR_VERSION_AT_MOST(5,1)
  if (mJointP.mActuatorType == FORCE)
    mCommands = mForces;
  // TODO: Remove at DART 5.1.
#endif

  assert(!math::isNan(mForces));
  assert(!math::isInf(mForces));

  assert(!math::isNan(mCommands));
  assert(!math::isInf(mCommands));
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getForces() const
{
  return mForces;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setForceLowerLimit(size_t index,
                                                       double force)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setForceLowerLimit, index);
    return;
  }

  mGenericP.mForceLowerLimits[index] = force;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getForceLowerLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getForceLowerLimit, index);
    return 0.0;
  }

  return mGenericP.mForceLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setForceUpperLimit(size_t index,
                                                       double force)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setForceUpperLimit, index);
    return;
  }

  mGenericP.mForceUpperLimits[index] = force;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getForceUpperLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getForceUpperLimit, index);
    return 0.0;
  }

  return mGenericP.mForceUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetForces()
{
  mForces.setZero();

#if DART_MAJOR_MINOR_VERSION_AT_MOST(5,2)
  if (mJointP.mActuatorType == FORCE)
    mCommands = mForces;
#endif

  assert(!math::isNan(mForces));
  assert(!math::isInf(mForces));

  assert(!math::isNan(mCommands));
  assert(!math::isInf(mCommands));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setVelocityChange(
    size_t index, double velocityChange)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setVelocityChange, index);
    return;
  }

  mVelocityChanges[index] = velocityChange;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getVelocityChange(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getVelocityChange, index);
    return 0.0;
  }

  return mVelocityChanges[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetVelocityChanges()
{
  mVelocityChanges.setZero();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setConstraintImpulse(size_t index,
                                                         double impulse)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setConstraintImpulse, index);
    return;
  }

  mConstraintImpulses[index] = impulse;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getConstraintImpulse(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getConstraintImpulse, index);
    return 0.0;
  }

  return mConstraintImpulses[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetConstraintImpulses()
{
  mConstraintImpulses.setZero();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::integratePositions(double dt)
{
  const Point& point = detail::integratePosition<ConfigSpaceType>(
        detail::mapToManifoldPoint<ConfigSpaceType>(getPositionsStatic()),
        getVelocitiesStatic(), dt);

  setPositionsStatic(detail::mapToEuclideanPoint<ConfigSpaceType>(point));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::integrateVelocities(double dt)
{
  setVelocitiesStatic(detail::integrateVelocity<ConfigSpaceType>(
                        getVelocitiesStatic(),
                        getAccelerationsStatic(), dt));
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getPositionDifferences(
    const Eigen::VectorXd& q2, const Eigen::VectorXd& q1) const
{
  if (static_cast<size_t>(q1.size()) != getNumDofs()
      || static_cast<size_t>(q2.size()) != getNumDofs())
  {
    dterr << "[GenericJoint::getPositionsDifference] q1's size [" << q1.size()
          << "] or q2's size [" << q2.size() << "] must both equal the dof ["
          << getNumDofs() << "] for Joint [" << getName() << "].\n";
    assert(false);
    return Eigen::VectorXd::Zero(getNumDofs());
  }

  return getPositionDifferencesStatic(q2, q1);
}

//==============================================================================
template <class ConfigSpaceType>
typename ConfigSpaceType::Vector
GenericJoint<ConfigSpaceType>::getPositionDifferencesStatic(
    const Vector& q2, const Vector& q1) const
{
  return q2 - q1;
  // TODO(JS): Move this implementation to each configuration space classes.
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setSpringStiffness(size_t index, double k)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setSpringStiffness, index);
    return;
  }

  assert(k >= 0.0);

  mGenericP.mSpringStiffnesses[index] = k;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getSpringStiffness(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getSpringStiffness, index);
    return 0.0;
  }

  return mGenericP.mSpringStiffnesses[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setRestPosition(size_t index, double q0)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setRestPosition, index);
    return;
  }

  if (mGenericP.mPositionLowerLimits[index] > q0
      || mGenericP.mPositionUpperLimits[index] < q0)
  {
    dtwarn << "[GenericJoint::setRestPosition] Value of _q0 [" << q0
           << "], is out of the limit range ["
           << mGenericP.mPositionLowerLimits[index]
           << ", "
           << mGenericP.mPositionUpperLimits[index]
           << "] for index ["
           << index << "] of Joint [" << getName() << "].\n";
    return;
  }

  mGenericP.mRestPositions[index] = q0;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getRestPosition(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getRestPosition, index);
    return 0.0;
  }

  return mGenericP.mRestPositions[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setDampingCoefficient(size_t index,
                                                          double d)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setDampingCoefficient, index);
    return;
  }

  assert(d >= 0.0);

  mGenericP.mDampingCoefficients[index] = d;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getDampingCoefficient(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getDampingCoefficient, index);
    return 0.0;
  }

  return mGenericP.mDampingCoefficients[index];
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setCoulombFriction(size_t index,
                                                       double friction)
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(setCoulombFriction, index);
    return;
  }

  assert(friction >= 0.0);

  mGenericP.mFrictions[index] = friction;
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getCoulombFriction(size_t index) const
{
  if (index >= getNumDofs())
  {
    GENERICJOINT_REPORT_OUT_OF_RANGE(getCoulombFriction, index);
    return 0.0;
  }

  return mGenericP.mFrictions[index];
}

//==============================================================================
template <class ConfigSpaceType>
double GenericJoint<ConfigSpaceType>::getPotentialEnergy() const
{
  // Spring energy
  Vector displacement = getPositionsStatic() - mGenericP.mRestPositions;

  const double pe = 0.5 * displacement.dot(
        mGenericP.mSpringStiffnesses.cwiseProduct(displacement));

  return pe;
}

//==============================================================================
template <class ConfigSpaceType>
const math::Jacobian
GenericJoint<ConfigSpaceType>::getLocalJacobian() const
{
  return getLocalJacobianStatic();
}

//==============================================================================
template <class ConfigSpaceType>
const typename GenericJoint<ConfigSpaceType>::JacobianMatrix&
GenericJoint<ConfigSpaceType>::getLocalJacobianStatic() const
{
  if (mIsLocalJacobianDirty)
  {
    updateLocalJacobian(false);
    mIsLocalJacobianDirty = false;
  }

  return mJacobian;
}

//==============================================================================
template <class ConfigSpaceType>
const math::Jacobian GenericJoint<ConfigSpaceType>::getLocalJacobian(
    const Eigen::VectorXd& positions) const
{
  return getLocalJacobianStatic(positions);
}

//==============================================================================
template <class ConfigSpaceType>
const math::Jacobian
GenericJoint<ConfigSpaceType>::getLocalJacobianTimeDeriv() const
{
  return getLocalJacobianTimeDerivStatic();
}

//==============================================================================
template <class ConfigSpaceType>
const typename GenericJoint<ConfigSpaceType>::JacobianMatrix&
GenericJoint<ConfigSpaceType>::getLocalJacobianTimeDerivStatic() const
{
  if (mIsLocalJacobianTimeDerivDirty)
  {
    updateLocalJacobianTimeDeriv();
    mIsLocalJacobianTimeDerivDirty = false;
  }

  return mJacobianDeriv;
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::Vector6d
GenericJoint<ConfigSpaceType>::getBodyConstraintWrench() const
{
  assert(mChildBodyNode);
  return mChildBodyNode->getBodyForce() - getLocalJacobianStatic() * mForces;
}

//==============================================================================
template <class ConfigSpaceType>
GenericJoint<ConfigSpaceType>::GenericJoint(
    const Properties& properties)
  : Joint(properties),
    mGenericP(properties),
    mCommands(Vector::Zero()),
    mPositions(EuclideanPoint::Zero()),
    mPositionDeriv(EuclideanPoint::Zero()),
    mVelocities(Vector::Zero()),
    mVelocitiesDeriv(Vector::Zero()),
    mAccelerations(Vector::Zero()),
    mAccelerationsDeriv(Vector::Zero()),
    mForces(Vector::Zero()),
    mForcesDeriv(Vector::Zero()),
    mVelocityChanges(Vector::Zero()),
    mImpulses(Vector::Zero()),
    mConstraintImpulses(Vector::Zero()),
    mJacobian(JacobianMatrix::Zero()),
    mJacobianDeriv(JacobianMatrix::Zero()),
    mInvProjArtInertia(Matrix::Zero()),
    mInvProjArtInertiaImplicit(Matrix::Zero()),
    mTotalForce(Vector::Zero()),
    mTotalImpulse(Vector::Zero())
{
  for (size_t i = 0; i < NumDofs; ++i)
    mDofs[i] = createDofPointer(i);

  assert(!math::isNan(mForces));
  assert(!math::isInf(mForces));

  assert(!math::isNan(mCommands));
  assert(!math::isInf(mCommands));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::registerDofs()
{
  const SkeletonPtr& skel = mChildBodyNode->getSkeleton();
  for (size_t i = 0; i < NumDofs; ++i)
  {
    mGenericP.mDofNames[i]
        = skel->mNameMgrForDofs.issueNewNameAndAdd(mDofs[i]->getName(),
                                                   mDofs[i]);
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateLocalSpatialVelocity() const
{
  mSpatialVelocity = getLocalJacobianStatic() * getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateLocalSpatialAcceleration() const
{
  mSpatialAcceleration = getLocalPrimaryAcceleration()
                    + getLocalJacobianTimeDerivStatic() * getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateLocalPrimaryAcceleration() const
{
  mPrimaryAcceleration = getLocalJacobianStatic() * getAccelerationsStatic();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addVelocityTo(Eigen::Vector6d& vel)
{
  // Add joint velocity to _vel
  vel.noalias() += getLocalJacobianStatic() * getVelocitiesStatic();

  // Verification
  assert(!math::isNan(vel));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::setPartialAccelerationTo(
    Eigen::Vector6d& partialAcceleration,
    const Eigen::Vector6d& childVelocity)
{
  // ad(V, S * dq) + dS * dq
  partialAcceleration = math::ad(childVelocity,
                      getLocalJacobianStatic() * getVelocitiesStatic())
                    + getLocalJacobianTimeDerivStatic() * getVelocitiesStatic();
  // Verification
  assert(!math::isNan(partialAcceleration));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addAccelerationTo(
    Eigen::Vector6d& acc)
{
  // Add joint acceleration to _acc
  acc.noalias() += getLocalJacobianStatic() * getAccelerationsStatic();

  // Verification
  assert(!math::isNan(acc));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addVelocityChangeTo(
    Eigen::Vector6d& velocityChange)
{
  // Add joint velocity change to _velocityChange
  velocityChange.noalias() += getLocalJacobianStatic() * mVelocityChanges;

  // Verification
  assert(!math::isNan(velocityChange));
}

//==============================================================================
template <class ConfigSpaceType>
const typename GenericJoint<ConfigSpaceType>::Matrix&
GenericJoint<ConfigSpaceType>::getInvProjArtInertia() const
{
  Joint::updateArticulatedInertia();

  return mInvProjArtInertia;
}

//==============================================================================
template <class ConfigSpaceType>
const typename GenericJoint<ConfigSpaceType>::Matrix&
GenericJoint<ConfigSpaceType>::getInvProjArtInertiaImplicit() const
{
  Joint::updateArticulatedInertia();

  return mInvProjArtInertiaImplicit;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildArtInertiaTo(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaToDynamic(parentArtInertia,
                                       childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildArtInertiaToKinematic(parentArtInertia,
                                             childArtInertia);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildArtInertiaTo);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildArtInertiaToDynamic(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  // Child body's articulated inertia
  JacobianMatrix AIS = childArtInertia * getLocalJacobianStatic();
  Eigen::Matrix6d PI = childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertia * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(getLocalTransform().inverse(), PI);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildArtInertiaToKinematic(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              childArtInertia);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaImplicitToDynamic(parentArtInertia,
                                                childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildArtInertiaImplicitToKinematic(parentArtInertia,
                                                childArtInertia);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildArtInertiaImplicitTo);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildArtInertiaImplicitToDynamic(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  // Child body's articulated inertia
  JacobianMatrix AIS = childArtInertia * getLocalJacobianStatic();
  Eigen::Matrix6d PI = childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertiaImplicit * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(getLocalTransform().inverse(), PI);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildArtInertiaImplicitToKinematic(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              childArtInertia);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateInvProjArtInertia(
    const Eigen::Matrix6d& artInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaDynamic(artInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateInvProjArtInertiaKinematic(artInertia);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateInvProjArtInertia);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateInvProjArtInertiaDynamic(
    const Eigen::Matrix6d& artInertia)
{
  // Projected articulated inertia
  const JacobianMatrix& Jacobian = getLocalJacobianStatic();
  const Matrix projAI = Jacobian.transpose() * artInertia * Jacobian;

  // Inversion of projected articulated inertia
  mInvProjArtInertia = detail::inverse<ConfigSpaceType>(projAI);

  // Verification
  assert(!math::isNan(mInvProjArtInertia));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateInvProjArtInertiaKinematic(
    const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& artInertia,
    double timeStep)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaImplicitDynamic(artInertia, timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateInvProjArtInertiaImplicitKinematic(artInertia, timeStep);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(
            updateInvProjArtInertiaImplicit);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateInvProjArtInertiaImplicitDynamic(
    const Eigen::Matrix6d& artInertia,
    double timeStep)
{
  // Projected articulated inertia
  const JacobianMatrix& Jacobian = getLocalJacobianStatic();
  Matrix projAI = Jacobian.transpose() * artInertia * Jacobian;

  // Add additional inertia for implicit damping and spring force
  projAI +=
      (timeStep * mGenericP.mDampingCoefficients
       + timeStep * timeStep * mGenericP.mSpringStiffnesses).asDiagonal();

  // Inversion of projected articulated inertia
  mInvProjArtInertiaImplicit = detail::inverse<ConfigSpaceType>(projAI);

  // Verification
  assert(!math::isNan(mInvProjArtInertiaImplicit));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateInvProjArtInertiaImplicitKinematic(
    const Eigen::Matrix6d& /*artInertia*/, double /*timeStep*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildBiasForceTo(
    Eigen::Vector6d& parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce,
    const Eigen::Vector6d& childPartialAcc)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasForceToDynamic(parentBiasForce,
                                 childArtInertia,
                                 childBiasForce,
                                 childPartialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildBiasForceToKinematic(parentBiasForce,
                                   childArtInertia,
                                   childBiasForce,
                                   childPartialAcc);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildBiasForceTo);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildBiasForceToDynamic(
    Eigen::Vector6d& parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce,
    const Eigen::Vector6d& childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = childBiasForce
        + childArtInertia
          * (childPartialAcc
             + getLocalJacobianStatic() * getInvProjArtInertiaImplicit()
               *mTotalForce);

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildBiasForceToKinematic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce,
    const Eigen::Vector6d& childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = childBiasForce
        + childArtInertia*(childPartialAcc
                            + getLocalJacobianStatic()*getAccelerationsStatic());

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildBiasImpulseTo(
    Eigen::Vector6d& parentBiasImpulse,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasImpulseToDynamic(parentBiasImpulse,
                                   childArtInertia,
                                   childBiasImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildBiasImpulseToKinematic(parentBiasImpulse,
                                     childArtInertia,
                                     childBiasImpulse);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildBiasImpulseTo);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildBiasImpulseToDynamic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasImpulse)
{
  // Compute beta
  const Eigen::Vector6d beta
      = childBiasImpulse
        + childArtInertia*getLocalJacobianStatic()
          *getInvProjArtInertia()*mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildBiasImpulseToKinematic(
    Eigen::Vector6d& parentBiasImpulse,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasImpulse)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  parentBiasImpulse += math::dAdInvT(getLocalTransform(), childBiasImpulse);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateTotalForce(
    const Eigen::Vector6d& bodyForce,
    double timeStep)
{
  assert(timeStep > 0.0);

  switch (mJointP.mActuatorType)
  {
    case FORCE:
      mForces = mCommands;
      assert(!math::isNan(mForces));
      assert(!math::isInf(mForces));
      updateTotalForceDynamic(bodyForce, timeStep);
      break;
    case PASSIVE:
    case SERVO:
      mForces.setZero();
      updateTotalForceDynamic(bodyForce, timeStep);
      break;
    case ACCELERATION:
      setAccelerationsStatic(mCommands);
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    case VELOCITY:
      setAccelerationsStatic( (mCommands - getVelocitiesStatic()) / timeStep );
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    case LOCKED:
      setVelocitiesStatic(Vector::Zero());
      setAccelerationsStatic(Vector::Zero());
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateTotalForce);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateTotalForceDynamic(
    const Eigen::Vector6d& bodyForce,
    double timeStep)
{
  // Spring force
  const Vector springForce
      = -mGenericP.mSpringStiffnesses.cwiseProduct(
        getPositionsStatic()
        - mGenericP.mRestPositions
        + getVelocitiesStatic() * timeStep);

  // Damping force
  const Vector dampingForce
      = -mGenericP.mDampingCoefficients.cwiseProduct(getVelocitiesStatic());

  //
  mTotalForce = mForces
      + springForce
      + dampingForce
      - getLocalJacobianStatic().transpose() * bodyForce;

  assert(!math::isNan(mTotalForce));
  assert(!math::isInf(mTotalForce));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateTotalForceKinematic(
    const Eigen::Vector6d& bodyForce,
    double timeStep)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateTotalImpulse(
    const Eigen::Vector6d& bodyImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateTotalImpulseDynamic(bodyImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateTotalImpulseKinematic(bodyImpulse);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateTotalImpulse);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateTotalImpulseDynamic(
    const Eigen::Vector6d& bodyImpulse)
{
  //
  mTotalImpulse = mConstraintImpulses
      - getLocalJacobianStatic().transpose() * bodyImpulse;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateTotalImpulseKinematic(
    const Eigen::Vector6d& /*bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::resetTotalImpulses()
{
  mTotalImpulse.setZero();
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateAcceleration(
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& spatialAcc)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateAccelerationDynamic(artInertia, spatialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateAccelerationKinematic(artInertia, spatialAcc);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateAcceleration);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateAccelerationDynamic(
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& spatialAcc)
{
  //
  setAccelerationsStatic( getInvProjArtInertiaImplicit()
        * (mTotalForce - getLocalJacobianStatic().transpose()
           *artInertia*math::AdInvT(getLocalTransform(), spatialAcc)) );

  // Verification
  assert(!math::isNan(getAccelerationsStatic()));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateAccelerationKinematic(
    const Eigen::Matrix6d& /*artInertia*/,
    const Eigen::Vector6d& /*spatialAcc*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateVelocityChange(
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& velocityChange)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateVelocityChangeDynamic(artInertia, velocityChange);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateVelocityChangeKinematic(artInertia, velocityChange);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateVelocityChange);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateVelocityChangeDynamic(
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& velocityChange)
{
  //
  mVelocityChanges
      = getInvProjArtInertia()
      * (mTotalImpulse - getLocalJacobianStatic().transpose()
         *artInertia*math::AdInvT(getLocalTransform(), velocityChange));

  // Verification
  assert(!math::isNan(mVelocityChanges));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateVelocityChangeKinematic(
    const Eigen::Matrix6d& /*artInertia*/,
    const Eigen::Vector6d& /*velocityChange*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateForceID(
    const Eigen::Vector6d& bodyForce,
    double timeStep,
    bool withDampingForces,
    bool withSpringForces)
{
  mForces = getLocalJacobianStatic().transpose()*bodyForce;

  // Damping force
  if (withDampingForces)
  {
    const typename ConfigSpaceType::Vector dampingForces
        = -mGenericP.mDampingCoefficients.cwiseProduct(getVelocitiesStatic());
    mForces -= dampingForces;
  }

  // Spring force
  if (withSpringForces)
  {
    const typename ConfigSpaceType::Vector springForces
        = -mGenericP.mSpringStiffnesses.cwiseProduct(
          getPositionsStatic()
          - mGenericP.mRestPositions
          + getVelocitiesStatic() * timeStep);
    mForces -= springForces;
  }

  assert(!math::isNan(mForces));
  assert(!math::isInf(mForces));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateForceFD(
    const Eigen::Vector6d& bodyForce,
    double timeStep,
    bool withDampingForces,
    bool withSpringForces)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateForceID(bodyForce, timeStep, withDampingForces, withSpringForces);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateForceFD);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateImpulseID(
    const Eigen::Vector6d& bodyImpulse)
{
  mImpulses = getLocalJacobianStatic().transpose()*bodyImpulse;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateImpulseFD(
    const Eigen::Vector6d& bodyImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateImpulseID(bodyImpulse);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateImpulseFD);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateConstrainedTerms(double timeStep)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateConstrainedTermsDynamic(timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateConstrainedTermsKinematic(timeStep);
      break;
    default:
      GENERICJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateConstrainedTerms);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateConstrainedTermsDynamic(
    double timeStep)
{
  const double invTimeStep = 1.0 / timeStep;

  setVelocitiesStatic(getVelocitiesStatic() + mVelocityChanges);
  setAccelerationsStatic(getAccelerationsStatic()
                         + mVelocityChanges * invTimeStep);
  mForces += mImpulses * invTimeStep;
  // Note: As long as this is only called from BodyNode::updateConstrainedTerms

  assert(!math::isNan(mForces));
  assert(!math::isInf(mForces));
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateConstrainedTermsKinematic(
    double timeStep)
{
  mForces += mImpulses / timeStep;

  assert(!math::isNan(mForces));
  assert(!math::isInf(mForces));
}


//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = childBiasForce;
  beta.noalias() += childArtInertia * getLocalJacobianStatic()
                    * getInvProjArtInertia() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = childBiasForce;
  beta.noalias() += childArtInertia * getLocalJacobianStatic()
                    * getInvProjArtInertiaImplicit() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& bodyForce)
{
  // Compute alpha
  mInvM_a = mForces - getLocalJacobianStatic().transpose() * bodyForce;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::getInvMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertia()
      * (mInvM_a - getLocalJacobianStatic().transpose()
         * artInertia * math::AdInvT(getLocalTransform(), spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  _invMassMat.block<NumDofs, 1>(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& invMassMat,
    const size_t col,
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertiaImplicit()
      * (mInvM_a - getLocalJacobianStatic().transpose()
         * artInertia * math::AdInvT(getLocalTransform(), spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  invMassMat.block<NumDofs, 1>(iStart, col) = mInvMassMatrixSegment;
}

//==============================================================================
template <class ConfigSpaceType>
void GenericJoint<ConfigSpaceType>::addInvMassMatrixSegmentTo(
    Eigen::Vector6d& acc)
{
  //
  acc += getLocalJacobianStatic() * mInvMassMatrixSegment;
}

//==============================================================================
template <class ConfigSpaceType>
Eigen::VectorXd GenericJoint<ConfigSpaceType>::getSpatialToGeneralized(
    const Eigen::Vector6d& spatial)
{
  return getLocalJacobianStatic().transpose() * spatial;
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_TEMPLATEDJOINT_H_
