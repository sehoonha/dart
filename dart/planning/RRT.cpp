/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** 
 * @file RRT.cpp
 * @author Tobias Kunz, Can Erdogan, Michael X. Grey
 * @date Jan 31, 2013
 * @brief The generic RRT implementation. It can be inherited for modifications to collision
 * checking, sampling and etc.
 */

#include "RRT.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/optimizer/GradientDescentSolver.h"
#include <flann/flann.hpp>

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace simulation;
using namespace dynamics;

namespace dart {
namespace planning {

RRT::Properties::Properties(double stepSize,
                            double maxStepSize,
                            size_t maxConstraintSolveAttempts,
                            double minDistanceBetweenConfigs,
                            const std::shared_ptr<optimizer::Solver>& solver)
  : mStepSize(stepSize),
    mMaxStepSize(maxStepSize),
    mMaxConstraintSolveAttempts(maxConstraintSolveAttempts),
    mMinDistanceBetweenConfigs(minDistanceBetweenConfigs),
    mSolver(solver)
{
  // Do nothing
}

//==============================================================================
RRT::RRT(const WorldPtr& world, const SkeletonPtr& robot,
         const std::vector<size_t>& dofs,
         const VectorXd& root,
         const Properties& properties)
  : mRD(),
    mMT(mRD()),
    mDistribution(0.0, std::nextafter(1.0, 2.0)) // This allows mDistribution to produce numbers in the range [0,1] inclusive
{
  std::vector<Eigen::VectorXd> roots;
  roots.push_back(root);
  reset(world, robot, dofs, roots, properties);
}

//==============================================================================
RRT::RRT(const WorldPtr& world, const SkeletonPtr& robot,
         const std::vector<size_t>& dofs,
         const vector<VectorXd>& roots,
         const Properties& properties)
  : mRD(),
    mMT(mRD()),
    mDistribution(0.0, std::nextafter(1.0, 2.0)) // This allows mDistribution to produce numbers in the range [0,1] inclusive
{
  reset(world, robot, dofs, roots, properties);
}

//==============================================================================
RRT::~RRT()
{
  // Do nothing
}

//==============================================================================
void RRT::reset(const WorldPtr& world, const SkeletonPtr& robot,
                const std::vector<size_t>& dofs,
                const std::vector<VectorXd>& roots)
{
  reset(world, robot, dofs, roots, mProperties);
}

//==============================================================================
void RRT::reset(const WorldPtr& world, const SkeletonPtr& robot,
                const std::vector<size_t>& dofs,
                const std::vector<VectorXd>& roots,
                const Properties& properties)
{
  mIndex = std::unique_ptr< flann::Index< flann::L2<double> > >(
        new flann::Index<flann::L2<double> >(flann::KDTreeSingleIndexParams()));

  mWorld = world;
  mRobot = robot;
  mDofs = dofs;
  mProperties = properties;

  mParentVectors.clear();
  mConfigs.clear();

  for(size_t i = 0; i < roots.size(); i++)
  {
    addNode(roots[i], -1);
  }
}


//==============================================================================
bool RRT::connect() {
	VectorXd qtry = getRandomConfig();
	return connect(qtry);
}

//==============================================================================
bool RRT::connect(const VectorXd &target) {

	// Get the index of the nearest neighbor in the tree to the given target
	int NNidx = getNearestNeighbor(target);

	// Keep taking steps towards the target until a collision happens
	StepResult result = STEP_PROGRESS;
	while(result == STEP_PROGRESS) {
		result = tryStepFromNode(target, NNidx);
    NNidx = mConfigs.size() - 1;
	}
	return (result == STEP_REACHED);
}

//==============================================================================
RRT::StepResult RRT::tryStep() {
	VectorXd qtry = getRandomConfig();
	return tryStep(qtry);
}

//==============================================================================
RRT::StepResult RRT::tryStep(const VectorXd &qtry) {
	int NNidx = getNearestNeighbor(qtry);
	return tryStepFromNode(qtry, NNidx);
}

//==============================================================================
RRT::StepResult RRT::tryStepFromNode(const VectorXd &qtry, int NNidx) {

	// Get the configuration of the nearest neighbor and check if already reached
  const VectorXd& qnear = *(mConfigs[NNidx]);
  if((qtry - qnear).norm() < mProperties.mStepSize) {
		return STEP_REACHED;
	}

	// Create the new node: scale the direction vector to stepSize and add to qnear
  VectorXd qnew = qnear + mProperties.mStepSize * (qtry - qnear).normalized();

	// Check for collision, make changes to the qNew and create intermediate points if necessary
	// NOTE: This is largely implementation dependent and in default, no points are created.
	list<VectorXd> intermediatePoints;
	bool collisionClear = newConfig(intermediatePoints, qnew, qnear, qtry);
	if(!collisionClear) return STEP_COLLISION;

	// Add the intermediate nodes and the final new node to the tree
	list <VectorXd>::iterator it = intermediatePoints.begin();
  for(; it != intermediatePoints.end(); ++it)
		NNidx = addNode(*it, NNidx);
	addNode(qnew, NNidx);
	return STEP_PROGRESS;
}

//==============================================================================
bool RRT::newConfig(list<VectorXd>& /*intermediatePoints*/, VectorXd& qnew,
                    const VectorXd& qnear, const VectorXd& /*qtarget*/) {

  if(mProperties.mSolver && mProperties.mSolver->getProblem())
  {
    const int dim = mProperties.mSolver->getProblem()->getDimension();
    if(dim != qnew.size())
    {
      dterr << "[RRT::newConfig] Mismatch between configuration size ("
            << qnew.size() << ") and Problem size (" << dim << ")\n";
      assert(false);
      return false;
    }

    size_t attempts = 0;
    while(attempts < mProperties.mMaxConstraintSolveAttempts)
    {
      mProperties.mSolver->getProblem()->setInitialGuess(qnew);

      if(!mProperties.mSolver->solve())
        return false;

      qnew = mProperties.mSolver->getProblem()->getOptimalSolution();

      if( (qnear - qnew).norm() <= mProperties.mMinDistanceBetweenConfigs )
        return false;

      if( (qnear - qnew).norm() <= mProperties.mMaxStepSize )
        break;

      qnew = qnear + mProperties.mStepSize*(qnew-qnear).normalized();
      ++attempts;
    }

    if(attempts == mProperties.mMaxConstraintSolveAttempts)
    {
//      std::cout << "Failed to find nearby valid solution" << std::endl;
      return false;
    }
  }

	return !checkCollisions(qnew);
}

//==============================================================================
int RRT::addNode(const VectorXd& qnew, int parentId)
{
  // Update the graph vector
  mConfigs.push_back(std::unique_ptr<VectorXd>(new VectorXd(qnew)));
  mParentVectors.push_back(parentId);

  const std::unique_ptr<const Eigen::VectorXd>& vec = mConfigs.back();

	// Update the underlying flann structure (the kdtree)
  unsigned int id = mConfigs.size() - 1;
  if(id == 0)
    mIndex->buildIndex(flann::Matrix<double>((double*)vec->data(), 1, vec->size()));
  else
    mIndex->addPoints(flann::Matrix<double>((double*)vec->data(), 1, vec->size()));

  mActiveNode = id;
	return id;
}

//==============================================================================
inline int RRT::getNearestNeighbor(const VectorXd& qsamp)
{
	int nearest;
	double distance;
	const flann::Matrix<double> queryMatrix((double*)qsamp.data(), 1, qsamp.size());
	flann::Matrix<int> nearestMatrix(&nearest, 1, 1);
	flann::Matrix<double> distanceMatrix(flann::Matrix<double>(&distance, 1, 1));
  mIndex->knnSearch(queryMatrix, nearestMatrix, distanceMatrix, 1,
		flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));
  mActiveNode = nearest;
	return nearest;
}

//==============================================================================
// random # between min & max
inline double RRT::randomInRange(double min, double max) const
{
	assert(max - min >= 0.0);
	assert(max - min < numeric_limits<double>::infinity());

  if(min == max)
    return min;

  return min + ((max-min) * mDistribution(mMT));
}


//==============================================================================
VectorXd RRT::getRandomConfig() const {
	// Samples a random point for qtmp in the configuration space, bounded by the provided 
	// configuration vectors (and returns ref to it)
  VectorXd config(mDofs.size());
  for (size_t i = 0; i < mDofs.size(); ++i)
  {
    config[i] = randomInRange(mRobot->getPositionLowerLimit(mDofs[i]),
                              mRobot->getPositionUpperLimit(mDofs[i]));
	}

	return config;
}


//==============================================================================
double RRT::getGap(const VectorXd& target)
{
  return (target - *(mConfigs[mActiveNode])).norm();
}


//==============================================================================
void RRT::tracePath(int node, std::list<VectorXd> &path, bool reverse) {

	// Keep following the "linked list" in the given direction
	int x = node;
	while(x != -1) {
    if(!reverse) path.push_front(*(mConfigs[x]));
    else path.push_back(*(mConfigs[x]));
    x = mParentVectors[x];
	}
}


//==============================================================================
bool RRT::checkCollisions(const VectorXd &c) const {
  mRobot->setPositions(mDofs, c);
  return mWorld->checkCollision();
}


//==============================================================================
size_t RRT::getSize() const
{
  return mConfigs.size();
}

} // namespace planning
} // namespace dart
