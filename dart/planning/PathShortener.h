#pragma once

#include <list>
#include <vector>
#include <Eigen/Core>

#include "dart/simulation/World.h"
#include "dart/planning/RRT.h"

namespace dart {

namespace simulation { class World; }
namespace dynamics { class Skeleton; }

namespace planning {

const size_t DefaultNumShortenAttempts = 10;

template <class R = RRT>
class PathShortener
{
public:

  simulation::WorldPtr mWorld;
  typename R::Properties mProperties;
  size_t mNumShortenAttempts;
  std::unique_ptr<R> mRRT;

  PathShortener(const simulation::WorldPtr& world = nullptr,
                const typename R::Properties& properties = typename R::Properties(),
                size_t numShortenAttempts = DefaultNumShortenAttempts)
    : mWorld(world), mProperties(properties), mNumShortenAttempts(numShortenAttempts),
      mRD(), mMT(mRD()), mDistribution(0.0, std::nextafter(1.0, 2.0))
  {
    // Do nothing
  }

  using Path = std::list<Eigen::VectorXd>;

  void shortenPath(const dynamics::SkeletonPtr& skel,
                   const std::vector<size_t>& dofs,
                   Path& rawPath);

  bool validSegment(Path& replacement,
                    const Eigen::VectorXd& config1,
                    const Eigen::VectorXd& config2);


protected:

  /// Randomization device
  std::random_device mRD;

  /// Mersene twister method
  mutable std::mt19937 mMT;

  /// Distribution
  mutable std::uniform_real_distribution<double> mDistribution;

};

//==============================================================================
template <class Container>
typename Container::iterator getIterator(Container& container, size_t location)
{
  typename Container::iterator it = container.begin();
  for(size_t i=0; i < location; ++i)
    ++it;

  return it;
}

//==============================================================================
template <class R>
void PathShortener<R>::shortenPath(
    const dynamics::SkeletonPtr& skel,
    const std::vector<size_t>& dofs, Path& rawPath)
{
  mRRT = std::unique_ptr<R>(
        new R(mWorld, skel, dofs, std::vector<Eigen::VectorXd>(), mProperties));

  for(size_t i=0; i < mNumShortenAttempts; ++i)
  {
    size_t num = rawPath.size();
    if(0 == num)
      return;

    Path::iterator begin;
    Path::iterator end;
    if(0 == i)
    {
      begin = rawPath.begin();
      end = --rawPath.end();
    }
    else
    {
      size_t start = static_cast<size_t>(num * mDistribution(mMT));
      size_t finish = static_cast<size_t>( (num-start)*mDistribution(mMT) + start );
      begin = getIterator(rawPath, start);
      end = getIterator(rawPath, finish);
    }

    if(begin == end)
      continue;

    Path replacement;
    if(validSegment(replacement, *begin, *end))
    {
      rawPath.erase(++begin, end);
      rawPath.insert(end, replacement.begin(), replacement.end());

      if(0 == i)
        return;
    }
  }
}

//==============================================================================
template <class R>
bool PathShortener<R>::validSegment(Path& replacement,
                                    const Eigen::VectorXd& config1,
                                    const Eigen::VectorXd& config2)
{
  double stepSize = mProperties.mStepSize;
  Eigen::VectorXd testConfig = config1;
  Eigen::VectorXd lastTestConfig = testConfig;
  std::list<Eigen::VectorXd> temp;

  while( (config2 - testConfig).norm() > stepSize )
  {
    lastTestConfig = testConfig;
    testConfig = testConfig + stepSize*(config2 - testConfig).normalized();
    if(mRRT->newConfig(temp, testConfig, lastTestConfig, config2))
      replacement.push_back(testConfig);
    else
      return false;
  }

  return true;
}

} // namespace planning
} // namespace dart
