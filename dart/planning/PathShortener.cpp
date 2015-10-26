#include "PathShortener.h"
#include "dart/simulation/World.h"
#include "RRT.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/dynamics/Skeleton.h"
#include <ctime>
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace simulation;

#define RAND12(N1,N2) N1 + ((N2-N1) * ((double)rand() / ((double)RAND_MAX + 1))) // random # between N&M

namespace dart {
namespace planning {

// TODO: Remove this file

} // namespace planning
} // namespace dart
