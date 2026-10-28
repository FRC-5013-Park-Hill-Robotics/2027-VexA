// #include "utility.h"

// using namespace TroLib;

// float linearInterpolator(std::vector<std::pair<float, float>> points, float interpolateTime)
// {
//      if (points.size() < 2)
//         throw std::invalid_argument("At least two points are required");

//     // Use the two most recent measurements.
//     const auto& p1 = points[points.size() - 2];
//     const auto& p2 = points[points.size() - 1];

//     if (p2.first == p1.first)
//         throw std::invalid_argument("Timestamps must be different");

//     // Calculate the rate of change per unit of time.
//     float slope = (p2.second - p1.second) /
//                   (p2.first - p1.first);

//     // Project the trend to the requested future time.
//     return p2.second + slope * (interpolateTime - p2.first);
// }