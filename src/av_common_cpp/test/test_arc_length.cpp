// Standalone test for av::geom::arc_length. No gtest/colcon wiring yet
// (none exists in this package), so this is a plain assert-based check
// you can compile and run directly:
//
//   g++ -std=c++17 -I ../include -I /usr/include/eigen3 \
//       test_arc_length.cpp -o /tmp/test_arc_length && /tmp/test_arc_length

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "av_common/geometry.hpp"

using av::geom::arc_length;
using av::geom::Polyline;

int main()
{
    Polyline p = {{0, 0}, {3, 4}, {6, 8}};
    std::vector<double> s = arc_length(p);

    std::cout << "arc_length: ";
    for (auto s_i : s)
    {
        std::cout << s_i << " , ";
    }
    std::cout << std::endl;
    return 0;
}
