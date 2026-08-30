// Standalone unit test for grid_dedup() -- no ROS, no OpenCV, no colcon.
// grid_dedup() itself only touches <cmath>/<unordered_map>/<vector>, so it
// can be copied out and exercised in complete isolation from the rest of
// lane_node.cpp.
//
// Build & run:
//   g++ -std=c++17 -Wall -Wextra -o /tmp/test_grid_dedup test_grid_dedup.cpp
//   /tmp/test_grid_dedup
//
// Exits 0 and prints "ALL PASSED" if every check succeeds; exits 1 and
// prints which check failed otherwise.

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------- the function under test
// Copied verbatim from lane_node.cpp's anonymous namespace / the Day 6 guide.
// Keep this in sync by hand if the real implementation changes.
struct ChainPt
{
    double x, y;
    bool yellow;
};
using Chain = std::vector<ChainPt>;

Chain grid_dedup(const Chain &pts, double cell_m)
{
    struct Cell
    {
        double sx = 0, sy = 0;
        int n = 0;
        bool yellow = false;
    };
    std::unordered_map<int64_t, Cell> grid;
    auto key = [&](double x, double y) -> int64_t
    {
        int32_t gx = static_cast<int32_t>(std::floor(x / cell_m));
        int32_t gy = static_cast<int32_t>(std::floor(y / cell_m));
        return (static_cast<int64_t>(gx) << 32) ^ static_cast<uint32_t>(gy);
    };
    for (const auto &p : pts)
    {
        auto &c = grid[key(p.x, p.y)];
        c.sx += p.x;
        c.sy += p.y;
        ++c.n;
        c.yellow = c.yellow || p.yellow;
    }
    Chain out;
    for (const auto &[k, c] : grid)
        out.push_back({c.sx / c.n, c.sy / c.n, c.yellow});
    return out;
}

// ---------------------------------------------------------------- tiny test harness
static int g_failures = 0;

void check(bool cond, const char *what)
{
    if (cond)
    {
        std::printf("[PASS] %s\n", what);
    }
    else
    {
        std::printf("[FAIL] %s\n", what);
        ++g_failures;
    }
}

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

// finds the single output point whose cell should contain (near) x,y; asserts exactly one match
const ChainPt *find_near(const Chain &out, double x, double y, double tol = 0.5)
{
    const ChainPt *hit = nullptr;
    for (const auto &p : out)
    {
        if (std::abs(p.x - x) < tol && std::abs(p.y - y) < tol)
        {
            if (hit != nullptr)
                return nullptr; // ambiguous: two candidates -- treat as not-found
            hit = &p;
        }
    }
    return hit;
}

// ---------------------------------------------------------------- test cases
void test_empty_input()
{
    Chain out = grid_dedup({}, 0.5);
    check(out.empty(), "empty input -> empty output");
}

void test_single_point_passes_through()
{
    Chain out = grid_dedup({{1.23, -4.56, true}}, 0.5);
    check(out.size() == 1, "single point -> single output cell");
    if (out.size() == 1)
    {
        check(near(out[0].x, 1.23) && near(out[0].y, -4.56), "single point: coordinates unchanged");
        check(out[0].yellow == true, "single point: yellow flag preserved");
    }
}

void test_two_points_same_cell_merge()
{
    // cell_m = 0.5, both points floor to gx=2 (1.0 <= x < 1.5), gy=-1 (-0.5 <= y < 0.0)
    Chain out = grid_dedup({{1.3, -0.2, false}, {1.4, -0.1, false}}, 0.5);
    check(out.size() == 1, "two points in same cell -> merged into one");
    if (out.size() == 1)
    {
        // averaged centroid: x=(1.3+1.4)/2=1.35, y=(-0.2-0.1)/2=-0.15
        check(near(out[0].x, 1.35) && near(out[0].y, -0.15), "same-cell merge: centroid is the mean, not endpoint");
    }
}

void test_two_points_different_cells_stay_separate()
{
    Chain out = grid_dedup({{1.3, -0.2, false}, {1.6, -0.1, false}}, 0.5); // second point: gx=3, different cell
    check(out.size() == 2, "two points in different cells -> two outputs, not merged");
}

void test_yellow_or_semantics()
{
    // same cell, one white one yellow -> cell must come out yellow
    Chain out = grid_dedup({{1.3, -0.2, false}, {1.4, -0.1, true}}, 0.5);
    check(out.size() == 1, "yellow-OR test: single merged cell");
    if (out.size() == 1)
        check(out[0].yellow == true, "yellow-OR test: any yellow evidence in the cell -> cell is yellow");
}

void test_all_white_cell_stays_white()
{
    Chain out = grid_dedup({{1.3, -0.2, false}, {1.4, -0.1, false}}, 0.5);
    check(out.size() == 1 && out[0].yellow == false, "all-white cell stays white");
}

void test_negative_coordinates_use_floor_not_truncation()
{
    // cell_m = 1.0. x=-0.3 and x=-0.9 both floor to gx=-1 (NOT 0, which naive
    // (int) truncation toward zero would give). This is the exact bug
    // static_cast<int32_t>(std::floor(...)) is written to avoid.
    Chain out = grid_dedup({{-0.3, -0.2, false}, {-0.9, -0.4, false}}, 1.0);
    check(out.size() == 1, "negative coords near origin: floor buckets both into cell (-1,-1), not split by truncation");

    // sanity: a point that truncation would ALSO put in the same cell as a
    // naive check, but is genuinely in a different cell under floor semantics.
    Chain out2 = grid_dedup({{-0.3, -0.2, false}, {0.3, 0.2, false}}, 1.0);
    check(out2.size() == 2, "straddling zero: (-0.3,-0.2) is cell (-1,-1), (0.3,0.2) is cell (0,0) -- must stay separate");
}

void test_point_count_conserved_across_cells()
{
    // every input point must land in exactly one output cell's running sum;
    // check via a weighted reconstruction: sum of (centroid * n) form isn't
    // exposed, so instead verify total INPUT count equals sum of cell sizes
    // indirectly by checking total number of distinct cells is <= input size
    // and that every input point is within cell_m of some output centroid.
    Chain pts = {{0.1, 0.1, false}, {0.2, 0.2, false}, {5.0, 5.0, true}, {5.1, 5.1, false}, {9.9, -9.9, true}};
    Chain out = grid_dedup(pts, 0.5);
    check(out.size() <= pts.size(), "output cell count never exceeds input point count");
    bool all_covered = true;
    for (const auto &p : pts)
        if (find_near(out, p.x, p.y, 0.5) == nullptr)
            all_covered = false;
    check(all_covered, "every input point has a nearby output centroid (no points silently dropped)");
}

void test_large_random_grid_no_crash_and_bounded_output()
{
    Chain pts;
    // deterministic pseudo-random spread over a 20m x 6m patch, dense enough
    // that many points collide into shared cells at cell_m=0.5
    uint32_t seed = 12345;
    auto next = [&]() { seed = seed * 1664525u + 1013904223u; return seed; };
    for (int i = 0; i < 5000; ++i)
    {
        double x = static_cast<double>(next() % 2000) / 100.0;        // [0, 20)
        double y = static_cast<double>(next() % 600) / 100.0 - 3.0;    // [-3, 3)
        pts.push_back({x, y, (next() % 5) == 0});
    }
    Chain out = grid_dedup(pts, 0.5);
    // 20m x 6m at 0.5m cells is at most 40*12 = 480 distinct cells
    check(out.size() <= 480, "5000-point dense cloud collapses to at most the grid's cell count");
    check(out.size() > 0, "large input still produces output");
    std::printf("       (5000 points -> %zu deduplicated cells)\n", out.size());
}

int main()
{
    test_empty_input();
    test_single_point_passes_through();
    test_two_points_same_cell_merge();
    test_two_points_different_cells_stay_separate();
    test_yellow_or_semantics();
    test_all_white_cell_stays_white();
    test_negative_coordinates_use_floor_not_truncation();
    test_point_count_conserved_across_cells();
    test_large_random_grid_no_crash_and_bounded_output();

    if (g_failures == 0)
    {
        std::printf("\nALL PASSED\n");
        return 0;
    }
    std::printf("\n%d CHECK(S) FAILED\n", g_failures);
    return 1;
}
