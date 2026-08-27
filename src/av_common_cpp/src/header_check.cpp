// Compile smoke: pulls every av_common header through the compiler.
#include "av_common/debug_tap.hpp"
#include "av_common/geometry.hpp"
#include "av_common/json_io.hpp"
#include "av_common/latch.hpp"
#include "av_common/map_model.hpp"
#include "av_common/projection.hpp"

int main()
{
    av::geom::Polyline p{{0, 0}, {1, 0}, {2, 0}, {3, 1}};
    auto r = av::geom::resample_uniform(p, 0.5);
    auto pr = av::geom::project_point(r, {1.5, 0.3});
    av::HysteresisLatch<int> latch(3, 25.0, 0);
    latch.update(1, 10.0);
    auto j = av::json{{"ok", true}, {"s", pr.s}};
    return j["ok"].get<bool>() && !r.empty() ? 0 : 1;
}