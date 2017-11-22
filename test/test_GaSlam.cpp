#include <boost/test/unit_test.hpp>

#include <chrono>

#include <ga_slam/GaSlam.hpp>

using namespace std::chrono;

namespace ga_slam
{

    BOOST_AUTO_TEST_CASE(global_map_interface)
    {
        auto now_ns = time_point_cast<nanoseconds>(system_clock::now());
        auto timestamp = now_ns.time_since_epoch().count();

        Map globalMap;
        globalMap.setTimestamp(timestamp);

        GaSlam gaSlam(globalMap);

        std::cout << globalMap.getTimestamp() << std::endl;
    }

}  // namespace camera

