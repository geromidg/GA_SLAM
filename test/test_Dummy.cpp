#include <boost/test/unit_test.hpp>
#include <ga_slam/Dummy.hpp>

using namespace ga_slam;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    ga_slam::DummyClass dummy;
    dummy.welcome();
}
