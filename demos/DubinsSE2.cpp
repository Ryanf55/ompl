// This utility finds the dubins path between N>=2 points.
// Optionally:
// * Graph the results
// * Show all 6 classifications
// * Show the optimal
// * Show the distances
// * Fun extension traveling saleseman dubins


// #include <boost/program_options/parsers.hpp>
// #include <boost/program_options/variables_map.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <iostream>

static constexpr auto DEFAULT_RHO = 2.0;
static constexpr auto DEFAULT_STATE1_X = 0.0;
static constexpr auto DEFAULT_STATE1_Y = 0.0;
static constexpr auto DEFAULT_STATE1_YAW = 0.0;

static constexpr auto DEFAULT_STATE2_X = 5.0;
static constexpr auto DEFAULT_STATE2_Y = 1.0;
static constexpr auto DEFAULT_STATE2_YAW = boost::math::constants::half_pi<double>();

int main() {

    using namespace ompl;

    auto state_space = base::DubinsStateSpace(DEFAULT_RHO);

    base::StateSpacePtr space(new base::SE2StateSpace());
    base::ScopedState<base::SE2StateSpace> se2_state1(space);
    se2_state1->setXY(DEFAULT_STATE1_X, DEFAULT_STATE1_Y);
    se2_state1->setYaw(DEFAULT_STATE1_YAW);

    base::ScopedState<base::SE2StateSpace> se2_state2(space);
    se2_state2->setXY(DEFAULT_STATE2_X, DEFAULT_STATE1_Y);
    se2_state2->setYaw(DEFAULT_STATE2_YAW);
    auto path = state_space.dubins(se2_state1.get(), se2_state2.get());

    auto const PathTypeToStr = [](const base::DubinsStateSpace::DubinsPathSegmentType type) -> std::string {
        switch(type) {
            case ompl::base::DubinsStateSpace::DUBINS_LEFT:
                return "LEFT";
            case ompl::base::DubinsStateSpace::DUBINS_STRAIGHT:
                return "STRAIGHT";
            case ompl::base::DubinsStateSpace::DUBINS_RIGHT:
                return "RIGHT";
            default:
                assert(false);
                return "INVALID";
        }
    };

    std::cout << "Total Length: " <<  path.length()<< "\n";
    std::cout << "Path Segment Lengths: [" <<  path.length_[0]  << ", " << path.length_[1] << ", " << path.length_[2] << "]\n";
    std::cout << "Path Types: [" <<  PathTypeToStr(path.type_[0])  << ", " << PathTypeToStr(path.type_[1]) << ", " << PathTypeToStr(path.type_[2]) << "]\n";

    return 0;
}
