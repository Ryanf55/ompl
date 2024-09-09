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
#include <vector>
#include "gnuplot-iostream.h"

static constexpr auto DEFAULT_RHO = 2.0;
static constexpr auto DEFAULT_STATE1_X = 0.0;
static constexpr auto DEFAULT_STATE1_Y = 0.0;
static constexpr auto DEFAULT_STATE1_YAW = 0.0;

static constexpr auto DEFAULT_STATE2_X = 5.0;
static constexpr auto DEFAULT_STATE2_Y = 3.0;
static constexpr auto DEFAULT_STATE2_YAW = boost::math::constants::half_pi<double>();

using namespace ompl;


void plot(const std::vector<ompl::base::ScopedState<ompl::base::SE2StateSpace>> states, const double rho) {
}

int main() {


    auto state_space = base::DubinsStateSpace(DEFAULT_RHO);

    base::StateSpacePtr space(new base::SE2StateSpace());
    base::ScopedState<base::SE2StateSpace> se2_state1(space);
    se2_state1->setXY(DEFAULT_STATE1_X, DEFAULT_STATE1_Y);
    se2_state1->setYaw(DEFAULT_STATE1_YAW);

    base::ScopedState<base::SE2StateSpace> se2_state2(space);
    se2_state2->setXY(DEFAULT_STATE2_X, DEFAULT_STATE2_Y);
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

    auto const xmin = std::min(DEFAULT_STATE1_X, DEFAULT_STATE2_X) - DEFAULT_RHO;
    auto const ymin = std::min(DEFAULT_STATE1_Y, DEFAULT_STATE2_Y) - DEFAULT_RHO;
    auto const xmax = std::max(DEFAULT_STATE1_X, DEFAULT_STATE2_X) + DEFAULT_RHO;
    auto const ymax = std::max(DEFAULT_STATE1_Y, DEFAULT_STATE2_Y) + DEFAULT_RHO;
    auto const dx = xmax - xmin;
    auto const dy = ymax - ymin;
    auto const xavg = (xmin + xmax) / 2.0;
    auto const yavg = (ymin + ymax) / 2.0;
    auto const xylim2 = std::max(dx, dy) / 2.0;


    Gnuplot gp;
    std::ostringstream os;
    os << "set xrange [" << xavg - xylim2 << ":" << xavg + xylim2 << "]\n";
    os << "set yrange [" << yavg - xylim2 << ":" << yavg + xylim2 << "]\n";
    os << "set title \"Dubins SE2 Demo\"\n";

    size_t i {1};
    const auto& states = {se2_state1, se2_state2};
    plot(states, DEFAULT_RHO);
    for (auto const& state: states) {
        // Plot using ellipse because the x limits and y limits may not be square.
        // https://stackoverflow.com/questions/56465882/gnuplot-not-showing-correct-scale-on-y-axis-for-a-circle
        os << "set object " << i << \
            " ellipse at " <<
            std::round(state->getX()) << "," <<  std::round(state->getY()) <<
            " size " << DEFAULT_RHO << ", " << DEFAULT_RHO <<
            "\n";
        // https://superuser.com/questions/148701/specifically-marking-a-point-in-gnuplot
        // os << "set object circle at <x>,<y> size <r>";      
        
        i++;
    }

    os << "set grid\n";
    os << "set size square\n";
    os << "plot NaN\n";
    
    std::cout << "\ngnuplot\n" << os.str();
    // set arrow from 2,3 to 5,4 ls 1
    gp << os.str();

    return 0;
}
