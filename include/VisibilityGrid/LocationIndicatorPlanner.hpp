#ifndef LOCATIONINDICATORPLANNER_HPP
#define LOCATIONINDICATORPLANNER_HPP

#include <Eigen/Geometry>
#include <move4d/API/forward_declarations.hpp>
#include <vector>
#include <string>
#include <move4d/Logging/Logger.h>
#include <move4d/API/moduleBase.hpp>


namespace move4d {
namespace mho{class SimpleGraph;}
namespace LocationIndicator{

struct TargetInformation
{
    Robot *robot;
    Robot *human;
    Eigen::Vector2d getPositionOfPhysicalTarget() const {return positionPhysicalTarget;}
    std::vector<std::pair<Robot*,float> > references;
    bool known;
    bool see_first;
    std::array<float,2> humanPosInit();
    std::array<float,2> robotPosInit();
    Eigen::Vector2d humanVPosInit();
    Eigen::Vector2d robotVPosInit();

    Eigen::Vector2d positionPhysicalTarget;
};
class LocationIndicatorPlanner : public ModuleBase
{
    static LocationIndicatorPlanner *__instance;
public:
    LocationIndicatorPlanner();

    void findHumanOnly(TargetInformation &target);

    Robot* isTargetVisibleFromInit(TargetInformation &target);
    void pointFromInit(TargetInformation &target, Robot *obj);

    void run(TargetInformation &target);
    void runCompare(TargetInformation &target);
    void run();

    void loadRouteGraph();

private:
    mho::SimpleGraph *mRouteGraph=nullptr;
};

}
} // namespace move4d

#endif // LOCATIONINDICATORPLANNER_HPP
