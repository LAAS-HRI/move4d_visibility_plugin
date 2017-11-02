#ifndef MOVE4D_VISIBILITYPLANNER_HPP
#define MOVE4D_VISIBILITYPLANNER_HPP

#include <move4d/API/moduleBase.hpp>
#include <move4d/Logging/Logger.h>

#include <move4d/API/forward_declarations.hpp>
#include <Eigen/Core>

namespace move4d {

class VisibilityCell;
class VisibilityPlanner : public move4d::ModuleBase
{
    MOVE3D_STATIC_LOGGER;
protected:
    VisibilityPlanner();
public:
    typedef std::tuple<bool,bool,double,double,double> CellCost_t;//is in col, cost , vis, dist

    static CellCost_t computeCost(move4d::Robot *target_rob, const Eigen::Vector2d &target, move4d::Robot *r, VisibilityCell *cell, int sum_dist);

    static std::string name(){return "VisibilityPlanner";}

    virtual void run() override;

protected:
    static double getCostSigmoid(double x, double min, double max, double low, double up);
private:
    static VisibilityPlanner *__instance;
};

} // namespace move4d

#endif // MOVE4D_VISIBILITYPLANNER_HPP
