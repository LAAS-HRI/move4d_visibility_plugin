#ifndef MOVE4D_POINTINGPLANNER_HPP
#define MOVE4D_POINTINGPLANNER_HPP

#include <move4d/API/moduleBase.hpp>
#include <move4d/API/Parameter.hpp>

namespace move4d {

class PointingPlanner : public ModuleBase
{
    MOVE3D_STATIC_LOGGER;
    PointingPlanner();
    static PointingPlanner *__instance;
public:
    static std::string name(){return "PointingPlanner";}

    virtual void initialize() override;
    virtual void run() override;
private:
    struct PlanningData *_data;
};

} // namespace move4d

#endif // MOVE4D_POINTINGPLANNER_HPP
