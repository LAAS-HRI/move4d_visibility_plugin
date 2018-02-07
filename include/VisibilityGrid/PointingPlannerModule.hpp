#ifndef POINTINGPLANNERMODULE_HPP
#define POINTINGPLANNERMODULE_HPP

#include <move4d/Logging/Logger.h>
#include <move4d/API/moduleBase.hpp>

namespace move4d {
struct PlanningData;

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

#endif // POINTINGPLANNERMODULE_HPP
