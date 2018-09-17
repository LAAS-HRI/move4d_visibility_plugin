#ifndef VISIBILITYMODULE_HPP
#define VISIBILITYMODULE_HPP

#include <move4d/API/moduleBase.hpp>
#include <move4d/Logging/Logger.h>

namespace move4d{
class VisibilityGrid3d;
class VisibilityGridCreator : public ModuleBase
{
    MOVE3D_STATIC_LOGGER;
protected:
    VisibilityGridCreator();
public:
    virtual ~VisibilityGridCreator();

    static std::string name(){return "VisibilityGridCreator";}

    virtual void run() override;

protected:
    void computeVisibilities(uint nb_threads);
    void writeGridsToFile(const std::string &name);

private:
    VisibilityGrid3d *_grid;
    static VisibilityGridCreator *__instance;
};
}

#endif // VISIBILITYMODULE_HPP
