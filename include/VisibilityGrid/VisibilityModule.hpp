#ifndef VISIBILITYMODULE_HPP
#define VISIBILITYMODULE_HPP

#include <move4d/API/moduleBase.hpp>

namespace move4d{
class VisibilityGrid;
class VisibilityModule : public ModuleBase
{
protected:
    VisibilityModule();
public:
    virtual ~VisibilityModule();

    static std::string name(){return "VisibilityModule";}

    virtual void run() override;
private:
    VisibilityGrid *_grid;
    static VisibilityModule *__instance;
};
}

#endif // VISIBILITYMODULE_HPP
