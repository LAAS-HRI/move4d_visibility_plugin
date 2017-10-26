#ifndef VISIBILITYMODULE_HPP
#define VISIBILITYMODULE_HPP

#include <move4d/API/moduleBase.hpp>

namespace move4d{
class VisibilityGrid;
class VisibilityGridCreator : public ModuleBase
{
protected:
    VisibilityGridCreator();
public:
    virtual ~VisibilityGridCreator();

    static std::string name(){return "VisibilityGridCreator";}

    virtual void run() override;

protected:
    void computeVisibilities();
    void writeGridsToFile(const std::string &name);

private:
    VisibilityGrid *_grid;
    static VisibilityGridCreator *__instance;
};
}

#endif // VISIBILITYMODULE_HPP
