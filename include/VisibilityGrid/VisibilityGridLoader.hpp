#ifndef MOVE4D_VISIBILITYGRIDLOADER_HPP
#define MOVE4D_VISIBILITYGRIDLOADER_HPP

#include <move4d/API/moduleBase.hpp>
#include <move4d/Logging/Logger.h>

namespace move4d {

class VisibilityGrid3d;

class VisibilityGridLoader : public move4d::ModuleBase
{
    MOVE3D_STATIC_LOGGER;
protected:
    VisibilityGridLoader();
public:
    virtual ~VisibilityGridLoader();
    static std::string name(){return "VisibilityGridLoader";}

    virtual void initialize() override;

    VisibilityGrid3d *grid() const;

private:
    VisibilityGrid3d *_grid=nullptr;
    static VisibilityGridLoader *__instance;
};

} // namespace move4d

#endif // MOVE4D_VISIBILITYGRIDLOADER_HPP
