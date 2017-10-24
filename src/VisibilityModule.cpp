#include "VisibilityGrid/VisibilityModule.hpp"
#include "VisibilityGrid/VisibilityGrid.hpp"

#include <move4d/API/project.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>
#include <move4d-gui/common/OgreBase.hpp>

#include <iostream>

using namespace std;

move4d::VisibilityModule* move4d::VisibilityModule::__instance = new move4d::VisibilityModule();
namespace move4d{
VisibilityModule::VisibilityModule():
    ModuleBase(),
    _grid(0)
{
    _name=name();
    addToRegister();
}

VisibilityModule::~VisibilityModule()
{
    if(_grid) delete _grid;
}

void VisibilityModule::run()
{
    cout<<"VisibilityModule::run()"<<endl;
    _grid = new VisibilityGrid(0.3,global_Project->getActiveScene()->getBounds());
    _grid->createAllCells();

    for(unsigned int i=0;i<global_Project->getActiveScene()->getNumberOfRobots();++i){
        Robot *r= global_Project->getActiveScene()->getRobot(i);
        Graphic::DrawablePool::sAdd2dGrid(std::shared_ptr<Graphic::Drawable2dGrid>(new Graphic::Drawable2dGrid("Vis"+r->getName(),_grid->matrix(r),_grid->getCellSize(),_grid->getOriginCorner())));
    }
}
}
