#include "VisibilityGrid/VisibilityGridLoader.hpp"
#include <move4d/API/project.hpp>
#include <move4d/API/scene.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>

#include "VisibilityGrid/VisibilityGrid.hpp"
#include <fstream>
#include <jsoncpp/json/json.h>

#include <boost/archive/binary_iarchive.hpp>

move4d::VisibilityGridLoader *move4d::VisibilityGridLoader::__instance = new move4d::VisibilityGridLoader();

INIT_MOVE3D_STATIC_LOGGER(move4d::VisibilityGridLoader,"move4d.visibilitygrid.loader");
using namespace std;
namespace move4d {

VisibilityGridLoader::VisibilityGridLoader():
    ModuleBase()
{
    _name = name();
    addToRegister();
}

VisibilityGridLoader::~VisibilityGridLoader()
{

}

void VisibilityGridLoader::initialize()
{
    if(_grid){return;}
    _grid = new VisibilityGrid3d();
    M3D_TRACE("VisibilityGridLoader::initialize");
    ifstream input;
    input.open("./data/visibility_grid_bin",ios::in | ios::binary);
    boost::archive::binary_iarchive ia(input);
    ia >> *_grid;
    input.close();

    if(_grid->getNumberOfCells()){
        for(auto it : _grid->getCell(0)){
            Robot *r=it.first;
            M3D_DEBUG("visibility of "<<r->getName());
            Graphic::DrawablePool::sAddGrid3Dfloat(std::shared_ptr<Graphic::Grid3Dfloat>(new Graphic::Grid3Dfloat{"Vis"+r->getName(),_grid->computeGridOf(r)}));
        }
    }
}

VisibilityGrid3d *VisibilityGridLoader::grid() const
{
    return _grid;
}

} // namespace move4d
