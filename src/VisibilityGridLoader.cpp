#include "VisibilityGrid/VisibilityGridLoader.hpp"
#include <move4d/API/project.hpp>
#include <move4d/API/scene.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>

#include "VisibilityGrid/VisibilityGrid.hpp"
#include <fstream>
#include <jsoncpp/json/json.h>

#include <move4d/database/DatabaseReader.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

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

    if(!(loadBinary() || loadText())){
        M3D_ERROR("could not read file visibility_grid_bin.txt nor visibility_grid_bin");
        return;
    }

    if(_grid->getNumberOfCells()){
        for(auto it : _grid->getCell(0)){
            Robot *r=it.first;
            M3D_DEBUG("visibility of "<<r->getName());
            Graphic::DrawablePool::sAddGrid3Dfloat(std::shared_ptr<Graphic::Grid3Dfloat>(new Graphic::Grid3Dfloat{"Vis"+r->getName(),_grid->computeGridOf(r)}));
        }
    }
}

bool VisibilityGridLoader::loadBinary(){
    ifstream input;
    std::string path=DatabaseReader::getInstance()->findFile("visibility_grid_bin");
    if(path.empty()){return false;}
    M3D_INFO("loading visibility grid from "<<path);
    input.open(path,ios::in | ios::binary);
    boost::archive::binary_iarchive ia(input);
    ia >> *_grid;
    input.close();
    return true;
}

bool VisibilityGridLoader::loadText()
{
    ifstream input;
    std::string path=DatabaseReader::getInstance()->findFile("visibility_grid_bin.txt");
    if(path.empty()){return false;}
    M3D_INFO("loading visibility grid from "<<path);
    input.open(path,ios::in);
    boost::archive::text_iarchive ia(input);
    ia >> *_grid;
    input.close();
    return true;
}

VisibilityGrid3d *VisibilityGridLoader::grid() const
{
    return _grid;
}

} // namespace move4d
