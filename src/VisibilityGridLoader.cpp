#include "VisibilityGrid/VisibilityGridLoader.hpp"
#include <move4d/API/project.hpp>
#include <move4d/API/scene.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>

#include "VisibilityGrid/VisibilityGrid.hpp"
#include <fstream>
#include <jsoncpp/json/json.h>

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

void VisibilityGridLoader::run()
{
    M3D_TRACE("VisibilityGridLoader::run");
    Json::Reader reader;
    Json::Value root;
    ifstream file;
    file.open("./visibility.json");
    bool ok = reader.parse(file,root);
    file.close();
    if(ok){
        Eigen::Vector2i dim(root["dim"][0].asInt(),root["dim"][1].asInt());
        Eigen::Vector2d cellsize(root["cellsize"][0].asDouble(),root["cellsize"][1].asDouble());
        std::vector<double> envsize=global_Project->getActiveScene()->getBounds();
        envsize[1]=envsize[0]+cellsize[0]*dim[0];
        envsize[3]=envsize[2]+cellsize[1]*dim[1];
        VisibilityGrid *grid = new VisibilityGrid(dim,envsize);
        M3D_ASSERT(grid->getCellSize() == cellsize)
        grid->createAllCells();
        M3D_TRACE("created grid of size "<<grid->getSize().transpose()<<" origin: "<<grid->getOriginCorner().transpose());
        for(Json::Value::iterator it=root["matrices"].begin();it!=root["matrices"].end();++it){
            Robot *r=global_Project->getActiveScene()->getRobotByName(it.memberName());
            M3D_TRACE("loading grid for robot "<<r->getName());
            for(unsigned int i=0;i<grid->getNumberOfCells();++i){
                double v=(*it)[i].asDouble();
                dynamic_cast<VisibilityCell*>(grid->getCell(i))->setVisibility(r,(*it)[i].asDouble());
            }
            Graphic::DrawablePool::sAdd2dGrid(std::shared_ptr<Graphic::Drawable2dGrid>(new Graphic::Drawable2dGrid("Vis"+r->getName(),grid->matrix(r),grid->getCellSize(),grid->getOriginCorner())));
            M3D_TRACE("mat "<<r->getName()<<"\n"<<grid->matrix(r));
        }
    }else{
        M3D_ERROR("failed to parse file "<<"./visibility.json");
    }



}

} // namespace move4d
