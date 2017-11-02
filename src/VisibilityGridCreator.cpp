#include "VisibilityGrid/VisibilityGridCreator.hpp"
#include "VisibilityGrid/VisibilityGrid.hpp"

#include <move4d/API/project.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>
#include <move4d-gui/common/OgreBase.hpp>
#include <move4d-gui/common/Robot.hpp>

#include <jsoncpp/json/json.h>

#include <iostream>

using namespace std;

extern Ogre::SceneNode *createOgreEntities(Ogre::SceneManager *sceneManager, MoveOgre::Robot *robot, Ogre::SceneNode *scNodeRobot);

move4d::VisibilityGridCreator* move4d::VisibilityGridCreator::__instance = new move4d::VisibilityGridCreator();

namespace move4d{
VisibilityGridCreator::VisibilityGridCreator():
    ModuleBase(),
    _grid(0)
{
    _name=name();
    addToRegister();
}

VisibilityGridCreator::~VisibilityGridCreator()
{
    if(_grid) delete _grid;
}

void VisibilityGridCreator::computeVisibilities()
{
    MoveOgre::OgreBase *base=MoveOgre::OgreBase::getInstance();
    for(unsigned int i=0;i<_grid->getNumberOfCells();++i){
        VisibilityCell *cell=dynamic_cast<VisibilityCell*>(_grid->getCell(i));
        Eigen::Vector2d corner = cell->getCorner();
        std::map<Robot*,double> vis;

        const uint nb_step=360*2; //.5deg step
        const uint nb_step_max = 20; //cost = 1 for x deg in the fov
        const double step=M_PI*2/nb_step;
        Ogre::Vector3 p;
        p[0]=corner[0] + _grid->getCellSize()[0]/2;
        p[1]=corner[1] + _grid->getCellSize()[1]/2;
        p[2]=1.5;

        for(double theta=0.;theta<M_PI*2;theta+=step){
            Ogre::Vector3 d(cos(theta),sin(theta),0);
            Robot *r=base->raycastRobot(p,d);
            if(r && r->getName().find("SIGN")!=std::string::npos)
                vis[r]+=1./nb_step_max;
        }
        cell->setVisibilities(std::move(vis));
    }
}

void VisibilityGridCreator::writeGridsToFile(const std::string &name){
    Json::Value root,matrices;
    Json::Value cellsize;
    cellsize.append(_grid->getCellSize()[0]);
    cellsize.append(_grid->getCellSize()[1]);
    root["cellsize"]=cellsize;
    Json::Value dim;
    dim.append(_grid->getSize()[0]);
    dim.append(_grid->getSize()[1]);
    root["dim"]=dim;
    for(int i=0;i<global_Project->getActiveScene()->getNumberOfRobots();++i){
        MoveOgre::Robot *r=dynamic_cast<MoveOgre::Robot*>(global_Project->getActiveScene()->getRobot(i));
        Json::Value array;
        for(unsigned int x=0;x<_grid->getNumberOfCells();++x){
            array.append(Json::Value(dynamic_cast<VisibilityCell*>(_grid->getCell(x))->visibility(r)));
        }
        matrices[r->getName()] = array;
    }

    root["matrices"]=matrices;
    ofstream file;
    file.open(name);
    if(file){
        Json::FastWriter writer;
        writer.enableYAMLCompatibility();
        file<<writer.write(root);
        file.close();
    }
}

void VisibilityGridCreator::run()
{
    cout<<"VisibilityModule::run()"<<endl;
    for(int i=0;i<global_Project->getActiveScene()->getNumberOfRobots();++i){
        MoveOgre::Robot *r=dynamic_cast<MoveOgre::Robot*>(global_Project->getActiveScene()->getRobot(i));
        if(r->getName().find("SIGN") != std::string::npos){
            Ogre::SceneNode *n=r->getRobotNode();
            createOgreEntities(MoveOgre::OgreBase::getInstance()->getSceneMgr(),r,n);
        }
    }

    MoveOgre::OgreBase::getInstance()->renderOneFrame();

    _grid = new VisibilityGrid(0.3,global_Project->getActiveScene()->getBounds());
    _grid->createAllCells();
    computeVisibilities();
    writeGridsToFile("./visibility.json");

    for(unsigned int i=0;i<global_Project->getActiveScene()->getNumberOfRobots();++i){
        Robot *r= global_Project->getActiveScene()->getRobot(i);
        Graphic::DrawablePool::sAdd2dGrid(std::shared_ptr<Graphic::Drawable2dGrid>(new Graphic::Drawable2dGrid("Vis"+r->getName(),_grid->matrix(r),_grid->getCellSize(),_grid->getOriginCorner())));
    }
}
}
