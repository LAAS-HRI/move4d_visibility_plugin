#include "VisibilityGrid/VisibilityGridCreator.hpp"
#include "VisibilityGrid/VisibilityGrid.hpp"

#include <move4d-gui/common/tools/VisibilityEngine.hpp>

#include <move4d/API/project.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>
#include <move4d-gui/common/OgreBase.hpp>
#include <move4d-gui/common/Robot.hpp>

#include <jsoncpp/json/json.h>

#include <iostream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

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
    MoveOgre::VisibilityEngine visibEngine(Ogre::Degree(360.),Ogre::Degree(360.),256u); //256 pixels per 90deg
    double nb_pixel_max = visibEngine.getPixelPerDegree() * 10;
    nb_pixel_max *= nb_pixel_max;
    visibEngine.prepareScene();
    for(unsigned int i=0;i<_grid->getNumberOfCells();++i){
        VisibilityGrid3d::reference vis=_grid->getCell(i);
        VisibilityGrid3d::SpaceCoord center = _grid->getCellCenter(_grid->getCellCoord(i));
        Eigen::Vector3d p(center[0],center[1],center[2]);
        Eigen::Affine3d transform{Eigen::Translation3d(p)};

        visibEngine.computeVisibilityFrom(transform);

        for(uint r=0;r<global_Project->getActiveScene()->getNumberOfRobots();++r){
            Robot *rob = global_Project->getActiveScene()->getRobot(r);
            vis[rob]=visibEngine.getVisibilityOf(rob) / nb_pixel_max;
        }
        //for(auto p : visibEngine.getVisibilityCounts()){
        //    vis[p.first] = p.second /  nb_pixel_max;
        //}
    }
    visibEngine.finish();

    //MoveOgre::OgreBase *base=MoveOgre::OgreBase::getInstance();
    //for(unsigned int i=0;i<_grid->getNumberOfCells();++i){
    //    VisibilityCell *cell=dynamic_cast<VisibilityCell*>(_grid->getCell(i));
    //    Eigen::Vector2d corner = cell->getCorner();
    //    std::map<Robot*,double> vis;

    //    const uint nb_step=360*2; //.5deg step
    //    const uint nb_step_max = 20; //cost = 1 for x deg in the fov
    //    const double step=M_PI*2/nb_step;
    //    Ogre::Vector3 p;
    //    p[0]=corner[0] + _grid->getCellSize()[0]/2;
    //    p[1]=corner[1] + _grid->getCellSize()[1]/2;
    //    p[2]=1.5;

    //    for(double theta=0.;theta<M_PI*2;theta+=step){
    //        Ogre::Vector3 d(cos(theta),sin(theta),0);
    //        Robot *r=base->raycastRobot(p,d);
    //        if(r && r->getName().find("SIGN")!=std::string::npos)
    //            vis[r]+=1./nb_step_max;
    //    }
    //    cell->setVisibilities(std::move(vis));
    //}
}

void VisibilityGridCreator::writeGridsToFile(const std::string &name){
    //boost serialization
    {
    ofstream of;
    of.open(name,ios::out | ios::binary);
    boost::archive::binary_oarchive oa(of);
    oa << *_grid;
    of.close();
    }

    {
    ofstream of;
    of.open(name+".txt",ios::out);
    boost::archive::text_oarchive oa(of);
    oa << *_grid;
    of.close();
    }
}

void VisibilityGridCreator::run()
{
    cout<<"VisibilityModule::run()"<<endl;
    //for(int i=0;i<global_Project->getActiveScene()->getNumberOfRobots();++i){
    //    MoveOgre::Robot *r=dynamic_cast<MoveOgre::Robot*>(global_Project->getActiveScene()->getRobot(i));
    //    if(r->getName().find("SIGN") != std::string::npos){
    //        Ogre::SceneNode *n=r->getRobotNode();
    //        createOgreEntities(MoveOgre::OgreBase::getInstance()->getSceneMgr(),r,n);
    //    }
    //}

    //MoveOgre::OgreBase::getInstance()->renderOneFrame();

    bool adapt_cellsize=true;
    std::vector<double> envSize=global_Project->getActiveScene()->getBounds();
    envSize[4]=0.8;
    envSize[5]=2.;
    _grid = new VisibilityGrid3d({{0.3,0.3,0.3}},adapt_cellsize,envSize);
    std::cout<<"VisibilityGrid3d nb cell="<<_grid->getNumberOfCells()<<std::endl;
    computeVisibilities();
    writeGridsToFile("./data/visibility_grid_bin");

    for(unsigned int i=0;i<global_Project->getActiveScene()->getNumberOfRobots();++i){
        Robot *r= global_Project->getActiveScene()->getRobot(i);
        Graphic::DrawablePool::sAddGrid3Dfloat(std::shared_ptr<Graphic::Grid3Dfloat>(new Graphic::Grid3Dfloat{"Vis"+r->getName(),_grid->computeGridOf(r)}));
    }
}
}
