#include "VisibilityGrid/VisibilityGridCreator.hpp"
#include "VisibilityGrid/VisibilityGrid.hpp"

#include <move4d-gui/common/tools/VisibilityEngine.hpp>

#include <move4d/API/project.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>
#include <move4d-gui/common/OgreBase.hpp>
#include <move4d-gui/common/Robot.hpp>

#include <move4d/API/Parameter.hpp>

#include <jsoncpp/json/json.h>

#include <iostream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <move4d/CTPL/ctpl.h>

INIT_MOVE3D_STATIC_LOGGER(move4d::VisibilityGridCreator,"move4d.visibilitygrid.creator");
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

void _countPixels(int id,MoveOgre::VisibilityEngine::PixelImage::ptr pixels, double nb_pixel_max, VisibilityGrid3d *grid, size_t cell_index, VisibilityGrid3d::SpaceCoord cell_center, std::mutex *grid_mutex){
    std::vector<uint> counts;

    //Ogre::Image image;
    //image = image.loadDynamicImage(static_cast<unsigned char*>(pixels->pixels.data), pixels->pixels.getWidth(),pixels->pixels.getHeight(),pixels->pixels.format);
    //image.save("/tmp/moveogreperspective"+std::to_string(cell_index)+".png");

    MoveOgre::VisibilityEngine::computeVisibilityFromImage(pixels,counts);

    grid_mutex->lock();

    VisibilityGrid3d::reference vis=grid->getCell(cell_index);
    for(uint r=0;r<counts.size();++r){
        Robot *rob = global_Project->getActiveScene()->getRobot(r);
        vis[rob]=counts[r] / nb_pixel_max;
    }
    grid_mutex->unlock();

    if(cell_index % size_t(std::ceil(double(grid->getNumberOfCells())/100)) == 0){
        std::cout <<"."; std::cout.flush();
    }
}

void VisibilityGridCreator::computeVisibilities(uint nb_threads)
{
    ctpl::thread_pool pool(nb_threads);
    //std::vector<std::future<void>> results(4);
    std::mutex grid_mutex;
    int pixPer90deg=API::Parameter::param<int>("VisibilityGrid/Creator/PixelPer90Deg",256);
    MoveOgre::VisibilityEngine visibEngine(Ogre::Degree(360.),Ogre::Degree(90.),static_cast<unsigned int>(pixPer90deg));
    M3D_INFO("The visibility will be computed using a resolution for the rendering of "<<visibEngine.getPixelPerDegree()*90<<" pixels per 90 degrees of field of view, that is a total of "<<visibEngine.getResolution()<<" pixels per point of view (=grid cell), or "<<visibEngine.getResolution()*_grid->getNumberOfCells()<<" pixels for all the grid");
    double nb_pixel_max = visibEngine.getPixelPerDegree() * 10;
    nb_pixel_max *= nb_pixel_max;
    visibEngine.prepareScene();
    std::cout<<"progress: (1 dot '.' = 1%; *=rendering complete)                                                    >100%"<<std::endl;
    for(unsigned int i=0;i<_grid->getNumberOfCells();++i){
        VisibilityGrid3d::reference vis=_grid->getCell(i);
        VisibilityGrid3d::SpaceCoord center = _grid->getCellCenter(_grid->getCellCoord(i));
        Eigen::Vector3d p(center[0],center[1],center[2]);
        Eigen::Affine3d transform{Eigen::Translation3d(p)};

        //visibEngine.computeVisibilityFrom(transform);
        auto pixels=visibEngine.renderSceneFrom(transform);

        // for(uint r=0;r<global_Project->getActiveScene()->getNumberOfRobots();++r){
        //     Robot *rob = global_Project->getActiveScene()->getRobot(r);
        //     vis[rob]=visibEngine.getVisibilityOf(rob) / nb_pixel_max;
        // }

        pool.push(_countPixels, pixels,nb_pixel_max,_grid,i,center,&grid_mutex);
        //f.get();
        //_countPixels(0,pixels,nb_pixel_max,_grid,i,&grid_mutex);
        //for(auto p : visibEngine.getVisibilityCounts()){
        //    vis[p.first] = p.second /  nb_pixel_max;
        //}
    }
    std::cout<<"*";std::cout.flush();
    pool.stop(/*wait = */ true); //wait for execution to be done.
    std::cout<<std::endl;
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
    M3D_DEBUG("VisibilityModule::run()");
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
    double minz=API::Parameter::param<double>("VisibilityGrid/Creator/MinZ",0.8);
    double maxz=API::Parameter::param<double>("VisibilityGrid/Creator/MaxZ",2.);
    envSize[4]=minz;
    envSize[5]=maxz;
    float pacexy=API::Parameter::param<double>("VisibilityGrid/Creator/StepXY",0.8);
    float pacez =API::Parameter::param<double>("VisibilityGrid/Creator/StepZ",1.2);
    uint nb_threads =API::Parameter::param<int>("VisibilityGrid/Creator/ThreadNumber",4);
    _grid = new VisibilityGrid3d({{pacexy,pacexy,pacez}},adapt_cellsize,envSize);
    M3D_INFO("Grid dimensions:"<<
            "\n\tGrid origin (x,y)=\t("<<envSize[0]<<","<<envSize[2]<<")"<<
            "\n\tGrid corner (x,y)=\t("<<envSize[1]<<","<<envSize[3]<<")"<<
            "\n\tGrid heigh span(z)=\t"<<envSize[4]<<" -> "<<envSize[5]<<
            "\n\tGrid Step size (x,y,z)=\t("<<pacexy<<","<<pacexy<<","<<pacez<<")"
            );
    M3D_INFO("Using "<<nb_threads<<" threads");
    M3D_INFO("VisibilityGrid3d nb cell="<<_grid->getNumberOfCells());
    computeVisibilities(nb_threads);
    writeGridsToFile("./data/visibility_grid_bin");
    M3D_INFO("Visibility Grids written to ./data/visibility_grid_bin");

    for(unsigned int i=0;i<global_Project->getActiveScene()->getNumberOfRobots();++i){
        Robot *r= global_Project->getActiveScene()->getRobot(i);
        Graphic::DrawablePool::sAddGrid3Dfloat(std::shared_ptr<Graphic::Grid3Dfloat>(new Graphic::Grid3Dfloat{"Vis"+r->getName(),_grid->computeGridOf(r)}));
    }
}
}
