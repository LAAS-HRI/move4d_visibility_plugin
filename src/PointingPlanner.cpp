#include <OGRE/Ogre.h>

#include <move4d-gui/common/tools/VisibilityEngine.hpp>
#include <move4d-gui/common/Robot.hpp>

#include "VisibilityGrid/PointingPlanner.hpp"

#undef  QT_LIBRARY
#include <move4d/API/project.hpp>
//#include <move4d/API/Grids/MultiGrid.hpp>
#include <move4d/API/Grids/NDGrid.hpp>
#include <move4d/API/Grids/NDGridAlgo.hpp>
#include "VisibilityGrid/VisibilityGrid.hpp"
#include "VisibilityGrid/VisibilityGridLoader.hpp"
#include <libmove3d/util/proto/p3d_angle_proto.h>

#include <move4d/API/Device/objectrob.hpp>
#include <move4d/utils/Geometry.h>
#include <move4d/utils/mathfunc.hpp>
#include <move4d/planner/cost_space.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>
#include <move4d/API/Collision/collisionInterface.hpp>
#include <move4d/API/Collision/CylinderCollision.hpp>

#include <boost/bind.hpp>

namespace move4d {



INIT_MOVE3D_STATIC_LOGGER(PlanningData,"move4d.visibilitygrid.pointingplanner.data");

struct CompareCellPtr
{
    bool operator()(PlanningData::Cell *const &a,PlanningData::Cell *const &b){
        return b<a;
    }
};

PlanningData::Cell PlanningData::run(bool read_parameters)
{
    M3D_DEBUG("PointingPlanner::run start");
    ENV.setBool(Env::isRunning,true);
    if(read_parameters) getParameters();
    this->resetFromCurrentInitPos();
    VisibilityGrid3d *vis_grid=dynamic_cast<VisibilityGridLoader*>(ModuleRegister::getInstance()->module(VisibilityGridLoader::name()))->grid();
    CompareCellPtr comp;
    std::vector<Cell*> open_heap;
    VisibilityGrid3d::SpaceCoord vis_cell_size=vis_grid->getCellSize();
    //API::MultiGrid<float,vis_size[0],vis_size[1],vis_size[0],vis_size[1]> grid;
    Grid::SpaceCoord cell_size;
    cell_size[0]=cell_size[2]=vis_cell_size[0];
    cell_size[1]=cell_size[3]=vis_cell_size[1];
    std::vector<double> envSize(8,0.);
    envSize[0]=envSize[4]=global_Project->getActiveScene()->getBounds()[0]; //x min
    envSize[1]=envSize[5]=global_Project->getActiveScene()->getBounds()[1]; //x max
    envSize[2]=envSize[6]=global_Project->getActiveScene()->getBounds()[2]; //y min
    envSize[3]=envSize[7]=global_Project->getActiveScene()->getBounds()[3]; //y max
    bool adjust=false;
    Grid grid(cell_size,adjust,envSize);
    Robot *r,*h;
    r=global_Project->getActiveScene()->getActiveRobot();
    h=global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");

    Grid::ArrayCoord coord;
    Grid::SpaceCoord pos;
    Eigen::Vector2d from;
    for(uint i=0;i<2;++i){
        from[i]=pos[0+i]=r->getCurrentPos()->at(6+i);
        pos[2+i]=h->getCurrentPos()->at(6+i);
    }
    coord=grid.getCellCoord(pos);
    Cell *start=createCell(coord,grid.getCellCenter(coord));
    start->open=true;
    grid.getCell(coord)=start;
    open_heap.push_back(start);
    std::push_heap(open_heap.begin(),open_heap.end(),comp);
    Cell *best=start;
    uint count(0);
    uint iter_of_best{0};
    for(count=0;count<160000 && open_heap.size();++count){
        std::pop_heap(open_heap.begin(),open_heap.end(),comp);
        Cell *current= open_heap.back();
        coord = current->coord;
        open_heap.pop_back();
        //std::cout<<"Current "<<current->cost<<std::endl;
        for (unsigned int i=0;i<grid.neighboursNumber();++i){
            Grid::ArrayCoord neigh=grid.getNeighbour(coord,i);
            Cell *c;
            try{
                bool compute_cost=false;
                c=grid[neigh];
                if(!c){
                    c=new Cell(neigh,grid.getCellCenter(neigh));
                    c->col=0.;
                    grid[neigh]=c;
                    c->cost.constraint(MyConstraints::COL)=std::numeric_limits<float>::infinity();
                    compute_cost=true;
                }
                if(isTooFar(*c,*start)){
                    c->open=true; //do not enter in the "if" bellow, hence ignores its neighbours
                }else if(compute_cost){
                    computeCost(c);
                }
                if(c->col>best->col){
                    //skip also if in collision (and we were not)
                    c->open=true;
                }
                if(!c->open){
                    open_heap.push_back(c);
                    balls->balls_values.push_back(std::make_pair(c->cost.toDouble(),std::vector<Eigen::Vector2d>{c->getPos(0),c->getPos(1)}));
                    std::push_heap(open_heap.begin(),open_heap.end(),comp);
                    c->open=true;
                    if(c->cost < best->cost){
                        best = c;
                        iter_of_best=count;
                        //setRobots(r,h,best);
                        //std::cout << "best: "<<best->cost<<std::endl;
                    }
                    assert(best->cost <= c->cost);
                }
            }catch (Grid::out_of_grid &e){
                //that's normal, just keep on going.
            }
        }
    }
    //visibEngine->finish();
    setRobots(r,h,best);
    M3D_DEBUG("done "<<best->cost.toDouble()<<" found at iteration #"<<iter_of_best
              <<"\nit: "<<count<<" / "<<grid.getNumberOfCells()
              <<"\n\tCcol="<<best->cost.constraint(MyConstraints::COL)
              <<"\n\tCvis="<<best->cost.constraint(MyConstraints::VIS)
              <<"\n\tcost="<<best->cost.cost(MyCosts::COST)
              );
    for(uint i=0;i<grid.getNumberOfCells();++i){
        if(grid.getCell(i)) delete grid.getCell(i);
    }
    auto &ball_values = balls->balls_values;
    using BallValue_t=std::pair<float,std::vector<Eigen::Vector2d> >;
    std::sort(ball_values.begin(),ball_values.end(), [](const BallValue_t &a, const BallValue_t &b) {return a.first < b.first;});
    ball_values.erase(ball_values.begin()+std::min<uint>(100u,ball_values.size()),ball_values.end());


    Graphic::DrawablePool::sAddLinkedBalls2d(balls);
    ENV.setBool(Env::isRunning,false);
    M3D_DEBUG("PointingPlanner::run end");
    return *best;
}

PlanningData::Cell *PlanningData::createCell(Grid::ArrayCoord coord, Grid::SpaceCoord pos){
    Cell *cell=new Cell(coord,pos);
    cell->col = 0;
    computeCost(cell);
    return cell;
}

void PlanningData::setRobots(Robot *a, Robot *b, Cell *cell){
    RobotState qa,qb;
    qa=*a->getCurrentPos();
    qb=*b->getCurrentPos();
    for(uint i=0;i<2;++i){
        qa[6+i]=cell->getPos(0)[i];
        qb[6+i]=cell->getPos(1)[i];
    }
    a->setAndUpdate(qa);
    b->setAndUpdate(qb);
    moveHumanToFaceTarget(cell);
    moveRobotToHalfAngle(cell);
}

PlanningData::Cost PlanningData::computeCost(Cell *c){
    float kh(1-mh), kr(1-mr);
    float cost;
    std::vector<float> angle_h(targets.size());
    std::vector<float> angle_r(targets.size());
    std::vector<float> angle_persp(targets.size());
    std::vector<float> visib(targets.size());

    Eigen::Vector3d hr = r->getHriAgent()->perspective->getVectorPos() - h->getHriAgent()->perspective->getVectorPos();
    if(hr.squaredNorm()){
        //if vector is non null
        hr.normalize();
    }
    Eigen::Vector2d pr,ph;
    pr=c->vPosRobot();
    ph=c->vPosHuman();
    //move agents to positions of the cell:
    m3dGeometry::setBasePosition2D(r,pr);
    m3dGeometry::setBasePosition2D(h,ph);

    //for each target get its related values
    visib = getVisibilites(h,c->vPosHuman());
    for (uint i=0;i<targets.size();++i){
        Eigen::Vector3d rt,ht;
        Robot *target=targets[i];
        rt =  target->getJoint(0)->getVectorPos() - r->getHriAgent()->perspective->getVectorPos();
        ht =  target->getJoint(0)->getVectorPos() - h->getHriAgent()->perspective->getVectorPos();
        rt.normalize();
        ht.normalize();

        angle_h[i] = std::acos(hr.dot(ht));//angle for the human to look at the target and the robot
        angle_r[i] = std::acos((-hr).dot(rt));//idem for robot -> human
        //visib[i] = std::max(0.f,(1-visibility(i,ph))); //visibility cost (i.e. 1=worst, 0=best) of the target for the human
        angle_persp[i] = std::acos((ht).dot(rt)) * visib[i]; // perspective difference weighted by the visibility of the target
    }
    float c_visib=element(visib);
    float c_angle_r,c_angle_h,c_angle_persp,c_dist_r,c_dist_h,c_prox,c_time;
    c_angle_h = bounded_affine<float>(element(angle_h),0.,float(M_PI));
    c_angle_r = bounded_affine<float>(element(angle_r),0,float(M_PI));
    c_angle_persp = bounded_affine<float>(element(angle_persp),0,float(M_PI)); //perspective difference
    int col;
    {
        float dist_r,dist_h;
        std::array<float,2> ah,ar;
        ar=c->posRobot();
        ah=c->posHuman();
        dist_r=distGrid_r.getCostPos(ar);
        dist_h=distGrid_h.getCostPos(ah);
        c_dist_r = std::pow(dist_r+1,kr*kd+1)-1.f;//dist robot
        c_dist_h = std::pow(dist_h+1,kh*kd+1)-1.f;//dist human
        c_time = std::max(dist_r*sr,dist_h*sh);

        API::CylinderCollision cylinderCol(global_Project->getCollision());
        col = 3;
        col -= int(freespace_h.getCell(ah));
        col -= int(freespace_r.getCell(ar));
        col -= int(cylinderCol.moveCheck(r,Eigen::Vector3d(pr[0],pr[1],0.),h,Eigen::Vector3d(ph[0],ph[1],0.)));
    }
    c_prox = std::abs(dp-float((pr-ph).norm())); //proxemics

    cost = c_angle_r * kr + c_angle_h * kh + c_angle_persp * ka + c_dist_r * kr*kd + c_dist_h * kh*kd + c_prox * kp + c_visib * kv + c_time*kt;
    cost = cost / (kr+kh+ka+kr*kd+kh*kd+kt+kp+kv);
    if(cost!=cost || cost>=std::numeric_limits<float>::max()){//nan or inf
        cost=std::numeric_limits<float>::infinity();
    }

    std::map<std::string,double> costDetails;
    costDetails["angle h"]=     double(c_angle_h);
    costDetails["angle r"]=     double(c_angle_r);
    costDetails["angle persp"]= double(c_angle_persp);
    costDetails["dist r"]=      double(c_dist_r);
    costDetails["dist h"]=      double(c_dist_h);
    costDetails["time"]=        double(c_time);
    costDetails["proxemics"]=   double(c_prox);
    costDetails["visib"]=       double(c_visib);
    global_costSpace->setCostDetails(std::move(costDetails));

    c->col = (col!=0);
    c->vis = c_visib < vis_threshold; // if visib is better than ..
    c->cost.constraint(MyConstraints::COL) = col;
    c->cost.constraint(MyConstraints::VIS) = std::max<float>(0.,c_visib - vis_threshold);

    c->cost.costs[0]=cost;
    return c->cost;
}

std::vector<float> PlanningData::getVisibilites(Robot *r, const Eigen::Vector2d &pos2d)
{
    std::vector<float> visib;
    Eigen::Affine3d jnt_pos = r->getHriAgent()->perspective->getMatrixPos();
    jnt_pos.translationExt()[0]=pos2d[0];
    jnt_pos.translationExt()[1]=pos2d[1];
    Eigen::Vector3f pos=jnt_pos.translation().cast<float>();
    VisibilityGrid3d::SpaceCoord pgrid{{pos[0],pos[1],pos[2]}};
    try{
        VisibilityGrid3d::reference cell=visibilityGrid->getCell(pgrid);
        VisibilityGrid3d::SpaceCoord c =visibilityGrid->getCellCenter(visibilityGrid->getCellCoord(pgrid));
        M3D_TRACE("cell "<<c[0]<<" "<<c[1]<<" "<<c[2]);
        for(uint i=0;i<targets.size();++i){
            M3D_TRACE("\t"<<targets[i]->getName()<<" "<<cell[targets[i]]);
            visib.push_back(1.f-cell[targets[i]]);
        }
    }catch(VisibilityGrid3d::out_of_grid &){
        visib.assign(targets.size(),1.f);
    }
    //ogre_rob->getRobotNode()->setVisible(false);
    //visibEngine->computeVisibilityFrom(jnt_pos);
    //for ( Robot *t : targets){
    //    visib.push_back(1.f-1.f/0.007f * float(visibEngine->getVisibilityInFovPercent(t)));
    //}
    //ogre_rob->getRobotNode()->setVisible(true);
    return visib;
}

float PlanningData::computeStateCost(RobotState &q)
{
    reinit();
    if(q.getRobot() == r){
        Grid::ArrayCoord coord{{0,0,0,0}};
        Grid::SpaceCoord pos;
        for(unsigned int i=0;i<2;i++){
            pos[i]=q.at(6+i);
            pos[i+2]=h->getCurrentPos()->at(6+i);
        }
        Cell c(coord,pos);
        try{
            computeCost(&c);
        }catch(std::out_of_range &){
            return 0.;
        }
        return c.cost.toDouble();
    }else if(q.getRobot() == h){
        Grid::ArrayCoord coord={0,0,0,0};
        Grid::SpaceCoord pos;
        for(unsigned int i=0;i<2;i++){
            pos[i]=r->getCurrentPos()->at(6+i);
            pos[i+2]=q.at(6+i);
        }
        Cell c(coord,pos);
        try{
            computeCost(&c);
        }catch(std::out_of_range &){
            return 0.;
        }
        return c.cost.toDouble();
    }
    return 0.f;
}

void PlanningData::moveHumanToFaceTarget(Cell *c, uint target_id)
{
    Eigen::Vector2d pt = m3dGeometry::getConfBase2DPos(*targets[target_id]->getCurrentPos());
    Eigen::Vector2d ph = c->vPosHuman();
    float angle=m3dGeometry::angle(pt-ph);
    RobotState q=*h->getCurrentPos();
    q[11]=angle;
    h->setAndUpdate(q);
}

void PlanningData::moveRobotToHalfAngle(Cell *c, uint target_id)
{
    Eigen::Vector2d pt = m3dGeometry::getConfBase2DPos(*targets[target_id]->getCurrentPos());
    Eigen::Vector2d ph(c->vPosHuman());
    Eigen::Vector2d pr(c->vPosRobot());
    using namespace m3dGeometry;
    float a= angle(ph-pr) + angle(pt-pr,ph-pr)/2;
    RobotState q=*r->getCurrentPos();
    q[11]=a;
    r->setAndUpdate(q);
}

float PlanningData::element(const std::vector<float> &values, float factor){
    assert(values.size());
    if(values.empty()){return 1.f;}
    return *std::max_element(values.begin(),values.end()) * factor;
}

float PlanningData::visibility(uint target_i, const Eigen::Vector3d &pos){
    VisibilityGrid3d::SpaceCoord p{pos[0],pos[1],pos[2]};
    try{
        VisibilityGrid3d::reference cell=visibilityGrid->getCell(p);
        return cell[targets[target_i]];
    }catch(VisibilityGrid3d::out_of_grid &){
        return 0.f;
    }
}

bool PlanningData::isTooFar(Cell &c, Cell &from)
{
    Eigen::Vector2d pr,ph,fr,fh;
    pr = c.getPos(0);
    ph = c.getPos(1);
    fr = from.getPos(0);
    fh = from.getPos(1);
    double dr,dh;
    dr=(pr-fr).norm();
    dh=(ph-fh).norm();

    if(mh<=0.f && dh>0.f) return true;// human moves with mob=0
    if(mr<=0.f && dr>0.f) return true;// robot moves with mob=0
    if(dh>max_dist || dr>max_dist){
        return true;//too far
    }else{
        return areAgentTooFarFromEachOther(c);
    }
}

bool PlanningData::areAgentTooFarFromEachOther(Cell &c)
{
    Eigen::Vector2d pr,ph;
    if(kp <= 0.f) return false; //ok (ignores inter agent distance)
    pr=c.vPosRobot();
    ph=c.vPosHuman();
    return ((pr-ph).norm() > dp*2);

}

PlanningData::PlanningData(Robot *r, Robot *h):
    r(r),h(h)
{
    visibilityGrid=dynamic_cast<VisibilityGridLoader*>(ModuleRegister::getInstance()->module(VisibilityGridLoader::name()))->grid();
    cyl_r = r->getObjectRob()->getCylinder();
    cyl_h = h->getObjectRob()->getCylinder();

    visibEngine = new MoveOgre::VisibilityEngine(Ogre::Degree(360.f),Ogre::Degree(90.f),64u);

    reinit();

}

PlanningData::~PlanningData()
{
    delete visibEngine;
}

void PlanningData::getParameters()
{
    API::Parameter::lock_t lock;
    mr=API::Parameter::root(lock)["PointingPlanner"]["mobrob"].asDouble();
    mh=API::Parameter::root(lock)["PointingPlanner"]["mobhum"].asDouble();
    sh=API::Parameter::root(lock)["PointingPlanner"]["speedhum"].asDouble();
    sr=API::Parameter::root(lock)["PointingPlanner"]["speedrob"].asDouble();
    kt=API::Parameter::root(lock)["PointingPlanner"]["ktime"].asDouble();
    ka=API::Parameter::root(lock)["PointingPlanner"]["kangle"].asDouble();
    kp=API::Parameter::root(lock)["PointingPlanner"]["kproxemics"].asDouble();
    dp=API::Parameter::root(lock)["PointingPlanner"]["distproxemics"].asDouble();
    kd=API::Parameter::root(lock)["PointingPlanner"]["kdist"].asDouble();
    kv=API::Parameter::root(lock)["PointingPlanner"]["kvisib"].asDouble();
    max_dist=API::Parameter::root(lock)["PointingPlanner"]["maxdist"].asDouble();
    vis_threshold=API::Parameter::root(lock)["PointingPlanner"]["vis_threshold"].asDouble();

    API::Parameter &ptargets = API::Parameter::root(lock)["PointingPlanner"]["targets"];
    targets.clear();
    for(uint i=0;i<ptargets.size();++i){
        targets.push_back(global_Project->getActiveScene()->getRobotByName(ptargets[i].asString()));
        assert(targets.back());//not null
    }
}

void PlanningData::resetFromCurrentInitPos()
{
    start_r = *r->getInitialPosition();
    start_h = *h->getInitialPosition();

    start_p_r = m3dGeometry::getConfBase2DPos(start_r);
    start_p_h = m3dGeometry::getConfBase2DPos(start_h);

    balls.reset(new Graphic::LinkedBalls2d);
    balls->name="PointingPlanner";

    initCollisionGrids();
    API::ndGridAlgo::Dijkstra<API::nDimGrid<bool,2>,float>::SpaceCoord fromr,fromh;
    fromr[0]=start_p_r[0];
    fromr[1]=start_p_r[1];
    fromh[0]=start_p_h[0];
    fromh[1]=start_p_h[1];
    distGrid_r = API::ndGridAlgo::Dijkstra<API::nDimGrid<bool,2>,float>(&freespace_r,freespace_r.getCellCoord(fromr));
    distGrid_h = API::ndGridAlgo::Dijkstra<API::nDimGrid<bool,2>,float>(&freespace_h,freespace_h.getCellCoord(fromh));
    Graphic::DrawablePool::sAddGrid2Dfloat(std::shared_ptr<Graphic::Grid2Dfloat>(new Graphic::Grid2Dfloat{"distance human",API::nDimGrid<float,2>(distGrid_h.getGrid()),true}));
    Graphic::DrawablePool::sAddGrid2Dfloat(std::shared_ptr<Graphic::Grid2Dfloat>(new Graphic::Grid2Dfloat{"distance robot",API::nDimGrid<float,2>(distGrid_r.getGrid()),true}));
}

void PlanningData::reinit(){
    getParameters();
    resetFromCurrentInitPos();
}

void PlanningData::initCollisionGrids()
{
    VisibilityGrid3d::SpaceCoord vis_cell_size=visibilityGrid->getCellSize();
    API::nDimGrid<bool,2>::SpaceCoord cell_size;
    cell_size[0]=vis_cell_size[0];
    cell_size[1]=vis_cell_size[1];
    std::vector<double> envSize(4,0.);
    envSize[0]=global_Project->getActiveScene()->getBounds()[0]; //x min
    envSize[1]=global_Project->getActiveScene()->getBounds()[1]; //x max
    envSize[2]=global_Project->getActiveScene()->getBounds()[2]; //y min
    envSize[3]=global_Project->getActiveScene()->getBounds()[3]; //y max
    bool adjust=false;
    freespace_h=API::nDimGrid<bool,2>(cell_size,adjust,envSize);
    freespace_r=API::nDimGrid<bool,2>(cell_size,adjust,envSize);
    initCollisionGrid(freespace_h,h);
    initCollisionGrid(freespace_r,r);

    Graphic::DrawablePool::sAddGrid2Dfloat(std::shared_ptr<Graphic::Grid2Dfloat>(new Graphic::Grid2Dfloat{"collision human",API::nDimGrid<float,2>(freespace_h)}));
    Graphic::DrawablePool::sAddGrid2Dfloat(std::shared_ptr<Graphic::Grid2Dfloat>(new Graphic::Grid2Dfloat{"collision robot",API::nDimGrid<float,2>(freespace_r)}));
}

void PlanningData::initCollisionGrid(API::nDimGrid<bool,2> &grid, Robot *a)
{
    API::CollisionInterface *coll=global_Project->getCollision();
    assert(coll);
    API::CylinderCollision cylinderColl(coll);
    for(uint i=0;i<grid.getNumberOfCells();++i){
        API::nDimGrid<bool,2>::SpaceCoord c = grid.getCellCenter(grid.getCellCoord(i));
        Eigen::Vector3d p{c[0],c[1],0.};
        //m3dGeometry::setBasePosition2D(a,p);
        grid[i]=cylinderColl.moveCheck(a,p,API::CollisionInterface::CollisionChecks(API::CollisionInterface::COL_ENV | API::CollisionInterface::COL_OBJECTS));
    }

}

} // namespace move4d
