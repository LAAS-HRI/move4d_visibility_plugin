#include "VisibilityGrid/VisibilityPlanner.hpp"
#include "VisibilityGrid/VisibilityGridLoader.hpp"
#include <move4d/API/Grids/TwoDCostGrid.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>

#undef QT_LIBRARY
#include <move4d/API/planningAPI.hpp>

#include "VisibilityGrid/VisibilityGrid.hpp"

#include <move4d/API/Device/objectrob.hpp>
#include <move4d/API/Collision/collisionInterface.hpp>

#include <move4d/utils/Geometry.h>

INIT_MOVE3D_STATIC_LOGGER(move4d::VisibilityPlanner,"move4d.visibilitygrid.planner");
move4d::VisibilityPlanner *move4d::VisibilityPlanner::__instance = new move4d::VisibilityPlanner();
namespace move4d {

std::ostream &operator<<(std::ostream &stream, const VisibilityPlanner::CellCost_t &c){
    using namespace std;
    stream << "col:"<<get<0>(c)<<" !visok:"<<get<1>(c)<<" cost:"<<get<2>(c)<<" 1-vis:"<<get<3>(c)<<" dist:"<<get<4>(c);
    return stream;
}

VisibilityPlanner::VisibilityPlanner():
    ModuleBase()
{
    _name = name();
    _deps.push_back(VisibilityGridLoader::name());
    _deps.push_back("Entities");
    _deps.push_back("HriInfo");
    addToRegister();
}

double VisibilityPlanner::getCostSigmoid(double x,double min,double max,double low,double up)
{
    assert(max> min);
    double x_norm=(x-low)/(up - low);
    double c=( max-min)*( 1 / (1+ std::exp(-x_norm*12+6))) + min;
    assert(c<=max );
    assert(c>=min);
    return c;
}


VisibilityPlanner::CellCost_t VisibilityPlanner::computeCost(move4d::Robot *target_rob,const Eigen::Vector2d &target, move4d::Robot *r, VisibilityCell *cell,int sum_dist)
{
    bool col,vis_ok;
    double cost,vis,dist;
    assert(r->getObjectRob());
    Robot *cyl=r->getObjectRob()->getCylinder();
    assert(cyl);
    m3dGeometry::setBasePosition2D(cyl,cell->getCenter());
    col=!global_Project->getCollision()->check(cyl,move4d::API::CollisionInterface::COL_OBJECTS | API::CollisionInterface::COL_ENV);
    vis=cell->visibility(target_rob);
    vis_ok=vis > 0.2;
    dist = (target - cell->getCenter()).norm();
    double factor;
    if(vis_ok){
        factor=getCostSigmoid(vis,0,1,0.2,0.5);
        factor=factor*factor;
    }else{
        factor=0;
    }
    double slength = factor * getCostSigmoid(sum_dist,0,1,5,10)
            + getCostSigmoid(vis,0,1,1,0)
            + dist* (1-factor)*0.1;
    cost = slength;

    return std::make_tuple(col,!vis_ok,cost,1-vis,dist);
}

inline VisibilityCell *cast(API::TwoDCell *c){return dynamic_cast<VisibilityCell*>(c);}

void VisibilityPlanner::run()
{
    VisibilityGrid *grid=dynamic_cast<VisibilityGridLoader*>(ModuleRegister::getInstance()->module(VisibilityGridLoader::name()))->grid();
    API::TwoDCostGrid *costgrid=new API::TwoDCostGrid(grid->getSize(),global_Project->getActiveScene()->getBounds());
    costgrid->createAllCells();


    Robot *r=global_Project->getActiveScene()->getActiveRobot();
    Robot *target = global_Project->getActiveScene()->getRobotByName("SHOP_SIGN_2");

    Eigen::Vector2d vtarget(target->getCurrentPos()->at(6),target->getCurrentPos()->at(7));
    int count=500;
    VisibilityCell *cell=cast(grid->getCell(Eigen::Vector2d(r->getCurrentPos()->at(6),r->getCurrentPos()->at(7))));
    CellCost_t best_cost=computeCost(target,vtarget,r,cell,0);
    M3D_TRACE("start cell: "<<cell->getCenter().transpose() << " "<<best_cost);
    dynamic_cast<API::TwoDCostCell*>(costgrid->getCell(cell->getCenter()))->setCost(std::get<2>(best_cost));
    int path_length(0);
    while(count-->0){
        if(!cell) break;
        VisibilityCell *best_neig=0;
        ++path_length;
        for(unsigned int n_i=0;n_i<8;n_i++){
            VisibilityCell *neig=cast(grid->getNeighbour(cell,n_i));
            if(neig){
                CellCost_t cost = computeCost(target,vtarget,r,neig,path_length);
                dynamic_cast<API::TwoDCostCell*>(costgrid->getCell(neig->getCenter()))->setCost(std::get<2>(cost));
                if(cost < best_cost ){
                    best_cost = cost;
                    best_neig = neig;
                }
            }
        }
        if(!best_neig || !count){
            RobotState q=*r->getCurrentPos();
            q.at(6)=cell->getCenter()[0];
            q.at(7)=cell->getCenter()[1];
            r->setAndUpdate(q);
            M3D_TRACE("stoping; count="<<count);
            for(unsigned int n_i=0;n_i<8;n_i++){
                VisibilityCell *neig=cast(grid->getNeighbour(cell,n_i));
                if(neig){
                    CellCost_t cost=computeCost(target,vtarget,r,neig,path_length);
                    M3D_TRACE("cell "<<neig->getCenter().transpose()<<" "<<cost);
                }
            }
            break;
        }else{
            M3D_TRACE("select cell: "<<best_neig->getCenter().transpose() << " "<<best_cost);
            cell=best_neig;
        }
    }
    move4d::Graphic::DrawablePool::sAdd2dGrid(std::shared_ptr<Graphic::Drawable2dGrid>(new Graphic::Drawable2dGrid("VisPlanner",costgrid->matrix(),costgrid->getCellSize(),costgrid->getOriginCorner())));

}

} // namespace move4d
