#include "VisibilityGrid/PointingPlannerModule.hpp"

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


#include <boost/bind.hpp>


INIT_MOVE3D_STATIC_LOGGER(move4d::PointingPlanner,"move4d.visibilitygrid.pointingplanner");

namespace move4d{

PointingPlanner *PointingPlanner::__instance = new PointingPlanner();

PointingPlanner::PointingPlanner():
    ModuleBase()
{
    M3D_INFO("PointingPlanner::PointingPlanner");
    _name = name();
    _deps.push_back("VisibilityGridLoader");
    _deps.push_back("Entities");
    _deps.push_back("CostSpace");
    _deps.push_back("HriInfo");
    addToRegister();
}

void PointingPlanner::initialize()
{
    API::Parameter::lock_t lock;
    API::Parameter &parameter = API::Parameter::root(lock)["PointingPlanner"];
    if(parameter.type() == API::Parameter::NullValue){
        parameter["mobhum"] = API::Parameter(.5);
        parameter["mobrob"] = API::Parameter(1.);
        parameter["speedhum"]=API::Parameter(0.8);
        parameter["speedrob"]=API::Parameter(0.8);
        parameter["ktime"] = API::Parameter(1.);
        parameter["ktimerob"] = API::Parameter(1.);
        parameter["kangle"] = API::Parameter(1.);
        parameter["kdist"] = API::Parameter(1.);
        parameter["kproxemics"] = API::Parameter(1.);
        parameter["distproxemics"] = API::Parameter(0.75);
        parameter["proxemics_tolerance"]=API::Parameter(0.75*0.2);
        parameter["maxdist"] = API::Parameter(3.5);
        parameter["maxtime"] = API::Parameter(60.);
        parameter["vis_threshold"] = API::Parameter(0.5);
        parameter["kvisib"] = API::Parameter(5.);
        parameter["targets"] = API::Parameter(std::vector<API::Parameter>{global_Project->getActiveScene()->getRobot(0u)->getName()});
        parameter["use_physical_target"] = API::Parameter(false);
        parameter["physical_target_pos"] = API::Parameter(std::vector<API::Parameter>{0.,0.});
        parameter["desired_angle_h"]=API::Parameter(80*M_PI/180);
        parameter["desired_angle_h_tolerance"] = API::Parameter(15*M_PI/180);
        parameter["ask_to_move_dist_trigger"] = 0.4; // when to consider the robot will ask the human to move somewhere else
        parameter["ask_to_move_duration"] = 10.; // the time it takes to the robot to ask the human to move and indicate them where
    }
    lock.unlock();

    Robot *r,*h;
    r=global_Project->getActiveScene()->getActiveRobot();
    h=global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");

    _data=new PlanningData(r,h);

    global_costSpace->addCost("PointingPlanner",boost::bind(&PlanningData::computeStateCost,_data,_1));
    global_costSpace->addCost("PointingPlannerVisibility",boost::bind(&PlanningData::computeStateVisiblity,_data,_1));

}

void PointingPlanner::run()
{
    M3D_DEBUG("PointingPlanner::run start");
    _data->run();
    M3D_DEBUG("PointingPlanner::run end");
}


} // namespace move4d
