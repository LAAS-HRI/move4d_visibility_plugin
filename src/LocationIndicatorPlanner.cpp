#include "VisibilityGrid/LocationIndicatorPlanner.hpp"
#include "VisibilityGrid/PointingPlanner.hpp"
#include "VisibilityGrid/VisibilityGridLoader.hpp"
#include <move4d/utils/Geometry.h>
#include <move4d/API/project.hpp>
#include <move4d/planner/cost_space.hpp>
#include <move4d/utils/multiHandOver/SimpleGraph.h>
#include <move4d/utils/multiHandOver/graphAlgorithms.h>

namespace move4d { namespace LocationIndicator{

template<typename T>
class NodeBase : public mho::Node
{
public:
    using reference = T&;
    using const_reference = T const &;
    using data_type = T;

    NodeBase(const_reference data):
        mho::Node(),
        data(data)
    { }

    const_reference getData() const { return data;}
    void setData(const data_type &value){data=value;}

private:
    data_type data;
};

LocationIndicatorPlanner* LocationIndicatorPlanner::__instance = new LocationIndicatorPlanner();

LocationIndicatorPlanner::LocationIndicatorPlanner()
{
    _deps.push_back("Entities");
    _deps.push_back("HriInfo");
    _deps.push_back("VisibilityGridLoader");
    _deps.push_back("PointingPlanner");
    _deps.push_back("CostSpace");

    _name="LocationIndicator";

    addToRegister();
}

void LocationIndicatorPlanner::loadRouteGraph()
{
    API::Parameter::lock_t lock;
    API::Parameter &param=API::Parameter::root(lock)["LocationIndicator"]["routes"];
    for(uint i=0 ; i<param["nodes"].size() ; ++i){
        mRouteGraph->addNode(new NodeBase<API::Parameter>(param["nodes"][i]));
    }

    for(uint i=0 ; i<param["edges"].size() ; ++i){
        API::Parameter &e=param["edges"][i];
        mRouteGraph->connectNodes(mRouteGraph->at(e["from"].asInt()),mRouteGraph->at(e["to"].asInt()),e["cost"].asDouble());
    }

}

void LocationIndicatorPlanner::findHumanOnly(TargetInformation &target)
{
    const double max_pointing_cost = 1.;
    std::vector<PlanningData::Cell> costs;
    uint best=-1u;
    PlanningData::Cell best_cell({{0,0,0,0}},{{0,0,0,0}});
    RobotState robot_state,human_state;

    for(uint i =0; i<target.references.size();++i){
        PlanningData plan(target.robot,target.human);
        plan.targets.assign(1,target.references[i].first);
        plan.max_dist=50.;
        plan.mr=0.f;
        plan.mh=1.f;
        plan.ka=0.f;
        plan.kp=0.f;
        plan.vis_threshold=0.5f;

        costs.push_back(plan.run(false));
        costs.back().cost.cost(PlanningData::Cost::Costs::COST)+=std::pow(plan.kt*target.references[i].second,2.) +
                std::pow(plan.ktr*target.references[i].second,2.);
        costs.back().cost.cost(PlanningData::Cost::Costs::TIME) += target.references[i].second;
        if(best==-1u || costs.back() < best_cell){
            best=i;
            best_cell=costs.back();
            human_state=*target.human->getCurrentPos();
            robot_state=*target.robot->getCurrentPos();
        }
    }
    if(best!=-1u && best_cell.cost.toDouble() < 1000.){
        //found a good place for the human
        std::cout << "human goes at "<<best_cell.getPos(1).transpose()<<" and will see "<<target.references[best].first->getName()<<
                     " at "<< m3dGeometry::getConfBase2DPos(*target.references[best].first->getCurrentPos()).transpose() <<std::endl;
        target.human->setAndUpdate(human_state);
        target.robot->setAndUpdate(robot_state);
    }else if(best!=-1u){
        std::cout<<"DIY solution has cost "<<best_cell.cost.toDouble()<<" -> discarding"<<std::endl;
    }else{
        std::cout<<"no DIY solution found"<<std::endl;
    }
}

Robot* LocationIndicatorPlanner::isTargetVisibleFromInit(TargetInformation &target)
{
    VisibilityGridLoader *visibGridLoader=dynamic_cast<VisibilityGridLoader*>(move4d::ModuleRegister::getInstance()->module("VisibilityGridLoader"));
    float best_vis{0};
    Robot *best_target{0};
    try{
        VisibilityGrid3d::reference cell=visibGridLoader->grid()->getCell(target.human,target.humanVPosInit());
        for(auto t : target.references){
            auto it = cell.find(t.first);
            if (it!=cell.end()) {
                if(best_vis < it->second){
                    best_vis=it->second;
                    best_target=it->first;
                }
            }
        }
        if(best_target){
            return best_target;
        }
    }catch (VisibilityGrid3d::out_of_grid &){
        return nullptr;
    }
    return nullptr;
}

void LocationIndicatorPlanner::pointFromInit(TargetInformation &target, Robot *obj)
{
    PlanningData plan(target.robot,target.human);
    plan.targets.assign(1,obj);
    plan.max_dist=0.;
    plan.mr=0.f;
    plan.mh=0.f;
    plan.ka=0.f;
    plan.kp=0.f;
    plan.vis_threshold=0.f;
    plan.run(false);
}

void LocationIndicatorPlanner::run(TargetInformation &target)
{
    const double max_pointing_cost = 50.;
    std::vector<PlanningData::Cell> costs;
    uint best=-1u;
    PlanningData::Cell best_cell({{0,0,0,0}},{{0,0,0,0}});
    RobotState robot_state,human_state;

    Robot *best_target_init =
            isTargetVisibleFromInit(target);
    if(best_target_init){
        std::cout<<"a target is visible from initial position, try to show it: "<<best_target_init->getName()<<std::endl;
        pointFromInit(target,best_target_init);
        if(target.see_first){
            std::cout<<"the human see the target, job done"<<std::endl;
            return;
        }//else continue
    }
    if (target.known){
        std::cout<<"human knows the target, they can go by themselve"<<std::endl;
        findHumanOnly(target);
    }else{
        std::cout<<"do some more effort in showing the target to the human"<<std::endl;
        for(uint i =0; i<target.references.size();++i){
            PlanningData plan(target.robot,target.human);
            plan.getParameters();
            plan.targets.assign(1,target.references[i].first);
            costs.push_back(plan.run(false));
            costs.back().cost.cost(PlanningData::Cost::Costs::COST)+=std::pow(plan.kt*target.references[i].second,2.) +
                    std::pow(plan.ktr*target.references[i].second,2.);
            costs.back().cost.cost(PlanningData::Cost::Costs::TIME)+=target.references[i].second;
            if(best==-1u || costs.back() < best_cell){
                best=i;
                best_cell=costs.back();
                human_state=*target.human->getCurrentPos();
                robot_state=*target.robot->getCurrentPos();
            }
        }
        if(best_cell.cost.toDouble() < max_pointing_cost){
            //found a good pointing solution
            std::cout<<"found good pointing for target "<<target.references[best].first->getName()<<std::endl;
            target.human->setAndUpdate(human_state);
            target.robot->setAndUpdate(robot_state);
        }else{
            std::cout<<"no pointing found, search DIY solution"<<std::endl;
            findHumanOnly(target);
        }
    }
}

void LocationIndicatorPlanner::runCompare(TargetInformation &target)
{
    PlanningData plan(target.robot,target.human);
    plan.getParameters();
    plan.targets.clear();
    plan.routeDirTimes.clear();
    for(auto it : target.references){
        plan.targets.push_back(it.first);
        plan.routeDirTimes.push_back(it.second);
    }
    PlanningData::Cell cell = plan.run(false);
    std::cout<<"found good pointing for target "<<target.references[cell.target].first->getName();
    std::cout<<"cost="<<cell.cost.cost(PlanningData::MyCosts::COST)<<
               "\ntime="<<cell.cost.cost(PlanningData::MyCosts::TIME)<<std::endl;

}

void LocationIndicatorPlanner::run()
{
    Robot *r,*h;
    r=global_Project->getActiveScene()->getActiveRobot();
    h=global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");

    std::vector<std::pair<Robot*,float> > targets;
    API::Parameter ptargets;
    TargetInformation data;
    {
    API::Parameter::lock_t lock;
    ptargets = API::Parameter::root(lock)["LocationIndicator"]["references"];
    data.known=API::Parameter::root(lock)["LocationIndicator"]["known"].asBool();
    data.see_first=API::Parameter::root(lock)["LocationIndicator"]["see_first"].asBool();
    // data.positionPhysicalTarget[0]=API::Parameter::root(lock)["LocationIndicator"]["physical_target_pos"][0].asDouble();
    // data.positionPhysicalTarget[1]=API::Parameter::root(lock)["LocationIndicator"]["physical_target_pos"][1].asDouble();
    }
    loadRouteGraph();
    for(uint i=0;i<ptargets.size();++i){
        targets.push_back(std::make_pair(global_Project->getActiveScene()->getRobotByName(ptargets[i]["name"].asString()),
                          float(ptargets[i]["dir_cost"].asDouble())));
        assert(targets.back().first);//not null
    }
    data.human=h;
    data.robot=r;
    data.references=targets;

    runCompare(data);
    //run(data);
}

std::array<float, 2> TargetInformation::humanPosInit(){
    statePtr_t q=human->getInitialPosition();
    return {{float(q->at(6)),float(q->at(7))}};
}

std::array<float, 2> TargetInformation::robotPosInit()
{
    statePtr_t q=robot->getInitialPosition();
    return {{float(q->at(6)),float(q->at(7))}};
}

Eigen::Vector2d TargetInformation::humanVPosInit()
{
    statePtr_t q=human->getInitialPosition();
    return Eigen::Vector2d{q->at(6),q->at(7)};
}

Eigen::Vector2d TargetInformation::robotVPosInit()
{
    statePtr_t q=robot->getInitialPosition();
    return Eigen::Vector2d{q->at(6),q->at(7)};
}

}
} // namespace move4d
