#ifndef MOVE4D_POINTINGPLANNER_HPP
#define MOVE4D_POINTINGPLANNER_HPP
#include <OGRE/Ogre.h>

#include <move4d-gui/common/tools/VisibilityEngine.hpp>
#include <move4d-gui/common/Robot.hpp>

#undef  QT_LIBRARY
#include <move4d/API/Grids/NDGrid.hpp>
#include <move4d/API/Grids/NDGridAlgo.hpp>
#include <move4d/API/moduleBase.hpp>
#include <move4d/API/Parameter.hpp>
#include <move4d/API/Graphic/DrawablePool.hpp>

#include "VisibilityGrid/VisibilityGrid.hpp"


namespace move4d {

template<typename C>
struct MyCell;
template <size_t NCONS, size_t NCOST,typename CONSTRAINTS, typename COSTS>
struct MyCost
{
    MyCost():costs{},constraints{} {}
    using Constraints = CONSTRAINTS;
    using Costs = COSTS;
    using Type = MyCost<NCONS,NCOST,CONSTRAINTS,COSTS>;
    std::array<float,NCONS> constraints;
    std::array<float,NCOST> costs;
    float &constraint(Constraints c){return constraints.at((int)c);}
    float &cost(Costs c){return costs.at((int)c);}
    bool isValid(){for(float c : constraints){if (c>0.f) return false;} return true;}
    bool operator<(const Type &other) const{
        return (constraints != other.constraints ? constraints < other.constraints : costs < other.costs);
    }
    bool operator<=(const Type &other) const{
        return (*this)==other || (*this)<other;
    }
    bool operator==(const Type &other) const{
        return constraints == other.constraints && costs == other.costs;
    }

    double toDouble(){ // implicit conversion not wanted ! (so no operator double())
        double v(0);
        int nconst = constraints.size();
        int ncost  = costs.size();
        for(int i=0;i<nconst;++i){
            //assert(constraints[i]<=1.);
            v+=constraints[i] * std::pow(10,nconst + ncost - i);
        }
        for(int i=0;i<ncost;++i){
            //assert(costs[i]<=1.);
            v+=costs[i] * std::pow(10,ncost - i - 1);
        }
        return v;
    }
    std::string toString(){
        std::ostringstream oss;
        oss<<"constr: ";
        for(auto k : constraints){
            oss<<k<<" ";
        }
        oss<<"\ncosts : ";
        for(auto c : costs){
            oss<<c<<" ";
        }
        oss<<"\n ="<<toDouble();
        return oss.str();
    }
};

template<typename C>
struct MyCell{
    using Grid = typename API::nDimGrid<MyCell<C>*,4>;
    using ArrayCoord = typename Grid::ArrayCoord;
    using SpaceCoord = typename Grid::SpaceCoord;
    using CostType = C;
    bool col;
    bool vis;
    uint target;
    C cost;
    ArrayCoord coord;
    SpaceCoord pos; //pos[0:1] = robot, pos[2:3] = human
    bool open;
    inline Eigen::Vector2d getPos(int i) const {return Eigen::Vector2d(pos[i*2],pos[i*2+1]);}
    inline std::array<float,2> posRobot() const {return {{pos[0],pos[1]}};}
    inline std::array<float,2> posHuman() const {return {{pos[2],pos[3]}};}
    inline Eigen::Vector2d vPosRobot() const {return Eigen::Vector2d(pos[0],pos[1]);}
    inline Eigen::Vector2d vPosHuman() const {return Eigen::Vector2d(pos[2],pos[3]);}
    MyCell(ArrayCoord coord,SpaceCoord pos):
        cost{},
        coord(coord),
        pos(pos),
        open(0)
    {
    }
    bool operator<(const MyCell<C> &other) const {
        return (cost < other.cost);
    }
};

struct PlanningData
{
    MOVE3D_STATIC_LOGGER;
public:
    enum class MyConstraints {COL=0,VIS,DIST,RTIME,ANGLE};
    enum class MyCosts {COST=0,TIME,VISIB};
    using Cost = MyCost<5,3,MyConstraints,MyCosts>;
    using Cell = MyCell<Cost>;
    using Grid = typename API::nDimGrid<Cell*,4>;

    Cell run(bool read_parameters=true);
    Cell *createCell(Grid::ArrayCoord coord,Grid::SpaceCoord pos);
    void setRobots(Robot *a,Robot *b, Cell *cell);
    Cost computeCost(Cell *c);
    std::vector<float> getVisibilites(Robot *r, const Eigen::Vector2d &pos2d);

    float computeStateCost(RobotState &q);

    void moveHumanToFaceTarget(Cell *c, uint target_id=0);
    void moveRobotToHalfAngle(Cell *c, uint target_id=0);

    inline float element(const std::vector<float> &values, float factor=1);

    Cost targetCost(Cell *c, uint i, float visib, float visib_r);
    float getRouteDirTime(Cell *c, uint i);

    float visibility(uint target_i, const Eigen::Vector3d &pos);

    bool isTooFar(Cell &c, Cell &from);

    bool areAgentTooFarFromEachOther(Cell &c);

    PlanningData(Robot *r, Robot *h);
    ~PlanningData();

    /// reset the configuration of this from move4d::API::Parameter::root["PointingPlanner"]
    void getParameters();
    /// reset the initial positions of the agents from the move4d::Robot::getInitialPosition()
    /// computes the collision and navigation grids
    void resetFromCurrentInitPos();
    /// perform getParameters and resetFromCurrentInitPos
    void reinit();
    void initCollisionGrids();
    void initCollisionGrid(API::nDimGrid<bool,2> &grid,Robot *a);

    Robot *r;
    Robot *h;
    std::vector<Robot*> targets;
    std::vector<float> routeDirTimes;
    bool usePhysicalTarget;
    Eigen::Vector2d physicalTarget;///< where the human has to get in the end
    VisibilityGrid3d* visibilityGrid;
    MoveOgre::VisibilityEngine *visibEngine;

    float mr,mh,sr,sh;
    float ka,kd,kt,ktr,kp,kv;//factors
    float dp; //optimal distance (proxemics)
    float vis_threshold; //maximal visibility cost to consider a object is visible
    float max_dist;//maximal distance run by either agent
    float max_time_r;//maximal time for the robot
    Robot *cyl_r;
    Robot *cyl_h;
    RobotState start_r;
    RobotState start_h;
    Eigen::Vector2d start_p_r;
    Eigen::Vector2d start_p_h;

    std::shared_ptr<move4d::Graphic::LinkedBalls2d> balls;

    API::nDimGrid<bool,2> freespace_h,freespace_r;///< cell is true if in free space
    API::ndGridAlgo::Dijkstra<API::nDimGrid<bool,2>,float> distGrid_h,distGrid_r,distGrid_physicalTarget;
};

} // namespace move4d

#endif // MOVE4D_POINTINGPLANNER_HPP
