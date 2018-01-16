#include "VisibilityGrid/PointingPlanner.hpp"

#undef  QT_LIBRARY
#include <move4d/API/project.hpp>
//#include <move4d/API/Grids/MultiGrid.hpp>
#include <move4d/API/Grids/NDGrid.hpp>
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

namespace move4d {

PointingPlanner *PointingPlanner::__instance = new PointingPlanner();

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
    float &constraint(Constraints c){return constraints[(int)c];}
    float &cost(Costs c){return costs[(int)c];}
    Type &operator=(const Type &other){
        constraints = other.constraints;
        costs = other.costs;
        return *this;
    }
    bool operator<(const Type &other){
        return (constraints != other.constraints ? constraints < other.constraints : costs < other.costs);
    }
    bool operator<=(const Type &other){
        return (*this)==other || (*this)<other;
    }
    bool operator==(const Type &other){
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
            v+=costs[i] * std::pow(10,ncost - i);
        }
        return v;
    }
};

enum class MyConstraints {COL=0,VIS,DIST};
enum class MyCosts {COST=0};
using Cost = MyCost<3,1,MyConstraints,MyCosts>;
using Cell = MyCell<Cost>;
using Grid = typename API::nDimGrid<MyCell<Cost>*,4>;

template<typename C>
struct MyCell{
    using CostType = C;
    bool col;
    bool vis;
    C cost;
    Grid::ArrayCoord coord;
    Grid::SpaceCoord pos;
    bool open;
    inline Eigen::Vector2d getPos(int i) const {return Eigen::Vector2d(pos[i*2],pos[i*2+1]);}
    MyCell(Grid::ArrayCoord coord,Grid::SpaceCoord pos):
    open(0)
    {
        std::swap(coord,this->coord);
        std::swap(pos,this->pos);
    }
    bool operator<(const MyCell<C> &other) const {
        return (cost < other.cost);
    }
};


struct CompareCellPtr
{
    bool operator()(Cell *const &a,Cell *const &b){
        return b<a;
    }
};

struct PlanningData
{
public:
    Cell *createCell(Grid::ArrayCoord coord,Grid::SpaceCoord pos){
        Cell *cell=new Cell(coord,pos);
        cell->col = 0;
        computeCost(cell);
        return cell;
    }
    void setRobots(Robot *a,Robot *b, Cell *cell){
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
    Cost computeCost(Cell *c){
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
        for(uint i=0;i<2;i++){
            pr[i] = c->pos[i];
            ph[i] = c->pos[i+2];
        }
        //move agents to positions of the cell:
        m3dGeometry::setBasePosition2D(r,pr);
        m3dGeometry::setBasePosition2D(h,ph);

        //for each target get its related values
        for (uint i=0;i<targets.size();++i){
            Eigen::Vector3d rt,ht;
            Robot *target=targets[i];
            rt =  target->getJoint(0)->getVectorPos() - r->getHriAgent()->perspective->getVectorPos();
            ht =  target->getJoint(0)->getVectorPos() - h->getHriAgent()->perspective->getVectorPos();
            rt.normalize();
            ht.normalize();

            angle_h[i] = std::acos(hr.dot(ht));//angle for the human to look at the target and the robot
            angle_r[i] = std::acos((-hr).dot(rt));//idem for robot -> human
            visib[i] = std::max(0.f,(1-visibility(i,ph))); //visibility cost (i.e. 1=worst, 0=best) of the target for the human
            angle_persp[i] = std::acos((ht).dot(rt)) * visib[i]; // perspective difference weighted by the visibility of the target
        }
        float c_visib=element(visib);
        float c_angle_r,c_angle_h,c_angle_persp,c_dist_r,c_dist_h,c_prox;
        c_angle_h = bounded_affine<float>(element(angle_h),0.,M_PI);
        c_angle_r = bounded_affine<float>(element(angle_r),0,M_PI);
        c_angle_persp = bounded_affine<float>(element(angle_persp),0,M_PI); //perspective difference
        c_dist_r = std::pow((start_p_r - pr).norm(),kr*kd+1);//dist robot
        c_dist_h = std::pow((start_p_h - ph).norm(),kh*kd+1);//dist human
        c_prox = std::abs(dp-(pr-ph).norm()); //proxemics

        cost = c_angle_r * kr + c_angle_h * kh + c_angle_persp * ka + c_dist_r * kr*kd + c_dist_h * kh*kd + c_prox * kp + c_visib * kv;
        cost = cost / (kr+kh+ka+kr*kd+kh*kd+kp+kv);

        std::map<std::string,double> costDetails;
        costDetails["angle h"]=c_angle_h;
        costDetails["angle r"]=c_angle_r;
        costDetails["angle persp"]=c_angle_persp;
        costDetails["dist r"]=c_dist_r;
        costDetails["dist h"]=c_dist_h;
        costDetails["proxemics"]=c_prox;
        costDetails["visib"]=c_visib;
        global_costSpace->setCostDetails(std::move(costDetails));


        m3dGeometry::setBasePosition2D(r->getObjectRob()->getCylinder(),pr);
        m3dGeometry::setBasePosition2D(h->getObjectRob()->getCylinder(),ph);
        API::CollisionInterface *coll=global_Project->getCollision();
        assert(coll);
        c->col = 2 - coll->check(r->getObjectRob()->getCylinder(),API::CollisionInterface::COL_ENV | API::CollisionInterface::COL_OBJECTS)
                - coll->check(h->getObjectRob()->getCylinder(),API::CollisionInterface::COL_ENV | API::CollisionInterface::COL_OBJECTS);

        c->vis = c_visib < vis_threshold; // if visib is better than ..
        c->cost.constraint(MyConstraints::COL) = c->col;
        c->cost.constraint(MyConstraints::VIS) = std::max<float>(0.,c_visib - vis_threshold);

        c->cost.costs[0]=cost;
        return c->cost;
    }

    float computeStateCost(RobotState &q)
    {
        reinit();
        if(q.getRobot() == r){
            Grid::ArrayCoord coord={0,0,0,0};
            Grid::SpaceCoord pos;
            for(unsigned int i=0;i<2;i++){
                pos[i]=q.at(6+i);
                pos[i+2]=h->getCurrentPos()->at(6+i);
            }
            Cell c(coord,pos);
            computeCost(&c);
            return c.cost.toDouble();
        }else if(q.getRobot() == h){
            Grid::ArrayCoord coord={0,0,0,0};
            Grid::SpaceCoord pos;
            for(unsigned int i=0;i<2;i++){
                pos[i]=r->getCurrentPos()->at(6+i);
                pos[i+2]=q.at(6+i);
            }
            Cell c(coord,pos);
            computeCost(&c);
            return c.cost.toDouble();
        }
    }

    void moveHumanToFaceTarget(Cell *c, uint target_id=0)
    {
        Eigen::Vector2d pt = m3dGeometry::getConfBase2DPos(*targets[target_id]->getCurrentPos());
        Eigen::Vector2d ph(c->pos[2],c->pos[3]);
        float angle=m3dGeometry::angle(pt-ph);
        RobotState q=*h->getCurrentPos();
        q[11]=angle;
        h->setAndUpdate(q);
    }
    void moveRobotToHalfAngle(Cell *c, uint target_id=0)
    {
        Eigen::Vector2d pt = m3dGeometry::getConfBase2DPos(*targets[target_id]->getCurrentPos());
        Eigen::Vector2d ph(c->pos[2],c->pos[3]);
        Eigen::Vector2d pr(c->pos[0],c->pos[1]);
        using namespace m3dGeometry;
        float a= angle(ph-pr) + angle(pt-pr,ph-pr)/2;
        RobotState q=*r->getCurrentPos();
        q[11]=a;
        r->setAndUpdate(q);
    }

    inline float element(const std::vector<float> &values, float factor=1){
        return *std::max_element(values.begin(),values.end()) * factor;
    }

    inline float visibility(uint target_i, Eigen::Vector2d &pos){
        VisibilityCell *cell=dynamic_cast<VisibilityCell*>(visibilityGrid->getCell(pos));
        if(cell)
            return cell->visibility(targets[target_i]);
        else
            return 0.;
    }

    bool isTooFar(Cell *c,Eigen::Vector2d &from,float threshold)
    {
        Eigen::Vector2d pr,ph;
        for(uint i=0;i<2;i++){
            pr[i] = c->pos[i];
            ph[i] = c->pos[i+2];
        }
        if((pr-from).norm() > threshold || (ph-from).norm() > threshold){
            return true;//too far
        }else{
            return areAgentTooFarFromEachOther(c);
        }
    }

    bool areAgentTooFarFromEachOther(Cell *c)
    {
        Eigen::Vector2d pr,ph;
        for(uint i=0;i<2;i++){
            pr[i] = c->pos[i];
            ph[i] = c->pos[i+2];
        }
        return ((pr-ph).norm() > dp*2);

    }

    PlanningData(Robot *r, Robot *h,std::vector<Robot*> targets):
        r(r),h(h)
    {
        this->targets.swap(targets);
        visibilityGrid=dynamic_cast<VisibilityGridLoader*>(ModuleRegister::getInstance()->module(VisibilityGridLoader::name()))->grid();
        cyl_r = r->getObjectRob()->getCylinder();
        cyl_h = h->getObjectRob()->getCylinder();

        reinit();
    }

    void reinit(){
        targets.clear();

        start_r = *r->getInitialPosition();
        start_h = *h->getInitialPosition();

        start_p_r = m3dGeometry::getConfBase2DPos(start_r);
        start_p_h = m3dGeometry::getConfBase2DPos(start_h);

        API::Parameter::lock_t lock;
        mr=API::Parameter::root(lock)["PointingPlanner"]["mobrob"].asDouble();
        mh=API::Parameter::root(lock)["PointingPlanner"]["mobhum"].asDouble();
        ka=API::Parameter::root(lock)["PointingPlanner"]["kangle"].asDouble();
        kp=API::Parameter::root(lock)["PointingPlanner"]["kproxemics"].asDouble();
        dp=API::Parameter::root(lock)["PointingPlanner"]["distproxemics"].asDouble();
        kd=API::Parameter::root(lock)["PointingPlanner"]["kdist"].asDouble();
        kv=API::Parameter::root(lock)["PointingPlanner"]["kvisib"].asDouble();
        vis_threshold=API::Parameter::root(lock)["PointingPlanner"]["vis_threshold"].asDouble();

        API::Parameter &ptargets = API::Parameter::root(lock)["PointingPlanner"]["targets"];
        for(uint i=0;i<ptargets.size();++i){
            targets.push_back(global_Project->getActiveScene()->getRobotByName(ptargets[i].asString()));
            assert(targets.back());//not null
        }

        balls.reset(new Graphic::LinkedBalls2d);
        balls->name="PointingPlanner";
    }

    Robot *r;
    Robot *h;
    std::vector<Robot*> targets;
    VisibilityGrid* visibilityGrid;
    float mr,mh;
    float ka,kd,kp,kv;//factors
    float dp; //optimal distance (proxemics)
    float vis_threshold; //maximal visibility cost to consider a object is visible
    Robot *cyl_r;
    Robot *cyl_h;
    RobotState start_r;
    RobotState start_h;
    Eigen::Vector2d start_p_r;
    Eigen::Vector2d start_p_h;

    std::shared_ptr<move4d::Graphic::LinkedBalls2d> balls;
};

PointingPlanner::PointingPlanner():
    ModuleBase()
{
    M3D_INFO("PointingPlanner::PointingPlanner");
    _name = name();
    _deps.push_back("VisibilityPlanner");
    _deps.push_back("CostSpace");
    addToRegister();
}

void PointingPlanner::initialize()
{
    API::Parameter::lock_t lock;
    API::Parameter &parameter = API::Parameter::root(lock)["PointingPlanner"];
    if(parameter.type() == API::Parameter::NullValue){
        parameter["mobhum"] = API::Parameter(.5);
        parameter["mobrob"] = API::Parameter(1.);
        parameter["kangle"] = API::Parameter(1.);
        parameter["kdist"] = API::Parameter(1.);
        parameter["kproxemics"] = API::Parameter(1.);
        parameter["distproxemics"] = API::Parameter(0.75);
        parameter["vis_threshold"] = API::Parameter(0.5);
        parameter["kvisib"] = API::Parameter(5.);
        parameter["targets"] = API::Parameter(std::vector<API::Parameter>{"SHOP_SIGN_2"});
    }
    lock.unlock();

    Robot *r,*h;
    r=global_Project->getActiveScene()->getActiveRobot();
    h=global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");

    std::vector<Robot*> targets;
    targets.push_back(global_Project->getActiveScene()->getRobotByName("SHOP_SIGN_2"));
    _data=new PlanningData(r,h,std::move(targets));

    global_costSpace->addCost("PointingPlanner",boost::bind(&PlanningData::computeStateCost,_data,_1));

}

void PointingPlanner::run()
{
    ENV.setBool(Env::isRunning,true);
    this->_data->reinit();
    VisibilityGrid *vis_grid=dynamic_cast<VisibilityGridLoader*>(ModuleRegister::getInstance()->module(VisibilityGridLoader::name()))->grid();
    CompareCellPtr comp;
    std::vector<Cell*> open_heap;
    Eigen::Vector2i vis_size=vis_grid->getSize();
    //API::MultiGrid<float,vis_size[0],vis_size[1],vis_size[0],vis_size[1]> grid;
    Grid::ArrayCoord dim;
    dim[0]=dim[2]=vis_size[0];
    dim[1]=dim[3]=vis_size[1];
    std::vector<double> envSize(8,0.);
    envSize[0]=envSize[4]=global_Project->getActiveScene()->getBounds()[0]; //x min
    envSize[1]=envSize[5]=global_Project->getActiveScene()->getBounds()[1]; //x max
    envSize[2]=envSize[6]=global_Project->getActiveScene()->getBounds()[2]; //y min
    envSize[3]=envSize[7]=global_Project->getActiveScene()->getBounds()[3]; //y max
    Grid grid(dim,envSize);
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
    Cell *start=_data->createCell(coord,pos);
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
                c=grid[neigh];
                if(!c){
                    c=_data->createCell(neigh,grid.getCellCenter(neigh));
                    grid[neigh]=c;
                    if(_data->isTooFar(c,from,3.5)){
                        c->open=true; //do not enter in the "if" bellow, hence ignores its neighbours
                    }else if(c->col>best->col){
                        c->open=true;
                    }
                }
                if(!c->open){
                    open_heap.push_back(c);
                    _data->balls->balls_values.push_back(std::make_pair(c->cost.toDouble(),std::vector<Eigen::Vector2d>{c->getPos(0),c->getPos(1)}));
                    std::push_heap(open_heap.begin(),open_heap.end(),comp);
                    c->open=true;
                    if(c->cost < best->cost){
                        best = c;
                        iter_of_best=count;
                        //_data->setRobots(r,h,best);
                        //std::cout << "best: "<<best->cost<<std::endl;
                    }
                    assert(best->cost <= c->cost);
                }
            }catch (Grid::out_of_grid &e){
                //that's normal, just keep on going.
            }
        }
    }
    _data->setRobots(r,h,best);
    std::cout<<"done "<<best->cost.toDouble()<<" found at iteration #"<<iter_of_best<<std::endl;
    std::cout<<"it: "<<count<<" / "<<grid.getNumberOfCells()<<std::endl;
    for(uint i=0;i<grid.getNumberOfCells();++i){
        if(grid.getCell(i)) delete grid.getCell(i);
    }
    auto &ball_values = _data->balls->balls_values;
    using BallValue_t=std::pair<float,std::vector<Eigen::Vector2d> >;
    std::sort(ball_values.begin(),ball_values.end(), [](const BallValue_t &a, const BallValue_t &b) {return a.first < b.first;});
    ball_values.erase(ball_values.begin()+std::min<uint>(100u,ball_values.size()),ball_values.end());


    Graphic::DrawablePool::sAddLinkedBalls2d(_data->balls);
    ENV.setBool(Env::isRunning,false);
}

} // namespace move4d
