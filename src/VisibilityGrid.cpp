#include "VisibilityGrid/VisibilityGrid.hpp"
#include <move4d-gui/common/OgreBase.hpp>


move4d::VisibilityGrid::VisibilityGrid(Eigen::Vector2i size, const std::vector<double> &envSize):
    API::TwoDGrid(size,envSize)
{

}

move4d::VisibilityGrid::VisibilityGrid(double samplingRate, const std::vector<double> &envSize):
    API::TwoDGrid(samplingRate,envSize)
{

}

Eigen::MatrixXd move4d::VisibilityGrid::matrix(move4d::Robot *rob)
{
    Eigen::MatrixXd mat(this->_nbCellsX,this->_nbCellsY);
    for (unsigned int i=0;i<getNumberOfCells();++i){
        VisibilityCell *cell=dynamic_cast<VisibilityCell*>(getCell(i));
        Eigen::Vector2i coord =getCellCoord(cell);
        mat(coord[0],coord[1]) = cell->visibility(rob);
    }
    return mat;
}

move4d::VisibilityCell *move4d::VisibilityGrid::createNewCell(unsigned int index, unsigned int x, unsigned int y)
{
    std::map<Robot*,double> vis;
    MoveOgre::OgreBase *base=MoveOgre::OgreBase::getInstance();
    Eigen::Vector2d corner = computeCellCorner(x,y);

    const uint nb_step=360; //1deg step
    const uint nb_step_max = 90; //cost = 1 for 90deg in the fov
    const double step=M_PI*2/nb_step;
    Ogre::Vector3 p;
    p[0]=corner[0] + _cellSize[0]/2;
    p[1]=corner[1] + _cellSize[1]/2;
    p[2]=1.5;

    for(double theta=0.;theta<M_PI*2;theta+=step){
        Ogre::Vector3 d(cos(theta),sin(theta),0);
        Robot *r=base->raycastRobot(p,d);
        if(r)
            vis[r]+=1./nb_step_max;
    }
    return new VisibilityCell(index,corner,this,vis);
}
