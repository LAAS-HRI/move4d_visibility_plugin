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
    return new VisibilityCell(index,computeCellCorner(x,y),this);
}
