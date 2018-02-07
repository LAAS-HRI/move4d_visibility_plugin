#include "VisibilityGrid/VisibilityGrid.hpp"
#include <move4d/API/project.hpp>
#include <move4d-gui/common/OgreBase.hpp>


namespace move4d{
VisibilityGrid::VisibilityGrid(Eigen::Vector2i size, const std::vector<double> &envSize):
    API::TwoDGrid(size,envSize)
{

}

VisibilityGrid::VisibilityGrid(double samplingRate, const std::vector<double> &envSize):
    API::TwoDGrid(samplingRate,envSize)
{

}

Eigen::MatrixXd VisibilityGrid::matrix(Robot *rob)
{
    Eigen::MatrixXd mat(this->_nbCellsX,this->_nbCellsY);
    for (unsigned int i=0;i<getNumberOfCells();++i){
        VisibilityCell *cell=dynamic_cast<VisibilityCell*>(getCell(i));
        Eigen::Vector2i coord =getCellCoord(cell);
        mat(coord[0],coord[1]) = cell->visibility(rob);
    }
    return mat;
}

VisibilityCell *VisibilityGrid::createNewCell(unsigned int index, unsigned int x, unsigned int y)
{
    return new VisibilityCell(index,computeCellCorner(x,y),this);
}

VisibilityGrid3d::VisibilityGrid3d(SpaceCoord origin, ArrayCoord size, SpaceCoord cellSize):
    API::nDimGrid<std::unordered_map<Robot*,float>,3>(origin,size,cellSize)
{
}

VisibilityGrid3d::VisibilityGrid3d(ArrayCoord size, std::vector<double> envSize):
    API::nDimGrid<std::unordered_map<Robot*,float>,3>(size,envSize)
{
}

VisibilityGrid3d::VisibilityGrid3d(double samplingRate, bool adapt, std::vector<double> envSize):
    API::nDimGrid<std::unordered_map<Robot*,float>,3>(samplingRate,adapt,envSize)
{
}

VisibilityGrid3d::VisibilityGrid3d(SpaceCoord cellSize, bool adapt, std::vector<double> envSize):
    API::nDimGrid<std::unordered_map<Robot*,float>,3>(cellSize,adapt,envSize)
{
}

VisibilityGrid3d::VisibilityGrid3d():
    API::nDimGrid<std::unordered_map<Robot*,float>,3>()
{
}

void VisibilityGrid3d::merge(const std::map<Robot *, API::nDimGrid<float, 3> > &grids)
{

}

void VisibilityGrid3d::add(const std::string &robotName, std::vector<float> &values)
{
    move4d::Robot *r=move4d::global_Project->getActiveScene()->getRobotByName(robotName);
    assert(values.size() == this->getNumberOfCells());
    if(r){
        for(uint i=0;i<values.size();++i){
            this->getCell(i)[r]=values[i];
        }
    }
}

API::nDimGrid<float, 3> VisibilityGrid3d::computeGridOf(Robot *r) const
{
    API::nDimGrid<float,3> grid(m_originCorner,m_nbOfCell,m_cellSize);
    assert(grid.getNumberOfCells() == this->getNumberOfCells());
    for(uint i=0;i<grid.getNumberOfCells();++i){
        auto it=this->getCell(i).find(r);
        if(it!=this->getCell(i).end())
            grid.getCell(i) = it->second;
    }
    return grid;
}

float VisibilityGrid3d::getVisibility(Robot *agent, Eigen::Vector2d &pos2d, Robot *target)
{
    float visib{0.};
    Eigen::Affine3d jnt_pos = agent->getHriAgent()->perspective->getMatrixPos();
    jnt_pos.translationExt()[0]=pos2d[0];
    jnt_pos.translationExt()[1]=pos2d[1];
    Eigen::Vector3f pos=jnt_pos.translation().cast<float>();
    VisibilityGrid3d::SpaceCoord pgrid{{pos[0],pos[1],pos[2]}};
    try{
        VisibilityGrid3d::reference cell=getCell(pgrid);
        VisibilityGrid3d::SpaceCoord c =getCellCenter(getCellCoord(pgrid));
        auto it=cell.find(target);
        if(it!=cell.end()){
            visib = it->second;
        }
    }catch(VisibilityGrid3d::out_of_grid &){
        visib=0.f;
    }
    return visib;
}

VisibilityGrid3d::reference VisibilityGrid3d::getCell(Robot *agent, const Eigen::Vector2d &pos2d)
{
    Eigen::Affine3d jnt_pos = agent->getHriAgent()->perspective->getMatrixPos();
    jnt_pos.translationExt()[0]=pos2d[0];
    jnt_pos.translationExt()[1]=pos2d[1];
    Eigen::Vector3f pos=jnt_pos.translation().cast<float>();
    VisibilityGrid3d::SpaceCoord pgrid{{pos[0],pos[1],pos[2]}};
    VisibilityGrid3d::reference cell=getCell(pgrid);
    return cell;

}

std::map<Robot *, API::nDimGrid<float, 3> > VisibilityGrid3d::split() const
{
    std::map<Robot *, API::nDimGrid<float, 3> > grids;
    if(this->getNumberOfCells()){
        for (auto it: values_[0]) {
            grids[it.first]=computeGridOf(it.first);
        }
    }
    return grids;

}
}
