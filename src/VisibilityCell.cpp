#include "VisibilityGrid/VisibilityCell.hpp"

move4d::VisibilityCell::VisibilityCell():
    API::TwoDCell()
{
}

move4d::VisibilityCell::VisibilityCell(int i, Eigen::Vector2d corner, move4d::API::TwoDGrid *grid):
    API::TwoDCell(i,corner,grid)
{
}

move4d::VisibilityCell::VisibilityCell(int i, Eigen::Vector2d corner, move4d::API::TwoDGrid *grid, const std::map<move4d::Robot *, double> &visibilities):
    API::TwoDCell(i,corner,grid),
    _visibilities(visibilities)
{

}

void move4d::VisibilityCell::setVisibilities(std::map<Robot *, double> visibilities)
{
    std::swap(_visibilities,visibilities);
}

void move4d::VisibilityCell::setVisibility(move4d::Robot *r, double vis)
{
    _visibilities[r]=vis;
}

double move4d::VisibilityCell::visibility(Robot *r)
{
    auto it=_visibilities.find(r);
    if(it!=_visibilities.end()){
        return it->second;
    }
    return 0.;
}
