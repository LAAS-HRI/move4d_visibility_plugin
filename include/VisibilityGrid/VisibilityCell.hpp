#ifndef VISIBILITY_CELL_HPP
#define VISIBILITY_CELL_HPP

#include <map>
#include <move4d/API/Grids/TwoDCell.hpp>
#include <move4d/API/forward_declarations.hpp>

namespace move4d{

class VisibilityCell : public API::TwoDCell
{

public:
    VisibilityCell();
    VisibilityCell(int i, Eigen::Vector2d corner, API::TwoDGrid *grid, const std::map<Robot *, double> &visibilities);
    
    double visibility(Robot *r);
private:
    std::map<Robot*,double> _visibilities;

};
}//namespace move4d

#endif // VISIBILITY_CELL_HPP
