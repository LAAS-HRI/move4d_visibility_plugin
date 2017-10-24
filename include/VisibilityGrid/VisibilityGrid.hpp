#ifndef VISIBILITY_GRID_HPP
#define VISIBILITY_GRID_HPP

#include <move4d/API/Grids/TwoDGrid.hpp>
#include <move4d/API/forward_declarations.hpp>
#include "VisibilityGrid/VisibilityCell.hpp"
#include <Eigen/Core>

namespace move4d{

class VisibilityGrid : public API::TwoDGrid
{

public:
    VisibilityGrid(Eigen::Vector2i size, const std::vector<double> &envSize);
    VisibilityGrid(double samplingRate, const std::vector<double> &envSize);

    Eigen::MatrixXd matrix(Robot *rob);

protected:
    virtual VisibilityCell *createNewCell(unsigned int index, unsigned int x, unsigned int y);
};
}//namespace move4d

#endif // VISIBILITY_GRID_HPP
