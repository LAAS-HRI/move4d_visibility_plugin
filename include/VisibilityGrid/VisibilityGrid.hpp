#ifndef VISIBILITY_GRID_HPP
#define VISIBILITY_GRID_HPP

#include <unordered_map>
#include <move4d/API/Device/robot.hpp>
#include <move4d/API/Grids/TwoDGrid.hpp>
#include <move4d/API/forward_declarations.hpp>
#include "VisibilityGrid/VisibilityCell.hpp"
#include <Eigen/Core>

#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/string.hpp>
#include <move4d/API/Grids/NDGrid.hpp>

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


class VisibilityGrid3d : public API::nDimGrid<std::unordered_map<move4d::Robot*,float>,3>
{
public:
    VisibilityGrid3d(SpaceCoord origin, ArrayCoord size, SpaceCoord cellSize);
    VisibilityGrid3d(ArrayCoord size, std::vector<double> envSize);
    VisibilityGrid3d(double samplingRate, bool adapt, std::vector<double> envSize);
    VisibilityGrid3d(SpaceCoord cellSize, bool adapt, std::vector<double> envSize);
    VisibilityGrid3d();

    void merge(const std::map<Robot*,API::nDimGrid<float,3> > &grids);
    void add(const std::string &robotName,std::vector<float> &values);

    std::map<Robot*,API::nDimGrid<float,3> > split() const;
    API::nDimGrid<float,3> computeGridOf(Robot *r) const;

    float getVisibility(Robot *agent, Eigen::Vector2d &pos2d, Robot *target);
    reference getCell(Robot *agent, const Eigen::Vector2d &pos2d);
    using API::nDimGrid<std::unordered_map<move4d::Robot*,float>,3>::getCell;

protected:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version){
        boost::serialization::split_member(ar,*this,version);
    }


    template<class Archive>
    void load(Archive &ar, const unsigned int version){
        ar >> boost::serialization::make_array(m_nbOfCell.data(),m_nbOfCell.size());
        ar >> boost::serialization::make_array(m_cellSize.data(),m_cellSize.size());
        ar >> boost::serialization::make_array(m_originCorner.data(),m_originCorner.size());

        this->values_.assign((uint)m_nbOfCell[0]*m_nbOfCell[1]*m_nbOfCell[2],value_type());
        ulong n_rob;

        ar >> n_rob;
        for (ulong i=0;i<n_rob;++i){
            std::string name;
            ar >> name;
            std::vector<float> values;
            ar >> values;
            this->add(name,values);
        }
    }
    template<class Archive>
    void save(Archive &ar, const unsigned int version) const
    {
        ar << boost::serialization::make_array(m_nbOfCell.data(),m_nbOfCell.size());
        ar << boost::serialization::make_array(m_cellSize.data(),m_cellSize.size());
        ar << boost::serialization::make_array(m_originCorner.data(),m_originCorner.size());

        if(this->getNumberOfCells()){
            ulong nb_rob=values_[0].size();
            ar << nb_rob; // nb of robots
            for (auto it: values_[0]) {
                std::string name = it.first->getName();
                ar << name;
                std::vector<float> values = computeGridOf(it.first).getValues();
                ar << values;
            }
        }
    }


};

}//namespace move4d

#endif // VISIBILITY_GRID_HPP
