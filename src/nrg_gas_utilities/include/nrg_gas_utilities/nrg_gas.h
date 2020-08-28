#ifndef NRGGAS_H
#define NRGGAS_H

#include <nrg_gas_utilities/GasSource.h>
#include <nrg_gas_utilities/WindParameters.h>
#include <nrg_gas_utilities/GasConcentration.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <deque>

namespace nrg_gas
{

using nrg_gas_utilities::GasSource;
using nrg_gas_utilities::WindParameters;
using nrg_gas_utilities::GasConcentration;
using geometry_msgs::Vector3Stamped;

class NRGGas
{
public: //TODO make protected
    NRGGas(ros::NodeHandle *nh);

    double calculateConcentration(const GasSource &gs) const;

    WindParameters wp_;   

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener; 

    std::deque<Vector3Stamped> wind_measurements_;
    std::deque<GasConcentration> gas_measurements_;
};
    
} // namespace nrg_gas

#endif //NRGGAS_H