#ifndef NRGGAS_H
#define NRGGAS_H

#include <nrg_gas_utilities/GasSource.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

namespace nrg_gas
{

class NRGGas
{
protected:
    NRGGas(ros::NodeHandle *nh);

    double calculateConcentration(const GasSource &gs) const;

    WindParameters wp_;   

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener; 
}
    
} // namespace nrg_gas

#endif //NRGGAS_H