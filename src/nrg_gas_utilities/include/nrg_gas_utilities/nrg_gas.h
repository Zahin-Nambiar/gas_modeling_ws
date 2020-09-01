#ifndef NRGGAS_H
#define NRGGAS_H

#include <nrg_gas_utilities/GasSource.h>
#include <nrg_gas_utilities/WindParameters.h>
#include <nrg_gas_utilities/GasConcentration.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>


#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <deque>

namespace nrg_gas
{

using nrg_gas_utilities::GasSource;
using nrg_gas_utilities::WindParameters;
using nrg_gas_utilities::GasConcentration;
using geometry_msgs::Vector3Stamped;
using geometry_msgs::Vector3;
using geometry_msgs::TransformStamped;
using geometry_msgs::PointStamped;
using geometry_msgs::Point;


class NRGGas
{
public: //TODO make protected
    NRGGas();

    double calculateConcentration(const GasSource &gs, const Vector3Stamped &wind, const TransformStamped map_to_anemometer) const;

    ros::NodeHandle private_nh_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener; 

    WindParameters wp_;   

    std::deque<Vector3Stamped> wind_measurements_;
    std::deque<GasConcentration> gas_measurements_;
private:
    Point calculateSourceTransform(const PointStamped& source, const Vector3Stamped& wind, const TransformStamped& map_to_anemometer) const;
};
    
} // namespace nrg_gas

#endif //NRGGAS_H