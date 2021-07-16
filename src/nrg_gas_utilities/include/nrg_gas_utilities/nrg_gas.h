#ifndef NRGGAS_H
#define NRGGAS_H

#include <nrg_gas_utilities/GasSource.h>
#include <nrg_gas_utilities/WindParameters.h>
#include <nrg_gas_utilities/GasConcentration.h>
#include <nrg_gas_utilities/AnemometerMsg.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>


#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <deque>
#include <vector>

using namespace std;  

namespace nrg_gas
{

using nrg_gas_utilities::GasSource;
using nrg_gas_utilities::WindParameters;
using nrg_gas_utilities::GasConcentration;
using nrg_gas_utilities::AnemometerMsg;

using geometry_msgs::TransformStamped;
using geometry_msgs::PointStamped;
using geometry_msgs::Point;


class NRGGas
{
protected: 
    NRGGas();

    double calculateConcentration(const GasSource &gs, const AnemometerMsg &wind, const TransformStamped map_to_anemometer) const;

    ros::NodeHandle private_nh_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener; 
    tf2_ros::TransformBroadcaster tfb;

    int pasquill_;  
    int environment_;
    vector<vector<float>> pasquill_data_urban_ {{0.22,0.0001,0.5,0.2,0.0,0.0},
                                               {0.16,0.0001,0.5,0.12,0.0,0.0},
                                               {0.11,0.0001,0.5,0.08,0.0002,0.5},
                                               {0.08,0.0001,0.5,0.06,0.0015,0.5},
                                               {0.06,0.0001,0.5,0.03,0.0003,1.0},
                                               {0.04,0.0001,0.5,0.016,0.0003,1.0}};
    vector<vector<float>> pasquill_data_country_ {{0.32,0.0004,0.5,0.24,0.001,0.5},
                                                 {0.32,0.0004,0.5,0.24,0.001,0.5},
                                                 {0.22,0.0004,0.5,0.2,0.0,0.0},
                                                 {0.16,0.0004,0.5,0.14,0.0003,0.5},
                                                 {0.11,0.0004,0.5,0.08,0.00015,0.5},
                                                 {0.11,0.0004,0.5,0.08,0.00015,0.5}};

    std::deque<AnemometerMsg> wind_measurements_;
    std::deque<GasConcentration> gas_measurements_;
private:
    Point calculateSourceTransform(const PointStamped& source, const AnemometerMsg& wind, const TransformStamped& map_to_anemometer) const;
};
    
} // namespace nrg_gas

#endif //NRGGAS_H