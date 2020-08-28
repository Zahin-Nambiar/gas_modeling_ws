#ifndef NRGGASUTILITIES_H
#define NRGGASUTILITIES_H

#include <nrg_gas_utilities/GasConcentration.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <deque>

namespace nrg_gas
{

struct WindDeck
{
    Vector3Stamped get( ros::Time t );
    void put( Vector3Stamped vs ); 

private:
    std::deque<Vector3Stamped> deck_;
};

struct GasDeck
{
    GasConcentration get( ros::Time t );
    void put( GasConcentration gc ); 

private:
    std::deque<GasConcentration> deck_;
};
    
} // namespace nrg_gas

#endif //NRGGASUTILITIES_H