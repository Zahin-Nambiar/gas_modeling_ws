#ifndef NRG_GAS_CONCENTRATION_H
#define NRG_GAS_CONCENTRATION_H

#include <nrg_gas_utilities/nrg_gas.h>
#include <nrg_gas_concentration/SetSource.h>
#include <nrg_gas_concentration/GetConcentration.h>

#include <visualization_msgs/Marker.h>

#include <std_srvs/Empty.h>

//#include <ros/ros.h>

namespace nrg_gas
{

class NRGGasConcentration: protected NRGGas
{
public:
    NRGGasConcentration();
private:
    bool addSource( nrg_gas_concentration::SetSource::Request& req, nrg_gas_concentration::SetSource::Response& res );
    bool clearSources( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );
    bool getGasConcentration( nrg_gas_concentration::GetConcentration::Request &req, nrg_gas_concentration::GetConcentration::Response &res );

    ros::NodeHandle nh_;

    ros::Publisher visualization_pub_;

    ros::ServiceServer set_gas_source_srv_, 
                       clear_gas_source_srv_,
                       get_gas_concentration_srv_;

    std::vector<nrg_gas_utilities::GasSource> sources_;
 
};


} // namespace nrg_gas


#endif