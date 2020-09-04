#ifndef NRG_GAS_CONCENTRATION_H
#define NRG_GAS_CONCENTRATION_H

#include <nrg_gas_utilities/nrg_gas.h>

#include <nrg_gas_concentration/SetSource.h>


namespace nrg_gas
{

class NRGGasConcentration: protected NRGGas
{
public:

private:
    bool addSource( SetSource::Request& req, SetSource::Response& res );

    // bool getConcentration( GetConcentration::Request &req, GetConcentration::Response &res );

    // bool clearSources( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );

    std::vector<nrg_gas_utilities::GasSource> sources_;

    ros::ServiceServer set_gas_source_srv_;
};


} // namespace nrg_gas


#endif