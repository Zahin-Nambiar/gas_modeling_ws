#include <nrg_gas_utilities/nrg_gas.h>

namespace nrg_gas
{

NRGGas::NRGGas(ros::NodeHandle *nh)
: tfListener(tfBuffer_)
{
}

double NRGGas::calculateConcentration(const GasSource &gs) const
{  
}

} // namespace nrg_gas
