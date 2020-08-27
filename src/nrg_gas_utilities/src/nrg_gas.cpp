#include <nrg_gas.h>

namespace nrg_gas
{

NRGGas::NRGGas(ros::NodeHandle *nh)
: tfListener(tfBuffer_, nh)
{

}

} // namespace nrg_gas
