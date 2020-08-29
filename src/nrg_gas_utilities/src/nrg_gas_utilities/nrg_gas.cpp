#include <nrg_gas_utilities/nrg_gas.h>

namespace nrg_gas
{

NRGGas::NRGGas()
: private_nh_("~"),
  tfListener(tfBuffer_)
{
    private_nh_.param( "horizontal_dispersion_x", wp_.horizontal.x, 0.1 ); //TODO give these  guys default values 
    private_nh_.param( "horizontal_dispersion_y", wp_.horizontal.y, 0.1 ); 
    private_nh_.param( "horizontal_dispersion_z", wp_.horizontal.z, 0.1 ); 
    private_nh_.param( "vertical_dispersion_x", wp_.vertical.x, 0.1 );
    private_nh_.param( "vertical_dispersion_y", wp_.vertical.y, 0.1 );
    private_nh_.param( "vertical_dispersion_z", wp_.vertical.z, 0.1 );
}

double NRGGas::calculateConcentration(const GasSource &gs, const Vector3Stamped &wind ) const
{  
    return 0.0;
}

} // namespace nrg_gas
