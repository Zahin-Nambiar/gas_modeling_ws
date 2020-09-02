#include <nrg_gas_source_localization/nrg_gas_source_localization.h>


namespace nrg_gas
{

NRGGasSourceLocalization::NRGGasSourceLocalization()
: NRGGas(),
  state_space_( 4, std::vector<double>(2) )
{   
    int np;
    private_nh_.param( "minimum_number_of_particles", np); 
    private_nh_.param( "minimum_number_of_particles", np_min_); 
    private_nh_.getParam( "state_space_bounds_x", state_space_[0]); 
    private_nh_.getParam( "state_space_bounds_y", state_space_[1]); 
    private_nh_.getParam( "state_space_bounds_z", state_space_[2]); 
    private_nh_.getParam( "state_space_bounds_rate", state_space_[3]); 

    initialize(np);
}

} // namespace nrg_gas