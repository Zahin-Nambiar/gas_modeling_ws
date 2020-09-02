#include <nrg_gas_source_localization/nrg_gas_source_localization.h>
#include <nrg_gas_utilities/nrg_gas_utilities.h>



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

void NRGGasSourceLocalization::initialize(int& np)
{
    particle_set_.resize(np);
    for(auto& particle: particle_set_)
    {
        particle.weight = 1.0/np;
        auto rnv = uniform_rn(4);
        particle.source.position.point.x = state_space_[0][0] + rnv[0]*(state_space_[0][1] - state_space_[0][0]);
        particle.source.position.point.y = state_space_[1][0] + rnv[1]*(state_space_[1][1] - state_space_[1][0]);
        particle.source.position.point.z = state_space_[2][0] + rnv[2]*(state_space_[2][1] - state_space_[2][0]);
        particle.source.rate = state_space_[3][0] + rnv[3]*(state_space_[3][1] - state_space_[3][0]);
    }
}

void NRGGasSourceLocalization::reweight()
{
    
}

} // namespace nrg_gas