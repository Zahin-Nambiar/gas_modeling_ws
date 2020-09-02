#include <nrg_gas_source_localization/nrg_gas_source_localization.h>
#include <nrg_gas_utilities/nrg_gas_utilities.h>



namespace nrg_gas
{

NRGGasSourceLocalization::NRGGasSourceLocalization()
: NRGGas(),
  state_space_( 4, std::vector<double>(2) )
{   
    private_nh_.getParam( "state_space_bounds_x", state_space_[0]); 
    private_nh_.getParam( "state_space_bounds_y", state_space_[1]); 
    private_nh_.getParam( "state_space_bounds_z", state_space_[2]); 
    private_nh_.getParam( "state_space_bounds_rate", state_space_[3]); 
    int np;
    private_nh_.param( "minimum_number_of_particles", np); 
    private_nh_.param( "minimum_number_of_particles", np_min_); 
    private_nh_.param( "minimum_number_of_particles", R_, 1.0); 


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
    return;
}
//calculateConcentration( const GasSource &gs, const Vector3Stamped &wind, TransformStamped map_to_anemometer )
void NRGGasSourceLocalization::reweight(GasConcentration& gas_measurement, Vector3Stamped& wind_measurement)
{   
    TransformStamped map_to_anemometer_tf;
    try{
      map_to_anemometer_tf = tfBuffer_.lookupTransform("map", wind_measurement.header.frame_id,
                               ros::Time(0), ros::Duration(0.25) );
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN_NAMED("NRGGasSourceLocalization::reweight()","%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    double max_weight = 0.0;
    // Reweight particles based on similarity between measured concentration and theoretical concentration
    for( auto& particle: particle_set_ )
    {
        particle.weight *= gaussian( calculateConcentration(particle.source, wind_measurement, map_to_anemometer_tf ), gas_measurement.concentration, R_ );   
        if( particle.weight > max_weight ) max_weight = particle.weight;
    }
    
    // Log-likelihood reweight to prevent numerical underflow
    double weight_sum = 0;
    for( auto& particle: particle_set_ )
    {
        particle.weight = std::exp( std::log(particle.weight/max_weight) );
        weight_sum += particle.weight; 
    }
    
    // Normalize the cdf to 1
    for( auto& particle: particle_set_ )
    {
        particle.weight /= weight_sum;
    }
    return;
}

} // namespace nrg_gas