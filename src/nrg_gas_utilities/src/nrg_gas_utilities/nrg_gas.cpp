#include <nrg_gas_utilities/nrg_gas.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

#include <iostream>

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

//23.771413852933193 1.0 -2.0
double NRGGas::calculateConcentration( const GasSource &gs, const Vector3Stamped &wind, TransformStamped map_to_anemometer ) const
{  
    calculateSourceTransform(gs.position, wind, map_to_anemometer);
    //If upwind- conc = 0 return
    //Else calculate conc return
    return 0.0;
}

void NRGGas::calculateSourceTransform(const PointStamped& source, const Vector3Stamped& wind, TransformStamped& map_to_anemometer) const
{
    map_to_anemometer.transform.translation.x -= source.point.x;
    map_to_anemometer.transform.translation.y -= source.point.y;
    map_to_anemometer.transform.translation.z -= source.point.z;

    tf2::Quaternion q_wind_azimuth, q_base;
    q_wind_azimuth.setRPY( 0, 0, atan2(wind.vector.y, wind.vector.x) );
    tf2::convert( map_to_anemometer.transform.rotation , q_base );

    map_to_anemometer.transform.rotation = tf2::toMsg( q_wind_azimuth*q_base );

    std::cout<<"Function"<<std::endl;
    std::cout<<map_to_anemometer.transform<<std::endl;

}

} // namespace nrg_gas
