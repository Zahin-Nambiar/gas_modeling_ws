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
    Point source_local_point;
    source_local_point = calculateSourceTransform(gs.position, wind, map_to_anemometer);
    

    //If upwind- conc = 0 return
    //Else calculate conc return
    return 0.0;
}

Point NRGGas::calculateSourceTransform(const PointStamped& source, const Vector3Stamped& wind, const TransformStamped& map_to_anemometer) const
{
    Point t_in;
    // Source local translation
    t_in.x = map_to_anemometer.transform.translation.x - source.point.x;
    t_in.y = map_to_anemometer.transform.translation.y - source.point.y;
    t_in.z = map_to_anemometer.transform.translation.z - source.point.z;

    tf2::Quaternion q_wind_azimuth, q_base;
    q_wind_azimuth.setRPY( 0, 
                           0, 
                           atan2(wind.vector.y, wind.vector.x) );       // Wind azimuth in anemometer frame
    tf2::convert( map_to_anemometer.transform.rotation,
                  q_base );                                             // Anemometer rotation in map frame
    TransformStamped rot;
    rot.transform.rotation = tf2::toMsg( q_wind_azimuth*q_base );       // Wind azimuth transformed to map frame

    Point t_out;
    // Source local rotation
    tf2::doTransform(t_in, t_out, rot); 

    return t_out;
}

} // namespace nrg_gas
