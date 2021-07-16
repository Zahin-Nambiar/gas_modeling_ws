#include <nrg_gas_utilities/nrg_gas.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

namespace nrg_gas
{


NRGGas::NRGGas()
: private_nh_("~"),
  tfListener(tfBuffer_)
{
    private_nh_.param( "pasquill_type", pasquill_, 0 ); //0=A, 1=B .... 
    private_nh_.param( "environment_type", environment_, 0 ); //0=urban, 1=open country .... 
}

double NRGGas::calculateConcentration( const GasSource &gs, const AnemometerMsg &wind, TransformStamped map_to_anemometer ) const
{  
    Point source_local_point;
    source_local_point = calculateSourceTransform(gs.position, wind, map_to_anemometer);

    if( source_local_point.x<0 )
    {
        //Downwind concentration is zero if the measurement point is not downwind of source 
        return 0.0;
    }else{
        std::cout<<"Downwind"<<std::endl;

        float h_x; //horizontal_dispersion_x
        float h_y; //horizontal_dispersion_y
        float h_z; //horizontal_dispersion_z
        float v_x; //vertical_dispersion_x
        float v_y; //vertical_dispersion_y
        float v_z; //vertical_dispersion_z

        if (environment_ == 0) {
            h_x = pasquill_data_urban_[pasquill_][0]; 
            h_y = pasquill_data_urban_[pasquill_][1]; 
            h_z = pasquill_data_urban_[pasquill_][2]; 
            v_x = pasquill_data_urban_[pasquill_][3]; 
            v_y = pasquill_data_urban_[pasquill_][4]; 
            v_z = pasquill_data_urban_[pasquill_][5]; 
        }
        else {
            h_x = pasquill_data_country_[pasquill_][0];
            h_y = pasquill_data_country_[pasquill_][1];
            h_z = pasquill_data_country_[pasquill_][2];
            v_x = pasquill_data_country_[pasquill_][3];
            v_y = pasquill_data_country_[pasquill_][4];
            v_z = pasquill_data_country_[pasquill_][5];
        }

        const double sy = h_x*source_local_point.x*std::pow( 1.0+h_y*source_local_point.x, -h_z );
        const double sz = v_x*source_local_point.x*std::pow( 1.0+v_y*source_local_point.x, -v_z );
        const double expy = std::exp(-( source_local_point.y*source_local_point.y )/( 2*sy*sy ));  
        const double expz = std::exp(-( source_local_point.z*source_local_point.z )/( 2*sz*sz ));
        const double norm = ( gs.rate/wind.speed )/( 2*M_PI*sy*sz );
        return norm*expy*expz;
    }
}

Point NRGGas::calculateSourceTransform(const PointStamped& source, const AnemometerMsg& wind, const TransformStamped& map_to_anemometer) const
{
    Point t_in;
    // Source local translation
    t_in.x = map_to_anemometer.transform.translation.x - source.point.x;
    t_in.y = map_to_anemometer.transform.translation.y - source.point.y;
    t_in.z = map_to_anemometer.transform.translation.z - source.point.z;

    tf2::Quaternion q_wind_azimuth, q_base;
    
    q_wind_azimuth.setRPY( 0, 
                           0,
                           M_PI - wind.azimuth);       // Wind azimuth in anemometer frame:
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
