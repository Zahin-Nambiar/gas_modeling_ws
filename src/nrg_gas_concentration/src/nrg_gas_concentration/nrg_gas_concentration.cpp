#include <nrg_gas_concentration/nrg_gas_concentration.h>


using nrg_gas_utilities::GasSource;
using nrg_gas_concentration::SetSource;
using nrg_gas_concentration::GetConcentration;
using std_srvs::Empty;

namespace nrg_gas
{

NRGGasConcentration::NRGGasConcentration()
{
    visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("/gas_source_visualization_marker", 10, false);

    set_gas_source_srv_ = private_nh_.advertiseService( "set_source", &NRGGasConcentration::addSource, this );

    clear_gas_source_srv_ = private_nh_.advertiseService( "clear_sources", &NRGGasConcentration::clearSources, this );
    
    get_gas_concentration_srv_ = private_nh_.advertiseService( "get_gas_concentration", &NRGGasConcentration::getGasConcentration, this );
}

bool NRGGasConcentration::addSource( SetSource::Request& req, SetSource::Response& res )
{
    if (req.source.rate <= 0) 
    {
        ROS_WARN_NAMED("NRGGasConcentration::addSource()", "Source activity must be positive.");
        return 0;
    }

    sources_.push_back(req.source);
    const visualization_msgs::Marker source_marker = createSourceMarker(req.source);
    visualization_pub_.publish(source_marker);
    return 1;
}

bool NRGGasConcentration::clearSources( Empty::Request &req, Empty::Response &res )
{
    sources_.clear();
    return 1;
}

bool NRGGasConcentration::getGasConcentration( GetConcentration::Request &req, GetConcentration::Response &res )
{
  TransformStamped map_to_anemometer_tf;
  try
  {
    map_to_anemometer_tf = tfBuffer_.lookupTransform( "map", 
                                                       req.wind.header.frame_id,
                                                       ros::Time(0), 
                                                       ros::Duration(0.1) );
  }catch ( tf2::TransformException &ex ) 
  {
    ROS_WARN_NAMED( "NRGGasConcentration::getConcentration()","%s",ex.what() );
    ros::Duration( 1.0 ).sleep();
    return 0;
  }

  res.concentration.concentration = 0;
  for(const auto& source: sources_)
  {
    res.concentration.concentration += 
      calculateConcentration( source, req.wind, map_to_anemometer_tf );
  }
  res.concentration.header.frame_id = "map";
  res.concentration.header.stamp = ros::Time::now();
  return 1;
}


visualization_msgs::Marker NRGGasConcentration::createSourceMarker(const GasSource& source)
{
  visualization_msgs::Marker marker;
  
  marker.header = source.position.header;
  marker.ns = "gas_sources";
  marker.id = sources_.size();
  marker.pose.position = source.position.point;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = 1;
  marker.color.a = 1;
  
  return marker;
}

} // namespace nrg_gas