#include <nrg_gas_utilities/nrg_gas_utilities.h>

namespace nrg_gas
{

using nrg_gas_utilities::GasConcentration;
using geometry_msgs::Vector3Stamped;

Vector3Stamped WindDeck::get( ros::Time t )
{   
    Vector3Stamped val;
    for( auto it = deck_.end(); it != deck_.begin(); --it )
    {
        if ( it->header.stamp.sec == t.sec )
        { 
            val = *it;
            deck_.erase(it);
            ROS_DEBUG_STREAM_NAMED("WindDeck::get", "Getting current measurement at time: " << t.sec );
            return val;
        }
        else if ( it->header.stamp.sec - t.sec <= -2 )
        {
            val = *it;
            deck_.erase(it);
            ROS_DEBUG_STREAM_NAMED("WindDeck::get", "Getting future measurement at time: " << t.sec );
            return val;
        }
        else if ( it->header.stamp.sec - t.sec >= 2 )
        {
            val = *it;
            deck_.erase(it);
            ROS_DEBUG_STREAM_NAMED("WindDeck::get", "Getting past measurement at time: " << t.sec );
            return val;
        }else
        {
            val.header.seq = -1;
            ROS_DEBUG_STREAM_NAMED("WindDeck::get", "No valid measurement to return");
        }
    }
    return val;
}

void WindDeck::put(Vector3Stamped vs)
{
    deck_.push_front( vs );
}

} // namespace nrg_gas