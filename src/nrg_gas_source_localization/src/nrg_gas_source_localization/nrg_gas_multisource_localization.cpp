#include <nrg_gas_source_localization/nrg_gas_multisource_localization.h>
#include <nrg_gas_utilities/nrg_gas_utilities.h>
#include <nrg_gas_utilities/GasSource.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h> 
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <math.h> 
#include <string> 



using visualization_msgs::MarkerArray;
using nrg_gas_utilities::GasSource;
using namespace std;

namespace nrg_gas
{

NRGGasMultisourceLocalization::NRGGasMultisourceLocalization(const visualization_msgs::InteractiveMarkerInit& ss_hypotheses)
: NRGGas(),
  state_space_( 4, std::vector<double>(2) )
{   
    
    visualization_pub_ = nh_.advertise<MarkerArray>("/particle_set_visualization", 10, true);
    ss_grids_ = ss_hypotheses;

    
    
    private_nh_.getParam( "state_space_bounds_x", state_space_[0] ); 
    private_nh_.getParam( "state_space_bounds_y", state_space_[1] ); 
    private_nh_.getParam( "state_space_bounds_z", state_space_[2] ); 
    private_nh_.getParam( "state_space_bounds_rate", state_space_[3] ); 
    
    int np;
    private_nh_.param( "number_of_particles", np, 2000 ); 
    private_nh_.param( "minimum_number_of_particles", np_min_, 500 );
    ROS_DEBUG_COND_NAMED( np_min_> np, "NRGGasMultisourceLocalization::NRGGasMultisourceLocalization()", 
                          "Number of particles is less than the minimum number of particles, resampling will be triggered by default" ); 
    private_nh_.param( "measurement_noise", R_, 1.0 ); 
    private_nh_.param( "process_noise", Q_, 1.0 );
    private_nh_.param( "source_number", sources_, 1 );
    private_nh_.param( "stationary_data_collection", stationary_mode_, 0 );   

    
    initialize(np);
}

void NRGGasMultisourceLocalization::update( const GasConcentration& gas_measurement, 
                                            const AnemometerMsg& wind_measurement, 
                                            const Odometry& odom_measurement)
{
    
    visualization_pub_.publish(createParticleSetVisualization());

    float v_x = odom_measurement.twist.twist.linear.x;
    float v_y = odom_measurement.twist.twist.linear.y;
    float angular_speed = odom_measurement.twist.twist.angular.z;
    
    if (stationary_mode_ == 1){ //Only update when stationary
        if(abs(v_x) + abs(v_y) + abs(angular_speed) < .05) {
            std::cout<<"Stationary Mode:: Robot is NOT moving: Updating particle filter!"<<std::endl;
            reweight(gas_measurement, wind_measurement);
            if( isDegenerate() )
            {
                resample();
            }
        }
        else {
            std::cout<<"Stationary Mode:: Robot is moving: NOT updating particle filter!"<<std::endl;
        }
    }

    else { //Update at all times
        std::cout<<"Dynamic Mode:: Updating particle filter!"<<std::endl;
        reweight(gas_measurement, wind_measurement);
        if( isDegenerate() )
        {
            resample();
        }
    }


    return;
}

void NRGGasMultisourceLocalization::initialize( const int& np)
{
    particle_set_.resize( np );
    
    if(ss_grids_.markers.size() == 0){  //If no grids created in rviz, use the default state space bounds
        std::cout<<"Using default ss bounds!!!"<<std::endl;

        for( auto& particle: particle_set_ )
        {
            
            particle.weight = 1.0/np;
            particle.sources.resize(sources_);
        
            for (auto& source: particle.sources) {
            
                auto rnv = uniform_rn(4);
                float x = state_space_[0][0] + rnv[0]*(state_space_[0][1] - state_space_[0][0]);
                float y = state_space_[1][0] + rnv[1]*(state_space_[1][1] - state_space_[1][0]);
                float z = state_space_[2][0] + rnv[2]*(state_space_[2][1] - state_space_[2][0]);
                float rate = state_space_[3][0] + rnv[3]*(state_space_[3][1] - state_space_[3][0]);
            
                populateSource(source,x,y,z,rate);
            
            }
        
        }
        
        return;
    }
    else {
        
        for( auto& particle: particle_set_ )
        {
            
            particle.weight = 1.0/np;
            particle.sources.resize(sources_);
        
            for (auto& source: particle.sources) {

                int grid_random = rand()%(ss_grids_.markers.size());
                int num_points = ss_grids_.markers[grid_random].controls[0].markers[0].points.size();
                geometry_msgs::Quaternion quat = ss_grids_.markers[grid_random].pose.orientation;

                tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                
                //Search for rectangular prism bounds
                float rect_x = 0.0;
                float rect_y = 0.0;
                float rect_z = 0.0;

                for(auto& point:ss_grids_.markers[grid_random].controls[0].markers[0].points){
                    if(point.x > rect_x){
                        rect_x = point.x;
                    }
                    if(point.y > rect_y){
                        rect_y = point.y;
                    }
                    if(point.z > rect_z){
                        rect_z = point.z;
                    }
                }
                 
                float x_anchor = ss_grids_.markers[grid_random].pose.position.x;
                float y_anchor = ss_grids_.markers[grid_random].pose.position.y;

                auto rnv = uniform_rn(4);
                float x = cos(yaw)*rnv[0]*rect_x - sin(yaw)*rnv[1]*rect_y + x_anchor;
                float y = sin(yaw)*rnv[0]*rect_x + cos(yaw)*rnv[1]*rect_y + y_anchor;
                float z = rnv[2]*(rect_z);
                float rate = state_space_[3][0] + rnv[3]*(state_space_[3][1] - state_space_[3][0]);

                
            
                populateSource(source,x,y,z,rate);
            
            }
        
        }
        return;
       

    }
}

void NRGGasMultisourceLocalization::populateSource(nrg_gas_utilities::GasSource& source, const float x, const float y, const float z, const float rate) {
    
    
    source.position.header.frame_id = "map";
    source.position.point.x = x;
    source.position.point.y = y;
    source.position.point.z = z;
    source.rate = rate;
    

}

void NRGGasMultisourceLocalization::reweight( const GasConcentration& gas_measurement, 
                                         const AnemometerMsg& wind_measurement )
{   
    TransformStamped map_to_anemometer_tf;
    try{
      map_to_anemometer_tf = tfBuffer_.lookupTransform( "map", 
                                                        wind_measurement.header.frame_id,
                                                        ros::Time(0), 
                                                        ros::Duration(0.1) );
    }
    catch ( tf2::TransformException &ex ) {
      ROS_WARN_NAMED( "NRGGasMultisourceLocalization::reweight()","%s",ex.what() );
      ros::Duration( 1.0 ).sleep();
      return;
    }

    double max_weight = 0.0;
    // Reweight particles based on similarity between measured concentration and theoretical concentration
    
    
    for( auto& particle: particle_set_ )
    {
        // Average "source" for particle-----------------------
        nrg_gas_utilities::GasSource s_average;
        s_average.position.header.frame_id = "map";

        for (auto& s: particle.sources) {

            s_average.position.point.x += s.position.point.x;
            s_average.position.point.y += s.position.point.y;
            s_average.position.point.z += s.position.point.z;
            s_average.rate += s.rate;

        }

        s_average.position.point.x /= sources_;
        s_average.position.point.y /= sources_;
        s_average.position.point.z /= sources_;
        s_average.rate /= sources_;
        //------------------------------------------------------

        particle.weight *= gaussian( calculateConcentration( s_average, 
                                                             wind_measurement, 
                                                             map_to_anemometer_tf ), 
                                     gas_measurement.concentration,
                                     R_ );   
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

void NRGGasMultisourceLocalization::resample()
{
    
    // See Multinomial ResampliMultisourceng in: " filters and resampling techniques: Importance in computational complexity analysis" IEEE 2013
    const int np = particle_set_.size();
    std::vector<MultisourceParticle> new_particle_set;
    new_particle_set.resize( np );
    
    // Find incremental sum of weight vector
    std::vector<double> weight_sum{0.0};    //Final size will be np+1 elements
    for( const auto& particle: particle_set_ )
    {
        weight_sum.push_back( weight_sum.back() + particle.weight ); 
    }
    
    // Pick a new particles from old particle set 
    for( auto& new_particle: new_particle_set )
    {        
        const double pick = uniform_rn();
        for(int i = 0; i < np; i++)
        {
            if( pick > weight_sum[i] &&
                pick < weight_sum[i+1] )
            {
                new_particle = particle_set_[i];
                new_particle.weight = 1.0/np;

                for (auto& s: new_particle.sources) {
                    s.position.point.x += uniform_rn()*Q_;
                    s.position.point.y += uniform_rn()*Q_;
                    s.position.point.z += uniform_rn()*Q_;
                }
                
            }
        }
    }
    particle_set_ = std::move( new_particle_set );
    return;
}

bool NRGGasMultisourceLocalization::isDegenerate()
{
    
    double sum = 0;
    for( auto& particle: particle_set_)
    {
        sum += particle.weight*particle.weight;
    }
    
    if( 1.0/sum < np_min_ )
    {    
        // Degenerate
        return 1;
    }
    // Not degenerate
    return 0;
}

visualization_msgs::MarkerArray NRGGasMultisourceLocalization::createParticleSetVisualization()
{
    
    
    for (int i =0; i < sources_; i++) {         //Update predicted source frames

        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = std::to_string(i);

        for( const auto& particle: particle_set_) {

            transformStamped.transform.translation.x += particle.sources[i].position.point.x;
            transformStamped.transform.translation.y += particle.sources[i].position.point.y;
            transformStamped.transform.translation.z += particle.sources[i].position.point.z;

        }
        transformStamped.transform.translation.x /= particle_set_.size();
        transformStamped.transform.translation.y /= particle_set_.size();
        transformStamped.transform.translation.z /= particle_set_.size();

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        transformStamped.header.stamp = ros::Time::now();
        tfb.sendTransform(transformStamped);
        

    }


    int id{0};
    MarkerArray particle_set_visualization;


    for( const auto& particle: particle_set_)
    {
        for (int i =0; i < sources_; i++) {
                
            particle_set_visualization.markers.push_back( createSourceMarker(particle.sources[i],i+1,sources_) );
            particle_set_visualization.markers.back().id = id;  //Each marker must have unique ID
            particle_set_visualization.markers.back().ns = "particles"; // Unique namespace 
            ++id;

        }

    } 
    


    return particle_set_visualization;
}

} // namespace nrg_gas