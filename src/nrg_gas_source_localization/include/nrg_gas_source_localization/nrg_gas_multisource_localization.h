#include <nrg_gas_utilities/nrg_gas.h>
#include <nrg_gas_utilities/GasSource.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <nav_msgs/Odometry.h>
#include <vector>

namespace nrg_gas
{

using nav_msgs::Odometry;

struct MultisourceParticle
{
    std::vector<nrg_gas_utilities::GasSource> sources;
    double weight = 0.0;
};



class NRGGasMultisourceLocalization: protected NRGGas
{
public:
    NRGGasMultisourceLocalization(const visualization_msgs::InteractiveMarkerInit& ss_hypotheses);
    void update( const GasConcentration& gas_measurement, const AnemometerMsg& wind_measurement, const Odometry& odom_measurement);
private:
    void initialize(const int& np);
    void reweight( const GasConcentration& gas_measurement, const AnemometerMsg& wind_measurement );
    void resample();
    bool isDegenerate();
    void populateSource(nrg_gas_utilities::GasSource& source, const float x, const float y, const float z, const float rate);

    std::vector<MultisourceParticle> particle_set_;
    std::vector<std::vector<double>> state_space_;
    
    double R_; 
    double Q_;
    int sources_;
    int stationary_mode_;
    int np_min_;

    visualization_msgs::InteractiveMarkerInit ss_grids_;

    ros::NodeHandle nh_;

    ros::Publisher visualization_pub_;

    visualization_msgs::MarkerArray createParticleSetVisualization();

};

} // namespace nrg_gas