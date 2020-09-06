#include <nrg_gas_concentration/nrg_gas_concentration.h>

//////ENTRY POINT//////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nrg_gas_concentration_server");

    nrg_gas::NRGGasConcentration concentration_server;
    
    ros::spin();
  
    return(0);

}