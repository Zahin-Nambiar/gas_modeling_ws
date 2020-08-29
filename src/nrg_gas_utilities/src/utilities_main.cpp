#include <nrg_gas_utilities/nrg_gas.h>


int main(int argc, char** argv) {
  // Initialize ROS.
  ros::init(argc, argv, "nrg_utilities");
  ros::NodeHandle nh;
  nrg_gas::NRGGas nrg_gas_;

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

