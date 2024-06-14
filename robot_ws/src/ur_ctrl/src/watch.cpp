//ft bias->0force->admittance->up->save data->calc data->ft bias
//ILC_in_mbk1
//save_res in file cost.txt(append), x_f.txt(rewrite)
#include "utils.hpp"
#include <fstream>
#include <queue>
#include <string>
#include <csignal>
#include <cmath>

double fts[6]={0.0,0.0,0.0,0.0,0.0,0.0};
//all needed global params
//base ros handle
ros::NodeHandle *n;
//arm communication
arm_com *arm;

int main(int argc, char **argv){
  ros::init(argc, argv, "ur_watch");

  //ros nodehandle init
  n=new ros::NodeHandle();
  //arm driver init
  arm=new arm_com(*n);
  
  arm->start_filtered_coll();
  int t;
  std::cin>>t;
  arm->stop_filtered_coll();
  return 0;
}
