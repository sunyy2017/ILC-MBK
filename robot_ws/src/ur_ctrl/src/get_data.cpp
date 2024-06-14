#include "utils.hpp"
#include <queue>
#include <string>
#include <csignal>

//all needed global params
//base ros handle
ros::NodeHandle *n;
//arm communication
arm_com *arm;
//0 for no press emerg
//1 for pressed emerg
//2 for push
//3 for knob clock-wise
//4 for knob anti-clock-wise
int choice=1;
double fts[6]={0.0};
//line for 0force
double a_0=0.1;
double v_0=0.05;
//line for push
double a_l=0.1;
double v_l=0.005;
//line for knob,emerg
//double a_l=0.1;
//double v_l=0.07;
double a_j=1.0;
double v_j=0.1;

//calculate new ft zeros
int move0force(){
  arm->GetFT(fts);
  double pos_ori_t[7]={0.0,0.0,0.0005,1.0,0.0,0.0,0.0};
  double tmp_f=fts[2];
  double now_f=fts[2];
  while(abs(now_f-tmp_f)<0.5 || now_f>-0.5){
    tmp_f=now_f;
    //cartesian move block
    arm->MoveRelaLineC(pos_ori_t,a_l,v_l);
    ros::Duration(0.25).sleep();
    arm->GetFT(fts);
    now_f=fts[2];
    std::cout<<tmp_f<<'\t'<<now_f<<'\n';
  }
  //cartesian move block
  if(choice==3||choice==4)
    pos_ori_t[2]=-0.0025;
  else if(choice>1)
    pos_ori_t[2]=-0.0005;
  else if(choice==0)
    pos_ori_t[2]=0.0035;
  else if(choice==1)
    pos_ori_t[2]=0.001;
  arm->MoveRelaLineC(pos_ori_t,a_l,v_l);
  ros::Duration(0.75).sleep();
  return 1;
}
int main(int argc, char **argv){
  ros::init(argc, argv, "my_ur_driver");
  //ros nodehandle init
  n=new ros::NodeHandle();
  ros::Rate r100(100);
  ros::Rate r50(50);
  ros::Rate r10(10);
  //arm driver init
  arm=new arm_com(*n);
  ROS_INFO("Init ok");
  double button11[7]={0.0,0.0,0.006,1.0,0.0,0.0,0.0};
  double button12[7]={0.0,0.0,-0.006,1.0,0.0,0.0,0.0};
  double button21[7]={0.0,0.0,0.0,cos(0.05),0.0,0.0,sin(0.05)};
  double button22[7]={0.0,0.0,0.0,cos(-0.05),0.0,0.0,sin(-0.05)};
  double button41[7]={0.0,0.0,0.006,1.0,0.0,0.0,0.0};
  double button42[7]={0.0,0.0,-0.006,1.0,0.0,0.0,0.0};
  double end2[7]={-0.065,0.0375,0.0,1.0,0.0,0.0,0.0};
  double end3[7]={0.0,-0.075,0.0,1.0,0.0,0.0,0.0};
  double away[7]={0.0,0.0,-0.006,1.0,0.0,0.0,0.0};
  std::cout<<"input numbers to decide the object\n1 for pressed emergency, 2 for press, 3 for knob, clock-wise, 4 for press emergency\n";
  std::cin>>choice;

  double orig[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  arm->GetCartState(orig);
  arm->start_coll();
  move0force();
  switch(choice){
    case 1:
      for(int i=0;i<18;++i){
        arm->MoveRelaJointC(button21,end3,a_j,v_j);
        ros::Duration(1).sleep();
      }/*
      for(int i=0;i<18;++i){
        arm->MoveRelaJointC(button22,end3,a_j,v_j);
        ros::Duration(1).sleep();
      }*/
      break;
    case 2:
      arm->MoveRelaLineC(button11,a_l,v_l);
      ros::Duration(3).sleep();
      arm->MoveRelaLineC(button12,a_l,v_l);
      ros::Duration(3).sleep();
      break;
    case 3:
      for(int i=0;i<16;++i){
        arm->MoveRelaJointC(button21,end2,a_j,v_j);
        ros::Duration(1).sleep();
      }
      for(int i=0;i<16;++i){
        arm->MoveRelaJointC(button22,end2,a_j,v_j);
        ros::Duration(1).sleep();
      }
      break;
    case 4:
      arm->MoveRelaLineC(button41,a_l,v_l);
      ros::Duration(3).sleep();
      arm->MoveRelaLineC(button42,a_l,v_l);
      ros::Duration(3).sleep();
      break;
  }
  arm->MoveRelaLineC(away,a_l,v_l);
  ros::Duration(5).sleep();
  arm->stop_coll();
  arm->MoveLineC(orig,a_l,v_l);
  ros::Duration(5).sleep();
  return 0;
}
