#include "utils.hpp"
#include <queue>
#include <string>

int main(int argc, char **argv){
  ros::init(argc, argv, "my_ur_test");
  //ros nodehandle init
  ros::NodeHandle*n=new ros::NodeHandle();
  ros::Rate r100(100);
  ros::Rate r50(50);
  ros::Rate r20(20);
  ros::Rate r10(10);
  ros::Rate r1(1);
  //arm driver init
  arm_com*arm=new arm_com(*n);
  double a_j=0.1,v_j=0.001;
  double a_l=0.1,v_l=0.001;
  double jts_move1[6]={0.1,0.0,0.0,0.0,0.0,0.0};
  double jts_move2[6]={-0.1,0.0,0.0,0.0,0.0,0.0};
  double pos_move1[7]={0.0,0.0,0.02,1.0,0.0,0.0,0.0};
  double pos_move2[7]={0.0,0.0,-0.02,1.0,0.0,0.0,0.0};
  double pos_end1[7]={-0.065,0.0375,0.0,1.0,0.0,0.0,0.0};
  double pos_end2[7]={0.0,-0.075,0.0,1.0,0.0,0.0,0.0};
  double pos_rot1[7]={0.0,0.0,0.0,0.707,0.0,0.0,0.707};
  double pos_rot2[7]={0.0,0.0,0.0,0.707,0.0,0.0,-0.707};
  double pos_rot3[7]={0.0,0.0,0.0,0.866,0.0,0.0,0.5};
  double pos_rot4[7]={0.0,0.0,0.0,0.866,0.0,0.0,-0.5};
  double an=7;
  double pos_rotd1[7]={0.0,0.0,0.0,cos(an/180.0*M_PI),0.0,0.0,sin(an/180.0*M_PI)};
  double pos_rotd2[7]={0.0,0.0,0.0,cos(-an/180.0*M_PI),0.0,0.0,sin(-an/180.0*M_PI)};
  //data collect
  arm->start_coll();
  ros::Duration(3).sleep();
  arm->stop_coll();

  arm->start_filtered_sense();
  double fts[6]={0.0};
  arm->GetFT(fts);
  for(int i=0;i<6;++i){
    printf("%f\t",fts[i]);
  }
  printf("\n");
  double jts[6]={0.0};
  arm->GetJointState(jts);
  for(int i=0;i<6;++i){
    printf("%f\t",jts[i]);
  }
  printf("\n");
  double pos[7]={0.0};
  arm->GetCartState(pos);
  for(int i=0;i<7;++i){
    printf("%f\t",pos[i]);
  }
  printf("\n");
  double end[7]={0.0,0.0,0.1,1.0,0.0,0.0,0.0};
  arm->GetCartState(pos,end);
  for(int i=0;i<7;++i){
    printf("%f\t",pos[i]);
  }
  printf("\n");

  //move func
  r20.sleep();
  for(int i=0;i<4;++i){
    arm->GetCartState(pos);
    printf("1st %f\t",pos[2]);
    arm->MoveRelaLineC(pos_move1,a_l,v_l);
    r20.sleep();
  }
  for(int i=0;i<4;++i){
    arm->GetCartState(pos);
    printf("2nd %f\t",pos[2]);
    arm->MoveRelaLineC(pos_move2,a_l,v_l);
    r20.sleep();
  }
  r1.sleep();
  arm->GetCartState(pos);
  printf("%f\t",pos[2]);
  /*
  arm->MoveRelaLineJ(jts_move1,a_l,v_l);
  ros::Duration(3).sleep();

  arm->MoveRelaJointJ(jts_move2,a_j,v_j);
  ros::Duration(3).sleep();

  arm->MoveRelaLineC(pos_move1,a_l,v_l);
  ros::Duration(3).sleep();

  arm->MoveRelaJointC(pos_move2,a_j,v_j);
  ros::Duration(3).sleep();

  arm->MoveRelaLineC(pos_move1,pos_end1,a_l,v_l);
  ros::Duration(3).sleep();

  arm->MoveRelaJointC(pos_move2,pos_end1,a_j,v_j);
  ros::Duration(3).sleep();

  arm->MoveRelaRotC(pos_rot1,a_l,v_l);
  ros::Duration(3).sleep();

  arm->MoveRelaRotC(pos_rot2,a_l,v_l);
  ros::Duration(3).sleep();
  
  arm->MoveRelaRotC(pos_rot1,pos_end1,a_l,v_l*5);
  ros::Duration(30).sleep();

  arm->MoveRelaRotC(pos_rot2,pos_end1,a_l,v_l*5);
  ros::Duration(30).sleep();
  
  arm->MoveRelaRotC(pos_rot3,pos_end2,a_l,v_l*5);
  ros::Duration(20).sleep();

  arm->MoveRelaRotC(pos_rot4,pos_end2,a_l,v_l*5);
  ros::Duration(20).sleep();
  
  for(int i=0;i<3;++i){
    arm->MoveRelaRotC(pos_rotd1,pos_end2,a_l,v_l*10);
    ros::Duration(1).sleep();
  }
  for(int i=0;i<3;++i){
    arm->MoveRelaRotC(pos_rotd2,pos_end2,a_l,v_l*10);
    ros::Duration(1).sleep();
  }
  */
  arm->stop_filtered_sense();
  return 0;
}
