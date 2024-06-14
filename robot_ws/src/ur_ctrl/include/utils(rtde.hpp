#ifndef UTILS_HPP_
#define UTILS_HPP_
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "geometry_msgs/WrenchStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <cmath>
#include <pthread.h>

//util functions
void double2Angleaxis(double*pos_ori,double*pos_aa){
  Eigen::Quaterniond q_(pos_ori[3],pos_ori[4],
                        pos_ori[5],pos_ori[6]);
  Eigen::AngleAxisd aa(q_);
  double ang=aa.angle();
  pos_aa[0]=pos_ori[0];
  pos_aa[1]=pos_ori[1];
  pos_aa[2]=pos_ori[2];
  pos_aa[3]=aa.axis().x()*ang;
  pos_aa[4]=aa.axis().y()*ang;
  pos_aa[5]=aa.axis().z()*ang;
}
void Angleaxis2double(double*pos_aa,double*pos_ori){
  double ang=sqrt(pos_aa[3]*pos_aa[3]+pos_aa[4]*pos_aa[4]+pos_aa[5]*pos_aa[5]);
  Eigen::AngleAxisd aa(ang,Eigen::Vector3d(pos_aa[3]/ang,pos_aa[4]/ang,pos_aa[5]/ang));
  Eigen::Quaterniond q_(aa);
  q_.normalize();
  pos_ori[0]=pos_aa[0];
  pos_ori[1]=pos_aa[1];
  pos_ori[2]=pos_aa[2];
  pos_ori[3]=q_.w();
  pos_ori[4]=q_.x();
  pos_ori[5]=q_.y();
  pos_ori[6]=q_.z();
}
Eigen::Matrix4d double2Mat(double*pos_ori){
  Eigen::Matrix4d pose_out=Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q_(pos_ori[3],pos_ori[4],
                        pos_ori[5],pos_ori[6]);
  q_.normalize();
  Eigen::Matrix3d R_=q_.matrix();
  pose_out.block<3,3>(0,0)=R_;
  pose_out.block<3,1>(0,3)<<pos_ori[0],pos_ori[1],pos_ori[2];
  return pose_out;
}
void Mat2double(Eigen::Matrix4d pose_in,double*pos_ori){
  Eigen::Quaterniond q_(pose_in.block<3,3>(0,0));
  q_.normalize();
  pos_ori[0]=pose_in(0,3);
  pos_ori[1]=pose_in(1,3);
  pos_ori[2]=pose_in(2,3);
  pos_ori[3]=q_.w();
  pos_ori[4]=q_.x();
  pos_ori[5]=q_.y();
  pos_ori[6]=q_.z();
}

//only for z-axis
double DeltaDist(double*pos_ori_0,double*pos_ori){
  Eigen::Matrix4d ori0=double2Mat(pos_ori_0);
  Eigen::Matrix4d ori=double2Mat(pos_ori);
  Eigen::Matrix4d trans=ori0.inverse()*ori;
  return trans(2,3);
}
//delta angle of quaterion, only for z-axis
double DeltaQuat(double*pos_ori_0,double*pos_ori){
  Eigen::Matrix4d ori0=double2Mat(pos_ori_0);
  Eigen::Matrix4d ori=double2Mat(pos_ori);
  Eigen::Matrix4d trans=ori0.inverse()*ori;
  return atan2(trans(1,0),trans(0,0));
}
class arm_com{
public:
  //init
  arm_com(ros::NodeHandle&n,std::string&ip):n_(n),ip_(ip),force_spin(4){
    ur_ctrl=new ur_rtde::RTDEControlInterface(ip_);
    ur_rcev=new ur_rtde::RTDEReceiveInterface(ip_);
  }
  ~arm_com(){}
  ros::NodeHandle &n_;
  std::string ip_;
  ur_rtde::RTDEControlInterface*ur_ctrl;
  ur_rtde::RTDEReceiveInterface*ur_rcev;
  std::ofstream fft,fjt,fpo;

  //collect force-pos data
  ros::AsyncSpinner force_spin;
  ros::Subscriber ft_sub;
  void force_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg_){
    pthread_mutex_lock(&m1);
    f_t[0]=msg_->wrench.force.x;
    f_t[1]=msg_->wrench.force.y;
    f_t[2]=msg_->wrench.force.z;
    f_t[3]=msg_->wrench.torque.x;
    f_t[4]=msg_->wrench.torque.y;
    f_t[5]=msg_->wrench.torque.z;
    pthread_mutex_unlock(&m1);
    fft<<msg_->header.stamp.nsec<<'\t'<<msg_->wrench.force.x<<'\t'<<msg_->wrench.force.y<<'\t'<<msg_->wrench.force.z<<'\t'
      <<msg_->wrench.torque.x<<'\t'<<msg_->wrench.torque.y<<'\t'<<msg_->wrench.torque.z<<'\n';
    std::vector<double> jt=ur_rcev->getActualQ();
    std::vector<double> po=ur_rcev->getActualTCPPose();
    double pos_aa[6]={0.0};
    double pos_ori[7]={0.0};
    fjt<<msg_->header.stamp.nsec<<'\t'<<jt[0]<<'\t'<<jt[1]<<'\t'<<jt[2]<<'\t'
      <<jt[3]<<'\t'<<jt[4]<<'\t'<<jt[5]<<'\n';
    for(int i=0;i<6;++i){
      pos_aa[i]=po[i];
    }
    Angleaxis2double(pos_aa,pos_ori);
    fpo<<msg_->header.stamp.nsec<<'\t'<<pos_ori[0]<<'\t'<<pos_ori[1]<<'\t'<<pos_ori[2]
      <<'\t'<<pos_ori[3]<<'\t'<<pos_ori[4]<<'\t'<<pos_ori[5]<<'\t'<<pos_ori[6]<<'\n';
  }
  void start_coll(){
    ft_sub=n_.subscribe("/ethdaq_data", 1, &arm_com::force_callback, this);
    pthread_mutex_init(&m1, NULL);
    fft.open("for_tor_collect.txt",std::ios::trunc);
    fjt.open("joint_collect.txt",std::ios::trunc);
    fpo.open("pos_ori_collect.txt",std::ios::trunc);
    force_spin.start();
  }
  void stop_coll(){
    ft_sub.~Subscriber();
    pthread_mutex_destroy(&m1);
    fft.close();
    fjt.close();
    fpo.close();
    force_spin.stop();
  }
  
  //sense realtime force pos data
  double f_t[6]={0.0};
  pthread_mutex_t m1;
  void f_t_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg_){
    pthread_mutex_lock(&m1);
    f_t[0]=msg_->wrench.force.x;
    f_t[1]=msg_->wrench.force.y;
    f_t[2]=msg_->wrench.force.z;
    f_t[3]=msg_->wrench.torque.x;
    f_t[4]=msg_->wrench.torque.y;
    f_t[5]=msg_->wrench.torque.z;
    pthread_mutex_unlock(&m1);
  }
  void start_sense(){
    ft_sub=n_.subscribe("/ethdaq_data", 1, &arm_com::f_t_callback, this);
    pthread_mutex_init(&m1, NULL);
    force_spin.start();
  }
  void stop_sense(){
    ft_sub.~Subscriber();
    pthread_mutex_destroy(&m1);
    force_spin.stop();
  }

  //Get function group
  void GetFT(double*res){
    printf("got 1\n");
    pthread_mutex_lock(&m1);
    printf("got 2\n");
    res[0]=f_t[0];
    res[1]=f_t[1];
    res[2]=f_t[2];
    res[3]=f_t[3];
    res[4]=f_t[4];
    res[5]=f_t[5];
    pthread_mutex_unlock(&m1);
  }
  void GetJointState(double*res){
    std::vector<double> jt=ur_rcev->getActualQ();
    for(int i=0;i<6;++i){
      res[i]=jt[i];
    }
  }
  void GetCartState(double*res,double*end_pos){
    std::vector<double> po=ur_rcev->getActualTCPPose();
    double pos_aa[6]={0.0};
    for(int i=0;i<6;++i){
      pos_aa[i]=po[i];
    }
    Angleaxis2double(pos_aa,res);
    Eigen::Matrix4d pose=double2Mat(res);
    Eigen::Matrix4d trans=double2Mat(end_pos);
    pose=pose*trans;
    Mat2double(pose,res);
  }
  void GetCartState(double*res){
    std::vector<double> po=ur_rcev->getActualTCPPose();
    double pos_aa[6]={0.0};
    for(int i=0;i<6;++i){
      pos_aa[i]=po[i];
    }
    Angleaxis2double(pos_aa,res);
  }
  //Move function group
  void MoveLineJ(double*tar,double a,double v){
    std::vector<double> t;
    for(int i=0;i<6;++i){
      t.emplace_back(tar[i]);
    }
    ur_ctrl->moveL_FK(t,v,a,true);
  }
  void MoveJointJ(double*tar,double a,double v){
    std::vector<double> t;
    for(int i=0;i<6;++i){
      t.emplace_back(tar[i]);
    }
    ur_ctrl->moveJ(t,v,a,true);
  }
  void MoveLineC(double*tar,double a,double v){
    double pos_aa[6];
    double2Angleaxis(tar,pos_aa);
    std::vector<double> t;
    for(int i=0;i<6;++i){
      t.emplace_back(pos_aa[i]);
    }
    ur_ctrl->moveL(t,v,a,true);
  }
  void MoveLineC(double*tar,double*end_pos,double a,double v){
    double mid[7];
    Eigen::Matrix4d pose=double2Mat(tar);
    Eigen::Matrix4d trans=double2Mat(end_pos);
    pose=pose*trans.inverse();
    Mat2double(pose,mid);
    MoveLineC(mid,a,v);
  }
  void MoveJointC(double*tar,double a,double v){
    double pos_aa[6];
    double2Angleaxis(tar,pos_aa);
    std::vector<double> t;
    for(int i=0;i<6;++i){
      t.emplace_back(pos_aa[i]);
    }
    ur_ctrl->moveJ_IK(t,v,a,true);
  }
  void MoveJointC(double*tar,double*end_pos,double a,double v){
    double mid[7];
    Eigen::Matrix4d pose=double2Mat(tar);
    Eigen::Matrix4d trans=double2Mat(end_pos);
    pose=pose*trans.inverse();
    Mat2double(pose,mid);
    MoveJointC(mid,a,v);
  }
  void MoveRelaLineJ(double*tar,double a,double v){
    double tmp[6]={0,0,0,0,0,0};
    GetJointState(tmp);
    for(int i=0;i<6;++i){
      tmp[i]+=tar[i];
    }
    MoveLineJ(tmp,a,v);
  }
  void MoveRelaJointJ(double*tar,double a,double v){
    double tmp[6]={0,0,0,0,0,0};
    GetJointState(tmp);
    for(int i=0;i<6;++i){
      tmp[i]+=tar[i];
    }
    MoveJointJ(tmp,a,v);
  }
  void MoveRelaLineC(double*tar,double a,double v){
    double tmp_c[7]={0,0,0,1,0,0,0};
    GetCartState(tmp_c);
    Eigen::Matrix4d tmp_m1=double2Mat(tmp_c);
    Eigen::Matrix4d tmp_m2=double2Mat(tar);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m2;
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    MoveLineC(tar2,a,v);
  }
  void MoveRelaLineC(double*tar,double*end_pos,double a,double v){
    double tmp_c[7]={0,0,0,1,0,0,0};
    GetCartState(tmp_c);
    Eigen::Matrix4d tmp_m1=double2Mat(tmp_c);
    Eigen::Matrix4d tmp_m2=double2Mat(tar);
    Eigen::Matrix4d tmp_m3=double2Mat(end_pos);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m3*tmp_m2;
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    MoveLineC(tar2,end_pos,a,v);
  }
  void MoveRelaJointC(double*tar,double a,double v){
    double tmp_c[7]={0,0,0,1,0,0,0};
    GetCartState(tmp_c);
    Eigen::Matrix4d tmp_m1=double2Mat(tmp_c);
    Eigen::Matrix4d tmp_m2=double2Mat(tar);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m2;
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    MoveJointC(tar2,a,v);
  }
  void MoveRelaJointC(double*tar,double*end_pos,double a,double v){
    double tmp_c[7]={0,0,0,1,0,0,0};
    printf("start get cart\n");
    GetCartState(tmp_c);
    Eigen::Matrix4d tmp_m1=double2Mat(tmp_c);
    Eigen::Matrix4d tmp_m2=double2Mat(tar);
    Eigen::Matrix4d tmp_m3=double2Mat(end_pos);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m3*tmp_m2;
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    MoveJointC(tar2,end_pos,a,v);
  }
  void MoveRelaRotC(double*tar,double a,double v){
    MoveRelaLineC(tar,a,v);
  }
  //specially for z-axis
  void MoveRelaRotC(double*tar,double*end_pos,double a,double v){
    double tmp_c[7]={0,0,0,1,0,0,0};
    GetCartState(tmp_c);
    double angle=atan2(tar[6],tar[3]);
    std::cout<<angle<<std::endl;
    double tar_2[7]={0,0,0,cos(angle),0,0,sin(angle)};
    Eigen::Matrix4d tmp_m1=double2Mat(tmp_c);
    Eigen::Matrix4d tmp_m2=double2Mat(tar_2);
    Eigen::Matrix4d tmp_m3=double2Mat(end_pos);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m3*tmp_m2*tmp_m3.inverse();
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    double tmp_a2[6]={0,0,0,0,0,0};
    double2Angleaxis(tar2,tmp_a2);

    std::vector<double> t;
    for(int i=0;i<6;++i){
      t.emplace_back(tmp_a2[i]);
    }
    ur_ctrl->servoC(t,v,a);
  }
};
#endif
