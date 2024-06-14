#ifndef UTILS_BEHAVIOR_HPP_
#define UTILS_BEHAVIOR_HPP_
#include "geometry_msgs/WrenchStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <fstream>
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

//head to end, initial 0
class cyclevec{
public:
  std::vector<double> data;
  int cap=0;
  int ptr=0;
  cyclevec(int n=3){
    cap=n;
    data.clear();
    for(int i=0;i<cap;++i){
      data.emplace_back(0.0);
    }
    ptr=0;
  }
  ~cyclevec(){
    data.clear();
  }
  void push(double val){
    data[ptr]=val;
    ptr=(ptr+1)%cap;
  }
  int size(){
    return data.size();
  }
  double get(int pos){
    double res=data[(ptr+pos)%cap];
    return res;
  }
  void clear(){
    data.clear();
    for(int i=0;i<cap;++i){
      data.emplace_back(0.0);
    }
    ptr=0;
  }
};

class arm_com{
public:
  //init
  arm_com(ros::NodeHandle&n):n_(n),force_spin(6){}
  ~arm_com(){}
  ros::NodeHandle &n_;
  ros::Publisher cmd_pub=n_.advertise<std_msgs::String>("/ur_hardware_interface/script_command",1000);
  std::ofstream fft,fjt,fpo;

  //collect force-pos data
  ros::AsyncSpinner force_spin;
  ros::Subscriber ft_sub,jt_sub,po_sub;
  //filtered parameters
  std::vector<double> for_a,for_b,tor_a,tor_b;
  //6-axis filtered buffer queue
  cyclevec buf1,buf2,buf3,buf4,buf5,buf6;
  //6-axis zero point
  double ft_zero[6]={0.0};
  
  void force_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg_){
    pthread_mutex_lock(&m1);
    f_t[0]=msg_->wrench.force.x;
    f_t[1]=msg_->wrench.force.y;
    f_t[2]=msg_->wrench.force.z;
    f_t[3]=msg_->wrench.torque.x;
    f_t[4]=msg_->wrench.torque.y;
    f_t[5]=msg_->wrench.torque.z;
    fft<<msg_->header.stamp.nsec<<'\t'<<f_t[0]<<'\t'<<f_t[1]<<'\t'<<f_t[2]<<'\t'
      <<f_t[3]<<'\t'<<f_t[4]<<'\t'<<f_t[5]<<'\n';
    pthread_mutex_unlock(&m1);
  }
  void joint_callback(const sensor_msgs::JointState::ConstPtr& msg_){
    pthread_mutex_lock(&m2);
    j_t[0]=msg_->position[2];
    j_t[1]=msg_->position[1];
    j_t[2]=msg_->position[0];
    j_t[3]=msg_->position[3];
    j_t[4]=msg_->position[4];
    j_t[5]=msg_->position[5];
    fjt<<msg_->header.stamp.nsec<<'\t'<<j_t[0]<<'\t'<<j_t[1]<<'\t'<<j_t[2]<<'\t'
      <<j_t[3]<<'\t'<<j_t[4]<<'\t'<<j_t[5]<<'\n';
    pthread_mutex_unlock(&m2);
  }
  void posori_callback(const tf2_msgs::TFMessage::ConstPtr& msg_){
    for(int i=0;i<msg_->transforms.size();++i){
      if(msg_->transforms[i].child_frame_id=="tool0_controller"){
        pthread_mutex_lock(&m3);
        p_o[0]=msg_->transforms[i].transform.translation.x;
        p_o[1]=msg_->transforms[i].transform.translation.y;
        p_o[2]=msg_->transforms[i].transform.translation.z;
        p_o[3]=msg_->transforms[i].transform.rotation.w;
        p_o[4]=msg_->transforms[i].transform.rotation.x;
        p_o[5]=msg_->transforms[i].transform.rotation.y;
        p_o[6]=msg_->transforms[i].transform.rotation.z;
        fpo<<msg_->transforms[i].header.stamp.nsec<<'\t'
          <<p_o[0]<<'\t'<<p_o[1]<<'\t'<<p_o[2]<<'\t'<<p_o[3]<<'\t'<<p_o[4]<<'\t'<<p_o[5]<<'\n';
        pthread_mutex_unlock(&m3);
        break;
      }
    }
  }
  void start_coll(){
    ft_sub=n_.subscribe("/ethdaq_data", 1, &arm_com::force_callback, this);
    jt_sub=n_.subscribe("/joint_states", 1, &arm_com::joint_callback, this);
    po_sub=n_.subscribe("/tf", 1, &arm_com::posori_callback, this);
    fft.open("for_tor_collect.txt",std::ios::trunc);
    fjt.open("joint_collect.txt",std::ios::trunc);
    fpo.open("pos_ori_collect.txt",std::ios::trunc);
    pthread_mutex_init(&m1, NULL);
    pthread_mutex_init(&m2, NULL);
    pthread_mutex_init(&m3, NULL);
    force_spin.start();
  }
  void stop_coll(){
    ft_sub.~Subscriber();
    jt_sub.~Subscriber();
    po_sub.~Subscriber();
    fft.close();
    fjt.close();
    fpo.close();
    pthread_mutex_destroy(&m1);
    pthread_mutex_destroy(&m2);
    pthread_mutex_destroy(&m3);
    force_spin.stop();
  }
private:
  //sense realtime force pos data
  double p_o[7]={0.0};
  double j_t[6]={0.0};
  double f_t[6]={0.0};
  pthread_mutex_t m1,m2,m3;
  void f_t_filtered_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg_){
    pthread_mutex_lock(&m1);
    f_t[0]=msg_->wrench.force.x+ft_zero[0];
    f_t[1]=msg_->wrench.force.y+ft_zero[1];
    f_t[2]=msg_->wrench.force.z+ft_zero[2];
    f_t[3]=msg_->wrench.torque.x+ft_zero[3];
    f_t[4]=msg_->wrench.torque.y+ft_zero[4];
    f_t[5]=msg_->wrench.torque.z+ft_zero[5];
    buf1.push(f_t[0]-for_a[1]*buf1.get(2)-for_a[2]*buf1.get(1));
    f_t[0]=for_b[0]*buf1.get(2)+for_b[1]*buf1.get(1)+for_b[2]*buf1.get(0);
    
    buf2.push(f_t[1]-for_a[1]*buf2.get(2)-for_a[2]*buf2.get(1));
    f_t[1]=for_b[0]*buf2.get(2)+for_b[1]*buf2.get(1)+for_b[2]*buf2.get(0);
    
    buf3.push(f_t[2]-for_a[1]*buf3.get(2)-for_a[2]*buf3.get(1));
    f_t[2]=for_b[0]*buf3.get(2)+for_b[1]*buf3.get(1)+for_b[2]*buf3.get(0);
    
    buf4.push(f_t[3]-tor_a[1]*buf4.get(2)-tor_a[2]*buf4.get(1));
    f_t[3]=tor_b[0]*buf4.get(2)+tor_b[1]*buf4.get(1)+tor_b[2]*buf4.get(0);
    
    buf5.push(f_t[4]-tor_a[1]*buf5.get(2)-tor_a[2]*buf5.get(1));
    f_t[4]=tor_b[0]*buf5.get(2)+tor_b[1]*buf5.get(1)+tor_b[2]*buf5.get(0);
    
    buf6.push(f_t[5]-tor_a[1]*buf6.get(2)-tor_a[2]*buf6.get(1));
    f_t[5]=tor_b[0]*buf6.get(2)+tor_b[1]*buf6.get(1)+tor_b[2]*buf6.get(0);
    
    pthread_mutex_unlock(&m1);
  }
  void f_t_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg_){
    pthread_mutex_lock(&m1);
    f_t[0]=msg_->wrench.force.x+ft_zero[0];
    f_t[1]=msg_->wrench.force.y+ft_zero[1];
    f_t[2]=msg_->wrench.force.z+ft_zero[2];
    f_t[3]=msg_->wrench.torque.x+ft_zero[3];
    f_t[4]=msg_->wrench.torque.y+ft_zero[4];
    f_t[5]=msg_->wrench.torque.z+ft_zero[5];
    pthread_mutex_unlock(&m1);
  }
  void j_t_callback(const sensor_msgs::JointState::ConstPtr& msg_){
    pthread_mutex_lock(&m2);
    j_t[0]=msg_->position[2];
    j_t[1]=msg_->position[1];
    j_t[2]=msg_->position[0];
    j_t[3]=msg_->position[3];
    j_t[4]=msg_->position[4];
    j_t[5]=msg_->position[5];
    pthread_mutex_unlock(&m2);
  }
  void p_o_callback(const tf2_msgs::TFMessage::ConstPtr& msg_){
    for(int i=0;i<msg_->transforms.size();++i){
      if(msg_->transforms[i].child_frame_id=="tool0_controller"){
        pthread_mutex_lock(&m3);
        p_o[0]=msg_->transforms[i].transform.translation.x;
        p_o[1]=msg_->transforms[i].transform.translation.y;
        p_o[2]=msg_->transforms[i].transform.translation.z;
        p_o[3]=msg_->transforms[i].transform.rotation.w;
        p_o[4]=msg_->transforms[i].transform.rotation.x;
        p_o[5]=msg_->transforms[i].transform.rotation.y;
        p_o[6]=msg_->transforms[i].transform.rotation.z;
        pthread_mutex_unlock(&m3);
        break;
      }
    }
  }
public:
  void start_sense(){
    ft_sub=n_.subscribe("/ethdaq_data", 1, &arm_com::f_t_callback, this);
    jt_sub=n_.subscribe("/joint_states", 1, &arm_com::j_t_callback, this);
    po_sub=n_.subscribe("/tf", 1, &arm_com::p_o_callback, this);
    pthread_mutex_init(&m1, NULL);
    pthread_mutex_init(&m2, NULL);
    pthread_mutex_init(&m3, NULL);
    force_spin.start();
  }
  void stop_sense(){
    ft_sub.~Subscriber();
    jt_sub.~Subscriber();
    po_sub.~Subscriber();
    pthread_mutex_destroy(&m1);
    pthread_mutex_destroy(&m2);
    pthread_mutex_destroy(&m3);
    force_spin.stop();
  }
  void start_filtered_sense(){
    for_a.emplace_back(1.0);
    for_a.emplace_back(-1.95557824031504);
    for_a.emplace_back(0.956543676511203);
    for_b.emplace_back(0.000241359049042);
    for_b.emplace_back(0.000482718098084);
    for_b.emplace_back(0.000241359049042);
    tor_a.emplace_back(1.0);
    tor_a.emplace_back(-1.99555712434579);
    tor_a.emplace_back(0.995566972065975);
    tor_b.emplace_back(2.46e-06);
    tor_b.emplace_back(4.92e-06);
    tor_b.emplace_back(2.46e-06);
    
    buf1.clear();
    buf2.clear();
    buf3.clear();
    buf4.clear();
    buf5.clear();
    buf6.clear();
        
    ft_sub=n_.subscribe("/ethdaq_data", 1, &arm_com::f_t_filtered_callback, this);
    jt_sub=n_.subscribe("/joint_states", 1, &arm_com::j_t_callback, this);
    po_sub=n_.subscribe("/tf", 1, &arm_com::p_o_callback, this);
    pthread_mutex_init(&m1, NULL);
    pthread_mutex_init(&m2, NULL);
    pthread_mutex_init(&m3, NULL);
    force_spin.start();
  }
  void stop_filtered_sense(){
    for_a.clear();
    for_b.clear();
    tor_a.clear();
    tor_b.clear();
    buf1.clear();
    buf2.clear();
    buf3.clear();
    buf4.clear();
    buf5.clear();
    buf6.clear();
    
    ft_sub.~Subscriber();
    jt_sub.~Subscriber();
    po_sub.~Subscriber();
    pthread_mutex_destroy(&m1);
    pthread_mutex_destroy(&m2);
    pthread_mutex_destroy(&m3);
    force_spin.stop();
  }
  void start_zero(){
    pthread_mutex_lock(&m1);
    ft_zero[0]=-f_t[0];
    ft_zero[1]=-f_t[1];
    ft_zero[2]=-f_t[2];
    ft_zero[3]=-f_t[3];
    ft_zero[4]=-f_t[4];
    ft_zero[5]=-f_t[5];
    pthread_mutex_unlock(&m1);
  }
  void stop_zero(){
    pthread_mutex_lock(&m1);
    ft_zero[0]=0.0;
    ft_zero[1]=0.0;
    ft_zero[2]=0.0;
    ft_zero[3]=0.0;
    ft_zero[4]=0.0;
    ft_zero[5]=0.0;
    pthread_mutex_unlock(&m1);
  }  

  //Get function group
  void GetFT(double*res){
    pthread_mutex_lock(&m1);
    res[0]=f_t[0];
    res[1]=f_t[1];
    res[2]=f_t[2];
    res[3]=f_t[3];
    res[4]=f_t[4];
    res[5]=f_t[5];
    pthread_mutex_unlock(&m1);
  }
  void GetJointState(double*res){
    pthread_mutex_lock(&m2);
    res[0]=j_t[0];
    res[1]=j_t[1];
    res[2]=j_t[2];
    res[3]=j_t[3];
    res[4]=j_t[4];
    res[5]=j_t[5];
    pthread_mutex_unlock(&m2);
  }
  void GetCartState(double*res,double*end_pos){
    pthread_mutex_lock(&m3);
    res[0]=p_o[0];
    res[1]=p_o[1];
    res[2]=p_o[2];
    res[3]=p_o[3];
    res[4]=p_o[4];
    res[5]=p_o[5];
    res[6]=p_o[6];
    pthread_mutex_unlock(&m3);
    Eigen::Matrix4d pose=double2Mat(res);
    Eigen::Matrix4d trans=double2Mat(end_pos);
    pose=pose*trans;
    Mat2double(pose,res);
  }
  void GetCartState(double*res){
    pthread_mutex_lock(&m3);
    res[0]=p_o[0];
    res[1]=p_o[1];
    res[2]=p_o[2];
    res[3]=p_o[3];
    res[4]=p_o[4];
    res[5]=p_o[5];
    res[6]=p_o[6];
    pthread_mutex_unlock(&m3);
  }
  //Move function group
  void MoveJ(double*tar,double t){
    std_msgs::String str;
    std::string res="servoj([";
    for(int i=0;i<6;i++){
      res+=std::to_string(tar[i]);
      if(i!=5){
        res+=",";
      }
    }
    res+="], 0, 0, ";
    res+=std::to_string(t);
    res+=", ";
    res+=std::to_string(t);
    res+=")";
    str.data=res;
    cmd_pub.publish(str);
  }
  void MoveC(double*tar,double t){
    double pos_aa[6];
    double2Angleaxis(tar,pos_aa);
    std_msgs::String str;
    std::string res="servoj(get_inverse_kin(p[";
    for(int i=0;i<6;i++){
      res+=std::to_string(pos_aa[i]);
      if(i!=5){
        res+=",";
      }
    }
    res+="]), 0, 0, ";
    res+=std::to_string(t);
    res+=", ";
    res+=std::to_string(t);
    res+=")";
    str.data=res;
    cmd_pub.publish(str);
  }
  void MoveC(double*tar,double*end_pos,double t){
    double mid[7];
    Eigen::Matrix4d pose=double2Mat(tar);
    Eigen::Matrix4d trans=double2Mat(end_pos);
    pose=pose*trans.inverse();
    Mat2double(pose,mid);
    MoveC(mid,t);
  }
  void MoveRelaJ(double*tar,double t){
    double tmp[6]={0,0,0,0,0,0};
    GetJointState(tmp);
    for(int i=0;i<6;++i){
      tmp[i]+=tar[i];
    }
    MoveJ(tmp,t);
  }
  void MoveRelaC(double*tar,double t){
    double tmp_c[7]={0,0,0,1,0,0,0};
    GetCartState(tmp_c);
    Eigen::Matrix4d tmp_m1=double2Mat(tmp_c);
    Eigen::Matrix4d tmp_m2=double2Mat(tar);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m2;
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    MoveC(tar2,t);
  }
  void MoveRelaC(double*tar,double*end_pos,double t){
    double tmp_c[7]={0,0,0,1,0,0,0};
    GetCartState(tmp_c);
    Eigen::Matrix4d tmp_m1=double2Mat(tmp_c);
    Eigen::Matrix4d tmp_m2=double2Mat(tar);
    Eigen::Matrix4d tmp_m3=double2Mat(end_pos);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m3*tmp_m2;
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    MoveC(tar2,end_pos,t);
  }
  //move to a relative position based on a base
  void MoveRela2C(double*base,double*tar,double t){
    Eigen::Matrix4d tmp_m1=double2Mat(base);
    Eigen::Matrix4d tmp_m2=double2Mat(tar);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m2;
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    MoveC(tar2,t);
  }
  void MoveRela2C(double*base,double*tar,double*end_pos,double t){
    Eigen::Matrix4d tmp_m1=double2Mat(base);
    Eigen::Matrix4d tmp_m2=double2Mat(tar);
    Eigen::Matrix4d tmp_m3=double2Mat(end_pos);
    Eigen::Matrix4d tmp_m4=tmp_m1*tmp_m3*tmp_m2;
    double tar2[7]={0,0,0,1,0,0,0};
    Mat2double(tmp_m4,tar2);
    MoveC(tar2,end_pos,t);
  }
};
#endif
