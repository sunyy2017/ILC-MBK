#ifndef FORCE_HPP_
#define FORCE_HPP_
#include <stdio.h>

class Admit_ctrl{
  double err_F;
  double xd,xd_t,xd_tt;
  double x,x_t,x_tt;
  double x_last;
  double x_t_last;
  double x_tt_last;
  double M,B,K;
  double T;
  double Same_dir;
public:
  Admit_ctrl(){}
  ~Admit_ctrl(){}
  void Admit_init(double m,double b,double k,double t,double same_dir,double start_x){
  //same_dir means the delta force and the delta x is in the same direction, 1 or -1
    M=m;
    B=b;
    K=k;
    T=t;
    err_F=0;
    xd=0;
    xd_t=0;
    xd_tt=0;
    x=0;
    x_t=0;
    x_tt=0;
    x_last=start_x;
    x_t_last=0;
    x_tt_last=0;
    Same_dir=same_dir;
  }
  double Admit(double force, double xd_in, double setf){
    xd=xd_in;
    err_F=force-setf;
    /*std::cout<<"[FORCE CONTROL]error of force:"<<err_F<<std::endl;
    std::cout<<"[FORCE CONTROL]error of x:"<<xd-x_last<<std::endl;
    std::cout<<"[FORCE CONTROL]error of v:"<<xd_t-x_t_last<<std::endl;*/
    x_tt=(err_F*Same_dir+ B*(xd_t - x_t_last) + K*(xd - x_last))/M + xd_tt;
    x_t = x_t_last + x_tt * T;
    x = x_last + x_t*T;
    //x_last = xd + x - x_last;
    x_last = x;
    x_t_last = x_t;
    return x_last;
  }
};

class Admit_kchange_ctrl{
  double err_F,err_F_t;
  double xd,xd_last,xd_t,xd_tt;
  double x,x_t,x_tt;
  double x_last;
  double x_t_last;
  double x_tt_last;
  double M,B,K,K_T,Kt;
  double T;
  double Same_dir;
public:
  Admit_kchange_ctrl(){}
  ~Admit_kchange_ctrl(){}
  void Admit_kchange_init(double m,double b,double k,double k_t,double t,double same_dir,double start_x){
    err_F=0;
    err_F_t=0;
    xd = 0;
    xd_last = 0;
    xd_t = 0;
    xd_tt = 0;
    x=0;
    x_t=0;
    x_tt=0;
    x_last=start_x;
    x_t_last = 0;
    x_tt_last = 0;
    M = m;
    B = b;
    K = k;
    K_T = k_t;
    Kt = 0;
    T = t;
    Same_dir=same_dir;
  }
  double Admit_kchange(double force,double xd_in, double setf){
    xd_last = xd;
    xd = xd_in;
    err_F_t=(force-setf-err_F)/T;
    err_F = force- setf;
    Kt=(K*err_F*Same_dir+K_T*err_F_t*Same_dir)/(xd-x_last);
    x_tt = (err_F*Same_dir+ B*(xd_t - x_t_last) + Kt*(xd - x_last))/M + xd_tt;
    x_t = x_t_last + x_tt * T;
    x = x_last + x_t*T;
    x_last = x;
    x_t_last = x_t;
    return x_last;
  }
};

#endif
