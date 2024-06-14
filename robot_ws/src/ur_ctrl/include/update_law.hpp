#ifndef UPDATE_HPP_
#define UPDATE_HPP_
#include <Eigen/Dense>
#include <vector>
#include <cmath>

Eigen::MatrixXd pinv(Eigen::MatrixXd A)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  double  pinvtoler = 1.e-8;
  int row = A.rows();
  int col = A.cols();
  int k = std::min(row, col);
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
  Eigen::MatrixXd singularValues_inv = svd.singularValues();
  Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
  for (long i = 0; i<k; ++i) {
    if (singularValues_inv(i) > pinvtoler)
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    else singularValues_inv(i) = 0;
  }
  for (long i = 0; i < k; ++i)
  {
    singularValues_inv_mat(i, i) = singularValues_inv(i);
  }
  X = (svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());

  return X;
}

Eigen::MatrixXd mat_sat(const Eigen::MatrixXd&src,double lower,double upper){
  int rows=src.rows();
  int cols=src.cols();
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> res;
  res.resize(rows,cols);
  for(int i=0;i<rows;++i){
    for(int j=0;j<cols;++j){
      if(src(i,j)<lower)  res(i,j)=lower;
      else if(src(i,j)>upper)  res(i,j)=upper;
      else  res(i,j)=src(i,j);
    }
  }
  return res;
}

/*
input y,y_d:3*t
input x:1*t, only delta_x is considered
input kk:1*t
input eta:1*1
input mu:1*1
middle C:3*2*t
output u_delta:2*3*t
output kk_o:1*t
*/

void inv_admit_fullu_estc(std::vector<Eigen::Matrix<double,2,3>,Eigen::aligned_allocator<Eigen::Matrix<double,2,3>>>&u_delta,std::vector<double>&kk_o,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>&y,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>&y_d,std::vector<double>&x,std::vector<double>&kk,double eta,double mu){
  int len=y.size();
  u_delta.clear();
  kk_o.clear();
  std::vector<Eigen::Matrix<double,3,2>,Eigen::aligned_allocator<Eigen::Matrix<double,3,2>>>C;
  for(int i=0;i<len;++i){
    Eigen::Matrix<double,3,2>Ct;
    Ct<<-1.0,0.0,0.0,-1.0,0.0,-kk[i];
    C.emplace_back(Ct);
  }
  for(int i=0;i<len-1;++i){
    kk_o.emplace_back(kk[i]-eta*(y[i](2)+kk[i]*(x[i]-y_d[i](1)))*(x[i]-y_d[i](1))/(mu+pow(x[i]-y_d[i](1),2)));
    u_delta.emplace_back(pinv(C[i])*(y_d[i+1]-y[i+1])*pinv(y_d[i]-y[i]));
  }
  int i=len-1;
  kk_o.emplace_back(kk[i]-eta*(y[i](2)+kk[i]*(x[i]-y_d[i](1)))*(x[i]-y_d[i](1))/(mu+pow(x[i]-y_d[i](1),2)));
  u_delta.emplace_back(u_delta.back());
}

//only for my ILC with negative m
//function in use outside
void update_fullu_estc(std::vector<double>&bm,std::vector<double>&km,std::vector<double>&m,std::vector<double>&kk,std::vector<double>&v_out,std::vector<double>&x_out,std::vector<double>&f_out,std::vector<double>&v_ref,std::vector<double>&x_ref,std::vector<double>&f_ref,double t_delta,double alpha,double eta,double mu,double lower,double upper){
  std::vector<Eigen::Matrix<double,2,3>,Eigen::aligned_allocator<Eigen::Matrix<double,2,3>>> u;
  std::vector<Eigen::Matrix<double,2,3>,Eigen::aligned_allocator<Eigen::Matrix<double,2,3>>> u_delta;
  std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> y;
  std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>y_d;
  std::vector<double> kk_o;
  std::vector<double> x;

  int len=v_out.size();
  for(int i=0;i<len;++i){
    Eigen::Matrix<double,2,3> ut;
    ut<<bm[i]/t_delta, (km[i]-1.0)/t_delta, m[i]/t_delta, bm[i], km[i], m[i];
    u.emplace_back(ut);
    Eigen::Vector3d yt;
    yt<<v_out[i], x_out[i], f_out[i];
    y.emplace_back(yt);
    Eigen::Vector3d ydt;
    ydt<<v_ref[i], x_ref[i], f_ref[i];
    y_d.emplace_back(ydt);
    x.emplace_back(x_ref[i]-x_out[i]);
  }
  inv_admit_fullu_estc(u_delta,kk_o,y,y_d,x,kk,eta,mu);
  for(int i=0;i<len;++i){
    u[i]=mat_sat(u[i]+alpha*u_delta[i],lower,upper);
    bm[i]=u[i](1,0);
    km[i]=u[i](1,1);
    m[i]=u[i](1,2);
    kk[i]=kk_o[i];
  }
}

//only for my ILC with negative m
//function in use outside
void update_fullu_estc_eso(std::vector<double>&bm,std::vector<double>&km,std::vector<double>&m,std::vector<double>&kk,std::vector<double>&err,std::vector<double>&v_out,std::vector<double>&x_out,std::vector<double>&f_out,std::vector<double>&v_ref,std::vector<double>&x_ref,std::vector<double>&f_ref,double t_delta,double alpha1,double alpha2,double eta,double mu,double lower,double upper){
  std::vector<Eigen::Matrix<double,2,3>,Eigen::aligned_allocator<Eigen::Matrix<double,2,3>>> u;
  std::vector<Eigen::Matrix<double,2,3>,Eigen::aligned_allocator<Eigen::Matrix<double,2,3>>> u_delta;
  std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> y;
  std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>y_d;
  std::vector<double> kk_o;
  std::vector<double> x;

  int len=v_out.size();
  for(int i=0;i<len;++i){
    Eigen::Matrix<double,2,3> ut;
    ut<<bm[i]/t_delta, (km[i]-1.0)/t_delta, m[i]/t_delta, bm[i], km[i], m[i];
    u.emplace_back(ut);
    Eigen::Vector3d yt;
    yt<<v_out[i], x_out[i], f_out[i];
    y.emplace_back(yt);
    Eigen::Vector3d ydt;
    ydt<<v_ref[i], x_ref[i], f_ref[i];
    y_d.emplace_back(ydt);
    x.emplace_back(x_ref[i]-x_out[i]);
  }
  inv_admit_fullu_estc(u_delta,kk_o,y,y_d,x,kk,eta,mu);
  for(int i=0;i<len;++i){
    u[i]=mat_sat(u[i]+alpha1*u_delta[i],lower,upper);
    bm[i]=u[i](1,0);
    km[i]=u[i](1,1);
    m[i]=u[i](1,2);
    kk[i]=kk_o[i];
    err[i]=err[i]+alpha2*(f_out[i]-err[i]-f_ref[i]);
  }
}

/*
for zhongsheng hou's origin CFDL
input phi:3*3*t
input u,u_last:3*t
input y,y_last,y_d:3*t
input rho,mu,eta,lambda:1*1
input low/up_phi:1*1
procedure:
1.phi(k+1)=phi(k)+u(k)+u_l(k)+y(k)+y_l(k)
2.u(k+1)=u(k)+theta(k+1)+y(k)+y_d(k)
*/

void CFDL_ILC(std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d>>&phi_o,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>&u_o,std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d>>&phi,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>&u,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>&u_l,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>&y,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>&y_l,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>&y_d,double rho,double mu,double eta,double lambda,double low_phi,double up_phi){
  int len=phi.size();
  phi_o.clear();
  u_o.clear();
  for(int i=0;i<len-1;++i){
    phi_o.emplace_back(phi[i]+eta*((y[i+1]-y_l[i+1])-phi[i]*(u[i]-u_l[i]))*(u[i]-u_l[i]).transpose()
                      /(mu+pow((u[i]-u_l[i]).norm(),2)));
    phi_o[i]=mat_sat(phi_o[i],low_phi,up_phi);
    u_o.emplace_back(u[i]+rho*(phi_o[i]).transpose()*(y_d[i+1]-y[i+1])/(lambda+pow(phi_o[i].norm(),2)));
  }
  int i=len-1;
  phi_o.emplace_back(phi[i]+eta*((y[i]-y_l[i])-phi[i]*(u[i]-u_l[i]))*(u[i]-u_l[i]).transpose()
                    /(mu+pow((u[i]-u_l[i]).norm(),2)));
  phi_o[i]=mat_sat(phi_o[i],low_phi,up_phi);
  u_o.emplace_back(u[i]+rho*(phi_o[i]).transpose()*(y_d[i]-y[i])/(lambda+pow(phi_o[i].norm(),2)));
}

//only for CFDL-ILC with positive m
//function not complete
void update_CFDL_ILC(std::vector<double>&km,std::vector<double>&bm,std::vector<double>&m,std::vector<double>&kk,std::vector<double>&v_out,std::vector<double>&x_out,std::vector<double>&f_out,std::vector<double>&v_ref,std::vector<double>&x_ref,std::vector<double>&f_ref,double t_delta,double alpha,double eta,double mu,double lower,double upper){
  std::vector<Eigen::Matrix<double,2,3>,Eigen::aligned_allocator<Eigen::Matrix<double,2,3>>> u;
  std::vector<Eigen::Matrix<double,2,3>,Eigen::aligned_allocator<Eigen::Matrix<double,2,3>>> u_delta;
  std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> y;
  std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>y_d;
  std::vector<double> kk_o;
  std::vector<double> x;

  int len=v_out.size();
  for(int i=0;i<len;++i){
    Eigen::Matrix<double,2,3> ut;
    ut<<1.0-bm[i]*t_delta, -km[i]*t_delta, -m[i]*t_delta, t_delta-bm[i]*t_delta*t_delta, 1.0-km[i]*t_delta*t_delta, -m[i]*t_delta;
    u.emplace_back(ut);
    Eigen::Vector3d yt;
    yt<<v_out[i], x_out[i], f_out[i];
    y.emplace_back(yt);
    Eigen::Vector3d ydt;
    ydt<<v_ref[i], x_ref[i], f_ref[i];
    y_d.emplace_back(ydt);
    x.emplace_back(x_ref[i]-x_out[i]);
  }
  inv_admit_fullu_estc(u_delta,kk_o,y,y_d,x,kk,eta,mu);
  for(int i=0;i<len;++i){
    u[i]=mat_sat(u[i]+alpha*u_delta[i],lower,upper);
    km[i]=abs(1.0-u[i](2,2))/t_delta/t_delta;
    bm[i]=abs(t_delta-u[i](2,1))/t_delta/t_delta;
    m[i]=abs(u[i](2,3))/t_delta/t_delta;
    kk[i]=kk_o[i];
  }
}

#endif
