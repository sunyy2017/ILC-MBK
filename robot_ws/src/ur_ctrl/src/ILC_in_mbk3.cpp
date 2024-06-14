//ft bias->0force->admittance->up->save data->calc data->ft bias
//ILC_in_mbk3
//save_res in file cost.txt(append), x_f.txt(rewrite)
#include "utils.hpp"
#include "update_law.hpp"
#include "button_data.hpp"
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
//double push_end[7]={0.065,-0.0375,0.0,1.0,0.0,0.0,0.0};
//double knob_end[7]={0.0,0.075,0,1.0,0.0,0.0,0.0};
//double emerg_end[7]={-0.065,-0.0375,0.0,1.0,0.0,0.0,0.0};
double knob_end[7]={-0.065,0.0375,0.0,1.0,0.0,0.0,0.0};
double emerg_end[7]={0.0,-0.075,0,1.0,0.0,0.0,0.0};
double push_end[7]={0.065,0.0375,0.0,1.0,0.0,0.0,0.0};
double len=0.075;
int choice=1;
//bm=t-bmtt,km=1-kmtt,m=-mtt
std::vector<double> x_state,v_state,x_out,v_out,f_out,bm,km,m,x_ref,v_ref,f_ref,kk_vec,err;
std::vector<double> push_km,push_bm,push_m,
                    knob_km,knob_bm,knob_m,
                    emerg_km,emerg_bm,emerg_m,
                    emerg_push_km,emerg_push_bm,emerg_push_m;
double kk=0.0;
double alpha=0.05;
//it needs 50ms to accelerate
double t_delta=0.05;
int iters=1;
double a_0=0.1;
double v_0=0.05;
double a_l=0.1;//1
double v_l=0.01;//0.005;
double a_j=1.0;
double v_j=0.1;
double low=-0.1,up=0.1;
double eta=1.0;
double mu=1.0;

void calc_v(std::vector<double>&x_t,std::vector<double>&v_t,double t_t){
  v_t.clear();
  int n=x_t.size();
  for(int i=0;i<n;++i){
    if(i==0)
      v_t.emplace_back(x_t[i]/t_t);
    else
      v_t.emplace_back((x_t[i]-x_t[i-1])/t_t);
  }
}

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
  if(choice==3)
    pos_ori_t[2]=-0.0025;
  else if(choice!=1)
    pos_ori_t[2]=-0.0005;
  else if(choice==1)
    pos_ori_t[2]=0.0015;
  arm->MoveRelaLineC(pos_ori_t,a_l,v_l);
  ros::Duration(0.75).sleep();
  return 1;
}

int admit_ctrl_press(){
  //init basic data
  ros::Rate r(0.2/t_delta);
  double pos_ori_t[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  double pos_ori_0[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  double pos_ori_m[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  r.sleep();
  arm->GetFT(fts);
  arm->GetCartState(pos_ori_0);
  double vs=0.0,xs=0.0,vo=0.0,xo=0.0;
  double fo=-fts[2];
  x_state.emplace_back(xs);
  v_state.emplace_back(vs);
  x_out.emplace_back(xo);
  v_out.emplace_back(vo);
  f_out.emplace_back(fo);

  for(int i=0;i<(int)x_ref.size()-1;i++){
    xs=bm[i]*(v_ref[i]-vo)+km[i]*(x_ref[i]-xo)+m[i]*(f_ref[i]-fo);

    double v_o=abs((x_ref[i+1]-xs-xo)/t_delta);
    xo=x_ref[i+1]-xs;
    if(xo<0.0)  xo=0.0;
    else if(xo>0.006) xo=0.006;
    //real temporary
    pos_ori_m[2]=xo;
    std::cout<<"target pos: "<<xo<<std::endl;
    arm->MoveRela2LineC(pos_ori_0,pos_ori_m,push_end,a_l,v_o);
    r.sleep();
    arm->GetFT(fts);
    arm->GetCartState(pos_ori_t);
    fo=-fts[2];
    std::cout<<"out force: "<<fo<<std::endl;
    xo=DeltaDist(pos_ori_0,pos_ori_t);
    vo=(xo-x_out.back())/t_delta;
    
    v_state.emplace_back(vs);
    x_state.emplace_back(xs);
    v_out.emplace_back(vo);
    x_out.emplace_back(xo);
    f_out.emplace_back(fo);
  }
  return 1;
}

int admit_ctrl_knob(){
  //init basic data
  ros::Rate r(0.2/t_delta);
  double pos_ori_t[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  double pos_ori_0[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  double pos_ori_m[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  r.sleep();
  arm->GetFT(fts);
  arm->GetCartState(pos_ori_0);
  double vs=0.0,xs=0.0,vo=0.0,xo=0.0;
  double fo=-fts[3];
  x_state.emplace_back(xs);
  v_state.emplace_back(vs);
  x_out.emplace_back(xo);
  v_out.emplace_back(vo);
  f_out.emplace_back(fo);

  for(int i=0;i<(int)x_ref.size()-1;i++){
    xs=bm[i]*(v_ref[i]-vo)+km[i]*(x_ref[i]-xo)+m[i]*(f_ref[i]-fo);

    double v_o=abs((x_ref[i+1]-xs-xo)/t_delta);//*len
    xo=x_ref[i+1]-xs;
    if(xo<0.0)  xo=0.0;
    else if(xo>1.6) xo=1.6;
    //real temporary
    std::cout<<"target pos: "<<xo<<std::endl;
    
    pos_ori_m[3]=cos(xo/2);
    pos_ori_m[6]=sin(xo/2);
    arm->MoveRela2LineC(pos_ori_0,pos_ori_m,knob_end,a_l,v_o);
    r.sleep();
    arm->GetFT(fts);
    arm->GetCartState(pos_ori_t);
    fo=-fts[3];
    std::cout<<"out force: "<<fo<<std::endl;
    xo=DeltaQuat(pos_ori_0,pos_ori_t);
    vo=(xo-x_out.back())/t_delta;
    v_state.emplace_back(vs);
    x_state.emplace_back(xs);
    v_out.emplace_back(vo);
    x_out.emplace_back(xo);
    f_out.emplace_back(fo);
  }
  return 1;
}

int admit_ctrl_emerg(){
  //init basic data
  ros::Rate r(0.2/t_delta);
  double pos_ori_t[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  double pos_ori_0[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  double pos_ori_m[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  r.sleep();
  arm->GetFT(fts);
  arm->GetCartState(pos_ori_0);
  double vs=0.0,xs=0.0,vo=0.0,xo=0.0;
  double fo=-fts[5];
  x_state.emplace_back(xs);
  v_state.emplace_back(vs);
  x_out.emplace_back(xo);
  v_out.emplace_back(vo);
  f_out.emplace_back(fo);

  for(int i=0;i<(int)x_ref.size()-1;i++){
    xs=bm[i]*(v_ref[i]-vo)+km[i]*(x_ref[i]-xo)+m[i]*(f_ref[i]-fo);

    double v_o=abs((x_ref[i+1]-xs-xo)/t_delta);
    xo=x_ref[i+1]-xs;
    if(xo<0.0)  xo=0.0;
    else if(xo>1.5) xo=1.5;
    //real temporary
    std::cout<<"target pos: "<<xo<<std::endl;
    
    pos_ori_m[3]=cos(xo/2);
    pos_ori_m[6]=sin(xo/2);
    arm->MoveRela2LineC(pos_ori_0,pos_ori_m,emerg_end,a_l,v_o);
    r.sleep();
    arm->GetFT(fts);
    arm->GetCartState(pos_ori_t);
    fo=-fts[5];
    std::cout<<"out force: "<<fo<<std::endl;
    xo=DeltaQuat(pos_ori_0,pos_ori_t);
    vo=(xo-x_out.back())/t_delta;
    v_state.emplace_back(vs);
    x_state.emplace_back(xs);
    v_out.emplace_back(vo);
    x_out.emplace_back(xo);
    f_out.emplace_back(fo);
  }
  return 1;
}

int admit_ctrl_emerg_press(){
  //init basic data
  ros::Rate r(0.2/t_delta);
  double pos_ori_t[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  double pos_ori_0[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  double pos_ori_m[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  r.sleep();
  arm->GetFT(fts);
  arm->GetCartState(pos_ori_0);
  double vs=0.0,xs=0.0,vo=0.0,xo=0.0;
  double fo=-fts[2];
  x_state.emplace_back(xs);
  v_state.emplace_back(vs);
  x_out.emplace_back(xo);
  v_out.emplace_back(vo);
  f_out.emplace_back(fo);

  for(int i=0;i<(int)x_ref.size()-1;i++){
    xs=bm[i]*(v_ref[i]-vo)+km[i]*(x_ref[i]-xo)+m[i]*(f_ref[i]-fo);

    double v_o=abs((x_ref[i+1]-xs-xo)/t_delta);
    xo=x_ref[i+1]-xs;
    if(xo<0.0)  xo=0.0;
    else if(xo>0.0075) xo=0.0075;
    //real temporary
    pos_ori_m[2]=xo;
    std::cout<<"target pos: "<<xo<<std::endl;
    arm->MoveRela2LineC(pos_ori_0,pos_ori_m,emerg_end,a_l,v_o);
    r.sleep();
    arm->GetFT(fts);
    arm->GetCartState(pos_ori_t);
    fo=-fts[2];
    std::cout<<"out force: "<<fo<<std::endl;
    xo=DeltaDist(pos_ori_0,pos_ori_t);
    vo=(xo-x_out.back())/t_delta;
    
    v_state.emplace_back(vs);
    x_state.emplace_back(xs);
    v_out.emplace_back(vo);
    x_out.emplace_back(xo);
    f_out.emplace_back(fo);
  }
  return 1;
}

int move2close_out(){
  double pos_ori_t[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
  if(choice<3)  pos_ori_t[2]=-0.015;
  else  pos_ori_t[2]=-0.0025;
  arm->MoveRelaLineC(pos_ori_t,a_l,v_l);
  ros::Duration(3).sleep();
  return 1;
}

void get_cost_in_mbk(double& x_cost,double& f_cost){
  int n=x_out.size();
  x_cost=0.0;
  f_cost=0.0;
  for(int i=0;i<n;++i){
    x_cost+=pow(x_out[i]-x_ref[i],2);
    f_cost+=pow(f_out[i]-f_ref[i],2);
  }
  x_cost/=n;
  f_cost/=n;
  x_cost=sqrt(x_cost);
  f_cost=sqrt(f_cost);
}

double my_sat(double src,double lower,double upper){
  double res=src;
  if(res>upper)  src=upper;
  else if(res<lower)  src=lower;
  return res;
}

int update_in_mbk(){
  update_fullu_estc(bm,km,m,kk_vec,v_out,x_out,f_out,v_ref,x_ref,f_ref,t_delta,alpha,eta,mu,low,up);
  return 1;
}

int save_res(double x_cost,double f_cost){
  std::ofstream fs1,fs2,fs3,fs4,fs5;
  fs1.open("x_f.txt",std::ios::trunc);
  fs2.open("bmkmm.txt",std::ios::trunc);
  fs3.open("cost.txt",std::ios::out|std::ios::app);
  fs4.open("x_f_out.txt",std::ios::trunc);
  fs5.open("estimate_C.txt",std::ios::trunc);
  int n=x_ref.size();
  for(int i=0;i<n;++i){
    fs1<<x_ref[i]<<'\t'<<f_ref[i]<<std::endl;
    double km2=(1-km[i])/t_delta/t_delta;
    double bm2=(t_delta-bm[i])/t_delta/t_delta;
    double m2=(-m[i])/t_delta/t_delta;
    fs2<<bm2<<'\t'<<km2<<'\t'<<m2<<'\n';
    fs4<<x_out[i]<<'\t'<<f_out[i]<<std::endl;
    fs5<<kk_vec[i]<<std::endl;
  }
  fs3<<x_cost<<'\t'<<f_cost<<std::endl;
  fs1.close();
  fs2.close();
  fs3.close();
  fs4.close();
  fs5.close();
  
  v_state.clear();
  x_state.clear();
  v_out.clear();
  x_out.clear();
  f_out.clear();
  return 1;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ur_ILC_in_mbk3");

  //ros nodehandle init
  n=new ros::NodeHandle();
  //arm driver init
  arm=new arm_com(*n);
  
  ROS_INFO("Init ok");

  std::cout<<"input numbers to decide the object\n0 for emergency press, 1 for emergency, 2 for press, 3 for knob, clock-wise\n";
  std::cin>>choice;

  int num;
  std::cout<<"input numbers to decide which iteration to be used\n";
  std::cin>>num;
  //get datas in file
  std::ifstream f1,f2,f3,f4,f5,f6,f7,f8;
  double inv=1.0;
  f1.open("trackdata/push_bmkmm"+std::to_string(num)+".txt",std::ios::in);
  f2.open("trackdata/knob_bmkmm"+std::to_string(num)+".txt",std::ios::in);
  f3.open("trackdata/emerg_bmkmm"+std::to_string(num)+".txt",std::ios::in);
  f4.open("trackdata/emerg_push_bmkmm"+std::to_string(num)+".txt",std::ios::in);
  
  //change initial value
  double bm_t,km_t,m_t;
  while(f1>>bm_t>>km_t>>m_t){
    push_bm.emplace_back(bm_t);
    push_km.emplace_back(km_t);
    push_m.emplace_back(m_t);
  }
  
  while(f2>>bm_t>>km_t>>m_t){
    knob_bm.emplace_back(bm_t);
    knob_km.emplace_back(km_t*1.2);
    knob_m.emplace_back(m_t);
  }
  
  while(f3>>bm_t>>km_t>>m_t){
    emerg_bm.emplace_back(bm_t);
    emerg_km.emplace_back(km_t);
    emerg_m.emplace_back(m_t);
  }
  
  while(f4>>bm_t>>km_t>>m_t){
    emerg_push_bm.emplace_back(bm_t);
    emerg_push_km.emplace_back(km_t*1.0);
    emerg_push_m.emplace_back(m_t);
  }
  
  f1.close();
  f2.close();
  f3.close();
  f4.close();
  
  arm->start_filtered_sense();
  //arm->start_sense();
  

  double orig[7]={0};
  double ori2[7]={0};
  arm->GetCartState(orig);
  
  //refine datas
  calc_v(push_x_data,push_v_data,t_delta);
  calc_v(push_x_ilc,push_v_ilc,t_delta);
  calc_v(knob_x_data,knob_v_data,t_delta);
  calc_v(knob_x_ilc,knob_v_ilc,t_delta);
  calc_v(emerg_x_data,emerg_v_data,t_delta);
  calc_v(emerg_x_ilc,emerg_v_ilc,t_delta);
  calc_v(emerg_push_x_data,emerg_push_v_data,t_delta);
  calc_v(emerg_push_x_ilc,emerg_push_v_ilc,t_delta);
  
  for(int i=0;i<120;++i){
    emerg_x_data.pop_back();
    emerg_v_data.pop_back();
    emerg_f_data.pop_back();
    emerg_x_ilc.pop_back();
    emerg_v_ilc.pop_back();
    emerg_f_ilc.pop_back();
  }
  
  if(choice==0){
    for(int i=0;i<(int)emerg_push_x_data.size();i++){
      bm.emplace_back(t_delta-emerg_push_bm[i]*t_delta*t_delta);
      km.emplace_back(1.0-emerg_push_km[i]*t_delta*t_delta);
      m.emplace_back(inv*emerg_push_m[i]*t_delta*t_delta);
      x_ref.emplace_back(emerg_push_x_data[i]);
      v_ref.emplace_back(emerg_push_v_data[i]);
      f_ref.emplace_back(emerg_push_f_data[i]);
      kk+=(emerg_push_f_data[i]/emerg_push_x_data[i])/emerg_push_x_data.size();
    }
    for(int i=0;i<(int)emerg_push_x_data.size();i++){
      kk_vec.emplace_back(kk);
    }
    for(int i=0;i<iters;++i){
      std::cout<<i<<std::endl;
      
      ros::Duration(1).sleep();
      arm->start_zero();
      move0force();
      arm->stop_zero();
      ros::Duration(1).sleep();
      arm->start_zero();
      //pos_ctrl_emerg_press();
      admit_ctrl_emerg_press();
      double x_cost,f_cost;
      get_cost_in_mbk(x_cost,f_cost);
      update_in_mbk();
      save_res(x_cost,f_cost);
      arm->stop_zero();
      
      arm->GetCartState(ori2);
      double d_quat=DeltaQuat(orig,ori2);
      while(d_quat<1.0){
        std::cout<<"Delta Quat: "<<d_quat<<std::endl;
        double p_o[7]={0.0,0.0,0.0,cos(0.1),0.0,0.0,sin(0.1)};
        arm->MoveRelaLineC(p_o,emerg_end,a_l,v_l);
        ros::Duration(1).sleep();
        arm->GetCartState(ori2);
        d_quat=DeltaQuat(orig,ori2);
      }
      move2close_out();
      arm->MoveJointC(orig,a_j,v_j);
    }
  }
  else if(choice==1){
    for(int i=0;i<(int)emerg_x_data.size();i++){
      bm.emplace_back(t_delta-emerg_bm[i]*t_delta*t_delta);
      km.emplace_back(1.0-emerg_km[i]*t_delta*t_delta);
      m.emplace_back(inv*emerg_m[i]*t_delta*t_delta);
      x_ref.emplace_back(emerg_x_data[i]);
      v_ref.emplace_back(emerg_v_data[i]);
      f_ref.emplace_back(emerg_f_data[i]);
      kk+=(emerg_f_data[i]/emerg_x_data[i])/emerg_x_data.size();
    }
    for(int i=0;i<(int)emerg_x_data.size();i++){
      kk_vec.emplace_back(kk);
    }
    for(int i=0;i<iters;++i){
      std::cout<<i<<std::endl;
      
      /*
      ros::Duration(1).sleep();
      arm->start_zero();
      ros::Duration(1).sleep();
      move0force();
      double p_o[7]={0.0,0.0,0.004,1.0,0.0,0.0,0.0};
      arm->MoveRelaLineC(p_o,emerg_end,a_l,v_l);
      ros::Duration(1.5).sleep();
      move2close_out();
      arm->MoveJointC(orig,a_j,v_j);
      arm->stop_zero();
      */
      ros::Duration(1).sleep();
      arm->start_zero();
      move0force();
      arm->stop_zero();
      ros::Duration(1).sleep();
      arm->start_zero();
      //pos_ctrl_emerg();
      admit_ctrl_emerg();
      
      arm->GetCartState(ori2);
      double d_quat=DeltaQuat(orig,ori2);
      /*while(d_quat<1.0){
        std::cout<<"Delta Quat: "<<d_quat<<std::endl;
        double p_o[7]={0.0,0.0,0.0,cos(0.1),0.0,0.0,sin(0.1)};
        arm->MoveRelaLineC(p_o,emerg_end,a_l,v_l);
        ros::Duration(1).sleep();
        arm->GetCartState(ori2);
        d_quat=DeltaQuat(orig,ori2);
      }*/
      
      move2close_out();
      double x_cost,f_cost;
      get_cost_in_mbk(x_cost,f_cost);
      update_in_mbk();
      save_res(x_cost,f_cost);
      arm->stop_zero();
      
      arm->MoveJointC(orig,a_j,v_j);
    }
  }
  else if(choice==2){
    for(int i=0;i<(int)push_x_data.size();i++){
      bm.emplace_back(t_delta-push_bm[i]*t_delta*t_delta);
      km.emplace_back(1.0-push_km[i]*t_delta*t_delta);
      m.emplace_back(inv*push_m[i]*t_delta*t_delta);
      x_ref.emplace_back(push_x_data[i]);
      v_ref.emplace_back(push_v_data[i]);
      f_ref.emplace_back(push_f_data[i]);
      kk+=(push_f_data[i]/push_x_data[i])/push_x_data.size();
    }
    for(int i=0;i<(int)push_x_data.size();i++){
      kk_vec.emplace_back(kk);
    }
    for(int i=0;i<iters;++i){
      std::cout<<i<<std::endl;
      
      ros::Duration(1).sleep();
      arm->start_zero();
      move0force();
      arm->stop_zero();
      ros::Duration(1).sleep();
      arm->start_zero();
      //pos_ctrl_press();
      admit_ctrl_press();
      move2close_out();
      double x_cost,f_cost;
      get_cost_in_mbk(x_cost,f_cost);
      update_in_mbk();
      save_res(x_cost,f_cost);
      arm->stop_zero();
      
      arm->MoveJointC(orig,a_j,v_j);
    }
  }
  else if(choice==3){
    for(int i=0;i<(int)knob_x_data.size();i++){
      bm.emplace_back(t_delta-knob_bm[i]*t_delta*t_delta);
      km.emplace_back(1.0-knob_km[i]*t_delta*t_delta);
      m.emplace_back(inv*knob_m[i]*t_delta*t_delta);
      x_ref.emplace_back(knob_x_data[i]);
      v_ref.emplace_back(knob_v_data[i]);
      f_ref.emplace_back(knob_f_data[i]);
      kk+=(knob_f_data[i]/knob_x_data[i])/knob_x_data.size();
    }
    for(int i=0;i<(int)knob_x_data.size();i++){
      kk_vec.emplace_back(kk);
    }
    for(int i=0;i<iters;++i){
      std::cout<<i<<std::endl;

      ros::Duration(1).sleep();
      arm->start_zero();
      move0force();
      arm->stop_zero();
      ros::Duration(1).sleep();
      arm->start_zero();
      //pos_ctrl_knob();
      admit_ctrl_knob();
      double x_cost,f_cost;
      get_cost_in_mbk(x_cost,f_cost);
      update_in_mbk();
      save_res(x_cost,f_cost);
      arm->stop_zero();

      arm->GetCartState(ori2);
      double d_quat=DeltaQuat(orig,ori2);
      /*while(d_quat>0.25){
        std::cout<<"Delta Quat: "<<d_quat<<std::endl;
        double p_o[7]={0.0,0.0,0.0,cos(-0.1),0.0,0.0,sin(-0.1)};
        arm->MoveRelaLineC(p_o,knob_end,a_l,v_l);
        ros::Duration(1).sleep();
        arm->GetCartState(ori2);
        d_quat=DeltaQuat(orig,ori2);
      }*/
      arm->MoveJointC(orig,a_j,v_j);
    }
  }
  arm->stop_filtered_sense();
  //arm->stop_sense();
  return 0;
}
