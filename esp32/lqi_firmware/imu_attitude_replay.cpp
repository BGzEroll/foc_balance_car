#include "../balance_car/user_lib/hw/attitude_eskf.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using attitude_eskf::filter;
constexpr float PI=3.14159265358979323846f;
struct legacy_filter {
 float e[3]={};bool first=true;
 void update(const float *a,const float *g,float dt){
  float x=std::atan2(a[1],a[2]),y=std::atan2(-a[0],std::hypot(a[1],a[2]));
  if(first){e[0]=x;e[1]=y;first=false;return;}
  e[0]=.98f*(e[0]+g[0]*dt)+.02f*x;e[1]=.98f*(e[1]+g[1]*dt)+.02f*y;e[2]+=g[2]*dt;
 }
};
struct quaternion_complementary {
 filter f;
 void update(const float *a,const float *g,float dt){
  if(!f.initialized){f.initialize(a);return;}
  float v[3],dq[4];for(int i=0;i<3;++i)v[i]=g[i]*dt;
  filter::rotation(v,dq);filter::multiply(f.q,dq,f.q);f.normalize();
  float h[3];f.gravity(h);float n=std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  v[0]=.02f*(a[1]*h[2]-a[2]*h[1])/n;v[1]=.02f*(a[2]*h[0]-a[0]*h[2])/n;v[2]=.02f*(a[0]*h[1]-a[1]*h[0])/n;
  filter::rotation(v,dq);filter::multiply(f.q,dq,f.q);f.normalize();
 }
};
std::vector<std::string> split(const std::string &s){std::vector<std::string> v;std::stringstream ss(s);std::string a;while(std::getline(ss,a,','))v.push_back(a);return v;}
int column(const std::vector<std::string> &h,const std::string &s){for(unsigned i=0;i<h.size();++i)if(h[i]==s)return i;throw std::runtime_error("Missing CSV column "+s);}
float wrap(float x){return std::atan2(std::sin(x),std::cos(x));}
float heading(const float *e){return e[2]+std::atan2(-std::sin(e[1])*std::sin(e[0]),std::cos(e[0]));}
float qdistance(const float *a,const float *b){double d=0,na=0,nb=0;for(int i=0;i<4;++i){d+=double(a[i])*b[i];na+=double(a[i])*a[i];nb+=double(b[i])*b[i];}d/=std::sqrt(na*nb);return float(2*std::acos(std::fmin(1.0,std::fabs(d))));}
float min_pivot(const float P[6][6]){
 double L[6][6]={};double lowest=1e30;
 for(int i=0;i<6;++i)for(int j=0;j<=i;++j){double s=.5*(P[i][j]+P[j][i]);for(int k=0;k<j;++k)s-=L[i][k]*L[j][k];
  if(i==j){lowest=std::fmin(lowest,s);if(s<=0)return float(s);L[i][j]=std::sqrt(s);}else L[i][j]=s/L[j][j];}
 return float(lowest);
}
struct comparison {
 filter kf,rest;legacy_filter legacy;quaternion_complementary qc;
 std::ofstream out;float previous_heading[4]={},unwrapped[4]={},hold_reference[4]={};
 double yaw_square[4]={},roll_square[4]={},pitch_square[4]={};unsigned hold_count=0,count=0;
 float truth_previous_heading=0,truth_heading_unwrapped=0;
 float max_q_norm_error=0,min_covariance_pivot=1e10f;double max_q_error[2]={};
 comparison(const char *path,float acc_std):out(path){kf.acc_direction_std=rest.acc_direction_std=acc_std;
  out<<"time_s,legacy_roll,legacy_pitch,legacy_heading,qcf_roll,qcf_pitch,qcf_heading,kf_roll,kf_pitch,kf_heading,rest_roll,rest_pitch,rest_heading,bx,by,bz,norm_error,min_covariance_pivot,nis,gravity_rejected,rest_updates,truth_heading,truth_roll,truth_pitch,kf_orientation_error,rest_orientation_error\n";out<<std::setprecision(9);}
 void step(double t,float dt,const float *a,const float *g,bool still,const float *truth=nullptr){
  legacy.update(a,g,dt);qc.update(a,g,dt);kf.update(a,g,dt,false);rest.update(a,g,dt,still);
  float e[4][3];std::memcpy(e[0],legacy.e,sizeof(legacy.e));qc.f.euler(e[1]);kf.euler(e[2]);rest.euler(e[3]);
  float true_e[3]={};float true_h=0;if(truth){filter z;std::memcpy(z.q,truth,sizeof(z.q));z.euler(true_e);float h=heading(true_e);if(count)truth_heading_unwrapped+=wrap(h-truth_previous_heading);truth_previous_heading=h;true_h=truth_heading_unwrapped;}
  for(int i=0;i<4;++i){float h=heading(e[i]);if(count==0){previous_heading[i]=h;unwrapped[i]=0;}else unwrapped[i]+=wrap(h-previous_heading[i]);previous_heading[i]=h;
   if(t<30.01)hold_reference[i]=unwrapped[i];
   if(t>=30.01){double error=unwrapped[i]-hold_reference[i]-true_h;yaw_square[i]+=error*error;roll_square[i]+=std::pow(wrap(e[i][0]-true_e[0]),2);pitch_square[i]+=std::pow(wrap(e[i][1]-true_e[1]),2);}
  }
  if(t>=30.01)++hold_count;
  float norm=0;for(float x:rest.q)norm+=x*x;max_q_norm_error=std::fmax(max_q_norm_error,std::fabs(norm-1));
  if(count%100==0){min_covariance_pivot=std::fmin(min_covariance_pivot,min_pivot(rest.P));min_covariance_pivot=std::fmin(min_covariance_pivot,min_pivot(kf.P));}
  if(truth){max_q_error[0]=std::fmax(max_q_error[0],qdistance(kf.q,truth));max_q_error[1]=std::fmax(max_q_error[1],qdistance(rest.q,truth));}
  if(count%10==0){out<<t;for(int i=0;i<4;++i)out<<','<<e[i][0]<<','<<e[i][1]<<','<<unwrapped[i];
   for(float b:rest.bias){out<<','<<b;}
   out<<','<<max_q_norm_error<<','<<min_covariance_pivot<<','<<rest.nis<<','<<rest.gravity_rejected<<','<<rest.rest_updates<<','<<true_h<<','<<true_e[0]<<','<<true_e[1]<<','<<(truth?qdistance(kf.q,truth):0)<<','<<(truth?qdistance(rest.q,truth):0)<<'\n';}
  ++count;
 }
 void finish(){
  std::cout<<std::setprecision(10)<<"{\"samples\":"<<count<<",\"holdout_samples\":"<<hold_count<<",\"yaw_drift_after_training_deg\":[";
  for(int i=0;i<4;++i)std::cout<<(i?",":"")<<(unwrapped[i]-hold_reference[i])*180/PI;
  std::cout<<"],\"yaw_holdout_rms_deg\":[";for(int i=0;i<4;++i)std::cout<<(i?",":"")<<std::sqrt(yaw_square[i]/std::fmax(1u,hold_count))*180/PI;
  std::cout<<"],\"yaw_final_error_deg\":[";for(int i=0;i<4;++i)std::cout<<(i?",":"")<<(unwrapped[i]-hold_reference[i]-truth_heading_unwrapped)*180/PI;
  std::cout<<"],\"final_residual_bias_deg_s\":[";for(int i=0;i<3;++i)std::cout<<(i?",":"")<<rest.bias[i]*180/PI;
  std::cout<<"],\"max_quaternion_norm_error\":"<<max_q_norm_error<<",\"min_covariance_cholesky_pivot\":"<<min_covariance_pivot<<",\"max_quaternion_truth_error_deg\":["<<max_q_error[0]*180/PI<<','<<max_q_error[1]*180/PI<<"],\"gravity_rejected\":"<<rest.gravity_rejected<<",\"rest_updates\":"<<rest.rest_updates<<"}\n";
  if(!(min_covariance_pivot>0) || max_q_norm_error>1e-5f)throw std::runtime_error("Invalid covariance/quaternion");
 }
};

int main(int argc,char **argv){
 if(argc<3)return 2;
 comparison c(argv[2],argc>3?std::stof(argv[3]):.003f);
 if(std::string(argv[1])=="synthetic"){
  // 30s confirmed rest, slow real yaw, coupled 3D motion, >90deg tilt, and acceleration pulses.
  float q[4]={1,0,0,0};unsigned random=17;auto noise=[&](){random=1664525u*random+1013904223u;return (float(random>>8)/16777216.f-.5f)*3.4641016f;};
  for(int k=0;k<=48000;++k){double t=k*.005;float omega[3]={};
   if(t>=30&&t<150)omega[2]=.02f*PI/180;
   if(t>=150&&t<190){omega[0]=.35f*std::sin(float(t-150)*.8f);omega[1]=.22f*std::cos(float(t-150)*.65f);omega[2]=.4f;}
   if(t>=190&&t<200)omega[1]=.22f;
   if(t>=200&&t<210)omega[1]=-.22f;
   float increment[3];for(int i=0;i<3;++i)increment[i]=omega[i]*.005f;
   float dq[4];filter::rotation(increment,dq);filter::multiply(q,dq,q);
   filter true_filter;std::memcpy(true_filter.q,q,sizeof(q));true_filter.normalize();std::memcpy(q,true_filter.q,sizeof(q));
   float a[3],g[3];true_filter.gravity(a);
   for(int i=0;i<3;++i){a[i]=.883f*a[i]+.0012f*noise();g[i]=omega[i]+(i==0?.004f:i==1?-.003f:.008f)*PI/180+.00070f*noise();}
   if((t>=170&&t<171)||(t>=220&&t<222))a[1]+=.15f;
   c.step(t,.005f,a,g,t<30,q);
  }
 }else{
  std::ifstream input(argv[1]);std::string line;std::getline(input,line);if(!line.empty()&&line.back()=='\r')line.pop_back();auto h=split(line);
  int ti=column(h,"time_us"),ai[3]={column(h,"ax"),column(h,"ay"),column(h,"az")},gi[3]={column(h,"gyro_x"),column(h,"gyro_y"),column(h,"gyro_z")};
  double first=-1,last=0;bool annotated_rest=false;
  while(std::getline(input,line)){auto v=split(line);if(v.size()!=h.size())continue;if(v[0]!="static" && v[0]!="rest_training")continue;
   double now=std::stod(v[ti])*1e-6;if(first<0){first=now;last=now-.005;annotated_rest=v[0]=="rest_training";}float dt=float(now-last);last=now;
   float a[3],g[3];for(int i=0;i<3;++i){a[i]=std::stof(v[ai[i]])/16384;g[i]=std::stof(v[gi[i]]);}
   c.step(now-first,dt,a,g,annotated_rest?v[0]=="rest_training":now-first<30);
  }
 }
 c.finish();
}
