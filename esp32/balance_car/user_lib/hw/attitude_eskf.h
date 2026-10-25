#pragma once
#include <cmath>
#include <cstring>

namespace attitude_eskf {
// Hamilton quaternion [w,x,y,z], body -> world, left/world-frame error.
// Six error states: world rotation error XYZ and body gyro bias XYZ (rad/s).
struct filter {
 float q[4]={1,0,0,0},bias[3]={},P[6][6]={};
 float gyro_std=.00070f,acc_direction_std=.003f,bias_walk=5e-6f;
 float gravity_norm=1,nis=0;
 unsigned gravity_updates=0,gravity_rejected=0,rest_updates=0;
 bool initialized=false;

 static void multiply(const float *a,const float *b,float *o){
  float z[4]={a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
   a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
   a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
   a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};std::memcpy(o,z,sizeof(z));
 }
 static void rotation(const float *v,float *o){
  float n=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  float s=n>1e-7f?std::sin(.5f*n)/n:.5f;
  o[0]=std::cos(.5f*n);for(int i=0;i<3;++i)o[i+1]=s*v[i];
 }
 static void skew(const float *v,float o[3][3]){
  o[0][0]=0;o[0][1]=-v[2];o[0][2]=v[1];
  o[1][0]=v[2];o[1][1]=0;o[1][2]=-v[0];
  o[2][0]=-v[1];o[2][1]=v[0];o[2][2]=0;
 }
 void normalize(){float n=0;for(float v:q)n+=v*v;n=1/std::sqrt(n);for(float &v:q)v*=n;}
 void euler(float *e)const{
  e[0]=std::atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
  float s=2*(q[0]*q[2]-q[3]*q[1]);s=std::fmax(-1.0f,std::fmin(1.0f,s));e[1]=std::asin(s);
  e[2]=std::atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
 }
 void gravity(float *g)const{
  g[0]=2*(q[1]*q[3]-q[0]*q[2]);g[1]=2*(q[2]*q[3]+q[0]*q[1]);g[2]=1-2*(q[1]*q[1]+q[2]*q[2]);
 }
 void matrix(float R[3][3])const{
  const float w=q[0],x=q[1],y=q[2],z=q[3];
  R[0][0]=1-2*(y*y+z*z);R[0][1]=2*(x*y-w*z);R[0][2]=2*(x*z+w*y);
  R[1][0]=2*(x*y+w*z);R[1][1]=1-2*(x*x+z*z);R[1][2]=2*(y*z-w*x);
  R[2][0]=2*(x*z-w*y);R[2][1]=2*(y*z+w*x);R[2][2]=1-2*(x*x+y*y);
 }
 void initialize(const float *acc){
  gravity_norm=std::sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
  float rx=std::atan2(acc[1],acc[2]),ry=std::atan2(-acc[0],std::hypot(acc[1],acc[2]));
  float cx=std::cos(rx/2),sx=std::sin(rx/2),cy=std::cos(ry/2),sy=std::sin(ry/2);
  q[0]=cy*cx;q[1]=cy*sx;q[2]=sy*cx;q[3]=-sy*sx;
  for(int i=0;i<6;++i)P[i][i]=i<3?.0009f:7.615435e-7f;
  initialized=true;
 }
 void transform_covariance(const float F[6][6]){
  float A[6][6]={},next[6][6]={};
  for(int i=0;i<6;++i)for(int k=0;k<6;++k)if(F[i][k]!=0)
   for(int j=0;j<6;++j)A[i][j]+=F[i][k]*P[k][j];
  for(int i=0;i<6;++i)for(int j=0;j<6;++j)
   for(int k=0;k<6;++k)next[i][j]+=A[i][k]*F[j][k];
  std::memcpy(P,next,sizeof(P));
 }
 static bool inverse3(const float A[3][3],float B[3][3]){
  float d=A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])-A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])+A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
  if(!(d>0))return false;
  B[0][0]=(A[1][1]*A[2][2]-A[1][2]*A[2][1])/d;B[0][1]=(A[0][2]*A[2][1]-A[0][1]*A[2][2])/d;B[0][2]=(A[0][1]*A[1][2]-A[0][2]*A[1][1])/d;
  B[1][0]=(A[1][2]*A[2][0]-A[1][0]*A[2][2])/d;B[1][1]=(A[0][0]*A[2][2]-A[0][2]*A[2][0])/d;B[1][2]=(A[0][2]*A[1][0]-A[0][0]*A[1][2])/d;
  B[2][0]=(A[1][0]*A[2][1]-A[1][1]*A[2][0])/d;B[2][1]=(A[0][1]*A[2][0]-A[0][0]*A[2][1])/d;B[2][2]=(A[0][0]*A[1][1]-A[0][1]*A[1][0])/d;return true;
 }
 bool correct(const float H[3][6],const float residual[3],float variance,bool gate){
  float PH[6][3]={},S[3][3]={},inverse[3][3],K[6][3]={};
  for(int i=0;i<6;++i)for(int j=0;j<3;++j)for(int k=0;k<6;++k)PH[i][j]+=P[i][k]*H[j][k];
  for(int i=0;i<3;++i)for(int j=0;j<3;++j){for(int k=0;k<6;++k)S[i][j]+=H[i][k]*PH[k][j];if(i==j)S[i][j]+=variance;}
  if(!inverse3(S,inverse))return false;
  float normalized=0;for(int i=0;i<3;++i)for(int j=0;j<3;++j)normalized+=residual[i]*inverse[i][j]*residual[j];
  if(gate){nis=normalized;if(nis>9.21f)return false;}
  for(int i=0;i<6;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)K[i][j]+=PH[i][k]*inverse[k][j];
  if(gate){
   // Acceleration supplies gravity direction, not heading. Preserve the
   // unobservable rotation/bias component along gravity in this correction.
   float g[3];gravity(g);
   for(int column=0;column<3;++column){
    K[2][column]=0;
    float along=0;for(int i=0;i<3;++i)along+=g[i]*K[3+i][column];
    for(int i=0;i<3;++i)K[3+i][column]-=g[i]*along;
   }
  }
  float dx[6]={};for(int i=0;i<6;++i)for(int j=0;j<3;++j)dx[i]+=K[i][j]*residual[j];
  // Joseph covariance update. World-frame yaw is explicitly unobservable in H,
  // avoiding a large yaw variance rotated into all three tilt coordinates.
  float A[6][6]={},AP[6][6]={},next[6][6]={};
  for(int i=0;i<6;++i)for(int j=0;j<6;++j){A[i][j]=i==j?1:0;for(int k=0;k<3;++k)A[i][j]-=K[i][k]*H[k][j];}
  for(int i=0;i<6;++i)for(int j=0;j<6;++j)for(int k=0;k<6;++k)AP[i][j]+=A[i][k]*P[k][j];
  for(int i=0;i<6;++i)for(int j=0;j<6;++j){for(int k=0;k<6;++k)next[i][j]+=AP[i][k]*A[j][k];for(int k=0;k<3;++k)next[i][j]+=variance*K[i][k]*K[j][k];}
  for(int i=0;i<6;++i)for(int j=0;j<6;++j)P[i][j]=.5f*(next[i][j]+next[j][i]);
  float dq[4];rotation(dx,dq);multiply(dq,q,q);normalize();for(int i=0;i<3;++i)bias[i]+=dx[i+3];
  float J[6][6]={},cross[3][3];skew(dx,cross);
  for(int i=0;i<6;++i)J[i][i]=1;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)J[i][j]+=.5f*cross[i][j];
  transform_covariance(J);return true;
 }
 void update(const float *acc,const float *raw_gyro,float dt,bool confirmed_rest){
  if(!initialized){initialize(acc);return;}
  float omega[3],increment[3];for(int i=0;i<3;++i){omega[i]=raw_gyro[i]-bias[i];increment[i]=omega[i]*dt;}
  float dq[4];rotation(increment,dq);multiply(q,dq,q);normalize();
  float F[6][6]={},R[3][3];matrix(R);
  for(int i=0;i<6;++i)F[i][i]=1;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)F[i][j+3]=-R[i][j]*dt;
  transform_covariance(F);
  for(int i=0;i<3;++i){P[i][i]+=gyro_std*gyro_std*dt*dt;P[i+3][i+3]+=bias_walk*bias_walk*dt;}
  float n=std::sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
  if(std::fabs(n/gravity_norm-1)<.08f){
   float g[3],H[3][6]={},residual[3];gravity(g);
   for(int i=0;i<3;++i){residual[i]=acc[i]/n-g[i];H[i][0]=R[1][i];H[i][1]=-R[0][i];}
   if(correct(H,residual,acc_direction_std*acc_direction_std,true))++gravity_updates;else ++gravity_rejected;
  }else ++gravity_rejected;
  if(confirmed_rest){
   float H[3][6]={},residual[3];for(int i=0;i<3;++i){H[i][i+3]=1;residual[i]=raw_gyro[i]-bias[i];}
   if(correct(H,residual,gyro_std*gyro_std,false))++rest_updates;
  }
 }
};
}
