function result=simulate_lqi(d,scenario,options)
% 非线性平面+偏航对象；1ms RK4，5ms控制；左右轮独立限流、积分冻结。
if nargin<3,options=struct;end
options=defaults(options,struct('Kt_scale',1,'delay_cycles',1,'current_lag_s',.001,...
 'noise_pitch',0,'noise_rate',0,'initial_deg',5,'seed',23,'duration',18,'fault_injection',false,...
 'stiction',false,'rolling_coulomb_N',.015,'friction_ff_N',0,'motion_integral_delta_limit',inf,...
 'motion_integral_return_tau',.3,'motion_reversal_reset',false));
p=d.parameters;Ts=d.Ts;N=round(options.duration/Ts)+1;t=(0:N-1)'*Ts;rng(options.seed);
p.stiction=options.stiction;p.rolling_coulomb_N=options.rolling_coulomb_N;
x=zeros(6,1);if strcmp(scenario,'recover'),x(3)=deg2rad(options.initial_deg);end
xi=zeros(2,1);iq=zeros(2,1);queue=zeros(2,options.delay_cycles+1);ref=zeros(6,1);
X=zeros(N,6);U=zeros(N,2);REF=zeros(N,6);XI=zeros(N,2);SAT=false(N,1);fault=false;fell=false;
nh=zeros(4,1);notchInitialized=false;
motion_anchor=0;motion_began=false;
for k=1:N
 now=t(k);ref(:)=0;force=0;torque=0;yawTorque=0;
 if strcmp(scenario,'drive'),[ref(1),ref(2)]=move(now,2,4,.15);[ref(5),ref(6)]=move(now,7,3,deg2rad(20));end
 if strcmp(scenario,'bounded_drive')
  if now>=2 && ~motion_began,motion_anchor=xi(1);motion_began=true;end
  if motion_began && isfinite(options.motion_integral_delta_limit)
   if now>=50,xi(1)=motion_anchor+(xi(1)-motion_anchor)*exp(-Ts/options.motion_integral_return_tau);
   else,xi(1)=max(motion_anchor-options.motion_integral_delta_limit,min(motion_anchor+options.motion_integral_delta_limit,xi(1)));end
   if options.motion_reversal_reset && ((now>=14&&now<15)||(now>=38&&now<39))
    xi(1)=motion_anchor+(xi(1)-motion_anchor)*exp(-Ts/.3);
   end
  end
  for seg=1:4
   amp=[.05 -.05 -.05 .05];[pos,vel]=move(now,2+12*(seg-1),6,amp(seg));
   ref(1)=ref(1)+pos;ref(2)=ref(2)+vel;
  end
 end
 ff_force=options.friction_ff_N*tanh(ref(2)/.003);
 ref(3)=asin(max(-.2,min(.2,ff_force*p.wheel_radius/(p.body_mass*p.gravity*hypot(p.com_height,p.com_forward)))));
 if strcmp(scenario,'push')&&now>=2&&now<2.15,force=.15;torque=.0005;end
 if strcmp(scenario,'bias')&&now>=2,force=.04;torque=.0005;yawTorque=.00015;end
 if strcmp(scenario,'noise'),force=.01*sin(2*pi*.95*now);end
 if options.fault_injection&&now>=3,fault=true;end
 m=x;m(3)=m(3)+options.noise_pitch*randn;m(4)=m(4)+options.noise_rate*randn;
 e=m-ref;e(5)=atan2(sin(e(5)),cos(e(5)));candidate=max(-d.integral_limit,min(d.integral_limit,xi+Ts*d.C*e));
 effort=-d.K*m+d.K*ref-d.Ki*xi;
 effort(1)=effort(1)+ff_force*p.wheel_radius/d.Kt_nominal;
 if isfield(options,'common_notch')
  n=options.common_notch;raw=effort(1);
  if ~notchInitialized,nh(:)=raw;notchInitialized=true;y=raw;
  else,y=n.b(1)*raw+n.b(2)*nh(1)+n.b(3)*nh(2)-n.a(2)*nh(3)-n.a(3)*nh(4);end
  nh=[raw;nh(1);y;nh(3)];effort(1)=raw+n.mix*(y-raw);
 end
 raw=[(effort(1)-effort(2))/2;(effort(1)+effort(2))/2];
 command=max(-d.current_limit_each,min(d.current_limit_each,raw));sat=any(abs(raw)>d.current_limit_each);
 if ~sat,xi=candidate;end
 if fault,command(:)=0;end
 queue=[queue(:,2:end),command];target=queue(:,1);
 X(k,:)=x';U(k,:)=command';REF(k,:)=ref';XI(k,:)=xi';SAT(k)=sat;
 if abs(x(3))>pi/4||any(~isfinite(x)),fell=true;X=X(1:k,:);U=U(1:k,:);REF=REF(1:k,:);XI=XI(1:k,:);SAT=SAT(1:k);t=t(1:k);break;end
 for sub=1:5
  h=.001;if options.current_lag_s>0,iq=target+(iq-target)*exp(-h/options.current_lag_s);else,iq=target;end
  total=d.Kt_nominal*options.Kt_scale*sum(iq);diffTau=d.Kt_nominal*options.Kt_scale*(iq(2)-iq(1));
  f=@(z)dynamics(z,total,diffTau,force,torque,yawTorque,p);
  a=f(x);b=f(x+h*a/2);c=f(x+h*b/2);v=f(x+h*c);x=x+h*(a+2*b+2*c+v)/6;
 end
end
mask=t>=t(end)-2;err=X-REF;
report=struct('scenario',scenario,'options',options,'completed',t(end)>=options.duration-Ts,...
 'fell',fell,'injected_timing_fault',options.fault_injection,'duration_s',t(end),'peak_pitch_deg',max(abs(rad2deg(X(:,3)))),...
 'tail_position_error_m',mean(err(mask,1)),'tail_speed_m_s',mean(X(mask,2)),...
 'tail_yaw_error_deg',rad2deg(mean(err(mask,5))),'pitch_rms_deg',rad2deg(sqrt(mean(X(:,3).^2))),...
 'peak_each_current_A',max(abs(U),[],'all'),'saturation_fraction',mean(SAT),...
 'pass',~fell&&t(end)>=options.duration-Ts&&abs(mean(err(mask,1)))<.02&&abs(mean(err(mask,5)))<deg2rad(2));
result=struct('t',t,'state',X,'reference',REF,'current_A',U,'integral',XI,'saturated',SAT,'report',report);
end
function y=dynamics(z,tau,diffTau,force,torque,yawTorque,p)
M=p.body_mass;l=hypot(p.com_height,p.com_forward);r=p.wheel_radius;theta=z(3);q=z(4);
D=[M+2*p.wheel_mass+2*p.wheel_inertia/r^2 M*l*cos(theta);M*l*cos(theta) p.pitch_inertia+M*l^2];
cart=tau/r+M*l*sin(theta)*q^2+force;body=M*p.gravity*l*sin(theta)-tau+torque-.0001*q;
if p.stiction
 required=cart-D(1,2)*body/D(2,2);
 if abs(z(2))<.001 && abs(required)<=p.rolling_coulomb_N
  a=[0;body/D(2,2)];
 else
  direction=sign(z(2));if abs(z(2))<.001,direction=sign(required);end
  a=D\[cart-.02*z(2)-p.rolling_coulomb_N*direction;body];
 end
else
 rolling=.02*z(2)+p.rolling_coulomb_N*tanh(z(2)/.01);
 a=D\[cart-rolling;body];
end
y=[z(2);a(1);q;a(2);z(6);(p.track/(2*r)*diffTau+yawTorque-.0002*z(6))/p.yaw_inertia];
end
function o=defaults(o,b)
n=fieldnames(b);for k=1:numel(n),if ~isfield(o,n{k}),o.(n{k})=b.(n{k});end,end
end
function [s,v]=move(t,start,T,amp)
u=max(0,min(1,(t-start)/T));s=amp*(10*u^3-15*u^4+6*u^5);v=amp/T*(30*u^2-60*u^3+30*u^4);
end
