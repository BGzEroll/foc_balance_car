function result=simulate_lqi(d,scenario,options)
% 非线性平面+偏航对象；1ms RK4，5ms控制；左右轮独立限流、积分冻结。
if nargin<3,options=struct;end
options=defaults(options,struct('Kt_scale',1,'delay_cycles',1,'current_lag_s',.001,...
 'noise_pitch',0,'noise_rate',0,'initial_deg',5,'seed',23,'duration',18,'fault_injection',false));
p=d.parameters;Ts=d.Ts;N=round(options.duration/Ts)+1;t=(0:N-1)'*Ts;rng(options.seed);
x=zeros(6,1);if strcmp(scenario,'recover'),x(3)=deg2rad(options.initial_deg);end
xi=zeros(2,1);iq=zeros(2,1);queue=zeros(2,options.delay_cycles+1);ref=zeros(6,1);
X=zeros(N,6);U=zeros(N,2);REF=zeros(N,6);XI=zeros(N,2);SAT=false(N,1);fault=false;fell=false;
for k=1:N
 now=t(k);ref(:)=0;force=0;torque=0;yawTorque=0;
 if strcmp(scenario,'drive'),[ref(1),ref(2)]=move(now,2,4,.15);[ref(5),ref(6)]=move(now,7,3,deg2rad(20));end
 if strcmp(scenario,'push')&&now>=2&&now<2.15,force=.15;torque=.0005;end
 if strcmp(scenario,'bias')&&now>=2,force=.04;torque=.0005;yawTorque=.00015;end
 if strcmp(scenario,'noise'),force=.01*sin(2*pi*.95*now);end
 if options.fault_injection&&now>=3,fault=true;end
 m=x;m(3)=m(3)+options.noise_pitch*randn;m(4)=m(4)+options.noise_rate*randn;
 e=m-ref;e(5)=atan2(sin(e(5)),cos(e(5)));candidate=max(-d.integral_limit,min(d.integral_limit,xi+Ts*d.C*e));
 effort=-d.K*m+d.K*ref-d.Ki*xi;
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
rolling=.02*z(2)+.015*tanh(z(2)/.01);
a=D\[tau/r+M*l*sin(theta)*q^2+force-rolling;M*p.gravity*l*sin(theta)-tau+torque-.0001*q];
y=[z(2);a(1);q;a(2);z(6);(p.track/(2*r)*diffTau+yawTorque-.0002*z(6))/p.yaw_inertia];
end
function o=defaults(o,b)
n=fieldnames(b);for k=1:numel(n),if ~isfield(o,n{k}),o.(n{k})=b.(n{k});end,end
end
function [s,v]=move(t,start,T,amp)
u=max(0,min(1,(t-start)/T));s=amp*(10*u^3-15*u^4+6*u^5);v=amp/T*(30*u^2-60*u^3+30*u^4);
end
