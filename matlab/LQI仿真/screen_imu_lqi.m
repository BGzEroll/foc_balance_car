function report=screen_imu_lqi(folder)
% Planar nonlinear LQI with measured colored IMU noise and a multirate sensor chain.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
a=readtable(fullfile(folder,'imu.csv'),'TextType','string');a=a(a.stage=="static",:);
assert(height(a)>2000,'Need at least ten seconds of raw stationary IMU');
s=jsondecode(fileread(fullfile(root,'esp32/lqi_firmware/selected_lqi.json')));
d=hardware_lqi_design;d.Kaug=d.Kaug.*s.scale(:)';d.K=d.Kaug(:,1:6);d.Ki=d.Kaug(:,7:8);
assert(max(abs(d.Kaug-s.Kaug),[],'all')<1e-6,'Freeze the actual saved LQI matrix');
t=(double(a.time_us)-double(a.time_us(1)))*1e-6;
native=double(a.native_sequence);nativeGrid=(native(1):native(end))';
noise=struct('acc',interp1(native,[a.ay a.az]/16384,nativeGrid),...
 'gyro',interp1(native,a.gyro_x,nativeGrid),'dt',diff(interp1(native,t,nativeGrid)));
noise.acc=noise.acc-mean(noise.acc,1);noise.dt=[median(noise.dt);noise.dt];
noise.gravity_scale=mean(vecnorm([a.ax a.ay a.az]/16384,2,2));
noise.read_duration=median(double(a.time_us)-double(a.read_began_us))*1e-6;
% Output-register age is unknown within one 1 kHz tick. Sweep, do not claim measured latency.
methods={'firmware_0p02','cf_0p5s','cf_1s','angle_bias_0p1Hz'};
cases=struct('name',{'nominal','weak_delayed','strong_offset'},...
 'kt',{1,.7,1.3},'current_lag',{.003,.005,.001},'command_delay',{1,2,0},...
 'imu_height',{0,.03,.03},'read_phase',{.0005,.0035,.0005},...
 'register_age',{.0005,.001,0},'friction_N',{.015,.25,.08},'noise_start_fraction',{0,.5,.9},...
 'accel_model',{'uniform_scale','z_offset','z_scale'});
for c=1:3,cases(c).control_sync=false;end
for c=1:3,cases(c+3)=cases(c);cases(c+3).control_sync=true;cases(c+3).name=[cases(c).name '_imu_sync'];end
scenarios={'recover','push','drive5cm'};rows=cell(1,numel(methods)*numel(cases)*numel(scenarios));
examples=cell(numel(methods),1);k=0;
for c=1:numel(cases)
 for j=1:numel(scenarios)
  for m=1:numel(methods)
   z=trial(d,noise,methods{m},scenarios{j},cases(c));k=k+1;rows{k}=z.metrics;
   if c==1 && j==3 && m==1,free_reference=z;end
   if c==4 && j==3,examples{m}=z;end
  end
  fprintf('IMU_LQI case=%s scenario=%s complete\n',cases(c).name,scenarios{j});
 end
end
items=[rows{:}];out=fullfile(here,'results/imu_lqi');if ~isfolder(out),mkdir(out);end
report=struct('source',folder,'Kaug',d.Kaug,'methods',{methods},'uncertainties',cases,...
 'measured_read_duration_us',noise.read_duration*1e6,'interpolated_native_samples',numel(nativeGrid)-height(a),...
 'cases',items,'hardware_estimator_changed',false,...
 'scope',['Nonlinear planar CAD plant with frozen measured-scaled LQI, 6A limit, anti-windup, current lag and command delay. '...
 'IMU: nominal 1kHz register updates, 200Hz host reads, approximate two-pole DLPF, measured colored accel/gyro noise and residual bias. '...
 'Both free-running and IMU-notified control are compared. Missing telemetry samples are interpolated by the native sample counter; they are not modeled as missing IMU acquisitions. '...
 'Accelerometer includes translational and angular acceleration at an uncertain IMU height. '...
 'Encoder AB4ms is approximated by two 2ms poles. Coulomb friction is swept as an unmeasured uncertainty with a low-speed sticking condition. '...
 'Yaw and roll are fixed; no slip, structural vibration or independent identified plant. '...
 'Fixed-stand noise is conditional on this pose. Its roughly 0.883g magnitude is screened as alternative uniform scale, Z offset or Z scale errors; none is an identified calibration. '...
 'Failed runs are retained. Results screen candidates only; no ground improvement is established.']);
fid=fopen(fullfile(out,'report.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
writetable(struct2table(items),fullfile(out,'cases.csv'));
save(fullfile(out,'traces.mat'),'examples','free_reference','report');
plot_imu_lqi;
disp(struct2table(items));
end

function z=trial(d,n,method,scenario,c)
h=.001;Ts=.005;duration=12;if strcmp(scenario,'drive5cm'),duration=24;end
count=round(duration/h);x=zeros(4,1);if strcmp(scenario,'recover'),x(3)=5*pi/180;end
p=d.parameters;r=p.wheel_radius;g=p.gravity;G=n.gravity_scale;
initial=[-sin(x(3))*g;cos(x(3))*g];a1=initial;a2=initial;w1=0;w2=0;
historyA=repmat(initial,1,12);historyW=zeros(1,12);
estimate_angle=-x(3);estimate_bias=0;gyro=0;distance=0;previous_speed=0;xi=0;
odometry=0;encoder1=0;encoder2=0;current=0;
queue=zeros(1,c.command_delay);nextRead=c.read_phase+.005;lastRead=nextRead-.005;
noiseIndex=1+floor(c.noise_start_fraction*(size(n.acc,1)-1));measured_pitch=x(3);speed=0;reference=0;targetSpeed=0;
N=ceil(count/5);T=zeros(N,1);X=zeros(N,4);U=zeros(N,1);MP=zeros(N,1);V=zeros(N,1);R=zeros(N,1);S=false(N,1);k=0;fell=false;
for tick=1:count
 now=tick*h;force=0;if strcmp(scenario,'push') && now>=2 && now<2.15,force=.15;end
 derivative=plant(x,current,c,d,force);
 % Specific force in forward/up body axes; pitch is positive leaning forward.
 ax=derivative(2)+c.imu_height*(cos(x(3))*derivative(4)-sin(x(3))*x(4)^2);
 az=-c.imu_height*(sin(x(3))*derivative(4)+cos(x(3))*x(4)^2);
 rawA=[cos(x(3))*ax-sin(x(3))*(az+g);sin(x(3))*ax+cos(x(3))*(az+g)];
 % Two-pole approximation matches roughly 42Hz gyro bandwidth and 4.8ms DC delay.
 fa=exp(-h/.00245);fw=exp(-h/.0024);
 a1=rawA+(a1-rawA)*fa;a2=a1+(a2-a1)*fa;
 w1=-x(4)+(w1+x(4))*fw;w2=w1+(w2-w1)*fw;
 historyA=[historyA(:,2:end),a2];historyW=[historyW(2:end),w2];
 didRead=false;
 if now>=nextRead
  didRead=true;
  dt=nextRead-lastRead;lastRead=nextRead;
  lag=n.read_duration+c.register_age;index=max(1,12-round(lag/h));
  measuredA=historyA(:,index)/g;
  if strcmp(c.accel_model,'uniform_scale'),measuredA=G*measuredA;
  elseif strcmp(c.accel_model,'z_offset'),measuredA(2)=measuredA(2)+G-1;
  else,measuredA(2)=G*measuredA(2);end
  measuredA=measuredA+n.acc(noiseIndex,:)';
  gyro=historyW(index)+n.gyro(noiseIndex);
  accAngle=atan2(measuredA(1),measuredA(2));pred=estimate_angle+dt*(gyro-estimate_bias);
  residual=atan2(sin(accAngle-pred),cos(accAngle-pred));
  if strcmp(method,'firmware_0p02'),alpha=.02;beta=0;
  elseif strcmp(method,'cf_0p5s'),alpha=dt/(.5+dt);beta=0;
  elseif strcmp(method,'cf_1s'),alpha=dt/(1+dt);beta=0;
  else,pole=exp(-2*pi*.1*dt);alpha=1-pole^2;beta=(1-pole)^2;end
  estimate_angle=pred+alpha*residual;estimate_bias=estimate_bias-beta/dt*residual;
  measured_pitch=-estimate_angle;
  noiseIndex=mod(noiseIndex,size(n.acc,1))+1;nextRead=nextRead+n.dt(noiseIndex);
 end
 % AB4ms approximation acts on wheel angular velocity relative to body.
 wheel=x(2)/r-x(4);f=exp(-h/.002);encoder1=wheel+(encoder1-wheel)*f;encoder2=encoder1+(encoder2-encoder1)*f;
 if (~c.control_sync && mod(tick,5)==0) || (c.control_sync && didRead)
  controlDt=Ts;if c.control_sync,controlDt=dt;end
  speed=r*(encoder2-gyro);odometry=odometry+controlDt*(previous_speed+speed)/2;previous_speed=speed;
  if strcmp(scenario,'drive5cm')
   [p1,v1]=move(now,2,6,.05);[p2,v2]=move(now,14,6,-.05);reference=p1+p2;targetSpeed=v1+v2;
  end
  error=[odometry-reference;speed-targetSpeed;measured_pitch;-gyro];
  effort=-d.K(1,1:4)*error-d.Ki(1,1)*xi;
  raw=effort/2;command=max(-d.current_limit_each,min(d.current_limit_each,raw));
  saturated=abs(raw)>d.current_limit_each;
  if ~saturated,xi=max(-d.integral_limit(1),min(d.integral_limit(1),xi+controlDt*(odometry-reference)));end
  if c.command_delay>0,target=queue(1);queue=[queue(2:end),command];else,target=command;end
  k=k+1;T(k)=now;X(k,:)=x';U(k)=command;MP(k)=measured_pitch;V(k)=speed;R(k)=reference;S(k)=saturated;
 end
 if k==0,target=0;end
 current=target+(current-target)*exp(-h/c.current_lag);
 f0=@(v)plant(v,current,c,d,force);
 k1=f0(x);k2=f0(x+h*k1/2);k3=f0(x+h*k2/2);k4=f0(x+h*k3);x=x+h*(k1+2*k2+2*k3+k4)/6;
 if abs(x(3))>pi/4 || any(~isfinite(x)),fell=true;break;end
end
T=T(1:k);X=X(1:k,:);U=U(1:k);MP=MP(1:k);V=V(1:k);R=R(1:k);S=S(1:k);
mask=T>=max(0,T(end)-3);quiet=abs(X(:,3))<.5*pi/180 & abs(X(:,2))<.01;
last=find(~quiet,1,'last');if isempty(last),settle=0;elseif last==numel(T),settle=NaN;else,settle=T(last);end
metrics=struct('method',method,'scenario',scenario,'uncertainty',c.name,'fell',fell,'duration_s',T(end),...
 'position_tracking_rms_m',rms(X(:,1)-R),'tail_position_rms_m',rms(X(mask,1)-R(mask)),...
 'true_speed_rms_m_s',rms(X(:,2)),'tail_true_speed_rms_m_s',rms(X(mask,2)),...
 'pitch_estimation_rms_deg',rms(MP-X(:,3))*180/pi,'true_pitch_rms_deg',rms(X(:,3))*180/pi,...
 'true_pitch_peak_deg',max(abs(X(:,3)))*180/pi,'each_current_rms_A',rms(U),'saturation_fraction',mean(S),...
 'last_exit_half_deg_and_1cms_s',settle,'peak_displacement_m',max(abs(X(:,1))));
z=struct('t',T,'x',X,'current',U,'measured_pitch',MP,'speed',V,'reference',R,'metrics',metrics);
end

function dx=plant(x,current,c,d,force)
p=d.parameters;M=p.body_mass;l=hypot(p.com_height,p.com_forward);r=p.wheel_radius;theta=x(3);q=x(4);
D=[M+2*p.wheel_mass+2*p.wheel_inertia/r^2 M*l*cos(theta);M*l*cos(theta) p.pitch_inertia+M*l^2];
tau=2*d.Kt_nominal*c.kt*current;
cart=tau/r+M*l*sin(theta)*q^2+force;body=M*p.gravity*l*sin(theta)-tau-.0001*q;
required=cart-D(1,2)*body/D(2,2);
if abs(x(2))<.001 && abs(required)<=c.friction_N
 a=[0;body/D(2,2)];
else
 direction=sign(x(2));if abs(x(2))<.001,direction=sign(required);end
 a=D\[cart-.02*x(2)-c.friction_N*direction;body];
end
dx=[x(2);a(1);q;a(2)];
end

function [s,v]=move(t,start,T,amp)
u=max(0,min(1,(t-start)/T));s=amp*(10*u^3-15*u^4+6*u^5);v=amp/T*(30*u^2-60*u^3+30*u^4);
end
