function report=compare_encoder_ab
% AS5600 speed estimation: matched-delay comparison, replay, and LQI simulation.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
out=fullfile(here,'results','encoder_ab');if ~isfolder(out),mkdir(out);end
methods={'LPF 3ms','AB matched 3ms','AB 6ms','AB 10ms'};tau=[.003 .003 .006 .010];
periods=[.00015 .0003 .0006 .005];noiseCounts=[0 .5 1];
names={'static','slow','reverse','10Hz','multi_turn','accelerate'};
quantum=2*pi/4096;rows=struct([]);frequency=struct([]);example=struct;
for ip=1:numel(periods)
 dt0=periods(ip);rng(100+ip);dt=dt0*(1+.15*randn(ceil(4/dt0)+1,1));dt=max(dt,.5*dt0);
 if dt0<.001,dt(round(.005/dt0):round(.005/dt0):end)=dt(round(.005/dt0):round(.005/dt0):end)+.0007;end
 t=[0;cumsum(dt)];t=t(t<=4);
 for ic=1:numel(names)
  [truth,velocity]=trajectory(t,names{ic});
  for in=1:numel(noiseCounts)
   rng(1000+ip*100+ic*10+in);
   raw=mod(round((truth+noiseCounts(in)*quantum*randn(size(t)))/quantum),4096);
   z=unwrap_counts(raw)*quantum;mask=t>.2;
   for im=1:numel(methods)
    [position,speed]=estimate(t,z,im,tau(im));
    item=struct('method',methods{im},'nominal_period_s',dt0,'noise_counts_rms',noiseCounts(in),'noise_correlation_s',0,...
     'scenario',names{ic},'speed_rmse_rad_s',rms(speed(mask)-velocity(mask)),...
     'speed_bias_rad_s',mean(speed(mask)-velocity(mask)),...
     'angle_rmse_rad',rms(position(mask)-truth(mask)),...
     'max_speed_error_rad_s',max(abs(speed(mask)-velocity(mask))));
    rows=[rows;item]; %#ok<AGROW>
    if ip==2&&in==2&&(ic==2||ic==4)
     key=['case' num2str(ic)];example.(key).t=t;example.(key).truth=velocity;
     example.(key).speed(:,im)=speed;
    end
   end
  end
 end
 for im=1:numel(methods)
  for hz=[1 5 10 20 40]
   [gain,lag,alpha,beta]=response(dt0,im,tau(im),hz);
   frequency=[frequency;struct('method',methods{im},'period_s',dt0,'frequency_Hz',hz,...
    'gain',gain,'phase_delay_ms',lag*1000,'alpha',alpha,'beta',beta)]; %#ok<AGROW>
  end
 end
 fprintf('ENCODER_SYNTHETIC nominal_dt_us=%g complete\n',dt0*1e6);
end
% Correlated angle noise matters: real sensor noise is not necessarily white.
dt0=.0003;rng(102);dt=dt0*(1+.15*randn(ceil(4/dt0)+1,1));dt=max(dt,.5*dt0);
dt(round(.005/dt0):round(.005/dt0):end)=dt(round(.005/dt0):round(.005/dt0):end)+.0007;
t=[0;cumsum(dt)];t=t(t<=4);
for correlation=[.002 .010]
 for ic=1:numel(names)
  [truth,velocity]=trajectory(t,names{ic});rng(1200+ic);w=randn(size(t));noise=w;
  for k=2:numel(t)
   a=exp(-(t(k)-t(k-1))/correlation);noise(k)=a*noise(k-1)+sqrt(1-a*a)*w(k);
  end
  raw=mod(round((truth+.5*quantum*noise)/quantum),4096);z=unwrap_counts(raw)*quantum;mask=t>.2;
  for im=1:numel(methods)
   [position,speed]=estimate(t,z,im,tau(im));
   item=struct('method',methods{im},'nominal_period_s',dt0,'noise_counts_rms',.5,'noise_correlation_s',correlation,...
    'scenario',names{ic},'speed_rmse_rad_s',rms(speed(mask)-velocity(mask)),...
    'speed_bias_rad_s',mean(speed(mask)-velocity(mask)),...
    'angle_rmse_rad',rms(position(mask)-truth(mask)),...
    'max_speed_error_rad_s',max(abs(speed(mask)-velocity(mask))));
   rows=[rows;item]; %#ok<AGROW>
  end
 end
 fprintf('ENCODER_COLORED_NOISE correlation_ms=%g complete\n',correlation*1000);
end
writetable(struct2table(rows),fullfile(out,'synthetic.csv'));
writetable(struct2table(frequency),fullfile(out,'frequency.csv'));
replay=encoder_replay(root,out,methods,tau);
fprintf('ENCODER_REPLAY complete (200Hz, no true-speed reference)\n');
d=hardware_lqi_design;s=jsondecode(fileread(fullfile(root,'esp32','lqi_firmware','selected_lqi.json')));
d.Kaug=d.Kaug.*s.scale(:)';closed=struct([]);
for delay=[0 .002]
 for scenario={'push','bounded_drive'}
  for im=1:numel(methods)
   c=closed_loop(d,im,tau(im),delay,scenario{1});c.method=methods{im};closed=[closed;c]; %#ok<AGROW>
   fprintf('ENCODER_LQI %s sensor_delay_ms=%g %s fell=%d speed_error=%.6f\n',...
    methods{im},delay*1000,scenario{1},c.fell,c.speed_estimation_rmse_m_s);
  end
 end
end
report=struct('synthetic',rows,'frequency',frequency,'replay',replay.report,'closed_loop',closed,...
 'scope',['Simulation only. Synthetic 150/300/600us timing and 0/0.5/1-count noise are assumptions, '...
 'not identified acquisition statistics; 2/10ms angle-noise correlation is also screened. 200Hz replay cannot reproduce the original asynchronous encoder filter or establish speed accuracy. '...
 'Closed-loop model uses CAD mechanics, nominal Kt, illustrative sensor delay/noise and ideal cancellation of cogging; it is not hardware validation.']);
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
save(fullfile(out,'comparison.mat'),'report','example','replay','methods','tau');
make_plot(out,example,replay,methods,rows,frequency);
disp('ENCODER_AB_COMPARISON_READY');
end

function z=unwrap_counts(raw)
delta=diff(raw);delta(delta>2048)=delta(delta>2048)-4096;delta(delta< -2048)=delta(delta< -2048)+4096;
z=raw(1)+[0;cumsum(delta)];
end

function [p,v]=trajectory(t,name)
switch name
 case 'static',p=zeros(size(t));v=p;
 case 'slow',p=.3*t;v=.3+zeros(size(t));
 case 'reverse',p=.2*sin(2*pi*1.5*t);v=.2*2*pi*1.5*cos(2*pi*1.5*t);
 case '10Hz',p=.4/(2*pi*10)*sin(2*pi*10*t);v=.4*cos(2*pi*10*t);
 case 'multi_turn',p=6*t+.2*sin(2*pi*.7*t);v=6+.2*2*pi*.7*cos(2*pi*.7*t);
 case 'accelerate',p=t.^2;v=2*t;
end
p=p+2*pi-.01;
end

function [position,speed]=estimate(t,z,method,tau)
position=zeros(size(z));speed=position;position(1,:)=z(1,:);
for k=2:numel(t)
 dt=t(k)-t(k-1);
 if method==1
  position(k,:)=z(k,:);speed(k,:)=(tau*speed(k-1,:)+z(k,:)-z(k-1,:))/(tau+dt);
 else
  % Repeated observer pole p gives the same low-frequency velocity delay as LPF(tau).
  p=tau/(tau+2*dt);alpha=1-p*p;beta=(1-p)^2;
  prediction=position(k-1,:)+dt*speed(k-1,:);error=z(k,:)-prediction;
  position(k,:)=prediction+alpha*error;speed(k,:)=speed(k-1,:)+beta/dt*error;
 end
end
end

function [gain,lag,alpha,beta]=response(dt,method,tau,hz)
q=exp(-1i*2*pi*hz*dt);alpha=NaN;beta=NaN;
if method==1
 h=(1-q)/(tau+dt)/(1-tau/(tau+dt)*q);
else
 p=tau/(tau+2*dt);alpha=1-p*p;beta=(1-p)^2;
 h=beta/dt*(1-q)/(1-(2-alpha-beta)*q+(1-alpha)*q*q);
end
h=h/(1i*2*pi*hz);gain=abs(h);lag=-angle(h)/(2*pi*hz);
end

function replay=encoder_replay(root,out,methods,tau)
source=fullfile(root,'esp32','lqi_firmware','results','anticogging','retained_natural_60s.csv');
T=readtable(source);t=(T.sample_us-T.sample_us(1))*1e-6;z=[T.angle_a T.angle_b];
report=struct([]);speed=zeros(numel(t),2,numel(methods));
for im=1:numel(methods)
 [~,speed(:,:,im)]=estimate(t,z,im,tau(im));m=t>1;
 item=struct('method',methods{im},'output_speed_rms_rad_s',rms(speed(m,:,im),'all'),...
  'output_difference_to_firmware_rms_rad_s',rms(speed(m,:,im)-[T.raw_a(m) T.raw_b(m)],'all'));
 report=[report;item]; %#ok<AGROW>
end
replay=struct('report',report,'t',t,'angle',z,'speed',speed,'firmware_speed',[T.raw_a T.raw_b],...
 'source',source,'max_sample_gap_s',max(diff(t)));
writetable(array2table([t z T.raw_a T.raw_b speed(:,:,1) speed(:,:,2)],...
 'VariableNames',{'time_s','angle_a','angle_b','firmware_speed_a','firmware_speed_b',...
 'replay_lpf_a','replay_lpf_b','replay_ab_a','replay_ab_b'}),fullfile(out,'replay.csv'));
end

function report=closed_loop(d,method,tau,sensorDelay,scenario)
h=.00025;Ts=d.Ts;stride=round(Ts/h);duration=12;if strcmp(scenario,'bounded_drive'),duration=60;end
p=d.parameters;M=p.body_mass;l=hypot(p.com_height,p.com_forward);r=p.wheel_radius;
D=[M+2*p.wheel_mass+2*p.wheel_inertia/r^2 M*l;M*l p.pitch_inertia+M*l^2];force=D\[1;0];F=[0;force(1);0;force(2)];
E=expm([d.A(1:4,1:4) d.B(1:4,1) F;zeros(2,6)]*h);A=E(1:4,1:4);B=E(1:4,5);Bf=E(1:4,6);
N=round(duration/h)+1;L=round(duration/Ts)+1;data=zeros(L,10);j=0;x=zeros(4,1);
xi=0;odometry=0;previousSpeed=0;common=0;target=0;queue=[0 0];fell=false;
offset=[2*pi-.01 2.58];delayN=round(sensorDelay/h);history=repmat(offset,delayN+1,1);
lastRaw=mod(round(offset/(2*pi)*4096),4096);angle=lastRaw*(2*pi/4096);position=angle;omega=[0 0];
pole=tau/(tau+2*h);alpha=1-pole^2;beta=(1-pole)^2;rng(71);
for k=1:N
 now=(k-1)*h;phi=offset+[-1 1]*(x(1)/r-x(3));
 history(mod(k-1,delayN+1)+1,:)=phi;sample=history(mod(k-1-delayN,delayN+1)+1,:);
 raw=mod(round(sample/(2*pi)*4096+.5*randn(1,2)),4096);delta=raw-lastRaw;
 delta(delta>2048)=delta(delta>2048)-4096;delta(delta< -2048)=delta(delta< -2048)+4096;
 z=angle+delta*(2*pi/4096);
 if k>1
  if method==1,omega=(tau*omega+z-angle)/(tau+h);position=z;
  else,prediction=position+h*omega;error=z-prediction;position=prediction+alpha*error;omega=omega+beta/h*error;end
 end
 angle=z;lastRaw=raw;
 if mod(k-1,stride)==0
  pitch=x(3)+.001*randn;rate=x(4)+.02*randn;
  speed=r*((-omega(1)+omega(2))/2+rate); % Encoder motion is relative to the body.
  if j>0,odometry=odometry+Ts*(previousSpeed+speed)/2;end
  previousSpeed=speed;[ref,refSpeed]=drive_reference(now,scenario);
  error=[odometry-ref;speed-refSpeed;pitch;rate];u=-d.Kaug(1,1:4)*error-d.Kaug(1,7)*xi;
  saturated=abs(u)>12;u=max(-12,min(12,u));if ~saturated,xi=max(-.3,min(.3,xi+Ts*error(1)));end
  target=queue(1);queue=[queue(2) u];j=j+1;data(j,:)=[now x' ref speed u saturated odometry];
 end
 common=target+(common-target)*exp(-h/.003);
 push=0;if strcmp(scenario,'push')&&now>=2&&now<2.15,push=.15;end
 drag=.02*x(2)+.015*tanh(x(2)/.01);x=A*x+B*common+Bf*(push-drag);
 if abs(x(3))>pi/4||any(~isfinite(x)),fell=true;break;end
end
data=data(1:j,:);mask=data(:,1)>1;
report=struct('scenario',scenario,'sensor_period_s',h,'sensor_delay_s',sensorDelay,'fell',fell,...
 'completed',data(end,1)>=duration-Ts,'peak_pitch_deg',max(abs(rad2deg(data(:,4)))),...
 'peak_position_m',max(abs(data(:,2))),'position_tracking_rmse_m',rms(data(mask,2)-data(mask,6)),...
 'speed_estimation_rmse_m_s',rms(data(mask,7)-data(mask,3)),...
 'odometry_rmse_m',rms(data(mask,10)-data(mask,2)),...
 'common_command_rms_A',rms(data(mask,8)),'saturation_fraction',mean(data(:,9)));
end

function [position,speed]=drive_reference(now,scenario)
position=0;speed=0;if ~strcmp(scenario,'bounded_drive'),return;end
for j=1:4
 amp=[.05 -.05 -.05 .05];u=max(0,min(1,(now-2-12*(j-1))/6));
 position=position+amp(j)*(10*u^3-15*u^4+6*u^5);speed=speed+amp(j)/6*(30*u^2-60*u^3+30*u^4);
end
end

function make_plot(out,example,replay,methods,rows,frequency)
fig=figure('Visible','off','Position',[60 60 1400 1000]);tiledlayout(2,2);
colors=[.55 .55 .55;0 .35 .75;.75 .4 0;.5 .15 .65];
nexttile;e=example.case2;m=e.t>=1&e.t<=1.2;hold on;plot(e.t(m),e.truth(m),'k--','LineWidth',1.4);
for k=1:2,plot(e.t(m),e.speed(m,k),'Color',colors(k,:),'LineWidth',1.1);end;grid on;
xlabel('时间 (s)');ylabel('轮速 (rad/s)');title('已知真值：0.3rad/s，量化及0.5计数噪声');legend('真值',methods{1:2});
nexttile;e=example.case4;m=e.t>=2&e.t<=2.2;hold on;plot(e.t(m),e.truth(m),'k--','LineWidth',1.4);
for k=1:4,plot(e.t(m),e.speed(m,k),'Color',colors(k,:),'LineWidth',1.1);end;grid on;
xlabel('时间 (s)');ylabel('轮速 (rad/s)');title('已知真值：10Hz动态，不能把真实运动滤掉');legend('真值',methods{:});
nexttile;hold on;
for im=1:numel(methods)
 a=rows(strcmp({rows.method},methods{im})&[rows.nominal_period_s]==.0003&[rows.noise_counts_rms]==.5&[rows.noise_correlation_s]==0&strcmp({rows.scenario},'static'));
 b=frequency(strcmp({frequency.method},methods{im})&[frequency.period_s]==.0003&[frequency.frequency_Hz]==10);
 plot(b.phase_delay_ms,a.speed_rmse_rad_s,'o','MarkerSize',9,'Color',colors(im,:),'DisplayName',methods{im});
 text(b.phase_delay_ms,a.speed_rmse_rad_s,['  ' methods{im}]);
end
grid on;xlim([2.5 12]);ylim([0 .32]);xlabel('10Hz速度相位延迟 (ms)');ylabel('静止速度误差 RMS (rad/s)');title('噪声与延迟的取舍：300μs名义采样');
nexttile;m=replay.t>=10&replay.t<=10.7;
hold on;plot(replay.t(m),replay.firmware_speed(m,1),':','Color',[.4 .65 .4]);
for k=1:2,plot(replay.t(m),replay.speed(m,1,k),'Color',colors(k,:),'LineWidth',1.1);end;grid on;
xlabel('时间 (s)');ylabel('左轮速度 (rad/s)');title('实测角度200Hz回放：没有速度真值');legend('原固件速度（高频更新）',methods{1:2});
sgtitle('AS5600 αβ观测器：同延迟对比与加延迟取舍，尚未修改实机');
exportgraphics(fig,fullfile(out,'comparison_ab.png'),'Resolution',130);close(fig);
end
