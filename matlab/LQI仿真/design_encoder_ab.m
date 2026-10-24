function result=design_encoder_ab
% Tune the critically damped alpha-beta family against measured encoder noise/timing.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
source=fullfile(root,'esp32','lqi_firmware','results','encoder_shadow');
out=fullfile(here,'results','encoder_ab_measured');if ~isfolder(out),mkdir(out);end
files=dir(fullfile(source,'**','encoder_*.bin'));traces=struct([]);stats=struct([]);
for j=1:numel(files)
 path=fullfile(files(j).folder,files(j).name);fid=fopen(path,'rb');bytes=fread(fid,inf,'*uint8');fclose(fid);
 header=typecast(bytes(1:16),'uint32');count=double(header(2));assert(numel(bytes)==24+16*count+4);
 block=reshape(bytes(25:end-4),16,[])';stamp=double(field(block,1:4,'uint32'));
 packed=field(block,5:6,'uint16');motor=bitshift(packed,-12);raw=double(bitand(packed,4095));
 delta=double(field(block,7:8,'int16'));lpf=double(field(block,9:12,'single'));ab=double(field(block,13:16,'single'));
 first=typecast(bytes(17:24),'uint64');relative=mod(stamp-double(bitand(first,uint64(4294967295)))+2147483648,4294967296)-2147483648;
 for side=0:1
  m=motor==side;t=relative(m)*1e-6;t=t-t(1);dt=diff(t);assert(all(dt>0));
  counts=raw(m);changes=delta(m);assert(all(mod(diff(counts)+2048,4096)-2048==changes(2:end)));
  z=cumsum(changes)*2*pi/4096;speed=lpf(m);shadow=ab(m);warm=t>.04;
  v=replay(t,z,2,double(header(3))*1e-6);old=replay(t,z,1,.003);
  item=struct('file',path,'motor',side,'samples',sum(m),'duration_s',t(end),...
   'median_period_us',median(dt)*1e6,'p99_period_us',prctile(dt,99)*1e6,'max_period_us',max(dt)*1e6,...
   'angle_variation_counts_rms',std(z*4096/(2*pi)),'angle_span_counts',range(z)*4096/(2*pi),...
   'lpf_speed_rms_rad_s',rms(speed(warm)),'ab_shadow_rms_rad_s',rms(shadow(warm)),...
   'shadow_replay_difference_rad_s',rms(v(warm)-shadow(warm)),...
   'lpf_replay_difference_rad_s',rms(old(warm)-speed(warm)));
  stats=[stats;item]; %#ok<AGROW>
  traces=[traces;struct('t',t,'z',z,'motor',side,'static',contains(files(j).folder,'static'))]; %#ok<AGROW>
 end
end
assert(any([traces.static]),'Collect stationary, motor-off captures first');
d=hardware_lqi_design;s=jsondecode(fileread(fullfile(root,'esp32','lqi_firmware','selected_lqi.json')));d.Kaug=d.Kaug.*s.scale(:)';
taus=[.001 .0015 .002 .003 .004 .005 .006 .008 .010];cases=struct([]);profiles={'static','slow','reverse','10Hz','reversal_step'};
for side=0:1
 dt=[];noise=[];
 for j=find([traces.static]&[traces.motor]==side)
  z=traces(j).z*4096/(2*pi);time=traces(j).t;
  dt=[dt;median(diff(time));diff(time)];noise=[noise;z-median(z)]; %#ok<AGROW>
 end
 t=cumsum(dt);t=t-t(1);medianDt=median(dt);
 for method=1:2
  for tau=taus
   errors=zeros(1,numel(profiles));
   for j=1:numel(profiles)
    [truth,velocity]=profile(t,profiles{j});z=round(truth*4096/(2*pi)+noise)*2*pi/4096;
    v=replay(t,z,method,tau);warm=t>.05;errors(j)=rms(v(warm)-velocity(warm));
   end
   [gain,lag]=frequency(medianDt,method,tau,10);
   radius=robust_radius(d,method,tau,medianDt);
   pole=tau/(tau+2*medianDt);
   item=struct('motor',side,'method',method,'tau_us',round(tau*1e6),...
    'alpha_at_median_dt',1-pole*pole,'beta_at_median_dt',(1-pole)^2,...
    'static_rmse',errors(1),'slow_rmse',errors(2),'reverse_rmse',errors(3),'ten_Hz_rmse',errors(4),...
    'reversal_step_rmse',errors(5),'score',mean(errors(2:end)),...
    'gain_10Hz',gain,'phase_delay_10Hz_ms',lag*1000,'worst_model_pole_radius',radius);
   cases=[cases;item]; %#ok<AGROW>
  end
 end
end
ranking=struct([]);
for tau=taus
 c=cases([cases.method]==2&[cases.tau_us]==round(tau*1e6));
 ranking=[ranking;struct('tau_us',round(tau*1e6),'score',mean([c.score]),...
  'worst_model_pole_radius',max([c.worst_model_pole_radius]),'max_phase_delay_10Hz_ms',max([c.phase_delay_10Hz_ms]),...
  'eligible',all([c.worst_model_pole_radius]<1)&all([c.phase_delay_10Hz_ms]<=5))]; %#ok<AGROW>
end
eligible=find([ranking.eligible]);assert(~isempty(eligible),'No candidate meets the model/lag screen');
[~,i]=min([ranking(eligible).score]);chosen=ranking(eligible(i));
result=struct('capture_statistics',stats,'cases',cases,'ranking',ranking,'candidate',chosen,...
 'scope',['Motor-off angle variation is treated as measurement noise under the stationary-wheel assumption. '...
 'Known-motion profiles use this measured noise and actual sample intervals; they are simulations, not speed ground truth from the moving car. '...
 'The critically damped gain family is screened at <=5ms phase delay at 10Hz and against a continuous-observer LQI model with Kt/delay uncertainty; powered A/B validation is still required.']);
writetable(struct2table(stats),fullfile(out,'capture_statistics.csv'));writetable(struct2table(cases),fullfile(out,'candidates.csv'));
fid=fopen(fullfile(out,'design.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
disp(chosen);
fig=figure('Visible','off');hold on;
for method=1:2
 c=cases([cases.method]==method);x=unique([c.tau_us]);y=arrayfun(@(q)mean([c([c.tau_us]==q).score]),x);
 plot(x/1000,y,'-o','LineWidth',1.3);
end
grid on;xlabel('设计时间参数 (ms)');ylabel('已知运动+实测噪声：平均速度误差 RMS (rad/s)');legend('差分+LPF','αβ');
title('编码器真实采样时序和静止噪声下的参数比较');exportgraphics(fig,fullfile(out,'parameter_tradeoff.png'));close(fig);
end

function x=field(block,columns,type)
x=typecast(reshape(block(:,columns)',[],1),type);x=x(:);
end
function v=replay(t,z,method,tau)
v=zeros(size(t));error=0;
for k=2:numel(t)
 dt=t(k)-t(k-1);delta=z(k)-z(k-1);
 if method==1,v(k)=(tau*v(k-1)+delta)/(tau+dt);
 else
  p=tau/(tau+2*dt);residual=delta-error-dt*v(k-1);
  error=-p*p*residual;v(k)=v(k-1)+(1-p)^2/dt*residual;
 end
end
end
function [z,v]=profile(t,name)
switch name
 case 'static',z=zeros(size(t));v=z;
 case 'slow',z=.3*t;v=.3+zeros(size(t));
 case 'reverse',z=.2*sin(2*pi*1.5*t);v=.2*2*pi*1.5*cos(2*pi*1.5*t);
 case '10Hz',z=.4/(2*pi*10)*sin(2*pi*10*t);v=.4*cos(2*pi*10*t);
 case 'reversal_step',u=(t-t(end)/2)/.02;z=-.006*log(cosh(u));v=-.3*tanh(u);
end
end
function [gain,lag]=frequency(dt,method,tau,hz)
q=exp(-1i*2*pi*hz*dt);
if method==1,h=(1-q)/(tau+dt)/(1-tau/(tau+dt)*q);
else,p=tau/(tau+2*dt);h=(1-p)^2/dt*(1-q)/(1-2*p*q+p*p*q*q);end
h=h/(1i*2*pi*hz);gain=abs(h);lag=-angle(h)/(2*pi*hz);
end

function radius=robust_radius(d,method,tau,dt)
radius=0;Ts=d.Ts;r=d.parameters.wheel_radius;
for kt=[.7 1 1.3]
 for sensorLag=[0 .002 .004]
  % Physical v,theta,q; sensor lag; first/second filter state; actual current.
  A=zeros(7);B=zeros(7,1);A(1:3,1:3)=d.A(2:4,2:4);A(1:3,7)=kt*d.B(2:4,1);
  if sensorLag==0,A(4,4)=-1000;input=zeros(1,7);input([1 3])=[1/r -1];
  else,A(4,[1 3 4])=[1/r -1 -1]/sensorLag;input=zeros(1,7);input(4)=1;end
  if method==1,w=1/tau;else,w=-log(tau/(tau+2*dt))/dt;end
  A(5,:)=w*input;A(5,5)=A(5,5)-w;A(6,[5 6])=[w -w];A(7,7)=-1/.003;B(7)=1/.003;
  E=expm([A B;zeros(1,8)]*Ts);Ad=E(1:7,1:7);Bd=E(1:7,8);
  for delay=0:2
   n=10+delay;F=zeros(n);
   for j=1:n
    z=zeros(n,1);z(j)=1;x=z(1:7);odometry=z(8);previous=z(9);xi=z(10);
    index=6;if method==1,index=5;end
    speed=r*(x(index)+x(3));odometry=odometry+Ts*(previous+speed)/2;
    u=-d.Kaug(1,[1 2 3 4 7])*[odometry;speed;x(2);x(3);xi];xi=xi+Ts*odometry;
    if delay>0,command=z(11);queue=[z(12:end);u];else,command=u;queue=[];end
    F(:,j)=[Ad*x+Bd*command;odometry;speed;xi;queue];
   end
   radius=max(radius,max(abs(eig(F))));
  end
 end
end
end
