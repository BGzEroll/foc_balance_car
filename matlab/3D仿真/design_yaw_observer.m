function report=design_yaw_observer
% Encoder-qualified zero-yaw-rate bias learning. No encoder-angle correction.
folder=fullfile(fileparts(mfilename('fullpath')),'results/yaw_bias');
t=readtable(fullfile(folder,'baseline.csv'));time=t.time_s-t.time_s(1);dt=[.005;diff(time)];
z=replay(dt,t.raw_yaw,t.gyro_rate,t.left,t.right,true(height(t),1));
reference=table(dt,t.raw_yaw,t.gyro_rate,t.left,t.right,z(:,1),z(:,2),z(:,3),z(:,4),z(:,5),z(:,6),...
 'VariableNames',{'dt_s','raw_heading','gyro_rate','left','right','raw_yaw','corrected_yaw','bias','encoder_yaw','quiet_time','learning'});
writetable(reference,fullfile(folder,'reference.csv'));
q=time>30;raw=slope(time(q),z(q,1))*180/pi*60;corrected=slope(time(q),z(q,2))*180/pi*60;encoder=slope(time(q),z(q,4))*180/pi*60;
report=struct('seconds',time(end),'accepted_seconds',z(end,5),'accepted_fraction',z(end,5)/time(end),'final_bias_deg_s',z(end,3)*180/pi,...
 'raw_rate_integral_drift_deg_min',raw,'corrected_integral_drift_deg_min',corrected,'encoder_drift_deg_min',encoder,...
 'window_s',1,'bias_learning_time_constant_s',15,'scope','Recorded uncorrected-control replay. Encoder angle is diagnostic only; only near-zero wheel-motion windows qualify gyro-mean bias updates.');
report.synthetic=cell(4,1);
for scenario=1:4
 dt=.005*(1+.03*sin(.7*(1:24000)'));time2=cumsum(dt);truth=zeros(size(dt));rate=truth;commanded=false(size(dt));
 for segment=1:4
  amp=[1 -1 -1 1]*pi/3;start=20+10*(segment-1);[a,b]=step(time2,start,3,amp(segment));truth=truth+a;rate=rate+b;commanded=commanded | (time2>=start & time2<start+5);
 end
 encoder2=1.05*truth;
 if scenario==2,encoder2=encoder2+4*pi/180*(time2>=80);end
 if scenario>=3
  slow=time2>=70 & time2<100;rate=rate+slow*.05*pi/180;truth=truth+max(0,min(30,time2-70))*.05*pi/180;
  if scenario==4,encoder2=encoder2+max(0,min(30,time2-70))*.05*pi/180;commanded=commanded | slow;end
 end
 k=.0325/.1325364;quant=2*pi/16384;l=round(-encoder2/(2*k)/quant)*quant;r=round(encoder2/(2*k)/quant)*quant;
 true_bias=.03*pi/180;gyro=rate+true_bias+.0004*sin(2*pi*17*time2);
 zz=replay(dt,truth,gyro,l,r,~commanded);moving=abs(rate)>.05;
 report.synthetic{scenario}=struct('scenario',scenario,'encoder_jump_deg',4*(scenario==2),'unseen_external_slow_turn',scenario==3,...
  'final_heading_error_deg',(zz(end,2)-truth(end))*180/pi,'final_bias_error_deg_s',(zz(end,3)-true_bias)*180/pi,...
  'learned_seconds_during_fast_turn',sum(dt(moving & zz(:,6)>0)),...
  'corrected_jump_at_encoder_step_deg',max(abs(diff(zz(time2>79.9 & time2<80.1,2))))*180/pi);
end
fid=fopen(fullfile(folder,'design.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
fig=figure('Visible','off','Position',[80 80 1100 720]);tiledlayout(3,1);
nexttile;plot(time,180/pi*[z(:,1)-z(1,1),z(:,2)-z(1,2),z(:,4)-z(1,4)]);grid on;ylabel('相对航向 (°)');legend('未修正陀螺积分','零偏修正后积分','编码器仅作对比');
nexttile;plot(time,z(:,3)*180/pi);grid on;ylabel('学习零偏 (°/s)');
nexttile;stairs(time,z(:,6));grid on;ylabel('上个窗口可学习');xlabel('时间 (s)');
sgtitle('编码器门控陀螺零偏：不融合编码器航向角');exportgraphics(fig,fullfile(folder,'bias_replay.png'),'Resolution',140);close(fig);disp(report);
end
function z=replay(dt,heading,rate,l,r,allow)
dt=single(dt);heading=single(heading);rate=single(rate);l=single(l);r=single(r);k=single(.0325)/single(.1325364);
n=numel(dt);z=zeros(n,6);raw=heading(1);yaw=raw;enc=raw;bias=single(0);seconds=single(0);integral=single(0);travel=single(0);minimum=enc;maximum=enc;wa=enc;quiet=single(0);candidate=true;learn=false;z(1,:)=[raw yaw bias enc quiet learn];
for i=2:n
 dl=l(i)-l(i-1);dr=r(i)-r(i-1);enc=heading(1)+k*((r(i)-r(1))-(l(i)-l(1)));
 seconds=seconds+dt(i);integral=integral+rate(i)*dt(i);travel=travel+single(.0325)*(abs(dl)+abs(dr))*single(.5);
 minimum=min(minimum,enc);maximum=max(maximum,enc);candidate=candidate && allow(i) && abs(rate(i)-bias)<single(.02);
 if seconds>=single(1)
  mean_rate=integral/seconds;learn=candidate && abs(enc-wa)<single(.00004712389) && maximum-minimum<single(.000872664626) && travel<single(.005) && abs(mean_rate-bias)<single(.001745329252);
  if learn,bias=bias+(single(1)-exp(-seconds/single(15)))*(mean_rate-bias);quiet=quiet+seconds;end
  seconds=single(0);integral=single(0);travel=single(0);wa=enc;minimum=enc;maximum=enc;candidate=true;
 end
 raw=raw+dt(i)*rate(i);yaw=yaw+dt(i)*(rate(i)-bias);z(i,:)=[raw yaw bias enc quiet learn];
end
end
function b=slope(t,y),t=t-mean(t);b=sum(t.*(y-mean(y)))/sum(t.*t);end
function [s,v]=step(t,start,T,amp)
u=max(0,min(1,(t-start)/T));s=amp*(10*u.^3-15*u.^4+6*u.^5);v=amp/T*(30*u.^2-60*u.^3+30*u.^4);
end
