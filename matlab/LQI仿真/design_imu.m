function design_imu(folder)
% Measured fixed-stand noise replay. Synthetic motion is not ground validation.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin==0
 d=dir(fullfile(root,'esp32/lqi_firmware/results/imu/*/imu.csv'));
 [~,i]=max([d.datenum]);folder=d(i).folder;
end
a=readtable(fullfile(folder,'imu.csv'),'TextType','string');
[~,label]=fileparts(folder);
out=fullfile(root,'matlab/LQI仿真/results/imu_measured',label);if ~exist(out,'dir'),mkdir(out);end
stages=unique(a.stage,'stable');stats=struct([]);
for k=1:numel(stages)
 q=a(a.stage==stages(k),:);t=(double(q.time_us)-double(q.time_us(1)))*1e-6;
 acc=[q.ax q.ay q.az]/16384;g=[q.gyro_x q.gyro_y q.gyro_z];
 dt=diff(t);theta=atan2(acc(:,2),acc(:,3));
 stats(k).stage=char(stages(k));stats(k).samples=height(q);stats(k).duration_s=t(end);
 stats(k).median_dt_us=median(dt)*1e6;stats(k).p99_dt_us=prctile(dt,99)*1e6;
 stats(k).max_dt_us=max(dt)*1e6;stats(k).long_gaps_over_7p5ms=sum(dt>.0075);
 stats(k).temperature_C_range=[min(q.temperature_raw) max(q.temperature_raw)]/340+36.53;
 stats(k).gyro_mean_deg_s=mean(g)*180/pi;stats(k).gyro_std_deg_s=std(g)*180/pi;
 stats(k).acc_std_mg=std(acc)*1000;stats(k).acc_pitch_std_deg=std(theta)*180/pi;
 stats(k).pitch_std_deg=std(q.pitch)*180/pi;
 stats(k).acc_norm_g_mean=mean(vecnorm(acc,2,2));
 words=[q.ax q.ay q.az q.temperature_raw q.gx q.gy q.gz];
 stats(k).identical_consecutive_raw_samples=sum(all(diff(words)==0,2));
end
q=a(a.stage=="static",:);t=(double(q.time_us)-double(q.time_us(1)))*1e-6;
dt=median(diff(t));tt=(0:dt:t(end))';
g=interp1(t,[q.gyro_x q.gyro_y q.gyro_z],tt,'linear');
accangle=interp1(t,atan2(double(q.ay),double(q.az)),tt,'linear');
temp=interp1(t,double(q.temperature_raw)/340+36.53,tt,'linear');
noise=accangle-mean(accangle);n=numel(tt);fs=1/dt;
periods=[.005 .01 .02 .05 .1 .2 .5 1 2 5 10];allan=nan(numel(periods),3);
for k=1:numel(periods)
 m=max(1,round(periods(k)/dt));nb=floor(n/m);
 for j=1:3
  means=mean(reshape(g(1:m*nb,j),m,nb),1);
  allan(k,j)=sqrt(mean(diff(means).^2)/2)*180/pi;
 end
end
% Known small-angle pitch, passed through nominal datasheet delays, plus measured noise.
T=tt;truth=(5*sin(2*pi*.5*T)+2*sin(2*pi*2*T)+.5*sin(2*pi*6*T))*pi/180;
rate=(5*2*pi*.5*cos(2*pi*.5*T)+2*2*pi*2*cos(2*pi*2*T)+.5*2*pi*6*cos(2*pi*6*T))*pi/180;
gyro=interp1(T,rate,max(0,T-.0048),'linear')+g(:,1);
acc=interp1(T,truth,max(0,T-.0049),'linear')+noise;
pulse=double(T>20 & T<21)*atan(.15);candidates=cell(1,8);idx=0;
for tau=[.1 .245 .5 1]
 idx=idx+1;r=estimate(g(:,1),accangle,dt,tau,0);y=estimate(gyro,acc,dt,tau,0);
 z=estimate(zeros(n,1),pulse,dt,tau,0);
 candidates{idx}=metrics('complementary',tau,r,y,z,truth,dt);
end
for fc=[.1 .2 .4 .8]
 idx=idx+1;r=estimate(g(:,1),accangle,dt,fc,1);y=estimate(gyro,acc,dt,fc,1);
 z=estimate(zeros(n,1),pulse,dt,fc,1);
 candidates{idx}=metrics('angle_bias_observer',fc,r,y,z,truth,dt);
end
candidates=[candidates{:}];
training=tt<min(30,tt(end)/3);validation=tt>tt(end)*.65;
coef=polyfit(temp(training),g(training,1),1);
pred=polyval(coef,temp(validation));base=mean(g(training,1));
thermal=struct('temperature_span_C',range(temp),'fit_deg_s_per_C',coef(1)*180/pi,...
 'heldout_rms_before_deg_s',rms(g(validation,1)-base)*180/pi,...
 'heldout_rms_after_deg_s',rms(g(validation,1)-pred)*180/pi,...
 'applied',false,'scope','Single warm-up trend; temperature and elapsed time are confounded. No thermal compensation is deployed.');
report=struct('source',folder,'stages',stats,'candidates',candidates,'thermal_screen',thermal,...
 'scope','Stand-still statistics and measured-noise synthetic pitch replay only. Stand rigidity is not independently measured. No ground acceleration, multi-pose calibration or closed-loop validation.',...
 'hardware_dlpf',struct('setting',3,'gyro_bandwidth_Hz',42,'gyro_nominal_delay_ms',4.8,'acc_bandwidth_Hz',44,'acc_nominal_delay_ms',4.9));
writetable(struct2table(candidates),fullfile(out,'candidates.csv'));
fid=fopen(fullfile(out,'report.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
fig=figure('Visible','off','Position',[100 100 1400 950]);tiledlayout(2,2,'TileSpacing','compact');
nexttile;plot(tt,temp);grid on;xlabel('时间 (s)');ylabel('芯片温度 (°C)');title('悬空静止温升');
nexttile;plot(tt,g*180/pi);grid on;xlabel('时间 (s)');ylabel('校准后角速度 (°/s)');legend('X','Y','Z');title('启动零偏校准后的实际漂移与噪声');
nexttile;loglog(periods,allan,'-o');grid on;xlabel('平均时间 (s)');ylabel('Allan 偏差 (°/s)');legend('X','Y','Z');title('非重叠 Allan 偏差：仅本次温升记录');
nexttile;[f,px]=spectrum(g(:,1)*180/pi,fs);[~,pa]=spectrum(noise*180/pi,fs);
semilogy(f,px,f,pa);grid on;xlim([0 100]);xlabel('频率 (Hz)');ylabel('功率谱密度');legend('陀螺 X [(°/s)^2/Hz]','加速度俯仰 [°^2/Hz]');title('静止功率谱：不同单位分别比较');
sgtitle('IMU 原始数据：当前 DLPF=3，200 Hz；不代表落地效果');
exportgraphics(fig,fullfile(out,'imu_noise.png'),'Resolution',160);close(fig);
fig=figure('Visible','off','Position',[100 100 1450 550]);tiledlayout(1,3,'TileSpacing','compact');
names=string({candidates.kind})+" "+string([candidates.parameter]);
nexttile;bar([candidates.static_pitch_std_deg]);xticks(1:8);xticklabels(names);xtickangle(60);ylabel('°');grid on;title('实测静止数据回放：俯仰标准差');
nexttile;bar([candidates.synthetic_tracking_rms_deg]);xticks(1:8);xticklabels(names);xtickangle(60);ylabel('°');grid on;title('合成已知运动＋实测噪声：跟踪误差');
nexttile;bar([candidates.synthetic_0p15g_false_pitch_peak_deg]);xticks(1:8);xticklabels(names);xtickangle(60);ylabel('°');grid on;title('合成0.15g加速1秒：错误俯仰峰值');
sgtitle('仅候选筛选，未替换实机融合；互补参数为秒，偏置观测器参数为Hz');
exportgraphics(fig,fullfile(out,'imu_candidates.png'),'Resolution',160);close(fig);
disp(struct2table(candidates));disp(thermal);
end

function y=estimate(g,a,dt,p,bias_observer)
y=zeros(size(g));y(1)=a(1);bias=0;
if bias_observer,pole=exp(-2*pi*p*dt);alpha=1-pole^2;beta=(1-pole)^2;else,alpha=dt/(p+dt);beta=0;end
for i=2:numel(g)
 pred=y(i-1)+(g(i)-bias)*dt;e=atan2(sin(a(i)-pred),cos(a(i)-pred));
 y(i)=pred+alpha*e;bias=bias-beta/dt*e;
end
end

function c=metrics(kind,param,r,y,z,truth,dt)
take=(1:numel(r))'>round(5/dt);
c=struct('kind',kind,'parameter',param,'static_pitch_std_deg',std(r(take))*180/pi,...
 'synthetic_tracking_rms_deg',rms(y(take)-truth(take))*180/pi,...
 'synthetic_0p15g_false_pitch_peak_deg',max(abs(z))*180/pi);
end

function [f,p]=spectrum(x,fs)
nfft=min(2048,2^floor(log2(numel(x))));w=.5-.5*cos(2*pi*(0:nfft-1)'/(nfft-1));p=zeros(nfft/2+1,1);count=0;
for k=1:nfft/2:numel(x)-nfft+1
 z=x(k:k+nfft-1);z=z-mean(z);q=abs(fft(z.*w)).^2/(fs*sum(w.^2));q=q(1:nfft/2+1);q(2:end-1)=2*q(2:end-1);p=p+q;count=count+1;
end
p=p/count;f=(0:nfft/2)'*fs/nfft;
end
