function result=analyze_sensor_noise
% Offline only. Raw IMU values already passed MPU hardware DLPF, before software fusion.
r=fileparts(mfilename('fullpath'));files=dir(fullfile(r,'still_*.csv'));A=[];G=[];E=[];means=[];rhoA=[];rhoG=[];dts=[];age=[];SA=[];SG=[];segments=cell(numel(files),1);accSegments=segments;gyroSegments=segments;
for k=1:numel(files)
 n=erase(files(k).name,'.csv');[T,J,a,g,t]=readburst(r,n);aa=a-mean(a);gg=g-mean(g);ee=[T.left_raw T.right_raw];ee=ee-mean(ee);
 A=[A;aa];G=[G;gg];E=[E;ee];means(end+1,:)=[mean(t) mean(a) mean(g) mean(double(T.temp_raw)/340+36.53)];
 rhoA=[rhoA;ar1(aa)];rhoG=[rhoG;ar1(gg)];dts=[dts;diff(double(T.imu_us))*1e-6];age=[age;double(T.captured_us-T.left_us)*1e-6 double(T.captured_us-T.right_us)*1e-6];
 [freq,pa]=psd(aa,1/median(diff(t)));[~,pg]=psd(gg,1/median(diff(t)));SA(:,:,k)=pa;SG(:,:,k)=pg;segments{k}=T;
 accSegments{k}=aa;gyroSegments{k}=gg;
end
Ca=cov(A);Cg=cov(G);pa=mean(SA,3);pg=mean(SG,3);ra=median(rhoA,'all');rg=median(rhoG,'all');
parameters=struct('sample_count_static',size(A,1),'sample_rate_hz',1/median(dts), ...
 'rho_acc',max(-.95,min(.98,ra)),'rho_gyro',max(-.95,min(.98,rg)), ...
 'acc_cov_g2',Ca,'gyro_cov_rad2_s2',Cg,'acc_sigma_g',sqrt(diag(Ca))', ...
 'gyro_sigma_rad_s',sqrt(diag(Cg))','gyro_residual_bias_rad_s',mean(means(:,5:7)), ...
 'encoder_jitter_counts',std(E),'encoder_counts_per_rev',4096,'encoder_jitter_includes_quantization',true, ...
 'stationary_mean_specific_force_g',mean(vecnorm(means(:,2:4),2,2)), ...
 'acc_common_scale_single_pose_prior',mean(vecnorm(means(:,2:4),2,2)), ...
 'acc_scale_10',1,'acc_scale_20',1,'gyro_scale_10',1,'gyro_scale_20',1, ...
 'acc_cov_10_g2',Ca,'acc_cov_20_g2',Ca,'gyro_cov_10_rad2_s2',Cg,'gyro_cov_20_rad2_s2',Cg, ...
 'rho_acc_10',ra,'rho_acc_20',ra,'rho_gyro_10',rg,'rho_gyro_20',rg, ...
 'imu_hardware_config',struct('SMPLRT_DIV',4,'DLPF_CFG',3,'gyro_full_scale_deg_s',500,'acc_full_scale_g',2), ...
 'software_filter',struct('complementary_acc_weight',.02,'encoder_speed_tf_s',.003), ...
 'assumptions',{{'stationary fixture orientation is not an accelerometer bias calibration','single-pose acceleration norm does not identify per-axis gain versus bias; common scale is only a modeling prior','motor-on captures describe suspended fixture vibration, not ground driving','AR1 covariance is an equivalent noise model, not a full PSD or long-term bias model'}});
parameters.acc_ar_static=fit_ar(accSegments);parameters.gyro_ar_static=fit_ar(gyroSegments);
moving=struct([]);movingPSD=cell(2,2);
for k=1:2
 speed=10*k;n=sprintf('moving_%d',speed);[T,J,a,g,t]=readburst(r,n);I=t-t(1)>1;
 aa=a(I,:)-mean(a(I,:));gg=g(I,:)-mean(g(I,:));cam=cov(aa);cgm=cov(gg);
 parameters.(sprintf('acc_scale_%d',speed))=sqrt(trace(cam)/trace(Ca));parameters.(sprintf('gyro_scale_%d',speed))=sqrt(trace(cgm)/trace(Cg));
 parameters.(sprintf('acc_cov_%d_g2',speed))=cam;parameters.(sprintf('gyro_cov_%d_rad2_s2',speed))=cgm;
 parameters.(sprintf('rho_acc_%d',speed))=max(-.95,min(.98,median(ar1(aa))));parameters.(sprintf('rho_gyro_%d',speed))=max(-.95,min(.98,median(ar1(gg))));
 parameters.(sprintf('acc_ar_%d',speed))=fit_ar({aa});parameters.(sprintf('gyro_ar_%d',speed))=fit_ar({gg});
 [fm,pam]=psd(aa,1/median(diff(t)));[~,pgm]=psd(gg,1/median(diff(t)));movingPSD{k,1}=pam;movingPSD{k,2}=pgm;
 row=struct('speed_command_rad_s',speed,'stable_samples',sum(I),'acc_sigma_g',std(aa),'gyro_sigma_rad_s',std(gg), ...
  'acc_variance_ratio_to_still',trace(cam)/trace(Ca),'gyro_variance_ratio_to_still',trace(cgm)/trace(Cg));
 if isempty(moving),moving=row;else,moving(end+1)=row;end
end
result=struct('parameters',parameters,'static_bursts',numel(files),'static_duration_recorded_s',sum(cellfun(@height,segments))/parameters.sample_rate_hz, ...
 'imu_interval_us',struct('median',median(dts)*1e6,'std',std(dts)*1e6,'min',min(dts)*1e6,'max',max(dts)*1e6), ...
 'snapshot_encoder_age_max_us',max(age)*1e6,'snapshot_not_full_encoder_interarrival',true, ...
 'burst_means_columns',{{'time_s','ax_g','ay_g','az_g','gx_rad_s','gy_rad_s','gz_rad_s','temperature_C'}}, ...
 'burst_means',means,'temperature_range_C',[min(means(:,8)) max(means(:,8))], ...
 'moving',moving,'long_term_allan_bias_identified',false,'external_motion_reference',false);
writejson(fullfile(r,'sensor_noise_parameters.json'),parameters);writejson(fullfile(r,'analysis.json'),result);
f=figure('Visible','off','Color','w','Position',[40 40 1400 900]);tl=tiledlayout(2,2,'Padding','compact');
T=segments{1};[~,~,a,g,t]=readburst(r,'still_01');
nexttile;plot(t-t(1),a-mean(a));grid on;xlabel('时间 / s');ylabel('去段均值加速度 / g');legend('X','Y','Z');title('静止：片内滤波后的原始加速度');
nexttile;plot(t-t(1),g*180/pi);grid on;xlabel('时间 / s');ylabel('校准后角速度 / deg/s');legend('X','Y','Z');title('静止：陀螺零偏校准后的残余');
nexttile;semilogy(freq,sum(pa,2),'LineWidth',1.2);hold on;for k=1:2,semilogy(fm,sum(movingPSD{k,1},2));end;grid on;xlabel('频率 / Hz');ylabel('三轴加速度PSD和 / g^2/Hz');legend('静止','10rad/s架空','20rad/s架空');title('振动与静止噪声的频谱对照');
nexttile;semilogy(freq,sum(pg,2),'LineWidth',1.2);hold on;for k=1:2,semilogy(fm,sum(movingPSD{k,2},2));end;grid on;xlabel('频率 / Hz');ylabel('三轴陀螺PSD和 / (rad/s)^2/Hz');legend('静止','10rad/s架空','20rad/s架空');
title(tl,'MPU6050：200Hz读取、DLPF_CFG=3；转动频谱包含架空机构振动');exportgraphics(f,fullfile(r,'sensor_noise.png'),'Resolution',150);savefig(f,fullfile(r,'sensor_noise.fig'));close(f);
disp(result);fprintf('SENSOR_NOISE_ANALYSIS_COMPLETE sigma_acc_g=%s sigma_gyro_deg_s=%s encoder_count_std=%s\n',mat2str(parameters.acc_sigma_g,4),mat2str(parameters.gyro_sigma_rad_s*180/pi,4),mat2str(parameters.encoder_jitter_counts,4));
end
function [T,J,a,g,t]=readburst(r,n)
T=readtable(fullfile(r,[n '.csv']));J=jsondecode(fileread(fullfile(r,[n '.json'])));
a=[T.ax_raw T.ay_raw T.az_raw]/16384;g=[T.gx_raw T.gy_raw T.gz_raw]*pi/(180*65.5)-J.meta.gyro_bias_rad_s(:)';t=double(T.imu_us)*1e-6;
end
function v=ar1(x)
v=sum(x(1:end-1,:).*x(2:end,:),1)./sqrt(sum(x(1:end-1,:).^2,1).*sum(x(2:end,:).^2,1));v(~isfinite(v))=0;
end
function [f,P]=psd(x,fs)
N=256;w=.5-.5*cos(2*pi*(0:N-1)'/N);P=zeros(N/2+1,size(x,2));n=0;
for a=1:N/2:size(x,1)-N+1
 z=x(a:a+N-1,:);z=z-mean(z);y=fft(z.*w);s=abs(y(1:N/2+1,:)).^2/(fs*sum(w.^2));s(2:end-1,:)=2*s(2:end-1,:);P=P+s;n=n+1;
end
assert(n>0);P=P/n;f=(0:N/2)'*fs/N;
end
function writejson(file,s)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(s,PrettyPrint=true));fclose(fid);
end
function f=fit_ar(segments)
P=12;coeff=zeros(P,3);residual=[];target=[];
for axis=1:3
 H=[];Y=[];
 for k=1:numel(segments)
  x=segments{k}(:,axis);h=zeros(numel(x)-P,P);for lag=1:P,h(:,lag)=x(P+1-lag:end-lag);end
  H=[H;h];Y=[Y;x(P+1:end)];
 end
 a=-(H'*H+eye(P)*1e-5*trace(H'*H)/P)\(H'*Y);z=roots([1;a]);z(abs(z)>.995)=.995*z(abs(z)>.995)./abs(z(abs(z)>.995));polyA=real(poly(z));a=polyA(2:end)';coeff(:,axis)=a;residual(:,axis)=Y+H*a;target(axis)=var(Y);
end
C=cov(residual);D=sqrt(diag(C));Corr=C./(D*D');innov=zeros(3,1);
for axis=1:3
 state=zeros(P,1);power=0;
 for k=1:5000,u=double(k==1)-coeff(:,axis)'*state;power=power+u*u;state=[u;state(1:end-1)];end
 innov(axis)=sqrt(target(axis)/power);
end
L=chol(diag(innov)*Corr*diag(innov)+eye(3)*1e-16,'lower');maxPole=0;
for axis=1:3,maxPole=max(maxPole,max(abs(roots([1;coeff(:,axis)]))));end
f=struct('order',P,'coefficients',coeff,'innovation_chol',L,'maximum_pole_magnitude',maxPole,'target_variance',target,'source','regularized AR12 per axis plus correlated innovations; finite stationary/motor-on segments');
power=[];
for k=1:numel(segments),[freq,sp]=psd(segments{k},200);if isempty(power),power=zeros(size(sp));end;power=power+sp/numel(segments);end
f.spectrum_frequency_hz=freq;f.spectrum_total_PSD=sum(power,2);mask=freq>=10&freq<=80;ff=freq(mask);pp=f.spectrum_total_PSD(mask);[~,idx]=max(pp);f.dominant_10_80Hz_peak_hz=ff(idx);
end
