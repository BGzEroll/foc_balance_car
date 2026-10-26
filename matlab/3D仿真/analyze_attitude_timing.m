function result=analyze_attitude_timing
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));folder=fullfile(root,'esp32/lqi_firmware/results/identification/linear_08_raw_validate_b_20261026');out=fullfile(here,'results/attitude_motion');if ~isfolder(out),mkdir(out);end
s=readtable(fullfile(folder,'samples.csv'));i=readtable(fullfile(folder,'imu.csv'));[~,idx]=unique(s.imu_us);s=s(idx,:);t=s.imu_us*1e-6;
angle=s.pitch;rate=s.pitch_rate;horizons=[.025 .05 .1];rms_error=[];peak_error=[];count=[];
for h=horizons
 errors=[];
 for a=1:5:height(s)-1
  b=find(t>=t(a)+h-.001,1);if isempty(b),continue;end
  k=a:b;if max(diff(t(k)))>.008 || t(b)-t(a)>h+.008,continue;end
  errors(end+1)=angle(b)-angle(a)-trapz(t(k),rate(k)); %#ok<AGROW>
 end
 rms_error(end+1)=rms(errors)*180/pi;peak_error(end+1)=max(abs(errors))*180/pi;count(end+1)=numel(errors); %#ok<AGROW>
end
lags=-.04:.001:.04;errors=zeros(size(lags));
for n=1:numel(lags)
 q=interp1(t,rate,t+lags(n),'linear',NaN);d=gradient(movmean(angle,5),t);good=isfinite(q);errors(n)=rms(q(good)-d(good));
end
[~,best]=min(errors);
result=struct('horizon_s',horizons,'pitch_integral_consistency_rms_deg',rms_error,'peak_deg',peak_error,'windows',count,'best_rate_shift_s',lags(best),'best_derivative_residual_rad_s',errors(best),'median_current_age_us',[median(s.current_age_a_us),median(s.current_age_b_us)],'median_imu_age_us',median(s.imu_age_us),'scope','Consistency between ESKF angle and bias-corrected gyro rate; no external attitude truth. Nonzero residual includes gravity corrections and timing, not solely sensor lag.');
fid=fopen(fullfile(out,'timing_analysis.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
qint=cumtrapz(t,rate)+angle(1);f=figure('Visible','off','Color','w','Position',[80 80 1150 800]);tiledlayout(3,1);
nexttile;plot(t-t(1),[angle,qint]*180/pi);legend('实机ESKF俯仰','同段角速度积分');ylabel('俯仰角 (度)');grid on;
nexttile;plot(t-t(1),(angle-qint)*180/pi);ylabel('姿态修正累计量 (度)');grid on;
nexttile;plot(lags*1000,errors);xlabel('角速度相对角度导数平移 (ms)');ylabel('一致性残差 (rad/s)');grid on;
sgtitle('8 cm动态验证：分离姿态修正与时间错位；无外部姿态真值');exportgraphics(f,fullfile(out,'timing_analysis.png'),'Resolution',140);savefig(f,fullfile(out,'timing_analysis.fig'));close(f);disp(result);
end
