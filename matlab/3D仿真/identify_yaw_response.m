function result=identify_yaw_response
% Empirical yaw response to commanded current difference; not a motor Kt estimate.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));folder=fullfile(root,'esp32/lqi_firmware/results/identification');out=fullfile(here,'results/yaw_speed');if ~isfolder(out),mkdir(out);end
files=dir(fullfile(folder,'yaw_speed*/report.json'));train={};valid={};
for n=1:numel(files)
 r=jsondecode(fileread(fullfile(files(n).folder,files(n).name)));t=readtable(fullfile(files(n).folder,'samples.csv'));t=t(t.sample_us>=r.start_us & t.sample_us<=r.start_us+4*(r.move_seconds+2)*1e6,:);
 if contains(files(n).folder,'validate'),valid{end+1}=t;else,train{end+1}=t;end %#ok<AGROW>
end
assert(numel(train)>=3 && ~isempty(valid));best=Inf;
for lag=[0 .005 .01 .02 .035 .05]
 [X,Z,y]=equations(train,lag);Xp=X-Z*(Z\X);yp=y-Z*(Z\y);c=lsqnonneg(Xp,yp);offset=Z\(y-X*c);loss=mean((X*c+Z*offset-y).^2);
 if loss<best,best=loss;chosen=struct('lag_s',lag,'input_gain_rad_s2_per_A',c(1),'viscous_per_s',c(2),'coulomb_rad_s2',c(3),'constant_rad_s2',offset);end
end
error=[];baseline=[];truth=[];prediction=[];
for n=1:numel(valid)
 t=valid{n};s=t.sample_us*1e-6;rate=movmean(t.yaw_rate,3);u=interp1(s,t.command_b-t.command_a,s-chosen.lag_s,'previous','extrap');
 for a=1:20:height(t)-21
  k=a:a+20;if max(diff(s(k)))>.008 || mean(abs(rate(k)))<.01,continue;end
  w=rate(a);
  for j=a:a+19
   h=(s(j+1)-s(j))/10;
   for sub=1:10,w=w+h*(chosen.input_gain_rad_s2_per_A*u(j)-chosen.viscous_per_s*w-chosen.coulomb_rad_s2*tanh(w/.03)+chosen.constant_rad_s2);end
  end
  error(end+1)=w-rate(a+20);baseline(end+1)=rate(a)-rate(a+20);truth(end+1)=rate(a+20);prediction(end+1)=w; %#ok<AGROW>
 end
end
result=chosen;result.train_runs=numel(train);result.validation_runs=numel(valid);result.validation_rate_rms_rad_s=rms(error);result.zero_acceleration_baseline_rms_rad_s=rms(baseline);result.improvement_vs_zero=1-rms(error)/rms(baseline);result.accepted_for_short_prediction=result.improvement_vs_zero>.25;
result.scope='Empirical command-current to yaw-rate response including actuator and ground losses. Not a measured Kt or inertia. 100 ms open-loop whole-run holdout against IMU rate; absolute yaw and long-horizon closed-loop prediction are not validated.';
fid=fopen(fullfile(out,'identified_yaw_response.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
f=figure('Visible','off','Color','w','Position',[80 80 1100 650]);tiledlayout(2,1);
nexttile;plot([truth;prediction]');legend('IMU实测角速度','100 ms经验模型预测');ylabel('偏航角速度 (rad/s)');grid on;
nexttile;plot([baseline;error]');legend('零加速度基线误差','经验模型误差');ylabel('预测误差 (rad/s)');xlabel('未参与拟合的验证窗口');grid on;
sgtitle('不同快慢转向：指令电流差→偏航角速度经验模型；非电机力矩常数辨识');exportgraphics(f,fullfile(out,'yaw_response_validation.png'),'Resolution',140);savefig(f,fullfile(out,'yaw_response_validation.fig'));close(f);disp(result);
end
function [X,Z,y]=equations(runs,lag)
X=[];Z=[];y=[];
for n=1:numel(runs)
 t=runs{n};s=t.sample_us*1e-6;rate=movmean(t.yaw_rate,3);u=interp1(s,t.command_b-t.command_a,s-lag,'previous','extrap');
 for a=1:10:height(t)-21
  k=a:a+20;dt=s(k(end))-s(k(1));if max(diff(s(k)))>.008 || dt<.095 || dt>.11 || any(t.fault(k)) || mean(abs(rate(k)))<.01,continue;end
  X=[X;trapz(s(k),u(k)),-trapz(s(k),rate(k)),-trapz(s(k),tanh(rate(k)/.03))];Z=[Z;dt];y=[y;rate(k(end))-rate(k(1))]; %#ok<AGROW>
 end
end
end
