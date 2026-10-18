function estimate=estimate_current_torque_scale
% 计算初始电流/力矩比例，并用已有3D坡中停车记录估计所需电流。
% 这里没有辨识算法；PID参数不能代替实测输入/响应数据。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
p=balance_current_parameters;
estimate=struct('method','nominal KV estimate, not identified from author PID', ...
 'KV_rpm_per_V',p.KV,'Kt_Nm_per_iq_A',p.Kt_nominal, ...
 'iq_convention','amplitude-invariant Park transform, phase-peak equivalent', ...
 'author_current_limit_A',p.current_limit_each);
currents=[.5;1;2;3;6];
mapping=table(currents,currents*p.Kt_nominal,'VariableNames',{'iq_A','torque_Nm'});
out=fullfile(paths.results,'current_torque_estimate');if ~isfolder(out),mkdir(out);end
writetable(mapping,fullfile(out,'nominal_mapping.csv'));
file=fullfile(paths.results,'suite','slope','slope_hold_pid','trajectory.csv');
if isfile(file)
 data=readtable(file);mask=data.time_s>=12&data.time_s<=18.5;
 torque=[data.tau_left_Nm data.tau_right_Nm];
 estimate.old_slope_hold_mean_each_torque_Nm=mean(abs(torque(mask,:)),'all');
 estimate.old_slope_hold_estimated_mean_each_iq_A=estimate.old_slope_hold_mean_each_torque_Nm/p.Kt_nominal;
 estimate.old_slope_run_peak_each_torque_Nm=max(abs(torque),[],'all');
 estimate.old_slope_run_estimated_peak_each_iq_A=estimate.old_slope_run_peak_each_torque_Nm/p.Kt_nominal;
end
fid=fopen(fullfile(out,'estimate.json'),'w');assert(fid>=0);
fprintf(fid,'%s',jsonencode(estimate,PrettyPrint=true));fclose(fid);
fprintf('标称KV估计Kt=%.8g Nm/A；不是PID反推或实测值。\n',p.Kt_nominal);
disp(mapping);disp(estimate);
end
