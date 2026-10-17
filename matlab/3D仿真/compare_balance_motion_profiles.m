function metrics=compare_balance_motion_profiles
% 对比同一个PID在常规和快速指令下的实际3D响应。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
names={'normal','agile'};rows=cell(2,1);data=cell(2,1);
for k=1:2
 folder=fullfile(balance_result_folder(paths.results,'pid',1,names{k}),'drive_pid');
 data{k}=readtable(fullfile(folder,'trajectory.csv'));
 r=jsondecode(fileread(fullfile(folder,'validation.json')));
 reverseStart=6;if k==2,reverseStart=3.8;end
 peakForward=max(data{k}.distance_m(data{k}.time_s>=1&data{k}.time_s<reverseStart));
 rows{k}=struct('profile',names{k},'forward_90percent_s',r.first_forward_90percent_time_s, ...
  'max_speed_mps',r.max_speed_mps,'peak_pitch_deg',r.max_tilt_error_deg, ...
  'forward_overshoot_m',max(0,peakForward-.3),'final_distance_error_m',r.final_distance_error_m,'peak_each_torque_Nm',r.peak_each_torque_Nm, ...
  'peak_approx_slip_mps',r.peak_approx_rolling_slip_mps,'saturation_fraction',r.saturated_fraction,'pass',r.pass);
end
folder=fullfile(paths.results,'suite','agile');metrics=struct2table(vertcat(rows{:}));
writetable(metrics,fullfile(folder,'normal_vs_agile.csv'));
f=figure('Visible','off','Color','w','Position',[60 40 1120 820]);tiledlayout(3,2);
for row=1:3
 for k=1:2
  tab=data{k};nexttile;
  if row==1,plot(tab.time_s,tab.distance_m,tab.time_s,tab.distance_ref_m,'--');ylabel('路程 / m');ylim([-.05 .4]);legend('实际','指令');
  elseif row==2,plot(tab.time_s,tab.speed_mps);ylabel('速度 / m/s');ylim([-.7 .7]);
  else,plot(tab.time_s,rad2deg(tab.tilt_error_rad),tab.time_s,rad2deg(tab.pid_pitch_target_rad),'--');ylabel('俯仰偏差 / deg');ylim([-25 25]);xlabel('时间 / s');legend('实际','目标');end
  grid on;title(names{k});
 end
end
sgtitle('同一3D小车与PID：常规指令 / 快速指令（允许更大倾角）');
exportgraphics(f,fullfile(folder,'normal_vs_agile.png'),'Resolution',140);close(f);disp(metrics);
end
