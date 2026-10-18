function summary=compare_slope_controllers
% 同一坡道、同一停车轨迹的真实Multibody结果对比。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
modes={'lqr','pid'};rows=cell(2,1);data=cell(2,1);
for k=1:2
 folder=fullfile(paths.suite_results,'slope',['slope_hold_' modes{k}]);
 r=jsondecode(fileread(fullfile(folder,'validation.json')));
 design=jsondecode(fileread(fullfile(folder,'controller_config.json')));settings=design.terrain;
 tab=readtable(fullfile(folder,'trajectory.csv'));pose=readtable(fullfile(folder,'body_pose.csv'));
 tab.world_y_m=interp1(pose.time_s,pose.y_m,tab.time_s);data{k}=tab;
 late=tab.time_s>=settings.hold_check_start&tab.time_s<=settings.hold_check_end;
 tab.hold_error_m=tab.world_y_m-settings.hold_y;data{k}=tab;
 rows{k}=struct('controller',upper(modes{k}),'preset','slope','slope_deg',r.terrain_angle_deg, ...
  'height_m',r.terrain_height_m,'hold_mean_y_error_m',r.hold_mean_y_error_m, ...
  'hold_y_span_m',r.hold_y_span_m,'hold_mean_abs_speed_mps',r.hold_mean_abs_speed_mps, ...
  'hold_on_incline_fraction',r.hold_on_incline_fraction,'hold_stability_pass',r.hold_stability_pass, ...
  'hold_tracking_pass',r.hold_tracking_pass,'hold_mean_pitch_deg',mean(rad2deg(tab.tilt_error_rad(late))), ...
  'hold_mean_each_torque_Nm',mean((tab.tau_left_Nm(late)+tab.tau_right_Nm(late))/2), ...
  'full_course_pass',r.pass,'final_distance_error_m',r.final_distance_error_m);
end
summary=struct2table(vertcat(rows{:}));folder=fullfile(paths.suite_results,'slope');
writetable(summary,fullfile(folder,'hold_comparison.csv'));
f=figure('Visible','off','Color','w','Position',[40 30 1150 850]);tiledlayout(2,2);
nexttile;hold on;
for k=1:2,plot(data{k}.time_s,data{k}.world_y_m,'LineWidth',1.2);end
plot(data{1}.time_s,data{1}.distance_ref_m,'k--');grid on;xline(7,':');xline(19,':');ylabel('水平位置 / m');legend('LQR','PID坡道整定','指令');title('7～19秒在上坡中段停车');
nexttile;hold on;
for k=1:2,plot(data{k}.time_s,data{k}.hold_error_m,'LineWidth',1.2);end
xlim([7 19]);yline(.02,'k:');yline(-.02,'k:');grid on;ylabel('停车位置误差 / m');legend('LQR','PID坡道整定');title('目标在斜面中段；虚线为±2cm');
nexttile;hold on;
for k=1:2,plot(data{k}.time_s,rad2deg(data{k}.tilt_error_rad),'LineWidth',1.1);end
grid on;ylabel('相对竖直平衡角的俯仰 / deg');xlabel('时间 / s');legend('LQR','PID坡道整定');
nexttile;hold on;
for k=1:2,plot(data{k}.time_s,data{k}.speed_mps,'LineWidth',1.1);end
xlim([7 19]);grid on;ylabel('速度 / m/s');xlabel('时间 / s');legend('LQR','PID坡道整定');title('停车阶段的残余运动');
sgtitle(sprintf('同一%.1f°坡道停车：保持直立 ≠ 停在指定位置（两种现有整定）',rows{1}.slope_deg));
exportgraphics(f,fullfile(folder,'hold_comparison.png'),'Resolution',140);close(f);disp(summary);
end
