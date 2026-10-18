function metrics=compare_balance_controllers
% 从两组真实3D报告生成对比，不使用线性模型代替Multibody结果。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
folder=paths.suite_results;names={'recover','drive','turn','route','push','terrain','bias'};
rows=cell(14,1);index=0;
for mode={'lqr','pid'}
 controller=mode{1};
 for k=1:numel(names)
  scenario=names{k};tab=readtable(fullfile(folder,[scenario '_' controller],'trajectory.csv'));
  r=jsondecode(fileread(fullfile(folder,[scenario '_' controller],'validation.json')));
  index=index+1;
  rows{index}=struct('scenario',scenario,'controller',upper(controller),'pass',r.pass, ...
   'distance_IAE_m_s',trapz(tab.time_s,abs(tab.distance_m-tab.distance_ref_m)), ...
   'yaw_IAE_deg_s',trapz(tab.time_s,abs(rad2deg(tab.yaw_rad-tab.yaw_ref_rad))), ...
   'final_distance_error_m',r.final_distance_error_m,'final_yaw_error_deg',r.final_yaw_error_deg, ...
   'max_tilt_error_deg',r.max_tilt_error_deg,'peak_each_torque_Nm',r.peak_each_torque_Nm);
 end
end
metrics=struct2table(vertcat(rows{:}));writetable(metrics,fullfile(folder,'comparison_lqr_pid.csv'));
f=figure('Visible','off','Color','w','Position',[50 50 1150 850]);tiledlayout(3,2);
selected={'drive','push'};
for row=1:3
 for column=1:2
  scenario=selected{column};nexttile;hold on;
  for mode={'lqr','pid'}
   tab=readtable(fullfile(folder,[scenario '_' mode{1}],'trajectory.csv'));
   if row==1,y=tab.distance_m;label='路程 / m';
   elseif row==2,y=rad2deg(tab.tilt_error_rad);label='俯仰偏差 / deg';
   else,y=tab.tau_left_Nm+tab.tau_right_Nm;label='总转矩 / N·m';end
   plot(tab.time_s,y,'LineWidth',1.1);
  end
  if row==1,plot(tab.time_s,tab.distance_ref_m,'k--');legend('LQR','PID','指令','Location','best');
  else,legend('LQR','PID','Location','best');end
  grid on;ylabel(label);title(scenario);if row==3,xlabel('时间 / s');end
 end
end
sgtitle('同一3D小车与工况：LQR / 级联PID（标称执行增益）');
exportgraphics(f,fullfile(folder,'comparison_lqr_pid.png'),'Resolution',140);close(f);
disp(metrics);
end
