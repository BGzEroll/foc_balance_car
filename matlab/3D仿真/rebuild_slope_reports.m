function tableOut=rebuild_slope_reports
% 仅从已保存的真实仿真数据重建报告，不重新求解物理系统。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
names={'slope_hold_lqr','slope_hold_pid','slope_lqr','slope_pid'};rows=cell(4,1);
for k=1:numel(names)
 folder=fullfile(paths.results,'suite','slope',names{k});
 source=load(fullfile(folder,'simulation.mat'),'o','r');snapshot=load(fullfile(folder,'controller_config.mat'),'d','cfg');
 old=source.r;d=snapshot.d;
 rows{k}=report_balance_case(source.o,old.scenario,old.expected_duration_s,snapshot.cfg(2),folder, ...
  old.controller,[old.actuator_gain_left old.actuator_gain_right],old.motion_profile,d.terrain);
end
tableOut=struct2table(vertcat(rows{:}));folder=fullfile(paths.results,'suite','slope');
writetable(tableOut,fullfile(folder,'selected_tests.csv'));
compare_slope_controllers;
end
