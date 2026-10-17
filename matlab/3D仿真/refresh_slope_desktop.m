function refresh_slope_desktop
% 在已有桌面中保留旧状态副本、重载新模型并运行坡中停车。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
model='balance_car_3d_suite';
if bdIsLoaded(model)
 assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'当前模型正在运行，请结束后再更新坡道演示');
 if strcmp(get_param(model,'Dirty'),'on')
  backup=fullfile(paths.matlab,'.cache','desktop_model_backups',datestr(now,'yyyymmdd_HHMMSS'));
  if ~isfolder(backup),mkdir(backup);end
  save_system(model,fullfile(backup,[model '.slx']),'SaveModelWorkspace',true);
  fprintf('旧桌面模型未保存状态已保留在：%s\n',backup);
 end
 close_system(model,0);
end
delete(findall(groot,'Type','figure','Tag','balance_car_demo_panel'));
clear balance_demo_panel run_balance_case report_balance_case balance_control_config ...
 balance_motion_profile pid_balance_parameters balance_result_folder;
demo_balance_slope;
end
