function demo_balance_slope
% 在当前MATLAB桌面运行12度坡中停车，无需新建桌面实例。
addpath(fileparts(fileparts(mfilename('fullpath'))));balance_setup;
f=balance_demo_panel;
set(findall(f,'Tag','case_selector'),'Value',9);
set(findall(f,'Tag','controller_selector'),'Value',2);
set(findall(f,'Tag','gain_left'),'String','1');set(findall(f,'Tag','gain_right'),'String','1');
p=pid_balance_parameters('slope');set(findall(f,'Tag','torque_limit'),'String',num2str(p.torque_limit_each));
profile=findall(f,'Tag','motion_profile_selector');set(profile,'Value',3);
cb=get(profile,'Callback');cb(profile,[]);
button=findall(f,'Tag','run_selected');cb=get(button,'Callback');cb(button,[]);
if evalin('base','exist(''report_selected'',''var'')')
 r=evalin('base','report_selected');
 if strcmp(r.scenario,'slope_hold')
  fprintf('SLOPE_DESKTOP_READY: pass=%d, hold_error=%.6g m, on_slope=%.3f\n',r.pass,r.hold_mean_y_error_m,r.hold_on_incline_fraction);
 end
end
end
