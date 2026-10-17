function demo_balance_agile
% 在当前MATLAB桌面展示PID快速档；复用同一个面板和3D模型窗口。
addpath(fileparts(fileparts(mfilename('fullpath'))));balance_setup;
f=balance_demo_panel;
set(findall(f,'Tag','case_selector'),'Value',2);
set(findall(f,'Tag','controller_selector'),'Value',2);
set(findall(f,'Tag','gain_left'),'String','1');set(findall(f,'Tag','gain_right'),'String','1');
p=pid_balance_parameters('agile');set(findall(f,'Tag','torque_limit'),'String',num2str(p.torque_limit_each));
profile=findall(f,'Tag','motion_profile_selector');set(profile,'Value',2);
cb=get(profile,'Callback');cb(profile,[]);
button=findall(f,'Tag','run_selected');cb=get(button,'Callback');cb(button,[]);
if evalin('base','exist(''report_selected'',''var'')')
 r=evalin('base','report_selected');
 if strcmp(r.motion_profile,'agile')&&r.pass
  fprintf('AGILE_DESKTOP_READY: peak_pitch=%.3f deg, peak_speed=%.3f m/s\n',r.max_tilt_error_deg,r.max_speed_mps);
 end
end
end
