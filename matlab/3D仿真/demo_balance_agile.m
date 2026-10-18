function demo_balance_agile(controller)
% 在当前MATLAB桌面展示PID快速档；复用同一个面板和3D模型窗口。
addpath(fileparts(fileparts(mfilename('fullpath'))));balance_setup;
if nargin<1,controller='pid';end
controller=validatestring(controller,{'pid','pid_current'});
f=balance_demo_panel;
set(findall(f,'Tag','case_selector'),'Value',2);
selector=findall(f,'Tag','controller_selector');index=2;if strcmp(controller,'pid_current'),index=3;end
set(selector,'Value',index);cb=get(selector,'Callback');cb(selector,[]);
set(findall(f,'Tag','gain_left'),'String','1');set(findall(f,'Tag','gain_right'),'String','1');
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
