function demo_balance_current
% 在当前桌面运行PID电流输出的12度坡中停车，复用面板与3D模型。
addpath(fileparts(fileparts(mfilename('fullpath'))));balance_setup;
f=balance_demo_panel;
set(findall(f,'Tag','case_selector'),'Value',9);
selector=findall(f,'Tag','controller_selector');set(selector,'Value',3);
callback=get(selector,'Callback');callback(selector,[]);
set(findall(f,'Tag','gain_left'),'String','1');
set(findall(f,'Tag','gain_right'),'String','1');
profile=findall(f,'Tag','motion_profile_selector');set(profile,'Value',3);
callback=get(profile,'Callback');callback(profile,[]);
evalin('base','clear simout_selected report_selected');
button=findall(f,'Tag','run_selected');callback=get(button,'Callback');callback(button,[]);
assert(evalin('base','exist(''report_selected'',''var'')')==1,'电流输出演示未完成，请查看命令窗口错误');
r=evalin('base','report_selected');
assert(strcmp(r.controller,'PID_CURRENT')&&strcmp(r.scenario,'slope_hold')&&r.completed);
fprintf('CURRENT_DESKTOP_READY: pass=%d, peak_iq=%.6g A, hold_error=%.6g m\n', ...
 r.pass,r.peak_each_iq_command_A,r.hold_mean_x_error_m);
end
