function checks=verify_usb_forward
% USB朝前的坐标与真实3D工况检查；原CAD/网格不变，重建两个MATLAB模型。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
assert(~bdIsLoaded('balance_car_3d_suite')&&~bdIsLoaded('balance_car_3d'), ...
 '请保存并关闭两个旧模型后运行验证，避免覆盖桌面未保存修改');
out=fullfile(paths.suite_results,'direction_checks');if ~isfolder(out),mkdir(out);end
backup=fullfile(paths.matlab,'.cache','usb_rear_model_backup');if ~isfolder(backup),mkdir(backup);end
for name={'balance_car_3d_suite','balance_car_3d'}
 file=fullfile(paths.simulation,[name{1} '.slx']);
 if isfile(file),copyfile(file,fullfile(backup,[datestr(now,'yyyymmdd_HHMMSS') '_' name{1} '.slx']));end
end
build_balance_3d;close_system('balance_car_3d',0);
build_balance_suite;cleanup=onCleanup(@()close_system('balance_car_3d_suite',0));
checks.coordinate_replay_max_difference=verify_coordinate_equivalence;
[cfg,d]=balance_control_config('pid_current',3,[],1,'agile');
pid=pid_balance_parameters('agile');assert(d.pitch_trim<0);assert(abs(cfg(3)-pid.pitch_trim)<1e-14);
% +偏航目标应产生右轮大于左轮的转矩，即朝车体左侧转弯。
clear balance_suite_controller
raw=[1.5;zeros(6,1);1;zeros(6,1)];control=balance_suite_controller(raw,cfg);
assert(control(2)>control(1));checks.positive_yaw_is_left=true;
[~,checks.lqr_recover]=run_balance_case('recover',false,[],'lqr');assert(checks.lqr_recover.pass);
[drive,checks.forward_reverse]=run_balance_case('drive',false,[],'pid_current',1,'agile');assert(checks.forward_reverse.pass);
c=squeeze(drive.control3d.Data);if size(c,1)~=numel(drive.control3d.Time),c=c';end
assert(max(c(:,3))>.25&&c(end,3)<.12);
raw=squeeze(drive.raw3d.Data);if size(raw,1)~=numel(drive.raw3d.Time),raw=raw';end
assert(max(raw(:,4))>.25&&max(abs(raw(:,2)))<.002); % 实际前进沿世界+Y
t=drive.control3d.Time;left=squeeze(drive.left_wheel_w.Data);right=squeeze(drive.right_wheel_w.Data);
wl=interp1(drive.left_wheel_w.Time,left,t);wr=interp1(drive.right_wheel_w.Time,right,t);
assert(mean((wl(c(:,4)>.15)+wr(c(:,4)>.15))/2)>0);checks.forward_wheel_speed_positive=true;
[turn,checks.left_turn]=run_balance_case('turn',false,[],'pid_current',1,'agile');assert(checks.left_turn.pass);
c=squeeze(turn.control3d.Data);if size(c,1)~=numel(turn.control3d.Time),c=c';end
assert(max(c(:,7))>deg2rad(50));checks.direction_convention='x_right_y_forward_v2';
[~,checks.slope_hold]=run_balance_case('slope_hold',false,[],'pid_current',1,'slope');assert(checks.slope_hold.pass);
plot_balance_axes(out);
fid=fopen(fullfile(out,'validation.json'),'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(checks,PrettyPrint=true));fclose(fid);
fprintf('USB_FORWARD_VERIFIED: LQR recover, PID current forward/reverse, left turn, slope hold all passed\n');
end
