function checks=verify_current_interface
% 电流接口验证：先回放同一传感序列，检验单位换算，再跑真实3D工况。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
motor=balance_current_parameters;kt=motor.Kt_nominal;
cfgTorque=balance_control_config('pid',9,6*kt,1,'slope');
cfgCurrent=balance_control_config('pid_current',9,6,1,'slope');
cfgDefault=balance_control_config('pid_current',1,[]);
assert(cfgDefault(2)==motor.current_limit_each);
source=fullfile(paths.results,'suite','slope','slope_hold_pid','simulation.mat');
data=load(source,'o');ts=data.o.raw3d;raw=squeeze(ts.Data);
if size(raw,1)~=numel(ts.Time),raw=raw';end
[times,uniqueIndex]=unique(ts.Time,'stable');raw=raw(uniqueIndex,:);
sampleTimes=(0:.005:34)';raw=interp1(times,raw,sampleTimes,'linear');raw(:,1)=sampleTimes;
clear balance_suite_controller
a=zeros(numel(sampleTimes),25);
for k=1:numel(sampleTimes),a(k,:)=balance_suite_controller(raw(k,:)',cfgTorque)';end
clear balance_suite_controller
b=zeros(size(a));
for k=1:numel(sampleTimes),b(k,:)=balance_suite_controller(raw(k,:)',cfgCurrent)';end
b(:,22:23)=b(:,22:23)*kt;
checks.replay_max_difference=max(abs(a-b),[],'all');assert(checks.replay_max_difference<1e-10);
clear balance_suite_controller
small=balance_control_config('pid_current',1,.05,[.8 1.2]);
r=[0;zeros(6,1);cos(pi/12);0;sin(pi/12);0;zeros(3,1)];
y=balance_suite_controller(r,small);
assert(all(abs(y(22:23))<=.05));assert(max(abs(y(22:23)))>.049);
assert(max(abs(y(1:2)-y(22:23).*kt.*[.8;1.2]))<1e-12);
checks.current_clamp_pass=true;
% 保存当前磁盘模型后同步嵌入函数；不生成另一份机械模型。
model='balance_car_3d_suite';file=fullfile(paths.simulation,[model '.slx']);
assert(~bdIsLoaded(model),'验证入口会同步磁盘模型，请先保存并关闭已打开的balance_car_3d_suite');
backup=fullfile(paths.matlab,'.cache','current_interface_backup');
if ~isfolder(backup),mkdir(backup);end
backupFile=fullfile(backup,[datestr(now,'yyyymmdd_HHMMSS') '_' model '.slx']);
copyfile(file,backupFile);load_system(file);cleanup=onCleanup(@()close_system(model,0));
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/Balance_Controller']);
chart.Script=fileread(fullfile(paths.simulation,'balance_suite_controller.m'));
mw=get_param(model,'ModelWorkspace');assignin(mw,'suite_config',balance_control_config('lqr'));
save_system(model,file);
[~,checks.recover]=run_balance_case('recover',false,[],'pid_current');assert(checks.recover.pass);
[~,checks.agile_drive]=run_balance_case('drive',false,[],'pid_current',1,'agile');assert(checks.agile_drive.pass);
[~,checks.slope_hold]=run_balance_case('slope_hold',false,[],'pid_current',1,'slope');assert(checks.slope_hold.pass);
out=fullfile(paths.suite_results,'current_interface_checks.json');
fid=fopen(out,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(checks,PrettyPrint=true));fclose(fid);
fprintf('CURRENT_INTERFACE_VERIFIED; replay error=%g\n',checks.replay_max_difference);
end
