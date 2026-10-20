function [o,r]=run_measured_car(scenario,showWindows,initialTiltDeg,sensorsOn,macroStep,durationOverride,tuning,envelope,stressSpeed,controllerTs)
% Current-limited, measured actuator variant of the existing Multibody car.
if nargin<1,scenario='recover';end
if nargin<2,showWindows=false;end
if nargin<3,initialTiltDeg=3;end
if nargin<4,sensorsOn=true;end
if nargin<7,tuning='selected';end
if nargin<8,if ismember(lower(scenario),{'mixed','limit'}),envelope='explore6';else,envelope='bench';end,end
if nargin<10,controllerTs=.005;end
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));paths=balance_setup;addpath(here,'-begin');sp=measured_sensor_parameters;
names={'recover','drive','turn','route','push','terrain','bias','slope','slope_hold','mixed','limit'};idx=find(strcmpi(scenario,names),1);assert(~isempty(idx));
[cfg,d,p]=measured_pid_config(tuning,idx,envelope,controllerTs);
addpath(fullfile(paths.root,'模型','仿真模型','通用URDF','meshes'));
if nargin>=5,p.mechanical_macro_step=macroStep;p.cfg(31)=round(macroStep/p.Ts);end
assert(p.mechanical_macro_step>=p.Ts&&abs(p.mechanical_macro_step/p.Ts-round(p.mechanical_macro_step/p.Ts))<1e-8,'机械宏步长必须是50us电气步长的正整数倍');
assert(abs(controllerTs/p.mechanical_macro_step-round(controllerTs/p.mechanical_macro_step))<1e-8,'机械宏步长必须整除PID周期');
d.current_parameters=p;
model='balance_car_pid_foc';file=fullfile(here,[model '.slx']);if ~isfile(file),build_measured_car;end
load_system(file);mw=get_param(model,'ModelWorkspace');
if ~showWindows,cleanup=onCleanup(@()close_model(model));end
course=measured_course_definition;
suffix=sprintf('_pid%gus',controllerTs*1e6);if idx==11
 if nargin<9,stressSpeed=2.5;end
 course.moves(1,3)=stressSpeed*course.moves(1,2)/1.875;cfg(23)=max(cfg(23),stressSpeed*1.15);cfg(24)=deg2rad(35);d.speed_limit=cfg(23);d.pitch_target_limit=cfg(24);d.config=cfg;suffix=[suffix sprintf('_v%g',stressSpeed)];
end
assignin(mw,'course_moves',course.moves);
fc=p.cfg;fc(17:18)=p.B(2);fc(19:20)=p.coulomb(2);sp.cfg(1)=double(sensorsOn);
assignin(mw,'suite_config',cfg);assignin(mw,'foc_config',fc);assignin(mw,'sensor_config',sp.cfg);assignin(mw,'terrain_on',double(idx==6||idx>=8));
if idx==10
 terrain=configure_measured_course(model,true);
else
 configure_measured_course(model,false);kind='mild';if ismember(idx,[8 9]),kind='steep';end;terrain=configure_balance_terrain(model,kind);
 if idx==11,assignin(mw,'terrain_on',0);end
end
angle=rad2deg(d.pitch_trim);if idx==1,angle=angle+initialTiltDeg;end
set_param([model '/FloatingBase'],'SphPositionTargetRotationAngle',num2str(angle,16));
duration=d.motion_profile.durations(idx);if nargin>=6&&~isempty(durationOverride),duration=durationOverride;end
set_param(model,'StopTime',num2str(duration),'SimMechanicsOpenEditorOnUpdate',onoff(showWindows));
timing=configure_measured_timing(model,controllerTs,p.mechanical_macro_step,sp.Ts);
if showWindows,open_system(model);end
fprintf('MEASURED_CAR_START %s, %gs, initial=%gdeg, iq<=%gA, sensor=%d\n',scenario,duration,initialTiltDeg,p.Imax,sensorsOn);
progress=timer('ExecutionMode','fixedSpacing','Period',20,'TimerFcn',@(~,~)print_progress(model));start(progress);timerCleanup=onCleanup(@()clear_timer(progress));
o=sim(model);out=fullfile(here,'results',[sprintf('%s_%s_%s_tilt%g_sensor%d_macro%gus',scenario,tuning,envelope,initialTiltDeg,sensorsOn,p.mechanical_macro_step*1e6) suffix]);
if ~isfolder(out),mkdir(out);end
save(fullfile(out,'simulation_measured.mat'),'o','p','sp','cfg','fc','d','scenario','duration','initialTiltDeg','sensorsOn','terrain','tuning','envelope','timing','-v7.3');
r=report_measured_car(o,scenario,duration,p,out,terrain,initialTiltDeg,sensorsOn);
r.pid_tuning=tuning;r.operating_envelope=envelope;r.pid_parameters=d;
r.timing=timing;r.result_folder=out;tick=unique(o.pid_tick.Time);r.logged_pid_period_s=median(diff(tick));r.pid_tick_count=numel(tick);
r.timing_validation_pass=all(abs(diff(tick)-controllerTs)<1e-8);assert(r.timing_validation_pass,'PID clock log does not match requested period');
if idx==10,r=report_measured_course(o,r,out,p,terrain);end
if idx==11
 r.requested_peak_speed_mps=stressSpeed;rawData=squeeze(o.raw3d.Data);if size(rawData,1)~=numel(o.raw3d.Time),rawData=rawData';end;r.maximum_true_speed_mps=max(hypot(rawData(:,3),rawData(:,5)));
 r.pass=r.completed&&~r.fell&&r.max_true_tilt_error_deg<40&&r.max_roll_deg<8&&abs(r.true_final_distance_error_m)<.03&&abs(r.true_final_yaw_error_deg)<2&&abs(r.true_final_forward_speed_mps)<.03;
end
fid=fopen(fullfile(out,'validation.json'),'w');fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);
save(fullfile(out,'configuration.mat'),'p','sp','cfg','fc','d');fprintf('MEASURED_CAR_COMPLETE %s pass=%d duration=%g tilt=%g\n',scenario,r.pass,r.duration_s,r.max_tilt_error_deg);
end
function s=onoff(v)
if v,s='on';else,s='off';end
end
function close_model(model)
if bdIsLoaded(model),close_system(model,0);end
end
function print_progress(model)
if bdIsLoaded(model)
 simTime=get_param(model,'SimulationTime');if ischar(simTime)||isstring(simTime),simTime=str2double(simTime);end
 fprintf('SIM_PROGRESS %s: %.4f s\n',get_param(model,'SimulationStatus'),simTime);
end
end
function clear_timer(t)
if isvalid(t),stop(t);delete(t);end
end
