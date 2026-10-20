function file=build_measured_car
% Derive a distinct model; preserve the validated ideal-actuator suite.
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));paths=balance_setup;addpath(here,'-begin');
model='balance_car_pid_foc';file=fullfile(here,[model '.slx']);source=fullfile(paths.simulation,'balance_car_3d_suite.slx');
if bdIsLoaded(model),error('Save and close measured model before rebuilding');end
load_system(source);save_system('balance_car_3d_suite',file);
if bdIsLoaded('balance_car_3d_suite'),close_system('balance_car_3d_suite',0);end
load_system(file);[control,~,p]=measured_pid_config('selected',10,'explore6');sp=measured_sensor_parameters;mw=get_param(model,'ModelWorkspace');
pidChart=find(sfroot,'-isa','Stateflow.EMChart','Path',path('Balance_Controller'));pidChart.Script=fileread(fullfile(here,'measured_car_pid.m'));
course=measured_course_definition;assignin(mw,'course_moves',course.moves);
add('simulink/Sources/Constant','Course_reference');set_param(path('Course_reference'),'Value','course_moves');wire('Course_reference/1','Balance_Controller/3');
cfg=p.cfg;cfg(17:18)=p.B(2);cfg(19:20)=p.coulomb(2); % final car has a wheel on each side
assignin(mw,'foc_config',cfg);assignin(mw,'sensor_config',sp.cfg);
assignin(mw,'suite_config',control);set_param([model '/FloatingBase'],'SphPositionTargetRotationAngle',num2str(rad2deg(control(3)),16));
% Two additional physical obstacle solids, each with left/right contact sensing.
worldPorts=get_param(path('World'),'PortHandles');
for j=5:6
 name=['terrain_' num2str(j)];add_block(path('terrain_4_pose'),path([name '_pose']));add_block(path('terrain_4'),path(name));
 tp=get_param(path([name '_pose']),'PortHandles');sp0=get_param(path(name),'PortHandles');add_line(model,worldPorts.RConn(1),tp.LConn(1),'autorouting','on');add_line(model,tp.RConn(1),sp0.RConn(1),'autorouting','on');
 for side={'left','right'}
  s=side{1};cn=[name '_' s];add_block(path(['terrain_4_' s]),path(cn));cp=get_param(path(cn),'PortHandles');dp=get_param(path([s '_disk']),'PortHandles');
  add_line(model,sp0.LConn(1),cp.LConn(1),'autorouting','on');add_line(model,dp.RConn(1),cp.RConn(1),'autorouting','on');
  add_block(path(['terrain_4_' s '_N']),path([cn '_N']));conv=get_param(path([cn '_N']),'PortHandles');add_line(model,cp.RConn(2),conv.LConn(1),'autorouting','on');
  add('simulink/Sinks/To Workspace',['log_' cn '_N']);set_param(path(['log_' cn '_N']),'VariableName',[cn '_N'],'SaveFormat','Timeseries');wire([cn '_N/1'],['log_' cn '_N/1']);
 end
end
configure_measured_course(model,true);
add('simulink/Sources/Digital Clock','FOC_clock');set_param(path('FOC_clock'),'SampleTime',num2str(p.mechanical_macro_step,16));
add('simulink/Sources/Constant','FOC_parameters');set_param(path('FOC_parameters'),'Value','foc_config');
add('simulink/Sources/Constant','Sensor_parameters');set_param(path('Sensor_parameters'),'Value','sensor_config');
add('simulink/Signal Routing/Mux','Motor_mechanics');set_param(path('Motor_mechanics'),'Inputs','4');
for k=1:4,names={'left_wheel_q','right_wheel_q','left_wheel_w','right_wheel_w'};wire([names{k} '/1'],['Motor_mechanics/' num2str(k)]);end
add('simulink/Discrete/Zero-Order Hold','Motor_sample');set_param(path('Motor_sample'),'SampleTime',num2str(p.mechanical_macro_step,16));wire('Motor_mechanics/1','Motor_sample/1');
add('simulink/Discrete/Unit Delay','Motor_mechanics_delay');set_param(path('Motor_mechanics_delay'),'SampleTime',num2str(p.mechanical_macro_step,16),'InitialCondition','zeros(4,1)');wire('Motor_sample/1','Motor_mechanics_delay/1');
chart('Measured_FOC','measured_vehicle_actuator.m');wire('Motor_mechanics_delay/1','Measured_FOC/1');
if getSimulinkBlockHandle(path('Unused_monitor_7'))>0,delete_block(path('Unused_monitor_7'));end
wire('Controller_outputs/7','Measured_FOC/2');wire('FOC_clock/1','Measured_FOC/3');wire('FOC_parameters/1','Measured_FOC/4');
add('simulink/Signal Routing/Demux','FOC_outputs');set_param(path('FOC_outputs'),'Outputs','[2 28]');wire('Measured_FOC/1','FOC_outputs/1');
delete_line(model,'Controller_outputs/1','Wheel_torques/1');wire('FOC_outputs/1','Wheel_torques/1');
add('simulink/Sinks/Terminator','Unused_ideal_torque');wire('Controller_outputs/1','Unused_ideal_torque/1');
add('simulink/Sinks/Terminator','FOC_unused_observations');wire('FOC_outputs/2','FOC_unused_observations/1');
add('simulink/Discrete/Unit Delay','Encoder_feedback_delay');set_param(path('Encoder_feedback_delay'),'SampleTime',num2str(p.mechanical_macro_step,16),'InitialCondition','zeros(30,1)');wire('Measured_FOC/1','Encoder_feedback_delay/1');
add('simulink/Discrete/Zero-Order Hold','Encoder_feedback_200Hz');set_param(path('Encoder_feedback_200Hz'),'SampleTime','.005');wire('Encoder_feedback_delay/1','Encoder_feedback_200Hz/1');
chart('Measured_sensors','measured_vehicle_sensors.m');delete_line(model,'Sample_200Hz/1','Balance_Controller/1');wire('Sample_200Hz/1','Measured_sensors/1');wire('Encoder_feedback_200Hz/1','Measured_sensors/2');wire('Sensor_parameters/1','Measured_sensors/3');wire('Measured_sensors/1','Balance_Controller/1');
% PID runs independently of the 200Hz IMU/noise/attitude estimator.
delete_line(model,'Measured_sensors/1','Balance_Controller/1');
add('simulink/Sources/Digital Clock','PID_clock');
add('simulink/Discrete/Zero-Order Hold','PID_imu_hold');wire('Measured_sensors/1','PID_imu_hold/1');
add('simulink/Discrete/Zero-Order Hold','PID_encoder_sample');wire('Encoder_feedback_delay/1','PID_encoder_sample/1');
chart('PID_observation','measured_pid_observation.m');wire('PID_imu_hold/1','PID_observation/1');wire('PID_encoder_sample/1','PID_observation/2');wire('PID_clock/1','PID_observation/3');wire('Sensor_parameters/1','PID_observation/4');wire('PID_observation/1','Balance_Controller/1');
add('simulink/Sinks/To Workspace','log_pid_tick');set_param(path('log_pid_tick'),'VariableName','pid_tick','SaveFormat','Timeseries');wire('PID_clock/1','log_pid_tick/1');
add('simulink/Sinks/To Workspace','log_pid_observation');set_param(path('log_pid_observation'),'VariableName','pid_observation','SaveFormat','Timeseries');wire('PID_observation/1','log_pid_observation/1');
add('simulink/User-Defined Functions/MATLAB Function','Actual_control_log');c=find(sfroot,'-isa','Stateflow.EMChart','Path',path('Actual_control_log'));c.Script=sprintf('function y=combine(c,tau)\n%%#codegen\ny=[tau;c(3:25)];\nend');
wire('Balance_Controller/1','Actual_control_log/1');wire('FOC_outputs/1','Actual_control_log/2');
delete_line(model,'Balance_Controller/1','log_control3d/1');wire('Actual_control_log/1','log_control3d/1');
add('simulink/Sinks/To Workspace','log_foc_actuator');set_param(path('log_foc_actuator'),'VariableName','foc_actuator','SaveFormat','Timeseries');wire('Measured_FOC/1','log_foc_actuator/1');
add('simulink/Sinks/To Workspace','log_sensed3d');set_param(path('log_sensed3d'),'VariableName','sensed3d','SaveFormat','Timeseries');wire('Measured_sensors/1','log_sensed3d/1');
logs=find_system(model,'BlockType','ToWorkspace');for k=1:numel(logs),if ~any(endsWith(logs{k},{'log_foc_actuator','log_sensed3d','log_pid_tick','log_pid_observation'})),set_param(logs{k},'Decimation','10');end,end
configure_measured_timing(model,control(31),p.mechanical_macro_step,sp.Ts);
set_param(model,'MaxStep',num2str(p.mechanical_macro_step,16),'StopTime','39','SimMechanicsOpenEditorOnUpdate','off');
set_param(model,'PostLoadFcn',"addpath(fileparts(get_param(bdroot,'FileName')));addpath(fileparts(fileparts(get_param(bdroot,'FileName'))));p=balance_setup;addpath(fullfile(p.root,'模型','仿真模型','通用URDF','meshes'));");
note=Simulink.Annotation(model,sprintf('PID ONLY at 200Hz (default; optional 1kHz): position P -> velocity PI -> pitch PID, yaw PID -> left/right target Iq\nEncoder observations and MPU6050/AR12/complementary filter at 200Hz by default\nMeasured base current PI, 5kHz per motor; 150us equivalent delay; electrical RK4 50us\nCourse: fast/slow motion, hill hold, platform, descent, staggered bumps, reversal/turn/push\nHigh-current envelopes are simulation extrapolations; hardware settings are unchanged.'));note.Position=[40 40];
Simulink.BlockDiagram.arrangeSystem(model);save_system(model,file);close_system(model,0);fprintf('MEASURED_CAR_BUILT %s\n',file);
 function pth=path(n),pth=[model '/' n];end
 function add(lib,n),add_block(lib,path(n));end
 function wire(a,b),add_line(model,a,b,'autorouting','on');end
 function chart(n,sourceFile)
  add('simulink/User-Defined Functions/MATLAB Function',n);c=find(sfroot,'-isa','Stateflow.EMChart','Path',path(n));c.Script=fileread(fullfile(here,sourceFile));
 end
end
