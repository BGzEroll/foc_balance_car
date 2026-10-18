function file=build_balance_suite
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
% BUILD_BALANCE_SUITE 独立生成扩展版，保留原balance_car_3d工程。
here=fileparts(mfilename('fullpath'));old=pwd;cleanup=onCleanup(@()cd(old));cd(here);configure_balance_cache;
d=design_balance_suite;model='balance_car_3d_suite';file=fullfile(here,[model '.slx']);
if bdIsLoaded(model),error('Close balance_car_3d_suite before rebuilding; save user changes first.');end
mesh=fullfile(paths.root,'模型','仿真模型','通用URDF','meshes');addpath(mesh);
smimport(fullfile(paths.root,'模型','仿真模型','通用URDF','urdf','balance_car.urdf'),'ModelName',model);
load_system('sm_lib');load_system('nesl_utility');load_system('simulink');
mw=get_param(model,'ModelWorkspace');assignin(mw,'suite_config',balance_control_config('lqr'));assignin(mw,'terrain_on',0);
assignin(mw,'direction_convention','x_right_y_forward_v2');
w=ports('World');bp=ports('base_link');delete_line(get_param(bp.LConn(1),'Line'));
add('sm_lib/Joints/6-DOF Joint','FloatingBase');
set_param(path('FloatingBase'),'PzPositionTargetSpecify','on','PzPositionTargetValue','.0326', ...
 'SphPositionTargetSpecify','on','SphPositionTargetRotationMethod','StandardAxis', ...
 'SphPositionTargetRotationStandardAxis','-X','SphPositionTargetRotationAngle',num2str(rad2deg(d.pitch_trim)+8,16), ...
 'SphSensingFrame','BaseFrame','PxSensePosition','on','PxSenseVelocity','on', ...
 'PySensePosition','on','PySenseVelocity','on','PzSensePosition','on','PzSenseVelocity','on', ...
 'SphSensePosition','on','SphSenseVelocity','on');
fp=ports('FloatingBase');wire(w.RConn(1),fp.LConn(1));wire(fp.RConn(1),bp.LConn(1));
set_param(path('MechanismConfiguration'),'GravityVector','[0 0 -9.80665]');
add('sm_lib/Curves and Surfaces/Infinite Plane','Ground');
set_param(path('Ground'),'GraphicPlaneWidth','2','GraphicPlaneHeight','1.2','GraphicDiffuseColor','[.72 .75 .78]');
gp=ports('Ground');wire(w.RConn(1),gp.LConn(1));
for k=1:2
 side={'left','right'};s=side{k};
 set_param(path([s '_wheel_joint']),'TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on');
 add('sm_lib/Curves and Surfaces/Disk',[s '_disk']);
 set_param(path([s '_disk']),'Radius',num2str(d.radius,16),'GraphicType','None');
 jp=ports([s '_wheel_joint']);dp=ports([s '_disk']);wire(jp.RConn(1),dp.LConn(1));
 contact([s '_ground'],gp.RConn(1),dp.RConn(1),true);
 sensor([s '_wheel_q'],jp.RConn(2),'rad');sensor([s '_wheel_w'],jp.RConn(3),'rad/s');
 logSignal([s '_wheel_q/1'],[s '_wheel_q']);logSignal([s '_wheel_w/1'],[s '_wheel_w']);
end
% 地形高差3mm，单侧凸起0.8mm。flat时整体移到地面下1m。
terrain=[.24 0 .0015 .12 .30 -atan(.003/.12);.35 0 .003 .10 .30 0; ...
 .46 0 .0015 .12 .30 atan(.003/.12);.62 d.track/2 .0008 .04 .06 0];
for j=1:4
 name=['terrain_' num2str(j)];tr=[name '_pose'];a=terrain(j,6);thick=.01;
 add('sm_lib/Frames and Transforms/Rigid Transform',tr);
 offset=sprintf('[%.16g %.16g %.16g-1+terrain_on]',terrain(j,1),terrain(j,2),terrain(j,3)-thick*cos(a)/2);
 set_param(path(tr),'TranslationMethod','Cartesian','TranslationCartesianOffset',offset, ...
  'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle',num2str(rad2deg(a),16));
 tp=ports(tr);wire(w.RConn(1),tp.LConn(1));
 add('sm_lib/Body Elements/Brick Solid',name);
 set_param(path(name),'BrickDimensions',mat2str([terrain(j,4)/cos(a) terrain(j,5) thick],16), ...
  'ExportEntireGeometry','on','GraphicDiffuseColor','[.65 .45 .20]');
 tp=ports(tr);sp=ports(name);wire(tp.RConn(1),sp.RConn(1));
 for k=1:2
  side={'left','right'};s=side{k};dp=ports([s '_disk']);contact([name '_' s],sp.LConn(1),dp.RConn(1),true);
 end
end
configure_balance_terrain(model,'mild');
for s={'base_link','left_wheel','right_wheel'}
 p=path([s{1} '/Visual']);[~,n,e]=fileparts(get_param(p,'ExtGeomFileName'));set_param(p,'ExtGeomFileName',[n e]);
end
% 原始传感向量：x vx y vy z vz quaternion(4) omegaWorld(3)。
signals={'x','vx','y','vy','z','vz','quat','omega'};units={'m','m/s','m','m/s','m','m/s','1','rad/s'};
add('simulink/Sources/Clock','Time');add('simulink/Signal Routing/Mux','Measurements');
set_param(path('Measurements'),'Inputs','9');add_line(model,'Time/1','Measurements/1');
for k=1:8
 sensor(['read_' signals{k}],fp.RConn(k+1),units{k});
 add_line(model,['read_' signals{k} '/1'],['Measurements/' num2str(k+1)]);
end
add('simulink/Discrete/Zero-Order Hold','Sample_200Hz');set_param(path('Sample_200Hz'),'SampleTime','.005');
add_line(model,'Measurements/1','Sample_200Hz/1');
add('simulink/Sources/Constant','Configuration');set_param(path('Configuration'),'Value','suite_config');
add('simulink/User-Defined Functions/MATLAB Function','Balance_Controller');
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',path('Balance_Controller'));
chart.Script=fileread(fullfile(here,'balance_suite_controller.m'));
add_line(model,'Sample_200Hz/1','Balance_Controller/1');add_line(model,'Configuration/1','Balance_Controller/2');
add('simulink/Signal Routing/Demux','Controller_outputs');set_param(path('Controller_outputs'),'Outputs','[2 6 6 3 3 1 2 2]');
add_line(model,'Balance_Controller/1','Controller_outputs/1');
for k=7:8
 name=['Unused_monitor_' num2str(k)];add('simulink/Sinks/Terminator',name);
 add_line(model,['Controller_outputs/' num2str(k)],[name '/1']);
end
add('simulink/Signal Routing/Demux','Wheel_torques');set_param(path('Wheel_torques'),'Outputs','2');
add_line(model,'Controller_outputs/1','Wheel_torques/1');
for k=1:2
 side={'left','right'};s=side{k};name=[s '_torque'];add('nesl_utility/Simulink-PS Converter',name);
 set_param(path(name),'Unit','N*m','FilteringAndDerivatives','zero');
 add_line(model,['Wheel_torques/' num2str(k)],[name '/1']);pp=ports(name);jp=ports([s '_wheel_joint']);wire(pp.RConn(1),jp.LConn(2));
end
add('sm_lib/Forces and Torques/External Force and Torque','External_disturbance');
set_param(path('External_disturbance'),'EnableForce','on','EnableTorque','on','ForceResolutionFrame','World','TorqueResolutionFrame','World');
ep=ports('External_disturbance');wire(bp.LConn(1),ep.RConn(1));
for k=1:2
 names={'Force_N','Torque_Nm'};units={'N','N*m'};name=names{k};
 add('nesl_utility/Simulink-PS Converter',name);set_param(path(name),'Unit',units{k},'FilteringAndDerivatives','zero');
 add_line(model,['Controller_outputs/' num2str(k+3)],[name '/1']);pp=ports(name);wire(pp.RConn(1),ep.LConn(k));
end
add('simulink/Sinks/Stop Simulation','Fall_stop');add_line(model,'Controller_outputs/6','Fall_stop/1');
logSignal('Measurements/1','raw3d');logSignal('Balance_Controller/1','control3d');
set_param(model,'Solver','ode15s','MaxStep','.001','RelTol','1e-5','AbsTol','1e-7', ...
 'StopTime','8','SimscapeLogType','none','ReturnWorkspaceOutputs','on','SimMechanicsOpenEditorOnUpdate','off');
set_param(model,'PostLoadFcn',"addpath(fileparts(fileparts(get_param(bdroot,'FileName'))));p=balance_setup;addpath(fullfile(p.root,'模型','仿真模型','通用URDF','meshes'));");
Simulink.BlockDiagram.arrangeSystem(model);save_system(model,file);
fprintf('Built LQR/PID 3D suite: %s\n',file);
    function p=path(n),p=[model '/' n];end
    function add(lib,n),add_block(lib,path(n));end
    function p=ports(n),p=get_param(path(n),'PortHandles');end
    function wire(a,b),add_line(model,a,b,'autorouting','on');end
    function sensor(n,source,unit)
        add('nesl_utility/PS-Simulink Converter',n);set_param(path(n),'Unit',unit);p=ports(n);wire(source,p.LConn(1));
    end
    function logSignal(source,n)
        block=['log_' n];add('simulink/Sinks/To Workspace',block);set_param(path(block),'VariableName',n,'SaveFormat','Timeseries');add_line(model,source,[block '/1']);
    end
    function contact(n,base,follower,sense)
        add('sm_lib/Forces and Torques/Spatial Contact Force',n);
        set_param(path(n),'NormalStiffness','1e4','NormalDamping','10','NormalTransitionRegionWidth','1e-4', ...
         'CoefficientOfStaticFriction','.9','CoefficientOfDynamicFriction','.8','FrictionalCriticalVelocity','.001');
        if sense,set_param(path(n),'SenseNormalForceMagnitude','on');end
        p=ports(n);wire(base,p.LConn(1));wire(follower,p.RConn(1));
        if sense,sensor([n '_N'],p.RConn(2),'N');logSignal([n '_N/1'],[n '_N']);end
    end
end


