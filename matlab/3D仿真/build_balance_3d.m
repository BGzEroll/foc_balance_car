function modelFile=build_balance_3d
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
% BUILD_BALANCE_3D 导入原有URDF，添加自由基座、空间接触和采样转矩反馈。
here=fileparts(mfilename('fullpath')); original=pwd; cleanup=onCleanup(@()cd(original)); cd(here); configure_balance_cache;
meshDir=fullfile(paths.root,'模型','仿真模型','通用URDF','meshes'); addpath(meshDir);
data=jsondecode(fileread(fullfile(paths.root,'模型','参数与图片','仿真刚体参数_SI.json')));
M=data.links(1).mass; m=data.links(2).mass; r=.0325;
com=data.links(1).com; l=hypot(com(2),com(3));
I=data.links(1).inertia(1,1); J=data.links(2).inertia(1,1); g_mps2=9.80665;
eq=atan2(-com(2),com(3)); Ts=.005;
D=[M+2*m+2*J/r^2 M*l;M*l I+M*l^2]; G=D\[0;M*g_mps2*l]; H=D\[1/r;-1];
A=[0 1 0 0;0 0 G(1) 0;0 0 0 1;0 0 G(2) 0]; B=[0;H(1);0;H(2)];
E=expm([A B;zeros(1,5)]*Ts); Ad=E(1:4,1:4); Bd=E(1:4,5);
Q=diag([100 4 1/deg2rad(8)^2 .25]); R=625; P=Q;
for i=1:100000
 Pn=Q+Ad'*P*Ad-Ad'*P*Bd*((R+Bd'*P*Bd)\(Bd'*P*Ad));
 if norm(Pn-P,'fro')/max(1,norm(Pn,'fro'))<1e-12,P=Pn;break;end
 P=(Pn+Pn')/2;
end
K=(R+Bd'*P*Bd)\(Bd'*P*Ad);
assert(max(abs(eig(Ad-Bd*K)))<1);
model='balance_car_3d'; modelFile=fullfile(here,[model '.slx']);
if bdIsLoaded(model), error('Close balance_car_3d before rebuilding; preserve any user edits first.'); end
smimport(fullfile(paths.root,'模型','仿真模型','通用URDF','urdf','balance_car.urdf'),'ModelName',model);
load_system('sm_lib'); load_system('nesl_utility'); load_system('simulink');
% Imported root is fixed to world: replace ONLY that branch with an actual 6-DOF joint.
world=ports('World'); body=ports('base_link');
delete_line(get_param(body.LConn(1),'Line'));
floating=add('sm_lib/Joints/6-DOF Joint','FloatingBase',[170 340 290 510]);
set_param(floating,'PzPositionTargetSpecify','on','PzPositionTargetValue','0.0326', ...
 'SphPositionTargetSpecify','on','SphPositionTargetRotationMethod','StandardAxis', ...
 'SphPositionTargetRotationStandardAxis','-X','SphPositionTargetRotationAngle',num2str(rad2deg(eq)+8,16), ...
 'PySensePosition','on','PySenseVelocity','on','PzSensePosition','on', ...
 'SphSensePosition','on','SphSenseVelocityX','on');
fp=ports('FloatingBase'); wire(world.RConn(1),fp.LConn(1)); wire(fp.RConn(1),body.LConn(1));
set_param([model '/MechanismConfiguration'],'GravityVector','[0 0 -9.80665]');
add('sm_lib/Curves and Surfaces/Infinite Plane','Ground',[460 600 540 660]);
set_param([model '/Ground'],'GraphicPlaneWidth','1','GraphicPlaneHeight','1', ...
 'GraphicDiffuseColor','[0.75 0.78 0.8]');
gp=ports('Ground'); wire(world.RConn(1),gp.LConn(1));
for side={'left','right'}
 s=side{1}; joint=[model '/' s '_wheel_joint'];
 set_param(joint,'TorqueActuationMode','InputTorque');
 % Joint follower Z coincides with original wheel +Y. Disk Z is its normal.
 disk=[s '_contact_disk']; contact=[s '_ground_contact'];
 add('sm_lib/Curves and Surfaces/Disk',disk,[470 720 530 780]);
 set_param([model '/' disk],'Radius','0.0325','GraphicType','None');
 jp=ports([s '_wheel_joint']); dp=ports(disk); wire(jp.RConn(1),dp.LConn(1));
 add('sm_lib/Forces and Torques/Spatial Contact Force',contact,[650 600 730 680]);
 set_param([model '/' contact],'NormalStiffness','1e4','NormalDamping','10', ...
  'NormalTransitionRegionWidth','1e-4','CoefficientOfStaticFriction','0.9', ...
  'CoefficientOfDynamicFriction','0.8','FrictionalCriticalVelocity','0.001', ...
  'SenseNormalForceMagnitude','on');
 cp=ports(contact); wire(gp.RConn(1),cp.LConn(1)); wire(dp.RConn(1),cp.RConn(1));
end
% Save mesh references by name; startup restores the repo-relative mesh search path.
for s={'base_link','left_wheel','right_wheel'}
 path=[model '/' s{1} '/Visual']; old=get_param(path,'ExtGeomFileName'); [~,n,e]=fileparts(old);
 set_param(path,'ExtGeomFileName',[n e]);
end
% Floating joint ports: F, y, vy, z, quaternion, angular velocity X.
units={'m','m/s','m','1','rad/s'}; signals={'x','vx','z','quat','wy'};
for i=1:5
 name=['read_' signals{i}]; add('nesl_utility/PS-Simulink Converter',name,[420 260+50*i 470 290+50*i]);
 set_param([model '/' name],'Unit',units{i}); pp=ports(name); wire(fp.RConn(i+1),pp.LConn(1));
end
add('simulink/Signal Routing/Mux','State',[630 300 635 490]);set_param([model '/State'],'Inputs','5');
for i=1:5
 source=['read_' signals{i} '/1'];
 if i==5
  add('simulink/Math Operations/Gain','Forward_pitch_rate',[510 510 560 540]);
  set_param([model '/Forward_pitch_rate'],'Gain','-1');
  add_line(model,source,'Forward_pitch_rate/1');source='Forward_pitch_rate/1';
 end
 add_line(model,source,['State/' num2str(i)],'autorouting','on');
end
% State=[y vy z qw qx qy qz forwardPitchRate]；最后一项=-世界wx。
add('simulink/Discrete/Zero-Order Hold','Sample_200Hz',[700 350 780 390]);
set_param([model '/Sample_200Hz'],'SampleTime','0.005');add_line(model,'State/1','Sample_200Hz/1');
add('simulink/User-Defined Functions/Fcn','LQR_each_wheel',[840 350 1000 390]);
expr=sprintf('-0.5*((%.16g)*u(1)+(%.16g)*u(2)+(%.16g)*(atan2(2*(-u(4)*u(5)+u(6)*u(7)),1-2*(u(5)^2+u(6)^2))-%.16g)+(%.16g)*u(8))',K(1),K(2),K(3),eq,K(4));
set_param([model '/LQR_each_wheel'],'Expr',expr); add_line(model,'Sample_200Hz/1','LQR_each_wheel/1');
add('simulink/Discontinuities/Saturation','Torque_limit',[1005 350 1035 390]);
set_param([model '/Torque_limit'],'UpperLimit','0.02','LowerLimit','-0.02');
add_line(model,'LQR_each_wheel/1','Torque_limit/1');
add('simulink/Math Operations/Gain','Control_enable',[1040 350 1100 390]);set_param([model '/Control_enable'],'Gain','1');
add_line(model,'Torque_limit/1','Control_enable/1');
for s={'left','right'}
 name=[s{1} '_torque_Nm']; add('nesl_utility/Simulink-PS Converter',name,[1180 340 1240 380]);
 set_param([model '/' name],'Unit','N*m','FilteringAndDerivatives','zero');
 add_line(model,'Control_enable/1',[name '/1'],'autorouting','on');
 pp=ports(name); jp=ports([s{1} '_wheel_joint']); wire(pp.RConn(1),jp.LConn(2));
end
logSignal('State/1','states3d','states3d');logSignal('Control_enable/1','torque3d','torque3d');
for s={'left','right'}
 name=[s{1} '_normal_N'];add('nesl_utility/PS-Simulink Converter',name,[790 640 840 670]);
 set_param([model '/' name],'Unit','N'); pp=ports(name); cp=ports([s{1} '_ground_contact']);
 wire(cp.RConn(2),pp.LConn(1)); logSignal([name '/1'],[s{1} '_contact_N'],[s{1} '_contact_N']);
end
set_param(model,'Solver','ode15s','MaxStep','0.001','RelTol','1e-5','AbsTol','1e-7', ...
 'StopTime','5','SimscapeLogType','all','ReturnWorkspaceOutputs','on', ...
 'SimMechanicsOpenEditorOnUpdate','off');
callback="addpath(fileparts(fileparts(get_param(bdroot,'FileName'))));p=balance_setup;addpath(fullfile(p.root,'模型','仿真模型','通用URDF','meshes'));";
set_param(model,'PostLoadFcn',char(callback));
Simulink.BlockDiagram.arrangeSystem(model);
save_system(model,modelFile);
fprintf('Built %s\nK=[%s]\n',modelFile,num2str(K));
    function h=add(lib,name,pos), h=add_block(lib,[model '/' name],'Position',pos);end
    function p=ports(name),p=get_param([model '/' name],'PortHandles');end
    function wire(a,b),add_line(model,a,b,'autorouting','on');end
    function logSignal(source,name,var)
        add('simulink/Sinks/To Workspace',name,[1350 300 1440 330]);
        set_param([model '/' name],'VariableName',var,'SaveFormat','Timeseries');
        add_line(model,source,[name '/1'],'autorouting','on');
    end
end

