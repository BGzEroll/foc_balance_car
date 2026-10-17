function file=build_foc_demo
% 使用原生Simulink API生成可查看的FOC框图，无需Motor Control Blockset。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
model='zd2808_foc_demo';file=fullfile(paths.foc,[model '.slx']);
if bdIsLoaded(model),error('请先保存并关闭zd2808_foc_demo再重建');end
p=foc_parameters;new_system(model);mw=get_param(model,'ModelWorkspace');
assignin(mw,'foc_config',p.config);assignin(mw,'foc_ts',p.Ts);
load_system('simulink');
add_block('simulink/Sources/Digital Clock',[model '/Time'],'SampleTime','foc_ts','Position',[30 50 70 80]);
add_block('simulink/Sources/Constant',[model '/Parameters'],'Value','foc_config','Position',[30 135 100 165]);
add_block('simulink/Discrete/Unit Delay',[model '/Motor_state'],'InitialCondition','zeros(7,1)', ...
 'SampleTime','foc_ts','Position',[735 265 785 315]);
nodes={'Reference','Current_FOC','Average_inverter_PMSM','Ideal_sensors'};
sources={'foc_reference.m','foc_current_controller.m','foc_motor_plant.m','foc_measurements.m'};
positions={[170 35 300 100],[390 90 555 210],[625 100 810 210],[175 250 330 330]};
for k=1:numel(nodes)
 path=[model '/' nodes{k}];add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',positions{k});
 chart=find(sfroot,'-isa','Stateflow.EMChart','Path',path);
 chart.Script=fileread(fullfile(paths.foc,sources{k}));
end
wire('Time/1','Reference/1');wire('Parameters/1','Reference/2');
wire('Motor_state/1','Ideal_sensors/1');wire('Parameters/1','Ideal_sensors/2');
wire('Ideal_sensors/1','Current_FOC/1');wire('Reference/1','Current_FOC/2');wire('Parameters/1','Current_FOC/3');
wire('Motor_state/1','Average_inverter_PMSM/1');wire('Current_FOC/1','Average_inverter_PMSM/2');
wire('Reference/1','Average_inverter_PMSM/3');wire('Parameters/1','Average_inverter_PMSM/4');
wire('Average_inverter_PMSM/1','Motor_state/1');
add_block('simulink/Signal Routing/Mux',[model '/Logged_signals'],'Inputs','4','Position',[885 100 890 245]);
wire('Motor_state/1','Logged_signals/1');wire('Reference/1','Logged_signals/2');
wire('Current_FOC/1','Logged_signals/3');wire('Ideal_sensors/1','Logged_signals/4');
add_block('simulink/Sinks/To Workspace',[model '/Save_data'],'VariableName','foc_data', ...
 'SaveFormat','Timeseries','Position',[960 105 1050 145]);wire('Logged_signals/1','Save_data/1');
add_block('simulink/Sinks/Scope',[model '/Waveforms'],'Position',[960 195 1010 235]);wire('Logged_signals/1','Waveforms/1');
note=Simulink.Annotation(model,sprintf('ZD2808 FOC: current PI + decoupling + SVPWM average voltages\nMotor_state holds [id iq omega theta energy integrals] at 25 kHz\nEdit foc_parameters.m; run run_foc_demo or launch_foc_demo.m\nThis is a standalone motor bench; the 3D robot remains a separate model.'));
note.Position=[30 390];
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','foc_ts', ...
 'StopTime','.42','ReturnWorkspaceOutputs','on','SignalLogging','off');
set_param(model,'PostLoadFcn',"addpath(fileparts(fileparts(get_param(bdroot,'FileName'))));balance_setup;");
save_system(model,file);close_system(model,0);
fprintf('已生成FOC模型：%s\n',file);
 function wire(a,b),add_line(model,a,b,'autorouting','on');end
end
