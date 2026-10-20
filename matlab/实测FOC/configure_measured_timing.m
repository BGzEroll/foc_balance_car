function timing=configure_measured_timing(model,controllerTs,mechanicalTs,imuTs)
% Keep controller, acquired IMU, encoder hold and electrical steps distinct.
assert(ismember(controllerTs,[.001 .005])&&imuTs==.005);
for period=[controllerTs imuTs]
 assert(abs(period/mechanicalTs-round(period/mechanicalTs))<1e-8,'机械步长必须整除PID及IMU周期');
end
for name={'Sample_200Hz','Encoder_feedback_200Hz'}
 set_param([model '/' name{1}],'SampleTime',num2str(imuTs,16));
end
for name={'PID_clock','PID_imu_hold','PID_encoder_sample'}
 set_param([model '/' name{1}],'SampleTime',num2str(controllerTs,16));
end
for name={'FOC_clock','Motor_sample','Motor_mechanics_delay','Encoder_feedback_delay'}
 set_param([model '/' name{1}],'SampleTime',num2str(mechanicalTs,16));
end
set_param(model,'MaxStep',num2str(mechanicalTs,16),'StartFcn','verify_measured_timing(bdroot);');
timing=struct('pid_period_s',controllerTs,'imu_period_s',imuTs,'encoder_period_s',controllerTs, ...
 'mechanical_macro_step_s',mechanicalTs,'current_update_period_s',.0002,'electrical_integration_step_s',.00005);
end
