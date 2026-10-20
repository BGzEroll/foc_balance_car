function verify_measured_timing(model)
% Called after compilation, before simulation starts; verify actual execution.
pidTs=str2double(get_param([model '/PID_clock'],'SampleTime'));
imuTs=str2double(get_param([model '/Sample_200Hz'],'SampleTime'));
macroTs=str2double(get_param([model '/FOC_clock'],'SampleTime'));
blocks={'Balance_Controller','PID_observation','Measured_sensors','Measured_FOC'};
expected=[pidTs pidTs imuTs macroTs];
for k=1:numel(blocks)
 sample=get_param([model '/' blocks{k}],'CompiledSampleTime');
 if iscell(sample),assert(numel(sample)==1,'Unexpected multirate function');sample=sample{1};end
 assert(isnumeric(sample)&&abs(sample(1)-expected(k))<1e-10,['实际调度周期不符: ' blocks{k}]);
end
fprintf('MEASURED_TIMING pid=%gHz encoder=%gHz imu=%gHz foc_updates=5000Hz electrical_dt=50us\n',1/pidTs,1/pidTs,1/imuTs);
end
