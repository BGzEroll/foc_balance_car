function r=verify_pid_rate_math
% Verify elapsed-time integration and held IMU semantics at both PID rates.
sp=measured_sensor_parameters;imu=zeros(14,1);imu(8)=1;imu(12)=.2;
[~,~,fastFoc]=measured_pid_config('selected',1,'explore6',.001);[~,~,slowFoc]=measured_pid_config('selected',1,'explore6',.005);
assert(isequal(fastFoc.cfg,slowFoc.cfg)&&fastFoc.control_period==.0002&&fastFoc.Ts==.00005,'PID rate change must not alter electrical/current-loop configuration');
act=zeros(30,1);act([15 29])=10.2;
observed=measured_pid_observation(imu,act,.004,sp.cfg);
assert(observed(1)==.004&&isequal(observed(8:14),imu(8:14)));
assert(abs(observed(5)-.325)<1e-12&&abs(observed(3))<1e-12);
act([15 29])=11.2;fresh=measured_pid_observation(imu,act,.0045,sp.cfg);
assert(abs(fresh(5)-observed(5)-.0325)<1e-12,'Encoder changes must be visible without a new IMU frame');
course=measured_course_definition;rows=struct([]);
for Ts=[.001 .005]
 [cfg,d,p]=measured_pid_config('selected',1,'explore6',Ts);cfg(3)=-.001;cfg([17 19 20 21 22 27 28 29])=0;cfg(18)=2;
 clear measured_car_pid
 act([15 29])=10;imu(12)=0;
 for now=0:Ts:1
  sample=measured_pid_observation(imu,act,now,sp.cfg);out=measured_car_pid(sample,cfg,course.moves);
 end
 assert(abs(out(3)-.325)<1e-10,'Distance integration must use elapsed seconds');
 expected=cfg(18)*.001*(1+Ts)/2;assert(max(abs(out(22:23)-expected))<1e-10,'Integral gain must not be multiplied by rate');
 assert(p.Kp==.060318579&&p.Ki==226.194672&&sp.Ts==.005&&d.Ts==Ts);
 row=struct('pid_period_s',Ts,'distance_after_1s_m',out(3),'each_current_A',out(22),'integral_expected_A',expected,'pass',true);
 if isempty(rows),rows=row;else,rows(end+1)=row;end
end
clear measured_car_pid
r=struct('pass',true,'rows',rows,'imu_period_s',sp.Ts,'current_pi_unchanged',true,'fresh_encoder_without_new_imu',true);
here=fileparts(mfilename('fullpath'));folder=fullfile(here,'pid_rate_validation');if ~isfolder(folder),mkdir(folder);end
fid=fopen(fullfile(folder,'integration_checks.json'),'w');fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);disp('PID_RATE_MATH_CHECKS_COMPLETE');
end
