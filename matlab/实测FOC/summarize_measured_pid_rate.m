function summary=summarize_measured_pid_rate
% Keep historical 200Hz evidence distinct from new 1kHz native simulations.
here=fileparts(mfilename('fullpath'));folder=fullfile(here,'pid_rate_validation');if ~isfolder(folder),mkdir(folder);end
coarse='mixed_selected_explore6_tilt0_sensor1_macro500us_pid1000us';
fine='mixed_selected_explore6_tilt0_sensor1_macro250us_pid1000us';
historical='mixed_selected_explore6_tilt0_sensor1_macro500us';
oldFine='mixed_selected_explore6_tilt0_sensor1_macro250us';
r=read(coarse);fineReport=read(fine);
s=load(fullfile(here,'results',coarse,'simulation_measured.mat'),'o','cfg','sp','p');
imuTick=unique(s.o.sensed3d.Time);encoderTick=unique(s.o.pid_observation.Time);
assert(s.cfg(31)==.001&&s.sp.Ts==.005&&s.p.Kp==.060318579&&s.p.Ki==226.194672&&s.p.control_period==.0002&&s.p.Ts==.00005);
summary=struct('date','2026-10-20','default_pid_period_s',.001,'pid_clock_verified',r.timing_validation_pass, ...
 'observed_imu_period_s',median(diff(imuTick)),'observed_encoder_period_s',median(diff(encoderTick)), ...
 'same_upper_gains_as_200Hz',true,'current_loop_changed',false,'hardware_changed',false, ...
 'native_1kHz_full_course',r,'native_1kHz_fast_250us',fineReport);
summary.functional_course_completed=r.complete_course&&r.completed&&~r.fell&&r.max_true_tilt_error_deg<40&&r.max_roll_deg<8&&r.terrain_pass&&r.trajectory_pass&&r.hill_hold_pass&&r.positive_and_negative_yaw;
summary.execution_rates_pass=r.timing_validation_pass&&all(abs(diff(imuTick)-.005)<1e-8)&&all(abs(diff(encoderTick)-.001)<1e-8);
recoveryFile=fullfile(here,'results','recover_selected_bench_tilt3_sensor1_macro500us_pid1000us','validation.json');
if isfile(recoveryFile)
 recovery=jsondecode(fileread(recoveryFile));summary.low_current_recovery=recovery;
 summary.low_current_recovery_pass=recovery.duration_s>=6-.01&&recovery.pass&&recovery.timing_validation_pass;
end
if isfile(fullfile(here,'results',historical,'validation.json'))
 old=read(historical);keys={'position_kp','velocity_kp','velocity_ki','pitch_kp','pitch_ki','pitch_kd','yaw_kp','yaw_ki','yaw_kd'};
 for k=1:numel(keys),assert(abs(old.pid_parameters.(keys{k})-r.pid_parameters.(keys{k}))<1e-12,'Upper gains changed');end
 summary.historical_200Hz=struct('pid_period_s',.005,'fast_velocity_rms_mps',old.fast_velocity_tracking_rms_mps, ...
  'hill_hold_mean_error_m',old.slope_hold_mean_world_y_error_m,'max_true_tilt_deg',old.max_true_tilt_error_deg,'peak_actual_each_iq_A',old.peak_actual_each_iq_A);
end
f=load(fullfile(here,'results',fine,'simulation_measured.mat'),'o');[tc,vc]=truth(s.o);[tf,vf]=truth(f.o);mask=tc<=min(4.8,tf(end));delta=vc(mask,:)-interp1(tf,vf,tc(mask),'linear');
summary.step_refinement=struct('duration_s',min(4.8,tf(end)),'max_pitch_difference_deg',max(abs(delta(:,1))), ...
 'max_world_position_difference_m',max(hypot(delta(:,2),delta(:,3))),'pitch_tolerance_deg',.15,'position_tolerance_m',.002, ...
 'pass',max(abs(delta(:,1)))<.15&&max(hypot(delta(:,2),delta(:,3)))<.002,'full_course_repeated_at_250us',false);
summary.strict_validation_pass=r.pass&&summary.execution_rates_pass&&summary.step_refinement.pass;
summary.scope='PID/encoder 1kHz with held 200Hz IMU; unchanged gains/current PI. Full course at 500us and fast segment at 250us. Historical speed boundaries remain 200Hz evidence.';
if isfile(fullfile(here,'results',oldFine,'simulation_measured.mat'))
 oldFineReport=read(oldFine);
 summary.same_step_fast_comparison=struct('mechanical_step_s',.00025, ...
  'old_200Hz_velocity_rms_mps',oldFineReport.fast_velocity_tracking_rms_mps,'new_1kHz_velocity_rms_mps',fineReport.fast_velocity_tracking_rms_mps, ...
  'velocity_rms_reduction_percent',100*(1-fineReport.fast_velocity_tracking_rms_mps/oldFineReport.fast_velocity_tracking_rms_mps), ...
  'old_200Hz_iq_tracking_rms_A',oldFineReport.actual_iq_tracking_rms_A,'new_1kHz_iq_tracking_rms_A',fineReport.actual_iq_tracking_rms_A, ...
  'comparison_changes','Both PID and encoder observation rate change; IMU stays 200Hz. Current-reference edge tracking RMS is not a constant-current noise metric.');
 old=load(fullfile(here,'results',oldFine,'simulation_measured.mat'),'o','cfg');assert(old.cfg(31)==.005);[to,vo]=truth(old.o);
 figureHandle=figure('Visible','off','Color','w','Position',[30 30 1250 850]);tiledlayout(3,1,'Padding','compact');
 nexttile;plot(to,vo(:,4),tf,vf(:,4));grid on;ylabel('真实前向速度 / m/s');legend('PID/编码器200Hz','PID/编码器1kHz');
 nexttile;plot(to,vo(:,1),tf,vf(:,1));grid on;ylabel('真实俯仰 / deg');
 nexttile;[ta,ya]=values(old.o.foc_actuator);[tb,yb]=values(f.o.foc_actuator);plot(ta,max(abs(ya(:,[14 28])),[],2),tb,max(abs(yb(:,[14 28])),[],2));grid on;ylabel('较大侧目标Iq / A');xlabel('时间 / s');
 sgtitle('相同250us机械步长、相同增益与200Hz IMU：上层调度对比');exportgraphics(figureHandle,fullfile(folder,'rate_comparison.png'),'Resolution',140);savefig(figureHandle,fullfile(folder,'rate_comparison.fig'));close(figureHandle);
end
copyfile(fullfile(here,'results',coarse,'mixed_course.png'),fullfile(folder,'mixed_course.png'));
copyfile(fullfile(here,'results',coarse,'truth_vs_sensed.png'),fullfile(folder,'truth_vs_sensed.png'));
fid=fopen(fullfile(folder,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
parametersFile=fullfile(here,'pid_selected_parameters.json');parameters=jsondecode(fileread(parametersFile));
parameters.controller_timing=struct('pid_period_s',.001,'encoder_period_s',.001,'imu_period_s',.005,'gains_retuned',false,'validation','pid_rate_validation/summary.json', ...
 'execution_rates_pass',summary.execution_rates_pass,'functional_course_completed',summary.functional_course_completed,'strict_validation_pass',summary.strict_validation_pass);
fid=fopen(parametersFile,'w');fprintf(fid,'%s',jsonencode(parameters,PrettyPrint=true));fclose(fid);
fid=fopen(fullfile(folder,'README.md'),'w');fprintf(fid,'# 上层PID 1kHz后台验证\n\n2026-10-20。PID及编码器观测1kHz；IMU、AR12创新和互补姿态滤波保持200Hz。未改变上层增益、base电流PI、每侧5kHz电流更新或50us电气积分，也未修改实物固件。\n\n');
fprintf(fid,'- 实际编译周期与时钟日志验证通过：%d；39秒PID时钟点数：%d。\n',summary.execution_rates_pass,r.pid_tick_count);
fprintf(fid,'- 完整39秒功能场景完成：%d；六块地形接触：%d；坡停门槛：%d。\n',summary.functional_course_completed,r.terrain_pass,r.hill_hold_pass);
fprintf(fid,'- 1kHz快速段速度RMS：%.6f m/s；12度坡停平均世界Y误差：%.6f m；真实俯仰峰值：%.3f度。\n',r.fast_velocity_tracking_rms_mps,r.slope_hold_mean_world_y_error_m,r.max_true_tilt_error_deg);
if isfield(summary,'historical_200Hz'),h=summary.historical_200Hz;fprintf(fid,'- 同增益、500us机械步长的历史200Hz：速度RMS %.6f m/s；坡停误差 %.6f m；俯仰峰值 %.3f度。\n',h.fast_velocity_rms_mps,h.hill_hold_mean_error_m,h.max_true_tilt_deg);end
if isfield(summary,'low_current_recovery_pass'),fprintf(fid,'- 0.7A、3度倾斜、6秒恢复检查：%d。\n',summary.low_current_recovery_pass);end
if isfield(summary,'same_step_fast_comparison'),a=summary.same_step_fast_comparison;fprintf(fid,'\n相同250us步长的快速段：200Hz/1kHz速度RMS为%.6f / %.6f m/s，降低%.2f%%；两侧Iq跟踪RMS由[%.6f, %.6f]增至[%.6f, %.6f] A。此次同时改变PID及编码器观测频率，IMU仍200Hz。电流指令边沿更频繁，因此不能把该跟踪误差当成恒定电流噪声，也不能宣称频率提高五倍就使整车性能提高五倍。\n',a.old_200Hz_velocity_rms_mps,a.new_1kHz_velocity_rms_mps,a.velocity_rms_reduction_percent,a.old_200Hz_iq_tracking_rms_A(1),a.old_200Hz_iq_tracking_rms_A(2),a.new_1kHz_iq_tracking_rms_A(1),a.new_1kHz_iq_tracking_rms_A(2));end
fprintf(fid,'\n1kHz快速段500us与250us宏步长的最大姿态差：%.6f度，最大水平位置差：%.6f m；原0.15度/2mm资格检查通过：%d。250us只复核前4.8秒，没有完整重跑39秒。严格综合资格通过：%d，不能把调度提高直接当成物理性能或极限速度提高。\n',summary.step_refinement.max_pitch_difference_deg,summary.step_refinement.max_world_position_difference_m,summary.step_refinement.pass,summary.strict_validation_pass);
fprintf(fid,'\n`rate_comparison.png`使用相同250us机械步长比较原200Hz与1kHz快速段；`mixed_course.png`是1kHz完整综合场景。250us工况只跑4.8秒，其综合场景pass字段按完整39秒条件计算；片段的调度与响应检查单独记录。原2.5/4m/s速度边界仍属于200Hz历史数据。运行`validate_measured_car`可重做当前验证；命令行第10项`1e-3`/`5e-3`选择周期，结果按PID周期分目录。\n');fclose(fid);
disp(summary);assert(summary.execution_rates_pass,'Execution rate validation failed');assert(summary.functional_course_completed,'1kHz course function failed');
 function report=read(name),report=jsondecode(fileread(fullfile(here,'results',name,'validation.json')));end
end
function [t,v]=truth(o)
[t,raw]=values(o.raw3d);a=balance_control_coordinates(raw);q=a(:,8:11);
yaw=atan2(2*(q(:,1).*q(:,4)+q(:,2).*q(:,3)),1-2*(q(:,3).^2+q(:,4).^2));
pitch=atan2(-2*(q(:,2).*q(:,4)-q(:,1).*q(:,3)),hypot(1-2*(q(:,3).^2+q(:,4).^2),2*(q(:,2).*q(:,3)+q(:,1).*q(:,4))));
v=[rad2deg(pitch) raw(:,2) raw(:,4) cos(yaw).*a(:,3)+sin(yaw).*a(:,5)];
end
function [t,v]=values(ts),t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end;[t,i]=unique(t,'stable');v=v(i,:);end
