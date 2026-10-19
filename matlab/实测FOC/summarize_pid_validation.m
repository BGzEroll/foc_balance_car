function summary=summarize_pid_validation
% Compact native evidence. Longitudinal screening never grants a 3D pass.
here=fileparts(mfilename('fullpath'));out=fullfile(here,'pid_validation');
base=read('mixed_baseline_explore6_tilt0_sensor1_macro500us');
selected=read('mixed_selected_explore6_tilt0_sensor1_macro500us');
summary=struct('date','2026-10-19','current_PI_unchanged',true,'hardware_high_current_validated',false, ...
 'baseline_partial_duration_s',base.duration_s,'selected_full_course',selected, ...
 'baseline_fast_velocity_rms_mps',base.fast_velocity_tracking_rms_mps, ...
 'selected_fast_velocity_rms_mps',selected.fast_velocity_tracking_rms_mps, ...
 'baseline_hill_hold_error_m',base.slope_hold_mean_world_y_error_m, ...
 'selected_hill_hold_error_m',selected.slope_hold_mean_world_y_error_m);
summary.baseline_and_selected_same_current_limit_A=6;
summary.baseline_is_full_course_pass=false;
auditFile=fullfile(out,'odometry_audit.json');if isfile(auditFile),summary.odometry_definition_audit=jsondecode(fileread(auditFile));end
summary.native_speed_probes=struct([]);
for speed=[2.5 4]
 name=sprintf('limit_selected_explore6_tilt0_sensor1_macro500us_v%g',speed);
 if isfile(fullfile(here,'results',name,'validation.json'))
  q=read(name);row=struct('requested_peak_mps',speed,'true_peak_mps',q.maximum_true_speed_mps, ...
   'max_true_pitch_deg',q.max_true_tilt_error_deg,'max_observed_pitch_deg',q.max_tilt_error_deg, ...
   'peak_Iq_A',q.peak_actual_each_iq_A,'voltage_saturation_fraction',q.voltage_saturation_fraction, ...
   'true_final_distance_error_m',q.true_final_distance_error_m,'pass',q.pass);
  if isempty(summary.native_speed_probes),summary.native_speed_probes=row;else,summary.native_speed_probes(end+1)=row;end
 end
end
coarse=fullfile(here,'results','mixed_selected_explore6_tilt0_sensor1_macro500us','simulation_measured.mat');
fine=fullfile(here,'results','mixed_selected_explore6_tilt0_sensor1_macro250us','simulation_measured.mat');
summary.step_refinement=struct('available',false,'full_course_repeated_at_250us',false);
if isfile(fine)
 a=load(coarse,'o');b=load(fine,'o');[ta,va]=truth(a.o);[tb,vb]=truth(b.o);mask=ta<=min(4.8,tb(end));
 delta=va(mask,:)-interp1(tb,vb,ta(mask),'linear');
 summary.step_refinement=struct('available',true,'duration_s',min(4.8,tb(end)), ...
  'max_pitch_difference_deg',max(abs(delta(:,1))),'max_world_position_difference_m',max(hypot(delta(:,2),delta(:,3))), ...
  'max_forward_speed_difference_mps',max(abs(delta(:,4))), ...
  'pitch_tolerance_deg',.15,'position_tolerance_m',.002, ...
  'pass',max(abs(delta(:,1)))<.15&&max(hypot(delta(:,2),delta(:,3)))<.002, ...
  'full_course_repeated_at_250us',false);
end
summary.pass=selected.pass;
summary.functional_course_completed=selected.complete_course&&selected.terrain_pass&&selected.hill_hold_pass&&selected.trajectory_pass&&selected.positive_and_negative_yaw&&~selected.fell;
summary.strict_validation_pass=selected.pass&&summary.step_refinement.available&&summary.step_refinement.pass;
summary.scope='Native 39s Multibody at 500us with electrical 50us; refinement checks first fast segment only. High current, speed, tire friction, L/Kt and high speed noise remain model extrapolations.';
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
copyfile(fullfile(here,'results','mixed_selected_explore6_tilt0_sensor1_macro500us','mixed_course.png'),fullfile(out,'mixed_course.png'));
copyfile(fullfile(here,'results','mixed_selected_explore6_tilt0_sensor1_macro500us','truth_vs_sensed.png'),fullfile(out,'truth_vs_sensed.png'));
f=figure('Visible','off','Color','w','Position',[30 30 1200 650]);tiledlayout(1,2,'Padding','compact');
nexttile;bar([base.fast_velocity_tracking_rms_mps selected.fast_velocity_tracking_rms_mps]);xticklabels({'原参数','选定参数'});ylabel('快速段速度跟踪 RMS / m/s');grid on;
nexttile;bar(100*[base.slope_hold_mean_world_y_error_m selected.slope_hold_mean_world_y_error_m]);xticklabels({'原参数','选定参数'});ylabel('12°坡中停车位置平均误差 / cm');grid on;
sgtitle('相同6A/电压包络下比较上层PID：完整3D机械真值');exportgraphics(f,fullfile(out,'pid_comparison.png'),'Resolution',140);savefig(f,fullfile(out,'pid_comparison.fig'));close(f);
parameterFile=fullfile(here,'pid_selected_parameters.json');s=jsondecode(fileread(parameterFile));
if summary.strict_validation_pass,s.status='native_39s_course_pass_at_500us';elseif summary.functional_course_completed,s.status='functional_course_completed_with_odometry_and_numerical_limits';else,s.status='native_course_incomplete_or_functional_failure';end
s.native_validation=struct('summary','pid_validation/summary.json','full_course_pass',selected.pass,'mechanical_step_s',.0005,'step_refinement',summary.step_refinement);
fid=fopen(parameterFile,'w');fprintf(fid,'%s',jsonencode(s,PrettyPrint=true));fclose(fid);
disp(summary);
 function r=read(name),r=jsondecode(fileread(fullfile(here,'results',name,'validation.json')));end
end
function [t,v]=truth(o)
t=o.raw3d.Time;raw=squeeze(o.raw3d.Data);if size(raw,1)~=numel(t),raw=raw';end
a=balance_control_coordinates(raw);q=a(:,8:11);yaw=atan2(2*(q(:,1).*q(:,4)+q(:,2).*q(:,3)),1-2*(q(:,3).^2+q(:,4).^2));
pitch=atan2(-2*(q(:,2).*q(:,4)-q(:,1).*q(:,3)),hypot(1-2*(q(:,3).^2+q(:,4).^2),2*(q(:,2).*q(:,3)+q(:,1).*q(:,4))));
v=[rad2deg(pitch) raw(:,2) raw(:,4) cos(yaw).*a(:,3)+sin(yaw).*a(:,5)];[t,i]=unique(t,'stable');v=v(i,:);
end
