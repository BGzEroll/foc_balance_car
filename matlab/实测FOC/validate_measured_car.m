function r=validate_measured_car
% Reproduce current PID-only validation. Native runs can take tens of minutes.
verify_measured_physics;verify_noise_profiles;validate_measured_foc;validate_exploration_foc;
% Original gains and selected gains use exactly the same 6A/voltage envelope.
run_measured_car('mixed',false,0,true,.0005,14.5,'baseline','explore6');
run_measured_car('mixed',false,0,true,.0005,39,'selected','explore6');
run_measured_car('mixed',false,0,true,.00025,4.8,'selected','explore6');
% A deliberately failed high speed case is evidence, not a regression failure.
run_measured_car('limit',false,0,true,.0005,7,'selected','explore6',2.5);
run_measured_car('limit',false,0,true,.0005,7,'selected','explore6',4);
r=analyze_course_odometry; %#ok<NASGU>
r=summarize_pid_validation;assert(r.functional_course_completed,'综合功能场景未完成，查看pid_validation/summary.json');
if ~r.strict_validation_pass,warning('PID:NumericalQualification','功能场景已完成；严格里程/数值精度资格仍未通过，查看summary.json。');end
end
