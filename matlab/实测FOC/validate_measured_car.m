function r=validate_measured_car
% Reproduce current PID-only validation. Native runs can take tens of minutes.
verify_measured_physics;verify_noise_profiles;validate_measured_foc;validate_exploration_foc;verify_pid_rate_math;
% Default 1kHz PID; original 200Hz reports remain separate historical evidence.
run_measured_car('mixed',false,0,true,.0005,39,'selected','explore6');
run_measured_car('mixed',false,0,true,.00025,4.8,'selected','explore6');
run_measured_car('recover',false,3,true,.0005,6,'selected','bench');
r=summarize_measured_pid_rate;
if ~r.strict_validation_pass,warning('PID:NumericalQualification','功能场景已完成；严格里程/数值精度资格仍未通过，查看pid_rate_validation/summary.json。');end
end
