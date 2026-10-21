function p=measured_ekf_parameters
%#codegen
% Fixed conservative tuning, selected before closed-loop evaluation.
% Noise priors are NOT claimed to be independently identified IMU parameters.
p.gyro_std_rad_s=.005; % Per-sample gyro uncertainty, 200Hz.
p.bias_walk_rad_s_sqrt_s=2e-5;
p.acc_direction_std=.02; % Includes unmodelled specific force / colored noise.
p.innovation_scale=.08; % Inflate R during inconsistent gravity observations.
p.acc_norm_gate=.20; % Fraction of calibrated 1g; reject above this mismatch.
p.initial_attitude_std_rad=2*pi/180;
p.initial_bias_std_rad_s=.5*pi/180;
end
