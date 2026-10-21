function p=encoder_yaw_parameters
%#codegen
% Encoder-only turn detection; fixed before closed-loop evaluation.
p.radius_m=.0325;p.track_m=.1325364;
p.window_s=.1;p.fast_tau_s=.02;p.confirm_s=.15;
p.enter_rad_s=.5*pi/180;p.exit_rad_s=1*pi/180;p.fast_exit_rad_s=3*pi/180;
p.bias_tau_s=2;
p.bias_initial_std_rad_s=.5*pi/180;
p.bias_walk_rad_s_sqrt_s=.015*pi/180;
p.zero_rate_std_rad_s=.15*pi/180;
end
