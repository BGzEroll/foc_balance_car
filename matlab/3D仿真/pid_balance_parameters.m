function p=pid_balance_parameters(profile)
% 经验整定入口，不通过电机Kt、质量或惯量计算这些增益。
% 输出为仿真等效N*m；实机直接输出A时需要重新整定数值尺度。
if nargin<1,profile='normal';end
profile=validatestring(profile,{'normal','agile','slope'});
p.Ts=.005;
p.pitch_trim=deg2rad(-.0896225217620993); % USB侧为前后的静态平衡偏置，可实测调整
p.torque_limit_each=.10;
% 位置P -> 速度PI -> 俯仰PID，另有偏航PID。
p.position_kp=1.3;
p.velocity_kp=1.0;
p.velocity_ki=.03;
p.pitch_kp=.40;
p.pitch_ki=.005;
p.pitch_kd=.040; % 直接使用俯仰角速度，避免差分和指令微分冲击
p.yaw_kp=.105;
p.yaw_ki=.010;
p.yaw_kd=.020;
p.speed_limit=.35; % m/s
p.pitch_target_limit=deg2rad(8);
p.velocity_integral_limit=.5; % m
p.pitch_integral_limit=.25; % rad*s
p.yaw_integral_limit=.5; % rad*s
if strcmp(profile,'agile')
 p.speed_limit=.90;
 p.pitch_target_limit=deg2rad(25);
end
if strcmp(profile,'slope')
 p.speed_limit=.30;p.pitch_target_limit=deg2rad(25);
 p.velocity_ki=.30;p.velocity_integral_limit=1.2;
end
p.profile=profile;
g=[p.position_kp p.velocity_kp p.velocity_ki p.pitch_kp p.pitch_ki p.pitch_kd ...
 p.yaw_kp p.yaw_ki p.yaw_kd];
assert(all(isfinite(g))&&all(g>=0),'PID增益须为有限非负数');
assert(all([p.Ts p.torque_limit_each p.speed_limit p.pitch_target_limit ...
 p.velocity_integral_limit p.pitch_integral_limit p.yaw_integral_limit]>0));
end
