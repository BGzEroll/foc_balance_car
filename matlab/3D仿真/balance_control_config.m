function [cfg,d]=balance_control_config(controller,scenario,limit,actuatorGain,profile)
% 末尾为运动档位、坡道停车水平位置和终点，配置共36项。
if nargin<1,controller='lqr';end
if nargin<2,scenario=1;end
if nargin<4,actuatorGain=[1 1];end
if nargin<5,profile='normal';end
motion=balance_motion_profile(profile);
controller=validatestring(controller,{'lqr','pid'});p=pid_balance_parameters(motion.name);
if strcmp(controller,'lqr'),d=design_balance_suite;mode=0;
else
 d=struct('Ts',p.Ts,'K',zeros(2,6),'pitch_trim',p.pitch_trim, ...
  'torque_limit_each',p.torque_limit_each,'parameters',p);mode=1;
end
if nargin<3||isempty(limit),limit=d.torque_limit_each;end
assert(isscalar(limit)&&limit>0,'Torque limit must be positive or Inf');
if isscalar(actuatorGain),actuatorGain=[actuatorGain actuatorGain];end
assert(numel(actuatorGain)==2&&all(isfinite(actuatorGain))&&all(actuatorGain>0),'执行增益须为正标量或左右两个正数');
cfg=[scenario limit d.pitch_trim d.K(:)' mode ...
 p.pitch_kp p.pitch_ki p.pitch_kd p.velocity_kp p.velocity_ki p.position_kp ...
 p.speed_limit p.pitch_target_limit p.velocity_integral_limit p.pitch_integral_limit ...
 p.yaw_kp p.yaw_ki p.yaw_kd p.yaw_integral_limit d.Ts actuatorGain(:)' motion.id];
terrain=balance_terrain_parameters('steep');cfg=[cfg terrain.hold_x terrain.finish_distance];
d.controller=upper(controller);d.config=cfg;d.motion_profile=motion;
end
