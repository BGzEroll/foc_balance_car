function p=balance_terrain_parameters(kind)
% 坡角/几何可修改后重跑，单位m、deg、s。
% 停车时序字段与控制器中的固定阶段一致；改时序需同步控制器并重建模型。
if nargin<1,kind='mild';end
p.kind=kind;p.width=.35;p.thickness=.02;
if strcmp(kind,'mild')
 p.width=.30;p.thickness=.01;
 p.angle_deg=rad2deg(atan(.003/.12));p.ramp_start=.18;p.ramp_run=.12;
 p.height=.003;p.platform_length=.10;p.bump_start=.60;p.bump_length=.04;p.bump_height=.0008;
else
 p.angle_deg=12;                 % 可改为15或18度；结果只验证实际运行值
 p.ramp_start=.25;p.ramp_run=.60;
 p.height=p.ramp_run*tand(p.angle_deg);p.platform_length=.20;
 p.bump_start=1.80;p.bump_length=.04;p.bump_height=.003;
end
p.ramp_end=p.ramp_start+p.ramp_run;
p.platform_end=p.ramp_end+p.platform_length;
p.downhill_end=p.platform_end+p.ramp_run;
p.hold_y=p.ramp_start+p.ramp_run/2; % 斜面中段，非平台
p.hold_reference_start=7;p.hold_reference_end=19;
p.hold_check_start=12;p.hold_check_end=18.5;
p.finish_distance=1.95;
assert(p.angle_deg>0&&p.angle_deg<35&&p.ramp_run>0);
end
