function p=balance_motion_profile(name)
% 运动配置独立于控制器：快慢不仅由LQR/PID决定，也取决于指令轨迹。
if nargin<1,name='normal';end
name=validatestring(name,{'normal','agile','slope'});
p=struct('name',name,'id',0,'durations',[6 11 9 10 9 13 18 26 34]);
if strcmp(name,'agile'),p.id=1;p.durations=[6 7 7 8 9 13 18 26 34];end
if strcmp(name,'slope'),p.id=2;end
end
