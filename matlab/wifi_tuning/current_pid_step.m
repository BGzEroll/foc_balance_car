function [out,s]=current_pid_step(m,s,g,cap)
% Direct-current controller, same 5 ms update as firmware. No Kt in this law.
if isempty(s),s=struct('distance',0,'prev_speed',0,'vi',0,'pi',0,'yi',0,'first',true);end
Ts=.005;clamp=@(x,l)max(-l,min(l,x));
if ~s.first,s.distance=s.distance+Ts*(s.prev_speed+m.speed)/2;end
s.first=false;s.prev_speed=m.speed;
target_speed=clamp(-g.position_kp*s.distance,.65);
ve=target_speed-m.speed;vi=clamp(s.vi+Ts*ve,1.2);
target_unclamped=g.velocity_kp*ve+g.velocity_ki*vi;
target_pitch=clamp(target_unclamped,deg2rad(8));
pe=m.pitch-g.pitch_trim_rad-target_pitch;pi=clamp(s.pi+Ts*pe,.25);
ye=atan2(sin(-m.yaw),cos(-m.yaw));yi=clamp(s.yi+Ts*ye,.5);
common=g.pitch_kp*pe+g.pitch_ki*pi+g.pitch_kd*m.pitch_rate;
diff=g.yaw_kp*ye+g.yaw_ki*yi-g.yaw_kd*m.yaw_rate;
u=[common-diff,common+diff]/2;saturated=max(abs(u))>cap;
if ~saturated
 s.pi=pi;s.yi=yi;
 if abs(target_unclamped)<=deg2rad(8),s.vi=vi;end
end
out=struct('current',clamp(u,cap),'target_speed',target_speed,...
 'target_pitch',target_pitch,'saturated',saturated,'distance',s.distance);
end
