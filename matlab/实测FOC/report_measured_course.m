function r=report_measured_course(o,r,out,p,terrain)
% Mechanical truth and actual contact forces decide route completion.
d=measured_course_definition;r.expected_full_course_duration_s=d.duration;r.complete_course=r.duration_s>=d.duration-.01;r.partial_diagnostic=~r.complete_course;
[t,raw]=values(o.raw3d);a=balance_control_coordinates(raw);q=a(:,8:11);
yaw=atan2(2*(q(:,1).*q(:,4)+q(:,2).*q(:,3)),1-2*(q(:,3).^2+q(:,4).^2));v=cos(yaw).*a(:,3)+sin(yaw).*a(:,5);
[tc,c]=values(o.control3d);refV=interp1(tc,c(:,10),t,'linear','extrap');fast=t>=1&t<=4.8;slow=t>=5&t<=7;holdMask=t>=d.hold_check(1)&t<=d.hold_check(2);
r.fast_peak_true_speed_mps=max(abs(v(fast)));r.fast_velocity_tracking_rms_mps=sqrt(mean((v(fast)-refV(fast)).^2));
r.slow_mean_absolute_speed_mps=mean(abs(v(slow)));r.slow_velocity_tracking_rms_mps=sqrt(mean((v(slow)-refV(slow)).^2));
if any(holdMask)
 r.slope_hold_mean_world_y_error_m=mean(raw(holdMask,4)-d.hold_y);r.slope_hold_position_span_m=max(raw(holdMask,4))-min(raw(holdMask,4));r.slope_hold_rms_speed_mps=sqrt(mean(v(holdMask).^2));
else
 r.slope_hold_mean_world_y_error_m=NaN;r.slope_hold_position_span_m=Inf;r.slope_hold_rms_speed_mps=Inf;
end
normal=zeros(numel(t),6);
for j=1:6
 for side={'left','right'}
  name=sprintf('terrain_%d_%s_N',j,side{1});[tn,n]=values(o.(name));normal(:,j)=normal(:,j)+interp1(tn,n,t,'linear',0);
 end
end
r.contact_peak_each_terrain_N=max(normal);r.all_six_terrain_contacts_observed=all(r.contact_peak_each_terrain_N>.1);
r.reached_platform=any(raw(:,4)>terrain.ramp_end&raw(:,4)<terrain.platform_end&raw(:,6)>terrain.height+.025);
r.descended_to_flat=any(raw(:,4)>terrain.downhill_end+.1&raw(:,6)<.045);
r.positive_and_negative_yaw=any(yaw>deg2rad(35))&&any(yaw<deg2rad(-8));
r.maximum_true_speed_mps=max(abs(v));r.terrain_slope_deg=d.slope_deg;r.course=d;
by=[2*(q(:,2).*q(:,3)-q(:,1).*q(:,4)) 1-2*(q(:,2).^2+q(:,4).^2) 2*(q(:,3).*q(:,4)+q(:,1).*q(:,2))];
bodyAxleRate=sum(by.*a(:,12:14),2);[tl,wl]=values(o.left_wheel_w);[tr,wr]=values(o.right_wheel_w);
rolling=.0325*(.5*(interp1(tl,wl,t,'linear','extrap')+interp1(tr,wr,t,'linear','extrap'))+bodyAxleRate);slip=rolling-v;
r.fast_true_slip_rms_mps=sqrt(mean(slip(fast).^2));r.peak_true_slip_mps=max(abs(slip));
[ta,act]=values(o.foc_actuator);r.estimated_copper_energy_J=trapz(ta,1.5*p.Rs*(act(:,[4 18]).^2+act(:,[5 19]).^2));
r.thermal_model_present=false;r.hardware_high_current_validated=false;
r.trajectory_pass=r.fast_peak_true_speed_mps>1.0&&r.fast_velocity_tracking_rms_mps<.30&&r.slow_velocity_tracking_rms_mps<.12;
r.terrain_pass=r.all_six_terrain_contacts_observed&&r.reached_platform&&r.descended_to_flat;
r.hill_hold_pass=abs(r.slope_hold_mean_world_y_error_m)<.05&&r.slope_hold_position_span_m<.05&&r.slope_hold_rms_speed_mps<.06;
r.pass=r.complete_course&&r.completed&&~r.fell&&r.max_true_tilt_error_deg<40&&r.max_roll_deg<8&&abs(r.true_final_distance_error_m)<.03&&abs(r.true_final_yaw_error_deg)<2&&abs(r.true_final_forward_speed_mps)<.03&&r.trajectory_pass&&r.terrain_pass&&r.hill_hold_pass&&r.positive_and_negative_yaw&&r.fast_true_slip_rms_mps<.25;
f=figure('Visible','off','Color','w','Position',[20 20 1400 1000]);tiledlayout(4,1,'Padding','compact');
nexttile;plot(t,v,tc,c(:,10),'--');grid on;ylabel('真实前向速度 / m/s');legend('机械真值','指令');
nexttile;plot(t,raw(:,6));hold on;yline(.0325+d.height,':');grid on;ylabel('轮轴高度 / m');
nexttile;plot(t,normal);grid on;ylabel('各地形支持力 / N');legend('上坡','平台','下坡','左凸起','右凸起','整轮减速带','Location','eastoutside');
nexttile;plot(t,slip);grid on;ylabel('真实滚动滑差 / m/s');xlabel('时间 / s');
sgtitle(sprintf('综合场景：PID / %s / %s / pass=%d',r.pid_tuning,r.operating_envelope,r.pass));exportgraphics(f,fullfile(out,'mixed_course.png'),'Resolution',140);savefig(f,fullfile(out,'mixed_course.fig'));close(f);
fid=fopen(fullfile(out,'validation.json'),'w');fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);
fprintf('MIXED_COURSE pass=%d fast=%.3fm/s hold=%.4fm terrain=%d\n',r.pass,r.fast_peak_true_speed_mps,r.slope_hold_mean_world_y_error_m,r.terrain_pass);
end
function [t,v]=values(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end
[t,i]=unique(t,'stable');v=v(i,:);
end
