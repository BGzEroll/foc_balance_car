function r=report_balance_case(o,scenario,duration,limit,out,controller,actuatorGain,profile,terrain)
if nargin<6,controller='LQR';end
if nargin<7,actuatorGain=[1 1];end
if nargin<8,profile='normal';end
if nargin<9,terrain=balance_terrain_parameters('mild');end
if ~isfolder(out),mkdir(out);end
[t,c]=values(o.control3d);[tr,raw]=values(o.raw3d);
state=c(:,3:8);ref=c(:,9:14);e=state-ref;tail=t>=t(end)-.5;
% 近似滚动滑差：轮组总自转与沿接触切线的速度之差。
[tl,wl]=values(o.left_wheel_w);[tw,wr]=values(o.right_wheel_w);
bodyY=[2*(raw(:,9).*raw(:,10)-raw(:,8).*raw(:,11)) ...
 1-2*(raw(:,9).^2+raw(:,11).^2) 2*(raw(:,10).*raw(:,11)+raw(:,8).*raw(:,9))];
bodyAxleRate=sum(bodyY.*raw(:,12:14),2);
rollingSpeed=.0325*(.5*(sample(tl,wl,t)+sample(tw,wr,t))+sample(tr,bodyAxleRate,t));
q=raw(:,8:11);roll=atan2(2*(q(:,1).*q(:,2)+q(:,3).*q(:,4)),1-2*(q(:,2).^2+q(:,3).^2));
contacts=zeros(numel(t),1);terrainForce=zeros(numel(t),1);uphillSupport=zeros(numel(t),1);downhillSupport=zeros(numel(t),1);
for side={'left','right'}
 s=side{1};[tt,yy]=values(o.get([s '_ground_N']));contacts=contacts+sample(tt,yy,t);
 for j=1:4
  [tt,yy]=values(o.get(['terrain_' num2str(j) '_' s '_N']));v=sample(tt,yy,t);
  contacts=contacts+v;terrainForce=terrainForce+v;
  if j==1,uphillSupport=uphillSupport+v;end
  if j==3,downhillSupport=downhillSupport+v;end
 end
end
% 支持力加权的接触切线；不把初始落地时的竖直速度误判为轮胎打滑。
alpha=deg2rad(terrain.angle_deg);flatSupport=contacts-uphillSupport-downhillSupport;
contactAngle=atan2(sin(alpha)*(uphillSupport-downhillSupport),flatSupport+cos(alpha)*(uphillSupport+downhillSupport));
tangentV=cos(contactAngle).*state(:,2)+sin(contactAngle).*sample(tr,raw(:,7),t);
rollingSlip=rollingSpeed-tangentV;
r=struct('scenario',scenario,'controller',controller,'motion_profile',profile,'expected_duration_s',duration,'duration_s',t(end), ...
 'completed',t(end)>=duration-.01,'fell',any(c(:,21)>.5),'max_tilt_error_deg',max(abs(rad2deg(state(:,3)))), ...
 'max_roll_deg',max(abs(rad2deg(roll))),'final_distance_m',state(end,1), ...
 'final_distance_error_m',mean(e(tail,1)),'final_yaw_error_deg',rad2deg(mean(e(tail,5))), ...
 'final_speed_mps',mean(state(tail,2)),'max_speed_mps',max(abs(state(:,2))), ...
 'peak_each_torque_Nm',max(abs(c(:,1:2)),[],'all'),'configured_torque_limit_each_Nm',limit, ...
 'saturated_fraction',mean(any(abs(c(:,22:23))>=limit-1e-8,2)), ...
 'actuator_gain_left',actuatorGain(1),'actuator_gain_right',actuatorGain(2), ...
 'peak_approx_rolling_slip_mps',max(abs(rollingSlip)),'first_forward_90percent_time_s',NaN, ...
 'max_terrain_normal_N',max(terrainForce),'max_axle_height_m',max(raw(:,6)), ...
 'min_axle_height_m',min(raw(:,6)),'final_world_x_m',raw(end,2),'final_world_y_m',raw(end,4), ...
 'final_total_normal_N',mean(contacts(tail)), ...
 'peak_external_force_N',max(vecnorm(c(:,15:17),2,2)), ...
 'peak_external_torque_Nm',max(vecnorm(c(:,18:20),2,2)), ...
 'terrain_angle_deg',terrain.angle_deg,'terrain_height_m',terrain.height, ...
 'hold_mean_x_error_m',NaN,'hold_x_span_m',NaN,'hold_mean_abs_speed_mps',NaN, ...
 'hold_peak_speed_mps',NaN,'hold_on_incline_fraction',NaN,'hold_rollback_m',NaN, ...
 'hold_stability_pass',NaN,'hold_tracking_pass',NaN, ...
 'climbed_ramp',NaN,'descended_ramp',NaN,'passed_bump',NaN);
if isinf(limit),r.configured_torque_limit_each_Nm='unlimited';end
allowedTilt=30;if strcmp(profile,'agile'),allowedTilt=40;end
r.allowed_tilt_check_deg=allowedTilt;
r.pass=r.completed && ~r.fell && r.max_tilt_error_deg<allowedTilt && r.max_roll_deg<8;
if ~strcmpi(scenario,'bias')
 r.pass=r.pass && abs(r.final_distance_error_m)<.02 && abs(r.final_yaw_error_deg)<2 && abs(r.final_speed_mps)<.02;
end
if strcmpi(scenario,'terrain'),r.pass=r.pass && r.max_terrain_normal_N>.1 && r.max_axle_height_m>.034;end
if strcmpi(scenario,'push'),r.pass=r.pass&&r.peak_external_force_N>.3&&r.peak_external_torque_Nm>.001;end
if strcmpi(scenario,'turn'),r.pass=r.pass&&max(abs(state(:,5)))>deg2rad(50);end
if strcmpi(scenario,'drive'),r.pass=r.pass&&max(state(:,1))>.25;end
if strcmpi(scenario,'route'),r.pass=r.pass&&max(state(:,1))>.55&&max(state(:,5))>deg2rad(50);end
if ismember(lower(scenario),{'slope','slope_hold'})
 r.climbed_ramp=double(r.max_axle_height_m>terrain.height+.025&&max(raw(:,2))>terrain.ramp_end+.03);
 r.descended_ramp=double(any(raw(:,2)>terrain.downhill_end+.04&raw(:,6)<.05));
 r.passed_bump=double(max(raw(:,2))>terrain.bump_start+terrain.bump_length+.05);
 r.pass=r.pass&&r.max_terrain_normal_N>.5&&r.max_axle_height_m>terrain.height+.025;
end
if strcmpi(scenario,'slope_hold')
 holdMask=t>=terrain.hold_check_start&t<=terrain.hold_check_end;
 x=sample(tr,raw(:,2),t);allHold=t>=terrain.hold_reference_start&t<terrain.hold_reference_end;
 if any(holdMask)&&any(allHold)
  r.hold_mean_x_error_m=mean(x(holdMask)-terrain.hold_x);
  r.hold_x_span_m=max(x(holdMask))-min(x(holdMask));
  r.hold_mean_abs_speed_mps=mean(abs(state(holdMask,2)));r.hold_peak_speed_mps=max(abs(state(holdMask,2)));
  r.hold_on_incline_fraction=mean(x(holdMask)>terrain.ramp_start+.03 & x(holdMask)<terrain.ramp_end-.03 & uphillSupport(holdMask)>.5);
  r.hold_rollback_m=max(0,x(find(allHold,1))-min(x(allHold)));
  r.hold_stability_pass=double(r.hold_on_incline_fraction>.95&&r.hold_x_span_m<.03&&r.hold_mean_abs_speed_mps<.01&&r.hold_peak_speed_mps<.03);
  r.hold_tracking_pass=double(abs(r.hold_mean_x_error_m)<.02);
  r.pass=r.pass&&r.hold_stability_pass>.5&&r.hold_tracking_pass>.5;
 else,r.pass=false;end
end
if strcmpi(scenario,'drive')
 first=find(t>=1&state(:,1)>=.27,1);
 r.first_forward_90percent_time_s=NaN;if ~isempty(first),r.first_forward_90percent_time_s=t(first)-1;end
end
fid=fopen(fullfile(out,'validation.json'),'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);
tab=array2table([t c contacts terrainForce rollingSlip],'VariableNames',{'time_s','tau_left_Nm','tau_right_Nm', ...
 'distance_m','speed_mps','tilt_error_rad','pitch_rate_radps','yaw_rad','yaw_rate_radps', ...
 'distance_ref_m','speed_ref_mps','tilt_ref_rad','pitch_rate_ref_radps','yaw_ref_rad','yaw_rate_ref_radps', ...
 'force_x_N','force_y_N','force_z_N', ...
 'torque_x_Nm','torque_y_Nm','torque_z_Nm','fall_signal', ...
 'command_left_Nm','command_right_Nm','pid_speed_target_mps','pid_pitch_target_rad','total_normal_N','terrain_normal_N','approx_rolling_slip_mps'});
writetable(tab,fullfile(out,'trajectory.csv'));
writetable(array2table([tr raw(:,2:14) rad2deg(roll)],'VariableNames', ...
 {'time_s','x_m','vx_mps','y_m','vy_mps','z_m','vz_mps','qw','qx','qy','qz','wx_radps','wy_radps','wz_radps','roll_deg'}), ...
 fullfile(out,'body_pose.csv'));
save(fullfile(out,'simulation.mat'),'o','r','-v7.3');
f=figure('Visible','off','Color','w','Position',[50 50 1100 850]);tiledlayout(3,2);
nexttile;plot(t,state(:,1),t,ref(:,1),'--');ylabel('纵向路程 (m)');grid on;legend('实际','指令');
nexttile;plot(t,rad2deg(state(:,5)),t,rad2deg(ref(:,5)),'--');ylabel('偏航 (°)');grid on;
nexttile;plot(t,rad2deg(state(:,3)),t,rad2deg(c(:,25)),'--');ylabel('俯仰偏差 (°)');grid on;legend('实际','目标');
nexttile;plot(tr,rad2deg(roll));ylabel('横滚 (°)');grid on;
nexttile;plot(t,c(:,1:2));ylabel('单轮转矩 (N·m)');grid on;legend('左','右');
nexttile;plot(t,contacts,t,terrainForce);ylabel('地面支持力 (N)');grid on;legend('全部接触','地形接触');
sgtitle(sprintf('原始3D模型 | %s | %s | %s | pass=%d',scenario,controller,profile,r.pass));
exportgraphics(f,fullfile(out,'results.png'),'Resolution',140);close(f);
f=figure('Visible','off','Color','w');plot(raw(:,2),raw(:,4),'LineWidth',1.5);axis equal;grid on;
xlabel('世界X (m)');ylabel('世界Y (m)');title(sprintf('%s / %s：轮轴中心实际行驶轨迹',scenario,controller));
exportgraphics(f,fullfile(out,'path.png'),'Resolution',140);close(f);
if ismember(lower(scenario),{'slope','slope_hold'})
 f=figure('Visible','off','Color','w','Position',[40 40 1100 750]);tiledlayout(2,2);
 nexttile;plot(tr,raw(:,2),t,ref(:,1),'--');grid on;ylabel('水平位置 / m');legend('实际世界X','目标路程');
 if strcmpi(scenario,'slope_hold'),xline(7,':','HandleVisibility','off');xline(19,':','HandleVisibility','off');end
 nexttile;plot(raw(:,2),raw(:,6));hold on;
 gx=[0 terrain.ramp_start terrain.ramp_end terrain.platform_end terrain.downhill_end terrain.finish_distance];
 gz=[0 0 terrain.height terrain.height 0 0];plot(gx,gz,'k-','LineWidth',2);axis equal;grid on;xlabel('世界X / m');ylabel('高度 / m');legend('轮轴中心','坡面');
 nexttile;plot(t,state(:,2));grid on;ylabel('速度 / m/s');xlabel('时间 / s');
 nexttile;plot(t,c(:,1:2),t,uphillSupport*.01,'--');grid on;ylabel('N·m，支持力×0.01');xlabel('时间 / s');legend('左轮转矩','右轮转矩','上坡面支持力×0.01');
 sgtitle(sprintf('%s / %s / %s：%.1f°坡，高%.1fcm；pass=%d',scenario,controller,profile,terrain.angle_deg,100*terrain.height,r.pass));
 exportgraphics(f,fullfile(out,'slope_details.png'),'Resolution',140);close(f);
end
fprintf('%s %s: pass=%d, position error=%.4g m, yaw error=%.4g deg, peak torque=%.4g Nm\n', ...
 scenario,controller,r.pass,r.final_distance_error_m,r.final_yaw_error_deg,r.peak_each_torque_Nm);
end

function [t,y]=values(ts)
t=ts.Time;y=squeeze(ts.Data);if size(y,1)~=numel(t),y=y';end
[t,i]=unique(t,'stable');y=y(i,:);
end
function y=sample(t,v,at)
y=interp1(t,v,at,'linear','extrap');
end

