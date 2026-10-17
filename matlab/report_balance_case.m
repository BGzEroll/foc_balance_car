function r=report_balance_case(o,scenario,controller,duration,limit,out)
if ~isfolder(out),mkdir(out);end
[t,c]=values(o.control3d);[tr,raw]=values(o.raw3d);
state=c(:,3:8);ref=c(:,9:14);e=state-ref;tail=t>=t(end)-.5;
q=raw(:,8:11);roll=atan2(2*(q(:,1).*q(:,2)+q(:,3).*q(:,4)),1-2*(q(:,2).^2+q(:,3).^2));
contacts=zeros(numel(t),1);terrainForce=zeros(numel(t),1);
for side={'left','right'}
 s=side{1};[tt,yy]=values(o.get([s '_ground_N']));contacts=contacts+sample(tt,yy,t);
 for j=1:4
  [tt,yy]=values(o.get(['terrain_' num2str(j) '_' s '_N']));v=sample(tt,yy,t);
  contacts=contacts+v;terrainForce=terrainForce+v;
 end
end
r=struct('scenario',scenario,'controller',controller,'expected_duration_s',duration,'duration_s',t(end), ...
 'completed',t(end)>=duration-.01,'fell',any(c(:,23)>.5),'max_tilt_error_deg',max(abs(rad2deg(state(:,3)))), ...
 'max_roll_deg',max(abs(rad2deg(roll))),'final_distance_m',state(end,1), ...
 'final_distance_error_m',mean(e(tail,1)),'final_yaw_error_deg',rad2deg(mean(e(tail,5))), ...
 'final_speed_mps',mean(state(tail,2)),'max_speed_mps',max(abs(state(:,2))), ...
 'peak_each_torque_Nm',max(abs(c(:,1:2)),[],'all'),'configured_torque_limit_each_Nm',limit, ...
 'saturated_fraction',mean(any(abs(c(:,1:2))>=limit-1e-8,2)), ...
 'max_integral_distance_ms',max(abs(c(:,15))),'max_integral_yaw_rads',max(abs(c(:,16))), ...
 'max_terrain_normal_N',max(terrainForce),'max_axle_height_m',max(raw(:,6)), ...
 'min_axle_height_m',min(raw(:,6)),'final_world_x_m',raw(end,2),'final_world_y_m',raw(end,4), ...
 'final_total_normal_N',mean(contacts(tail)), ...
 'peak_external_force_N',max(vecnorm(c(:,17:19),2,2)), ...
 'peak_external_torque_Nm',max(vecnorm(c(:,20:22),2,2)));
if isinf(limit),r.configured_torque_limit_each_Nm='unlimited';end
r.pass=r.completed && ~r.fell && r.max_tilt_error_deg<30 && r.max_roll_deg<8;
if ~strcmpi(scenario,'bias')
 r.pass=r.pass && abs(r.final_distance_error_m)<.02 && abs(r.final_yaw_error_deg)<2 && abs(r.final_speed_mps)<.02;
end
if strcmpi(scenario,'terrain'),r.pass=r.pass && r.max_terrain_normal_N>.1 && r.max_axle_height_m>.034;end
if strcmpi(scenario,'bias')&&strcmpi(controller,'LQI'),r.pass=r.pass&&abs(r.final_distance_error_m)<.005&&abs(r.final_yaw_error_deg)<.5;end
if strcmpi(scenario,'push'),r.pass=r.pass&&r.peak_external_force_N>.3&&r.peak_external_torque_Nm>.001;end
if strcmpi(scenario,'turn'),r.pass=r.pass&&max(abs(state(:,5)))>deg2rad(50);end
if strcmpi(scenario,'drive'),r.pass=r.pass&&max(state(:,1))>.25;end
if strcmpi(scenario,'route'),r.pass=r.pass&&max(state(:,1))>.55&&max(state(:,5))>deg2rad(50);end
fid=fopen(fullfile(out,'validation.json'),'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);
tab=array2table([t c contacts terrainForce],'VariableNames',{'time_s','tau_left_Nm','tau_right_Nm', ...
 'distance_m','speed_mps','tilt_error_rad','pitch_rate_radps','yaw_rad','yaw_rate_radps', ...
 'distance_ref_m','speed_ref_mps','tilt_ref_rad','pitch_rate_ref_radps','yaw_ref_rad','yaw_rate_ref_radps', ...
 'integral_distance_ms','integral_yaw_rads','force_x_N','force_y_N','force_z_N', ...
 'torque_x_Nm','torque_y_Nm','torque_z_Nm','fall_signal','total_normal_N','terrain_normal_N'});
writetable(tab,fullfile(out,'trajectory.csv'));
writetable(array2table([tr raw(:,2:14) rad2deg(roll)],'VariableNames', ...
 {'time_s','x_m','vx_mps','y_m','vy_mps','z_m','vz_mps','qw','qx','qy','qz','wx_radps','wy_radps','wz_radps','roll_deg'}), ...
 fullfile(out,'body_pose.csv'));
save(fullfile(out,'simulation.mat'),'o','r','-v7.3');
f=figure('Visible','off','Color','w','Position',[50 50 1100 850]);tiledlayout(3,2);
nexttile;plot(t,state(:,1),t,ref(:,1),'--');ylabel('纵向路程 (m)');grid on;legend('实际','指令');
nexttile;plot(t,rad2deg(state(:,5)),t,rad2deg(ref(:,5)),'--');ylabel('偏航 (°)');grid on;
nexttile;plot(t,rad2deg(state(:,3)));ylabel('俯仰偏差 (°)');grid on;
nexttile;plot(tr,rad2deg(roll));ylabel('横滚 (°)');grid on;
nexttile;plot(t,c(:,1:2));ylabel('单轮转矩 (N·m)');grid on;legend('左','右');
nexttile;plot(t,contacts,t,terrainForce);ylabel('地面支持力 (N)');grid on;legend('全部接触','地形接触');
sgtitle(sprintf('原始3D模型 | %s | %s | pass=%d',scenario,controller,r.pass));
exportgraphics(f,fullfile(out,'results.png'),'Resolution',140);close(f);
f=figure('Visible','off','Color','w');plot(raw(:,2),raw(:,4),'LineWidth',1.5);axis equal;grid on;
xlabel('世界X (m)');ylabel('世界Y (m)');title(sprintf('%s / %s：轮轴中心实际行驶轨迹',scenario,controller));
exportgraphics(f,fullfile(out,'path.png'),'Resolution',140);close(f);
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
