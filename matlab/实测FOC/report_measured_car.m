function r=report_measured_car(o,scenario,duration,p,out,terrain,initialTilt,sensorsOn)
% Preserve truthful units when reusing mechanical validation.
c=o.control3d;v=squeeze(c.Data);if size(v,1)~=numel(c.Time),v=v';end
v(:,22:23)=v(:,22:23)*p.Kt;patched=o;patched.control3d=timeseries(v,c.Time);
if isstruct(patched),snapshot=patched;patched.get=@(name)snapshot.(name);end
r=report_balance_case(patched,scenario,duration,p.Imax*p.Kt,out,'PID_FOC_BASE',[1 1],'normal',terrain,[],'x_right_y_forward_v2');
[t,y]=values(o.foc_actuator);ref=y(:,[14 28]);iq=y(:,[5 19]);miq=y(:,[7 21]);valid=t>min(1,duration/3);
r.current_loop='base Kp=0.060318579 Ki=226.194672; no decoupling feedforward';r.ideal_current_tracking=false;r.sensor_noise_enabled=logical(sensorsOn);
r.current_limit_A=p.Imax;r.Vbus_V=p.Vbus;r.voltage_cap_V=min(p.voltage_cap,p.ratio*p.Vbus);r.Kt_conditional_Nm_per_A=p.Kt;
r.requested_recovery_initial_tilt_deg=initialTilt;r.initial_tilt_error_deg=initialTilt*double(strcmpi(scenario,'recover'));r.electrical_mean_output_delay_us=p.output_delay*1e6;
r.actual_iq_tracking_rms_A=sqrt(mean((iq(valid,:)-ref(valid,:)).^2));r.measured_iq_tracking_rms_A=sqrt(mean((miq(valid,:)-ref(valid,:)).^2));
r.voltage_saturation_fraction=mean(y(:,[12 26])>.5);r.peak_actual_each_iq_A=max(abs(iq),[],'all');
r.motor_inertia_owned_by='existing Multibody wheel solids, not duplicated in actuator';r.hardware_ground_vehicle_validated=false;
r.mechanical_macro_step_s=p.mechanical_macro_step;r.electrical_integration_step_s=p.Ts;r.iq_statistics_sample_rate_hz=1/p.mechanical_macro_step;
[tr,raw]=values(o.raw3d);analysis=balance_control_coordinates(raw);Q=analysis(:,8:11);
R11=1-2*(Q(:,3).^2+Q(:,4).^2);R21=2*(Q(:,2).*Q(:,3)+Q(:,1).*Q(:,4));R31=2*(Q(:,2).*Q(:,4)-Q(:,1).*Q(:,3));
pitch=atan2(-R31,hypot(R11,R21));yaw=atan2(R21,R11);
forward=cos(yaw).*analysis(:,3)+sin(yaw).*analysis(:,5);distance=cumtrapz(tr,forward);trim=deg2rad(-.0896225217620993);tail=tr>tr(end)-.5;
refDistance=mean(v(c.Time>c.Time(end)-.5,9));refYaw=mean(v(c.Time>c.Time(end)-.5,13));
r.max_true_tilt_error_deg=max(abs(rad2deg(pitch-trim)));r.true_final_distance_error_m=mean(distance(tail))-refDistance;
r.true_final_yaw_error_deg=rad2deg(mean(atan2(sin(yaw(tail)-refYaw),cos(yaw(tail)-refYaw))));r.true_final_forward_speed_mps=mean(forward(tail));
r.controller_metrics_pass_before_truth=r.pass;r.pass=r.pass&&r.max_true_tilt_error_deg<30&&abs(r.true_final_distance_error_m)<.02&&abs(r.true_final_yaw_error_deg)<2&&abs(r.true_final_forward_speed_mps)<.02;
fid=fopen(fullfile(out,'validation.json'),'w');fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);
writetable(array2table([t y],'VariableNames',[{'time_s','net_tau_L','net_tau_R'} arrayfun(@(k)sprintf('motor_signal_%d',k),1:28,'UniformOutput',false)]),fullfile(out,'foc_actuator.csv'));
f=figure('Visible','off','Color','w','Position',[40 40 1300 850]);tl=tiledlayout(3,2,'Padding','compact');
for m=1:2
 nexttile(m);plot(t,ref(:,m),'k--',t,iq(:,m));grid on;ylabel('Iq / A');legend('指令','真实电气状态');title(sprintf('M%d',m));
 nexttile(m+2);cols=(m-1)*14+[10 11];plot(t,hypot(y(:,cols(1)),y(:,cols(2))));hold on;yline(r.voltage_cap_V,'k--');grid on;ylabel('命令电压矢量 / V');
 nexttile(m+4);plot(t,y(:,m));grid on;ylabel('净关节转矩 / Nm');xlabel('时间 / s');
end
title(tl,sprintf('实测FOC执行器 + 原3D机械模型：%s，传感器扰动=%d',scenario,sensorsOn));exportgraphics(f,fullfile(out,'foc_tracking.png'),'Resolution',140);savefig(f,fullfile(out,'foc_tracking.fig'));close(f);
f=figure('Visible','off','Color','w','Position',[40 40 1150 800]);tl=tiledlayout(3,1,'Padding','compact');
nexttile;plot(tr,rad2deg(pitch-trim),c.Time,rad2deg(v(:,5)),'--');grid on;ylabel('俯仰偏差 / deg');legend('真实机械状态','传感器估计');
nexttile;plot(tr,distance,c.Time,v(:,3),'--',c.Time,v(:,9),':');grid on;ylabel('前向路程 / m');legend('真实机械状态','轮速里程估计','指令');
nexttile;plot(tr,rad2deg(yaw),c.Time,rad2deg(v(:,7)),'--',c.Time,rad2deg(v(:,13)),':');grid on;ylabel('偏航 / deg');xlabel('时间 / s');legend('真实机械状态','传感器估计','指令');
title(tl,sprintf('%s：真实状态与带扰动观测分开检查，pass=%d',scenario,r.pass));exportgraphics(f,fullfile(out,'truth_vs_sensed.png'),'Resolution',140);savefig(f,fullfile(out,'truth_vs_sensed.fig'));close(f);
end
function [t,v]=values(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end
end
