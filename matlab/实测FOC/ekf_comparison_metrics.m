function row=ekf_comparison_metrics(o,r,scenario,mode,reused)
[t,y]=data(o.control3d);[tr,raw]=data(o.raw3d);a=balance_control_coordinates(raw);q=a(:,8:11);
r11=1-2*(q(:,3).^2+q(:,4).^2);r21=2*(q(:,2).*q(:,3)+q(:,1).*q(:,4));r31=2*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
pitch=atan2(-r31,hypot(r11,r21));yaw=unwrap(atan2(r21,r11));
keep=t>=tr(1)&t<=tr(end);t=t(keep);y=y(keep,:);
pe=y(:,5)+deg2rad(-.0896225217620993)-interp1(tr,pitch,t);
ye=atan2(sin(y(:,7)-interp1(tr,yaw,t)),cos(y(:,7)-interp1(tr,yaw,t)));
assert(~isempty(pe)&&all(isfinite([pe;ye])),'Invalid common-time metrics');
row=struct('scenario',scenario,'estimator',mode,'reference_reused',reused,'completed',r.completed,'fell',r.fell,'scenario_pass',r.pass, ...
 'pitch_estimate_rms_deg',rad2deg(sqrt(mean(pe.^2))), 'yaw_estimate_rms_deg',rad2deg(sqrt(mean(ye.^2))), ...
 'true_peak_tilt_deg',r.max_true_tilt_error_deg,'true_final_yaw_error_deg',r.true_final_yaw_error_deg, ...
 'true_final_distance_error_m',r.true_final_distance_error_m,'peak_iq_A',r.peak_actual_each_iq_A,'result_folder',r.result_folder);
end
function [t,v]=data(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end;[t,i]=unique(t,'stable');v=v(i,:);
end
