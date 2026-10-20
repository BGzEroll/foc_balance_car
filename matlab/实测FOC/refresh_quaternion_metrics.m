function summary=refresh_quaternion_metrics
% Recompute metrics only: log streams can have different last sample times.
here=fileparts(mfilename('fullpath'));out=fullfile(here,'quaternion_validation');
file=fullfile(out,'summary.json');summary=jsondecode(fileread(file));
assert(strcmp(summary.status,'complete'),'Wait for physics runs to finish');
for k=1:numel(summary.runs)
 s=load(fullfile(summary.runs(k).result_folder,'simulation_measured.mat'),'o');
 [t,y]=values(s.o.control3d);[tr,raw]=values(s.o.raw3d);
 a=balance_control_coordinates(raw);q=a(:,8:11);
 r11=1-2*(q(:,3).^2+q(:,4).^2);r21=2*(q(:,2).*q(:,3)+q(:,1).*q(:,4));r31=2*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
 pitch=atan2(-r31,hypot(r11,r21));yaw=unwrap(atan2(r21,r11));
 keep=t>=tr(1)&t<=tr(end);t=t(keep);y=y(keep,:);
 pe=y(:,5)+deg2rad(-.0896225217620993)-interp1(tr,pitch,t);
 ye=atan2(sin(y(:,7)-interp1(tr,yaw,t)),cos(y(:,7)-interp1(tr,yaw,t)));
 assert(~isempty(pe)&&all(isfinite([pe;ye])),'Invalid aligned attitude samples');
 summary.runs(k).pitch_estimate_rms_deg=rad2deg(sqrt(mean(pe.^2)));
 summary.runs(k).yaw_estimate_rms_deg=rad2deg(sqrt(mean(ye.^2)));
end
summary.metrics_time_alignment='Only common logged time interval; no extrapolation';
writetable(struct2table(summary.runs),fullfile(out,'comparison.csv'));
fid=fopen(file,'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
disp(struct2table(summary.runs));
end
function [t,v]=values(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end
[t,i]=unique(t,'stable');v=v(i,:);
end
