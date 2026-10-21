function rows=diagnose_encoder_yaw_gate
% Diagnostics only, from saved truth/observer logs; no new physics or tuning.
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));balance_setup;out=fullfile(here,'encoder_yaw_validation');
s=jsondecode(fileread(fullfile(out,'summary.json')));rows=struct([]);
for m=1:numel(s.runs)
 a=load(fullfile(s.runs(m).result_folder,'simulation_measured.mat'),'o');
 [t,d]=data(a.o.yaw_observer);[tr,raw]=data(a.o.raw3d);c=balance_control_coordinates(raw);q=c(:,8:11);
 yaw=unwrap(atan2(2*(q(:,2).*q(:,3)+q(:,1).*q(:,4)),1-2*(q(:,3).^2+q(:,4).^2)));
 k=t>=tr(1)&t<=tr(end);t=t(k);d=d(k,:);trueRate=interp1(tr,gradient(yaw,tr),t);
 straight=t>.5&((t<3.8)|(t>6.3&t<8.8)|(t>11.3));
 nearZero=straight&abs(trueRate)<deg2rad(.5);
 row=struct('mode',s.runs(m).estimator,'straight_sample_count',sum(straight), ...
  'straight_true_rate_median_abs_deg_s',rad2deg(median(abs(trueRate(straight)))), ...
  'straight_encoder_window_median_abs_deg_s',rad2deg(median(abs(d(straight,5)))), ...
  'straight_encoder_window_p95_abs_deg_s',rad2deg(prctile(abs(d(straight,5)),95)), ...
  'straight_fast_rate_p95_abs_deg_s',rad2deg(prctile(abs(d(straight,6)),95)), ...
  'straight_window_within_enter_fraction',mean(abs(d(straight,5))<deg2rad(.5)), ...
  'straight_fast_within_exit_fraction',mean(abs(d(straight,6))<deg2rad(3)), ...
  'straight_confirmed_fraction',mean(d(straight,7)>.5), ...
  'near_zero_true_rate_sample_count',sum(nearZero));
 if isempty(rows),rows=row;else,rows(end+1)=row;end
end
writetable(struct2table(rows),fullfile(out,'gate_diagnostics.csv'));
fid=fopen(fullfile(out,'gate_diagnostics.json'),'w');fprintf(fid,'%s',jsonencode(rows,PrettyPrint=true));fclose(fid);disp(struct2table(rows));
end
function [t,v]=data(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end;[t,i]=unique(t,'stable');v=v(i,:);
end
