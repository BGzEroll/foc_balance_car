function [T,report]=analyze_session(folder,make_plot)
if nargin<2,make_plot=false;end
% Import actual timestamps; do not pretend UDP samples are exactly 5 ms apart.
T=readtable(fullfile(folder,'samples.csv'),'VariableNamingRule','preserve');
assert(height(T)>1,'No telemetry');
assert(all(diff(T.sample_us)>=0),'Device reboot/time reversal: split the session');
sequence_missing=max(T.sequence)-min(T.sequence)+1-numel(unique(T.sequence));
[~,index]=unique(T.cycle,'stable');duplicates=height(T)-numel(index);T=T(index,:);
T.time_s=(T.sample_us-T.sample_us(1))/1e6;
gaps=diff(T.time_s);missing=sum(max(0,diff(T.cycle)-1));
report=struct('unique_cycles',height(T),'duplicates',duplicates,'missing_control_cycles',missing,...
 'missing_received_sequences',sequence_missing,...
 'max_sample_gap_s',max(gaps),'median_sample_period_s',median(gaps),...
 'pitch_rms_deg',rad2deg(rms(T.pitch)),'pitch_std_deg',rad2deg(std(T.pitch)),...
 'saturation_fraction',mean(T.saturated~=0),'fault_samples',sum(T.fault~=0),...
 'active_balance_samples',sum(T.mode==2),'parameter_revisions',unique(T.revision)');
% Current was sampled before this cycle's new command. Align by exact cycle id.
[present,prior]=ismember(T.cycle-1,T.cycle);
valid=present & T.current_age_a_us<1000 & T.current_age_b_us<1000 & bitand(uint32(T.flags),uint32(7))==7;
if any(valid)
 report.current_tracking_rms_A=[rms(T.iq_a(valid)+T.command_a(prior(valid))),...
     rms(T.iq_b(valid)-T.command_b(prior(valid)))];
else,report.current_tracking_rms_A=[NaN,NaN];end
fid=fopen(fullfile(folder,'metrics.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
if make_plot
fig=figure('Visible','off','Position',[100 100 1100 850]);tl=tiledlayout(4,1);
nexttile;plot(T.time_s,rad2deg([T.pitch,T.target_pitch]));ylabel('Pitch / deg');legend('actual','target');grid on
nexttile;plot(T.time_s,[T.speed,T.target_speed]);ylabel('Speed / m/s');grid on
nexttile;plot(T.time_s,[T.command_a,T.command_b,-T.iq_a,T.iq_b]);ylabel('Forward current / A');grid on
nexttile;stairs(T.time_s,[T.saturated,T.revision]);ylabel('Saturation / revision');xlabel('Actual device time / s');grid on
title(tl,'Wi-Fi telemetry: actual timestamps, control-cycle alignment');
exportgraphics(fig,fullfile(folder,'overview.png'));close(fig);
end
disp(report)
end
