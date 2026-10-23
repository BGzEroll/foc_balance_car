function reports=run_lqi_robustness
here=fileparts(mfilename('fullpath'));d=design_lqi;stats=analyze_lqi_data;reports={};
for kt=[.7 1 1.3]
 for delay=0:2
  o=struct('Kt_scale',kt,'delay_cycles',delay,'current_lag_s',.003,...
   'noise_pitch',stats.pitch_highpass_std_rad,'noise_rate',stats.rate_highpass_std_rad_s,'duration',18);
  result=simulate_lqi(d,'bias',o);reports{end+1}=result.report; %#ok<AGROW>
  fprintf('LQI robustness Kt=%.1f delay=%d pass=%d peak=%.3fdeg sat=%.3f\n',kt,delay,result.report.pass,result.report.peak_pitch_deg,result.report.saturation_fraction);
 end
end
fid=fopen(fullfile(here,'results','robustness_summary.json'),'w');fprintf(fid,'%s',jsonencode(reports,PrettyPrint=true));fclose(fid);
assert(all(cellfun(@(r)r.pass,reports)),'Retain failed robustness cases; do not claim universal success');
end
