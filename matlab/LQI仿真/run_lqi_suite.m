function results=run_lqi_suite(makePlots)
if nargin<1,makePlots=false;end
here=fileparts(mfilename('fullpath'));addpath(here);out=fullfile(here,'results');if ~isfolder(out),mkdir(out);end
stats=analyze_lqi_data;d=design_lqi;reports={};results=struct;
cases={'recover','push','bias','drive','noise','weak_actuator_delay','timing_fault'};
for k=1:numel(cases)
 name=cases{k};scenario=name;o=struct;
 if strcmp(name,'noise'),o.noise_pitch=stats.pitch_highpass_std_rad;o.noise_rate=stats.rate_highpass_std_rad_s;end
 if strcmp(name,'weak_actuator_delay'),scenario='push';o.Kt_scale=.7;o.delay_cycles=2;o.current_lag_s=.003;end
 if strcmp(name,'timing_fault'),scenario='push';o.fault_injection=true;end
 result=simulate_lqi(d,scenario,o);reports{k}=result.report;results.(name)=result;
 writetable(array2table([result.t,result.state,result.reference,result.current_A,result.integral,double(result.saturated)]),fullfile(out,[name '.csv']));
 fprintf('LQI %s pass=%d fall=%d peak=%.3fdeg tail_position=%.6fm saturation=%.3f\n',name,result.report.pass,result.report.fell,result.report.peak_pitch_deg,result.report.tail_position_error_m,result.report.saturation_fraction);
end
save(fullfile(out,'lqi_suite.mat'),'results','d','stats');
dj=d;dj.poles=[real(d.poles) imag(d.poles)];
fid=fopen(fullfile(out,'lqi_suite_summary.json'),'w');fprintf(fid,'%s',jsonencode(struct('controller','LQI_ONLY','design',dj,'reports',{reports},'constraints',stats),PrettyPrint=true));fclose(fid);
if makePlots,plot_lqi_results(results);end
assert(all(cellfun(@(r)r.pass,reports(1:4))),'A nominal LQI case failed; retain the failure and revise the design');
end
