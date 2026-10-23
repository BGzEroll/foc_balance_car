function report=run_lqi_stress
here=fileparts(mfilename('fullpath'));d=design_lqi;r=simulate_lqi(d,'recover',struct('initial_deg',15));
assert(r.report.pass&&any(r.saturated),'Large-angle recovery must exercise saturation and recover');
assert(max(abs(r.current_A),[],'all')<=2+1e-12);
previous=[zeros(1,2);r.integral(1:end-1,:)];assert(max(abs(r.integral(r.saturated,:)-previous(r.saturated,:)),[],'all')<1e-12,'Saturated integrator was not frozen');
report=r.report;save(fullfile(here,'results','large_recovery.mat'),'r','d');
fid=fopen(fullfile(here,'results','large_recovery.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);disp(report);
end
