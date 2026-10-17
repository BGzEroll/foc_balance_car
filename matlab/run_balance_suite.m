function summary=run_balance_suite
% 全部工况均由同一个3D多体被控对象求解；无桌面自动化。
names={'recover','drive','turn','route','push','terrain','bias'};rows=cell(14,1);i=0;
for kind={'LQR','LQI'}
 for scenario=names
  i=i+1;[~,rows{i}]=run_balance_case(scenario{1},kind{1},false);
 end
end
summary=vertcat(rows{:});here=fileparts(mfilename('fullpath'));
fid=fopen(fullfile(here,'results','suite','summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
tab=struct2table(summary);writetable(tab,fullfile(here,'results','suite','summary.csv'));
assert(all([summary.pass]),'Some 3D scenarios failed; inspect summary and individual reports');
lqr=summary(strcmp({summary.scenario},'bias')&strcmp({summary.controller},'LQR'));
lqi=summary(strcmp({summary.scenario},'bias')&strcmp({summary.controller},'LQI'));
assert(abs(lqi.final_distance_error_m)<.1*abs(lqr.final_distance_error_m),'LQI did not sufficiently reject steady force');
disp(tab(:,{'scenario','controller','pass','final_distance_error_m','final_yaw_error_deg','peak_each_torque_Nm'}));
end
