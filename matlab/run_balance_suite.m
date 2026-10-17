function summary=run_balance_suite
% 全部工况均由同一个3D多体被控对象求解；无桌面自动化。
names={'recover','drive','turn','route','push','terrain','bias'};rows=cell(numel(names),1);
for i=1:numel(names)
 [~,rows{i}]=run_balance_case(names{i},false);
end
summary=vertcat(rows{:});here=fileparts(mfilename('fullpath'));
fid=fopen(fullfile(here,'results','suite','summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
tab=struct2table(summary);writetable(tab,fullfile(here,'results','suite','summary.csv'));
d=design_balance_suite;save(fullfile(here,'results','suite','controller_design.mat'),'d');
fid=fopen(fullfile(here,'results','suite','controller_design.json'),'w');fprintf(fid,'%s',jsonencode(d,PrettyPrint=true));fclose(fid);
assert(all([summary.pass]),'Some 3D scenarios failed; inspect summary and individual reports');
disp(tab(:,{'scenario','controller','pass','final_distance_error_m','final_yaw_error_deg','peak_each_torque_Nm'}));
end
