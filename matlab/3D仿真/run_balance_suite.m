function summary=run_balance_suite(controller,actuatorGain,profile)
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
% 全部工况均由同一个3D多体被控对象求解；无桌面自动化。
if nargin<1,controller='lqr';end
if nargin<2,actuatorGain=[1 1];end
if nargin<3,profile='normal';end
controller=validatestring(controller,{'lqr','pid'});
names={'recover','drive','turn','route','push','terrain','bias'};rows=cell(numel(names),1);
for i=1:numel(names)
 [~,rows{i}]=run_balance_case(names{i},false,[],controller,actuatorGain,profile);
end
summary=vertcat(rows{:});folder=balance_result_folder(paths.results,controller,actuatorGain,profile);
fid=fopen(fullfile(folder,['summary_' controller '.json']),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
tab=struct2table(summary);writetable(tab,fullfile(folder,['summary_' controller '.csv']));
[~,d]=balance_control_config(controller,1,[],actuatorGain,profile);
save(fullfile(folder,['controller_design_' controller '.mat']),'d');
fid=fopen(fullfile(folder,['controller_design_' controller '.json']),'w');fprintf(fid,'%s',jsonencode(d,PrettyPrint=true));fclose(fid);
assert(all([summary.pass]),'Some 3D scenarios failed; inspect summary and individual reports');
disp(tab(:,{'scenario','controller','pass','final_distance_error_m','final_yaw_error_deg','peak_each_torque_Nm'}));
end

