function summary=run_foc_suite
% 后台验证四种电机工况，生成统一汇总。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
names={'current','reverse','load','voltage'};rows=cell(1,numel(names));
for k=1:numel(names),[~,rows{k}]=run_foc_demo(names{k},false);end
summary=vertcat(rows{:});folder=fullfile(paths.results,'foc');
close_system('zd2808_foc_demo',0);
writetable(struct2table(summary),fullfile(folder,'summary.csv'));
fid=fopen(fullfile(folder,'summary.json'),'w');assert(fid>=0);
fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
disp(struct2table(summary));assert(all([summary.pass]),'FOC验证失败，请查看各工况报告');
end
