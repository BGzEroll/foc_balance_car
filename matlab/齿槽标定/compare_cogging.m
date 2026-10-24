function compare_cogging(dataDir)
if nargin==0, dataDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','anticogging'); end
f=figure('Visible','off','Color','w','Name','抗齿槽补偿实机对比','Position',[80 80 1400 850]);
tiledlayout(2,2); rows=struct([]); k=0;
for m=0:1
    for s=[1 -1]
        k=k+1; tag='pos'; if s<0, tag='neg'; end
        a=readtable(fullfile(dataDir,sprintf('baseline_m%d_%s.csv',m,tag)));
        b=readtable(fullfile(dataDir,sprintf('verify_m%d_%s.csv',m,tag)));
        a=a(a.t>=3,:); b=b(b.t>=3,:); ref=s*0.5;
        nexttile; plot(a.t,a.speed); hold on; plot(b.t,b.speed); yline(ref,'k:'); grid on;
        title(sprintf('M%d，目标速度 %+.1f rad/s',m,ref)); xlabel('时间 (s)'); ylabel('速度 (rad/s)');
        legend('未补偿','补偿开启','目标','Location','best');
        rows(k).motor=m; rows(k).reference_rad_s=ref;
        rows(k).baseline_speed_mean=mean(a.speed); rows(k).compensated_speed_mean=mean(b.speed);
        rows(k).baseline_speed_std=std(a.speed); rows(k).compensated_speed_std=std(b.speed);
        rows(k).baseline_speed_rmse=sqrt(mean((a.speed-ref).^2));
        rows(k).compensated_speed_rmse=sqrt(mean((b.speed-ref).^2));
        rows(k).ripple_reduction_percent=100*(1-std(b.speed)/std(a.speed));
        rows(k).baseline_stall_fraction=mean(abs(a.speed)<0.05);
        rows(k).compensated_stall_fraction=mean(abs(b.speed)<0.05);
        rows(k).baseline_feedback_rms_A=rms(a.target);
        rows(k).compensated_feedback_rms_A=rms(b.target);
    end
end
writetable(struct2table(rows),fullfile(dataDir,'comparison_metrics.csv'));
fid=fopen(fullfile(dataDir,'comparison_metrics.json'),'w'); fprintf(fid,'%s',jsonencode(rows,PrettyPrint=true)); fclose(fid);
exportgraphics(f,fullfile(dataDir,'cogging_comparison.png'),'Resolution',160);
save(fullfile(dataDir,'cogging_comparison.mat'),'rows'); disp(struct2table(rows));
end


