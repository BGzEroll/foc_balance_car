function plot_dynamic_comparison(folders,labels,outfile)
% Shared axes and identical metric windows for same-calibration A/B trials.
fig=figure('Visible','off','Position',[80 80 1250 850]);tiledlayout(3,1);
colors=lines(numel(folders));
for k=1:numel(folders)
 T=readtable(fullfile(folders{k},'tracking.csv'));m=T.time_s>=0&T.time_s<=65;
 nexttile(1);hold on;plot(T.time_s(m),100*(T.position_m(m)-T.reference_m(m)),'Color',colors(k,:));
 nexttile(2);hold on;plot(T.time_s(m),100*T.speed_m_s(m),'Color',colors(k,:));
 nexttile(3);hold on;plot(T.time_s(m),T.body_pitch_deg(m),'Color',colors(k,:));
end
nexttile(1);grid on;ylabel('位置误差 (cm)');legend(labels,'Location','best');
nexttile(2);grid on;ylabel('估计速度 (cm/s)');
nexttile(3);grid on;ylabel('俯仰 (°)');xlabel('轨迹开始后时间 (s)');
for k=1:3,nexttile(k);xlim([0 65]);xline(48,'--','运动结束','HandleVisibility','off');end
sgtitle('同轮校准 ±5cm 往返：LQI 积分处理对比');
exportgraphics(fig,outfile,'Resolution',140);close(fig);
disp('DYNAMIC_COMPARISON_READY');
end
