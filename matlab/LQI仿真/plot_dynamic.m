function plot_dynamic(folder)
T=readtable(fullfile(folder,'tracking.csv'));
fig=figure('Visible','off','Position',[80 80 1250 850]);tiledlayout(3,1);
nexttile;plot(T.time_s,T.reference_m*100,'--',T.time_s,T.position_m*100);grid on;
ylabel('位移 (cm)');legend('轨迹参考','编码器估计','Location','best');xlim([0 75]);
nexttile;plot(T.time_s,T.reference_speed_m_s*100,'--',T.time_s,T.speed_m_s*100);grid on;
ylabel('速度 (cm/s)');legend('参考速度','估计速度','Location','best');xlim([0 75]);
nexttile;
if ismember('body_pitch_deg',T.Properties.VariableNames)
 plot(T.time_s,T.body_pitch_deg,T.time_s,T.reference_pitch_deg,'--');legend('车身俯仰','参考俯仰','Location','best');
else,plot(T.time_s,T.pitch_error_deg);end
grid on;
ylabel('俯仰角 (°)');xlabel('轨迹开始后时间 (s)');xlim([0 75]);
sgtitle(sprintf('±%.0f cm 慢速往返：实机动态跟踪',100*max(abs(T.reference_m))));
exportgraphics(fig,fullfile(folder,'tracking.png'),'Resolution',140);close(fig);
disp('DYNAMIC_FIGURE_READY');
end
