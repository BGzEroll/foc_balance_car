function plot_encoder_control
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
base=fullfile(root,'esp32','lqi_firmware','results','encoder_shadow');
folders={'lpf_A1','ab4_B1','lpf_A2','ab4_B2'};labels={'低通 A1','αβ B1','低通 A2','αβ B2'};
colors=[.5 .5 .5;0 .35 .8;.25 .25 .25;0 .6 .4];reports=cell(1,4);
fig=figure('Visible','off','Position',[60 60 1400 950]);tiledlayout(2,2);
nexttile;hold on;
for k=1:4
 T=readtable(fullfile(base,folders{k},'tracking.csv'));m=T.time_s>=0&T.time_s<=65;
 plot(T.time_s(m),T.angle_position_m(m)*100,'Color',colors(k,:),'LineWidth',1.1);
 reports{k}=jsondecode(fileread(fullfile(base,folders{k},'analysis.json')));
end
plot(T.time_s(m),T.reference_m(m)*100,'k--','LineWidth',1.2);grid on;xlim([0 65]);
xlabel('轨迹开始后时间 (s)');ylabel('位移 (cm)');title('统一用原始轮角+俯仰推算位移');legend([labels {'参考'}],'Location','best');
nexttile;
values=cellfun(@(q)[q.pitch_rate_rms_rad_s q.return_pitch_rate_rms_rad_s],reports,'UniformOutput',false);
bar(vertcat(values{:}));grid on;xticks(1:4);xticklabels(labels);ylabel('角速度 RMS (rad/s)');
title('实际车身俯仰角速度');legend('往返期间','回程后17秒','Location','best');
nexttile;
bar(100*cellfun(@(q)q.angle_speed_tracking_rms_20ms_m_s,reports));grid on;xticks(1:4);xticklabels(labels);
ylabel('速度误差 RMS (cm/s)');title('相同原始角度算法、相同20ms差分窗口');
nexttile;
values=cellfun(@(q)[q.actual_iq_each_rms_A q.return_actual_iq_each_rms_A],reports,'UniformOutput',false);
bar(vertcat(values{:}));grid on;xticks(1:4);xticklabels(labels);ylabel('实际每轮 Iq RMS (A)');
title('实际电流：包括齿槽补偿');legend('往返期间','回程后17秒','Location','best');
sgtitle('同轮校准：原3ms低通与4ms αβ速度反馈交替实测，LQI增益不变');
out=fullfile(here,'results','encoder_ab_measured');
exportgraphics(fig,fullfile(out,'control_comparison.png'),'Resolution',130);close(fig);
disp('ENCODER_CONTROL_FIGURE_READY');
end
