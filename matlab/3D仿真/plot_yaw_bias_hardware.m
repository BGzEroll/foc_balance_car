function plot_yaw_bias_hardware
folder=fullfile(fileparts(mfilename('fullpath')),'results/yaw_bias/hardware');
s=readtable(fullfile(folder,'postturn_static_yaw.csv'));s=s(s.time_us>=s.time_us(end)-240e6,:);t=(s.time_us-s.time_us(1))*1e-6;
fig=figure('Visible','off','Color','w','Position',[80 80 1200 850]);tiledlayout(3,1);
nexttile;plot(t/60,180/pi*[s.raw_yaw-s.raw_yaw(1),s.corrected_yaw-s.corrected_yaw(1),s.encoder_yaw-s.encoder_yaw(1)]);
grid on;ylabel('相对转角 (°)');legend('未扣新增零偏的陀螺积分','修正后四元数 yaw','编码器：仅作验证','Location','northwest');
nexttile;raw=s.raw_yaw-s.encoder_yaw;corrected=s.corrected_yaw-s.encoder_yaw;
plot(t/60,180/pi*[raw-raw(1),corrected-corrected(1)]);grid on;ylabel('相对编码器的角度差 (°)');legend('修正前','修正后','Location','northwest');
nexttile;yyaxis left;plot(t/60,s.bias*180/pi);ylabel('额外陀螺零偏 (°/s)');
yyaxis right;plot(t/60,s.quiet_time-s.quiet_time(1));ylabel('累计可学习时间 (s)');grid on;xlabel('时间 (min)');
sgtitle('实机最近四分钟近静止段：编码器门控零偏学习，无角度融合');
exportgraphics(fig,fullfile(folder,'static_hardware.png'),'Resolution',140);savefig(fig,fullfile(folder,'static_hardware.fig'));close(fig);
s=readtable(fullfile(folder,'turn_45_yaw.csv'));t=(s.time_us-s.time_us(1))*1e-6;
fig=figure('Visible','off','Color','w','Position',[80 80 1200 850]);tiledlayout(3,1);
nexttile;plot(t,180/pi*[s.target-s.target(1),s.corrected_yaw-s.corrected_yaw(1),s.encoder_yaw-s.encoder_yaw(1)]);
grid on;ylabel('相对转角 (°)');legend('目标轨迹','修正后四元数 yaw','编码器：仅作验证','Location','southwest');
nexttile;plot(t,s.bias*180/pi);grid on;ylabel('零偏 (°/s)');ylim(s.bias(1)*180/pi+[-.001 .001]);
nexttile;plot(t,s.quiet_time-s.quiet_time(1));grid on;ylabel('新增可学习时间 (s)');xlabel('时间 (s)');ylim([-.1 .1]);
sgtitle('实机 ±45° 往返：转向期间零偏不变，学习时间增量为零');
exportgraphics(fig,fullfile(folder,'turn_freeze_hardware.png'),'Resolution',140);savefig(fig,fullfile(folder,'turn_freeze_hardware.fig'));close(fig);
end
