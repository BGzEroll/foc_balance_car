function plot_imu_lqi
here=fileparts(mfilename('fullpath'));out=fullfile(here,'results/imu_lqi');
load(fullfile(out,'traces.mat'),'examples','free_reference','report');
methods=report.methods;items=report.cases;colors=lines(numel(methods));
labels={'原融合 0.02','互补 0.5s','互补 1s','角度＋零偏 0.1Hz'};
fig=figure('Visible','off','Position',[100 100 1450 900]);tiledlayout(2,2,'TileSpacing','compact');
nexttile;hold on;for m=1:numel(methods),z=examples{m};plot(z.t,z.x(:,1)*100,'Color',colors(m,:));end
plot(z.t,z.reference*100,'k--');plot(free_reference.t,free_reference.x(:,1)*100,':','Color',[.4 .4 .4]);
grid on;xlabel('时间 (s)');ylabel('真实位移 (cm)');title('标称模型：IMU同步控制下的5cm往返');legend([labels {'指令','原融合＋独立调度'}]);
nexttile;hold on;for m=1:numel(methods),z=examples{m};plot(z.t,(z.measured_pitch-z.x(:,3))*180/pi,'Color',colors(m,:));end
grid on;xlabel('时间 (s)');ylabel('估计－真实俯仰 (°)');title('包含实际加速造成的假倾角');
nexttile;hold on;for m=1:numel(methods),z=examples{m};plot(z.t,z.current,'Color',colors(m,:));end
grid on;xlabel('时间 (s)');ylabel('每轮电流指令 (A)');title('相同LQI增益下的控制输出');
metrics={'true_pitch_rms_deg','each_current_rms_A','position_tracking_rms_m'};values=zeros(4,3);
for m=1:4
 mask=strcmp({items.method},methods{m}) & endsWith(string({items.uncertainty}),'imu_sync');
 for j=1:3,values(m,j)=mean([items(mask).(metrics{j})]);end
end
nexttile;bar(100*(values./values(1,:)-1));yline(0);grid on;xticks(1:4);xticklabels(labels);xtickangle(15);
ylabel('相对原融合的均值变化 (%)');title('同步控制全部9工况：估计更准未必闭环更好');legend('真实俯仰RMS','每轮电流RMS','位置跟踪RMS','Location','northwest');
sgtitle(sprintf('实测噪声＋IMU融合＋LQI闭环仿真；%d/%d组未倾倒；尚未落地验证',sum(~[items.fell]),numel(items)));
exportgraphics(fig,fullfile(out,'comparison_metrics.png'),'Resolution',160);close(fig);
end
