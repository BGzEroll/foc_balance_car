p=fileparts(mfilename('fullpath'));s=jsondecode(fileread(fullfile(p,'summary.json')));a=readtable(fullfile(p,'fixed_trajectory.csv'));b=readtable(fullfile(p,'dynamic_trajectory.csv'));
f=figure('Visible','off','Position',[100 100 1250 850]);tiledlayout(2,2,'TileSpacing','compact');
nexttile;values=[s.fixed.sampling.x0.window_miss_percent s.dynamic.sampling.x0.window_miss_percent;s.fixed.sampling.x1.window_miss_percent s.dynamic.sampling.x1.window_miss_percent];bar(values);xticklabels({'左轮','右轮'});ylabel('窗口丢失率（百分比）');legend('固定触发实测','动态触发实测');title('同一台架轨迹：动态触发减少约两成窗口丢失');grid on;
nexttile;bar([30 30; s.dynamic.sampling.x0.trigger_min/80 s.dynamic.sampling.x0.trigger_max/80]);xticklabels({'固定触发','动态触发'});ylabel('相对周期零点（微秒）');legend('最早','最晚');title('动态触发位置实际随占空比变化');grid on;
nexttile;ta=double(a.sent_us)*1e-6;tb=double(b.sent_us)*1e-6;plot(ta,a.raw_b,tb,b.raw_b);xlabel('启动后时间（秒）');ylabel('右轮角速度（弧度/秒）');legend('固定触发','动态触发');xlim([10 55]);title('相同低电流正反转轨迹；不启动平衡控制');grid on;
nexttile;plot(ta,a.iq_b,tb,b.iq_b);xlabel('启动后时间（秒）');ylabel('右轮实测电流（安）');legend('固定触发','动态触发');xlim([10 55]);title('临时轮速驱动限流为正负0.30安');grid on;
sgtitle('中心调制保持不变的动态窗口采样：实机台架对比');exportgraphics(f,fullfile(p,'dynamic_adc_comparison.png'),'Resolution',140);savefig(f,fullfile(p,'dynamic_adc_comparison.fig'));close(f);
