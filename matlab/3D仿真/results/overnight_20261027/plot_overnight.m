p = fileparts(mfilename('fullpath'));
T = readtable(fullfile(p,'overview.csv'));
Y = readtable(fullfile(p,'segmented_yaw.csv'));
S = jsondecode(fileread(fullfile(p,'summary.json')));
f = figure('Visible','off','Color','w','Position',[100 100 1440 1050]);
set(groot,'defaultAxesFontName','Microsoft YaHei');
tl=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');
x=T.elapsed_min; t0=T.device_s(1); keep=true(size(x));
for e=S.exclusions'
    keep=keep & ~(T.device_s>=e.start_device_s & T.device_s<e.end_device_s);
end
nexttile;plot(x,T.bus_V,'LineWidth',1.2);grid on;ylabel('母线电压 / V');title('供电恢复单独标记；00:39 已确认人工插电');
nexttile;v=T.pitch_deg;v(~keep)=nan;plot(x,v,'LineWidth',1.1);hold on;v=T.roll_imu_deg;v(~keep)=nan;plot(x,v,'LineWidth',1.1);grid on;ylabel('估计角度 / °');legend('控制俯仰','原 IMU Y 角（对应横滚）','Location','northwest');title('排除操作窗口后的姿态趋势');
nexttile;hold on;
for s=S.segments'
    mask=T.device_s>=s.start_device_s & T.device_s<s.end_device_s;
    v=T.distance_cm(mask);plot(x(mask),v-v(1),'Color',[0 .447 .741],'LineWidth',1.2);
end
 grid on;ylabel('相对本段起点 / cm');title('前后里程：各段独立起点，不跨操作窗口累计');
nexttile;hold on;
for k=1:numel(S.segments)
    m=Y.segment==k;
    a=plot(Y.elapsed_min(m),Y.estimated_yaw_deg(m),'Color',[0 .447 .741],'LineWidth',1.2);
    b=plot(Y.elapsed_min(m),Y.encoder_yaw_deg(m),'Color',[.85 .325 .098],'LineWidth',1.2);
end
 grid on;ylabel('相对各段首分钟均值 / °');legend([a b],{'IMU 航向估计','双轮里程计航向'},'Location','southwest');title('每段重新定基准；估计稳定不等于实际无转动');
nexttile;hold on;
for k=1:numel(S.segments)
    m=Y.segment==k;plot(Y.elapsed_min(m),Y.raw_integral_deg(m),'Color',[0 .447 .741],'LineWidth',1.2);
end
 grid on;ylabel('相对各段首分钟均值 / °');title('未扣除此在线零偏的角速度积分（非关补偿实测）');
nexttile;yyaxis left;hold on;
for k=1:numel(S.segments)
    m=Y.segment==k;plot(Y.elapsed_min(m),Y.bias_deg_s(m),'Color',[0 .447 .741],'LineWidth',1.2);
end
 ylabel('在线零偏 / (°/s)');yyaxis right;hold on;
for k=1:numel(S.segments)
    m=Y.segment==k;plot(Y.elapsed_min(m),100*movmean(Y.learning(m),300),'Color',[.85 .5 .2],'LineWidth',1);
end
 ylim([0 100]);ylabel('5 min 窗口学习占比 / %');grid on;title('学习统计只在各段内部计算');
for ax=findall(f,'Type','axes')'
    xlabel(ax,'完整记录开始后 / min');xlim(ax,[0 ceil(x(end))]);
    hold(ax,'on');yl=ylim(ax);
    for e=S.exclusions'
        xx=([e.start_device_s e.end_device_s]-t0)/60;
        patch(ax,[xx(1) xx(2) xx(2) xx(1)],[yl(1) yl(1) yl(2) yl(2)],[.65 .65 .65],'FaceAlpha',.12,'EdgeColor','none','HandleVisibility','off');
    end
    xline(ax,(S.segments(end).start_device_s-t0)/60,'--','Color',[.4 .5 .4],'HandleVisibility','off');
    xline(ax,(S.retained_supply_event.device_s-t0)/60,':','Color',[.85 .5 .2],'HandleVisibility','off');
    ylim(ax,yl);
end
sgtitle(tl,{'夜间记录分段重算：仅剔除 00:39 人工接触前后各 2 分钟','灰带：人工接触；橙色点线：01:32 仅恢复供电，数据保留且不重定基准；绿色虚线：02:14 开始持续偏移'},'FontName','Microsoft YaHei');
exportgraphics(f,fullfile(p,'overnight_overview.png'),'Resolution',140);
savefig(f,fullfile(p,'overnight_overview.fig'));
fprintf('SEGMENTED_OVERNIGHT_FIGURE_OK\n');close(f);