function plot_rotation_demo(dataDir)
if nargin==0, dataDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','anticogging'); end
r=readtable(fullfile(dataDir,'demo_live.csv')); r=r(4:end,:); t=r.t-r.t(1);
f=figure('Visible','off','Color','w','Position',[80 80 1300 800]); tiledlayout(2,2);
for m=0:1
    nexttile; angle=r.(sprintf('angle%d',m)); angle=(angle-angle(1))*180/pi;
    plot(t,angle,'LineWidth',1.3); hold on; plot(t,.1*t*180/pi,'k:','LineWidth',1.2);
    xlabel('持续运行时间 (s)'); ylabel('累计角位移 (°)'); grid on;
    title(sprintf('M%d 双电机同时运行：每秒实测角度',m)); legend('实测','0.1 rad/s 参考','Location','northwest');
end
for m=0:1
    a=readtable(fullfile(dataDir,sprintf('dynamic_v01_m%d_pos.csv',m))); a=a(a.t>=3,:);
    error=unwrap(a.angle)-.1*a.t; error=(error-mean(error))*180/pi;
    nexttile; plot(a.t,error,'Color',[.1 .45 .7]); grid on;
    xlabel('独立高频验证时间 (s)'); ylabel('角度跟踪误差 (°)');
    title(sprintf('M%d，0.1 rad/s，误差标准差 %.3f°',m,std(error))); ylim([-.25 .25]);
end
exportgraphics(f,fullfile(dataDir,'final_smooth_rotation.png'),'Resolution',160);
end
