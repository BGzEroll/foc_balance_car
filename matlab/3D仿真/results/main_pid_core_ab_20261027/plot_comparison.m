p=fileparts(mfilename('fullpath'));
S=jsondecode(fileread(fullfile(p,'foc_velocity_ab_comparison.json')));
data=fullfile(p,'..','..','..','..','esp32','balance_car','build','core_audit','plot_data');
unzip(fullfile(p,'foc_velocity_ab_trials.zip'),data);
f=figure('Visible','off','Color','w','Position',[100 100 1400 850]);
set(groot,'defaultAxesFontName','Microsoft YaHei');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
for j=1:2
    if j==1, labels={'lpf_slow','ab_slow'};move=2;name='2 秒行程';else, labels={'lpf_fast','ab_fast'};move=1;name='1 秒行程';end
    ax1=nexttile(j);hold on;ax2=nexttile(j+2);hold on;
    for k=1:2
        report=jsondecode(fileread(fullfile(data,labels{k},'report.json')));
        T=readtable(fullfile(data,labels{k},'samples.csv'));t=(T.sample_us-report.start_us)*1e-6;
        if k==1,color=[0 .447 .741];else,color=[.85 .325 .098];end
        plot(ax1,t,100*T.distance,'Color',color,'LineWidth',1.1);
        plot(ax2,t,T.pitch*180/pi,'Color',color,'LineWidth',1.1);
    end
    tt=linspace(0,4*(move+2)+8,2000);target=zeros(size(tt));
    for n=1:numel(tt)
        if tt(n)>=4*(move+2),continue;end
        phase=floor(tt(n)/(move+2));u=min(1,(tt(n)-phase*(move+2))/move);a=[0 .1 0 -.1];b=[.1 0 -.1 0];target(n)=a(phase+1)+(b(phase+1)-a(phase+1))*u^3*(10-15*u+6*u*u);
    end
    plot(ax1,tt,100*target,'k--','LineWidth',1);
    grid(ax1,'on');grid(ax2,'on');xlabel(ax1,'时间 / s');xlabel(ax2,'时间 / s');ylabel(ax1,'里程计位置 / cm');ylabel(ax2,'估计俯仰 / °');
    title(ax1,[name '：前后各 10 cm']);title(ax2,[name '：加速与制动姿态']);
    legend(ax1,{'FOC 用 3 ms LPF','FOC 用 4 ms αβ','位置参考'},'Location','southeast');legend(ax2,{'FOC 用 3 ms LPF','FOC 用 4 ms αβ'},'Location','southeast');
    xlim(ax1,[0 tt(end)]);xlim(ax2,[0 tt(end)]);
end
sgtitle({'FOC 电角度外推轮速 A/B：相同 PID、6 A 上限及轨迹','每种速度/轮速源各一次，分别上电校准；位置与姿态均为车载估计'},'FontName','Microsoft YaHei');
exportgraphics(f,fullfile(p,'foc_velocity_ab.png'),'Resolution',140);savefig(f,fullfile(p,'foc_velocity_ab.fig'));close(f);
fprintf('FOC_AB_PLOT_OK\n');