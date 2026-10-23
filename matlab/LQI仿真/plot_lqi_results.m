function plot_lqi_results(results)
here=fileparts(mfilename('fullpath'));if nargin<1,a=load(fullfile(here,'results','lqi_suite.mat'));results=a.results;end
names={'recover','push','bias','drive'};
f=figure('Name','LQI 仿真：2A限流、200Hz','Color','w','Position',[80 80 1200 850]);tl=tiledlayout(4,3,'TileSpacing','compact');
for k=1:4
 r=results.(names{k});nexttile;plot(r.t,[r.state(:,1),r.reference(:,1)]);ylabel([names{k} ' / m']);grid on;if k==1,legend('实际','参考');end
 nexttile;plot(r.t,rad2deg(r.state(:,3)));ylabel('俯仰 / °');grid on
 nexttile;plot(r.t,r.current_A);ylabel('每轮电流 / A');ylim([-2.1 2.1]);grid on
end
xlabel(tl,'时间 / s');title(tl,'LQI；CAD参数与不确定性试验，不代表实机已验证');
for ax=reshape(findall(f,'Type','axes'),1,[]),ax.Toolbar.Visible='off';end
exportgraphics(f,fullfile(here,'results','lqi_overview.png'),'Resolution',140);
end
