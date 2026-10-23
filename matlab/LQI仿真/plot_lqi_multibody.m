function plot_lqi_multibody
here=fileparts(mfilename('fullpath'));names={'recover','push','bias'};labels={'5°恢复','脉冲扰动','恒定扰动'};
f=figure('Name','原生三维 LQI','Color','w','Position',[100 100 1200 800]);tl=tiledlayout(3,3,'TileSpacing','compact');
for k=1:3
 a=load(fullfile(here,'results',['multibody_' names{k} '.mat']),'o');t=a.o.control3d.Time;c=squeeze(a.o.control3d.Data);if size(c,1)~=numel(t),c=c';end
 nexttile;plot(t,[c(:,3) c(:,9)]);ylabel([labels{k} ' / m']);grid on;if k==1,legend('实际位置','参考位置');end
 nexttile;plot(t,rad2deg(c(:,5)));ylabel('俯仰偏差 / °');grid on
 nexttile;plot(t,c(:,22:23));ylabel('每轮电流 / A');ylim([-2.1 2.1]);grid on;if k==1,legend('左轮','右轮');end
end
title(tl,'原生CAD Multibody接触模型 · LQI · 每轮±2A（电流跟踪仍为理想假设）');xlabel(tl,'时间 / s');
for ax=reshape(findall(f,'Type','axes'),1,[]),ax.Toolbar.Visible='off';end
exportgraphics(f,fullfile(here,'results','lqi_multibody_overview.png'),'Resolution',140);
end
