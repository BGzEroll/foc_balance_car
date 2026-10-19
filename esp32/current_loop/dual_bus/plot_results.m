function plot_results
% Saved hardware data only. Does not open a serial port or run motors.
root=fileparts(mfilename('fullpath'));
set(groot,'defaultAxesFontName','Microsoft YaHei');
labels={'MA 裸转子','MB 带轮组'};
files=dir(fullfile(root,'dual_speed*_analysis.json'));
for k=1:numel(files)
 name=erase(files(k).name,'_analysis.json');J=jsondecode(fileread(fullfile(root,[name '.json'])));
 T=readtable(fullfile(root,[name '.csv']));M=J.meta;t=(T.us-M.profile_start)*1e-6;
 f=figure('Visible','off','Color','w','Position',[40 40 1450 1050]);tl=tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
 for motor=1:2
  I=T.motor==motor;tt=t(I);S=T(I,:);
  nexttile(motor);plot(tt,S.speed,'Color',[.35 .55 .85]);hold on;plot(tt,S.speed_ref,'k--');grid on;
  ylabel('速度 / rad/s');title(labels{motor});legend('编码器原有3ms低通','目标');
  nexttile(motor+2);plot(tt,S.iq,tt,S.iq_ref,'--',tt,S.id);grid on;ylabel('电流 / A');legend('Iq','Iq目标','Id');
  nexttile(motor+4);plot(tt,hypot(S.vd,S.vq),tt,S.vlimit,'--');grid on;ylabel('电压矢量 / V');legend('实际命令','动态限幅');
 end
 title(tl,strrep(name,'_','\_'));for ax=findall(f,'Type','axes')',xlabel(ax,'运行时间 / s');end
 exportgraphics(f,fullfile(root,[name '.png']),'Resolution',140);savefig(f,fullfile(root,[name '.fig']));close(f);
end
end
