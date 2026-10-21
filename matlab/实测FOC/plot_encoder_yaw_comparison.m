function plot_encoder_yaw_comparison(rows,out)
f=figure('Visible','off','Color','w','Position',[30 30 1500 950]);tl=tiledlayout(3,2,'Padding','compact');
names={'A 始终积分+零偏LP','B 航向保持','C 保持+零速KF'};
for panel=1:6
 nexttile;hold on;
 for m=1:3
  s=load(fullfile(rows(m).result_folder,'simulation_measured.mat'),'o');
  [t,d]=data(s.o.yaw_observer);[tr,raw]=data(s.o.raw3d);[tc,c]=data(s.o.control3d);
  a=balance_control_coordinates(raw);q=a(:,8:11);
  yaw=unwrap(atan2(2*(q(:,2).*q(:,3)+q(:,1).*q(:,4)),1-2*(q(:,3).^2+q(:,4).^2)));
  keep=t>=tr(1)&t<=tr(end);t=t(keep);d=d(keep,:);
  if panel==1
   plot(tr,rad2deg(yaw),'DisplayName',names{m});ylabel('真实偏航 / °');if m==1,plot(tc,rad2deg(c(:,13)),'k--','DisplayName','目标');end
  elseif panel==2
   e=atan2(sin(d(:,1)-interp1(tr,yaw,t)),cos(d(:,1)-interp1(tr,yaw,t)));
   plot(t,rad2deg(e),'DisplayName',names{m});ylabel('估计 - 真实偏航 / °');
  elseif panel==3
   plot(t,rad2deg(d(:,4)),'DisplayName',names{m});ylabel('等效偏航零偏估计 / °/s');
   if m==1,plot(t,.25+.015*t,'k--','DisplayName','额外体Z零偏（非完整等效偏航真值）');end
  elseif panel==4
   stairs(t,d(:,7)+1.2*(m-1),'DisplayName',names{m});ylabel('确认未转向（分层显示）');
  elseif panel==5
   plot(t,rad2deg(d(:,5)),'DisplayName',names{m});ylabel('编码器偏航角速度 / °/s');
  else
   plot(tc,rad2deg(c(:,5)),'DisplayName',names{m});ylabel('控制器俯仰偏差 / °');
  end
 end
 grid on;xlabel('时间 / s');legend('Location','best');
end
title(tl,'A/B/C原生闭环：直行-左转-直行-右转-直行；相同额外零偏漂移压力（非实测）');
exportgraphics(f,fullfile(out,'comparison.png'),'Resolution',150);savefig(f,fullfile(out,'comparison.fig'));close(f);
end
function [t,v]=data(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end;[t,i]=unique(t,'stable');v=v(i,:);
end
