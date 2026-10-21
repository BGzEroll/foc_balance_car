function plot_ekf_comparison(rows,out)
f=figure('Visible','off','Color','w','Position',[40 40 1500 850]);tl=tiledlayout(2,3,'Padding','compact');
labels={'欧拉角互补','四元数互补','误差状态EKF'};
for c=1:2
 for panel=1:3
  nexttile;hold on;
  for m=1:3
   s=load(fullfile(rows((c-1)*3+m).result_folder,'simulation_measured.mat'),'o');
   [t,y]=data(s.o.control3d);[tr,raw]=data(s.o.raw3d);a=balance_control_coordinates(raw);q=a(:,8:11);
   r11=1-2*(q(:,3).^2+q(:,4).^2);r21=2*(q(:,2).*q(:,3)+q(:,1).*q(:,4));r31=2*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
   pitch=atan2(-r31,hypot(r11,r21));yaw=unwrap(atan2(r21,r11));
   keep=t>=tr(1)&t<=tr(end);t=t(keep);y=y(keep,:);
   if panel==1
    plot(t,rad2deg(y(:,5)+deg2rad(-.0896225217620993)-interp1(tr,pitch,t)),'DisplayName',labels{m});ylabel('俯仰估计误差 / °');
   elseif panel==2
    e=atan2(sin(y(:,7)-interp1(tr,yaw,t)),cos(y(:,7)-interp1(tr,yaw,t)));
    plot(t,rad2deg(e),'DisplayName',labels{m});ylabel('偏航估计误差 / °');
   else
    plot(tr,rad2deg(pitch-deg2rad(-.0896225217620993)),'DisplayName',labels{m});ylabel('真实俯仰偏差 / °');
   end
  end
  xlabel('时间 / s');grid on;legend('Location','best');title(rows((c-1)*3+1).scenario);
 end
end
title(tl,'姿态估计闭环比较：相同PID / 200Hz IMU / 250µs机械步长 / 单次固定噪声种子');
exportgraphics(f,fullfile(out,'comparison.png'),'Resolution',150);savefig(f,fullfile(out,'comparison.fig'));close(f);
end
function [t,v]=data(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end;[t,i]=unique(t,'stable');v=v(i,:);
end
