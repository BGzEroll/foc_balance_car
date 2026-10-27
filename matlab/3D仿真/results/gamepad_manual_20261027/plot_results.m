p=fileparts(mfilename('fullpath'));
work=fullfile(p,'..','..','..','..','esp32','balance_car','build','gamepad_analysis','plot_data');
if ~isfolder(work),mkdir(work);end
unzip(fullfile(p,'measured_data.zip'),work);
T=readtable(fullfile(work,'aligned.csv'));P=readtable(fullfile(work,'gamepad.csv'));
S=jsondecode(fileread(fullfile(p,'summary.json')));
set(groot,'defaultAxesFontName','Microsoft YaHei');
lo=S.pad_first_s;hi=S.fault_time_s;idx=T.time_s>=lo;tt=T.time_s(idx)-lo;
padmask=P.time_s>=lo & P.time_s<=hi & P.fault==0;
f=figure('Visible','off','Color','w','Position',[80 80 1500 1000]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
ax=nexttile;hold on;stairs(P.time_s(padmask)-lo,P.forward(padmask),'k--');plot(tt,T.target_speed(idx),'Color',[.85 .32 .1]);plot(tt,T.speed(idx),'Color',[0 .45 .74]);
ylabel('速度 / m·s^{-1}');title('前后：摇杆参考、经过位置环后的参考、实际速度');legend({'摇杆速度参考','控制器速度参考','里程计速度'},'Location','northwest','Orientation','horizontal');grid on;
ax=nexttile;hold on;plot(tt,T.target_pitch(idx)*180/pi,'Color',[.85 .32 .1]);plot(tt,T.pitch(idx)*180/pi,'Color',[0 .45 .74]);ylabel('俯仰 / °');legend({'目标俯仰','估计俯仰'},'Location','northwest','Orientation','horizontal');grid on;
ax=nexttile;hold on;stairs(P.time_s(padmask)-lo,P.turn(padmask)*180,'k--');plot(tt,T.yaw_rate(idx)*180/pi,'Color',[0 .45 .74]);ylabel('转向角速度 / °·s^{-1}');legend({'摇杆角速度参考','估计角速度'},'Location','northwest','Orientation','horizontal');grid on;
ax=nexttile;hold on;plot(tt,max(abs([T.command_a(idx),T.command_b(idx)]),[],2),'Color',[.85 .32 .1]);plot(tt,max(abs([T.iq_a(idx),T.iq_b(idx)]),[],2),'Color',[0 .45 .74]);yline(6,'r--');ylabel('单轮最大 |Iq| / A');xlabel('距连续摇杆采集开始 / s');legend({'指令','实测','6 A 上限'},'Location','northwest','Orientation','horizontal');grid on;
axs=findall(f,'Type','axes');for k=1:numel(axs),xlim(axs(k),[0 hi-lo]);xline(axs(k),hi-lo-2,':r','末段单独排除','HandleVisibility','off');end
sgtitle({'手动遥控实测：速度参考已取消斜坡','末 2 秒单独标记，避免将碰撞段用于参数判断；速度和姿态来自车载估计'});
exportgraphics(f,fullfile(p,'manual_response.png'),'Resolution',140);savefig(f,fullfile(p,'manual_response.fig'));close(f);

release=[178.816529 181.426497];
f=figure('Visible','off','Color','w','Position',[80 80 1500 900]);tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for j=1:2
 t0=release(j);m=T.time_s>=t0-.4 & T.time_s<=t0+1.7;tr=T.time_s(m)-t0;q=find(T.time_s>=t0,1);origin=T.distance(q);
 ax=nexttile(j);hold on;plot(tr,T.target_speed(m),'Color',[.85 .32 .1]);plot(tr,T.speed(m),'Color',[0 .45 .74]);xline(0,'k--');ylabel('速度 / m·s^{-1}');title(sprintf('第 %d 次回中制动',j));legend({'位置环给出的速度参考','实际速度'},'Location','best');grid on;
 ax=nexttile(j+2);hold on;plot(tr,100*(T.distance(m)-origin),'Color',[0 .45 .74]);xline(0,'k--');yline(0,':k');ylabel('相对松杆位置 / cm');grid on;
 ax=nexttile(j+4);hold on;plot(tr,T.target_pitch(m)*180/pi,'Color',[.85 .32 .1]);plot(tr,T.pitch(m)*180/pi,'Color',[0 .45 .74]);xline(0,'k--');ylabel('俯仰 / °');xlabel('距摇杆回中 / s');legend({'目标','估计'},'Location','best');grid on;
end
sgtitle({'回中后的主要问题：制动后又返回松杆位置','先继续滑行，再反向回拉；需要将制动与静止位置保持分开处理'});
exportgraphics(f,fullfile(p,'release_braking.png'),'Resolution',140);savefig(f,fullfile(p,'release_braking.fig'));close(f);
disp('GAMEPAD_PLOTS_OK');