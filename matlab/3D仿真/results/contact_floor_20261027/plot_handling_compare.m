p=fileparts(mfilename('fullpath'));a=readtable(fullfile(p,'before_release.csv'));b=readtable(fullfile(p,'after_release.csv'));f=figure('Visible','off','Position',[100 100 1100 800]);tiledlayout(3,1);
nexttile;plot(a.time_s,a.speed,b.time_s,b.speed);yline(0,'k:');ylabel('速度（米/秒）');legend('调整前：松杆速度 0.88 米/秒','调整后：松杆速度 0.84 米/秒');title('实机松杆制动对比（两轮独立遥控记录）');grid on;xlim([0 3.5]);
nexttile;plot(a.time_s,a.position_cm,b.time_s,b.position_cm);ylabel('相对松杆位置（厘米）');grid on;xlim([0 3.5]);
nexttile;plot(a.time_s,a.integral,b.time_s,b.integral);yline(.5,'k:');ylabel('速度误差积分（米）');xlabel('摇杆回中后的时间（秒）');grid on;xlim([0 3.5]);
exportgraphics(f,fullfile(p,'handling_before_after.png'),'Resolution',140);savefig(f,fullfile(p,'handling_before_after.fig'));close(f);
