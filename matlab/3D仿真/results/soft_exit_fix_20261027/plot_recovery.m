p=fileparts(mfilename('fullpath'));a=readtable(fullfile(p,'recovery_old.csv'));b=readtable(fullfile(p,'recovery_new.csv'));f=figure('Visible','off','Position',[100 100 1100 650]);tiledlayout(2,1);
nexttile;plot(a.time_s,a.gravity_error_deg,b.time_s,b.gravity_error_deg);ylabel('估计重力方向与加速度方向夹角（度）');legend('原重力门控','修复后：降权并限制修正速率');title('实测原始数据回放：从记录中的错误姿态开始（内部状态近似）');grid on;
nexttile;plot(a.time_s,a.roll_deg,b.time_s,b.roll_deg);ylabel('横向倾角估计（度）');xlabel('恢复回放时间（秒）');legend('原重力门控','修复后');title('修正速率限制为每秒5度；本图不是实机闭环结果');grid on;
exportgraphics(f,fullfile(p,'gravity_recovery.png'),'Resolution',140);savefig(f,fullfile(p,'gravity_recovery.fig'));close(f);
