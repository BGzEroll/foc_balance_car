p=fileparts(mfilename('fullpath'));r=jsondecode(fileread(fullfile(p,'summary.json')));a=readtable(fullfile(p,'spectrum_3.csv'));b=readtable(fullfile(p,'spectrum_2.csv'));f=figure('Visible','off','Position',[100 100 1200 820]);tiledlayout(2,2);
nexttile;bar([r.runs.x3.accel_std_mg(:),r.runs.x2.accel_std_mg(:)]);xticklabels({'X轴','Y轴','Z轴'});ylabel('加速度标准差（mg，千分之一g）');legend('DLPF＝3','DLPF＝2');title('同一台架静止噪声：启动后20～50秒');grid on;
nexttile;bar([r.runs.x3.gyro_std_deg_s(:),r.runs.x2.gyro_std_deg_s(:)]);xticklabels({'X轴','Y轴','Z轴'});ylabel('角速度标准差（度/秒）');legend('DLPF＝3','DLPF＝2');grid on;
nexttile;semilogy(a.frequency_Hz,a.gyro_x_deg2_s2_Hz,b.frequency_Hz,b.gyro_x_deg2_s2_Hz);xlabel('频率（赫兹）');ylabel('X轴角速度功率谱密度（度²/秒²/赫兹）');legend('DLPF＝3','DLPF＝2');title('高频噪声对比（约200赫兹采样）');grid on;
nexttile;bar([4.9 3.0;4.8 2.8]);xticklabels({'加速度','陀螺仪'});ylabel('标称滤波延迟（毫秒）');legend('DLPF＝3','DLPF＝2');title('芯片手册标称值，非本次实测延迟');grid on;
exportgraphics(f,fullfile(p,'dlpf_comparison.png'),'Resolution',140);savefig(f,fullfile(p,'dlpf_comparison.fig'));close(f);
