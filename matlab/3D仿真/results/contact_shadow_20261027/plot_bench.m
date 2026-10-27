p=fileparts(mfilename('fullpath'));
w=fullfile(p,'..','..','..','..','esp32','balance_car','build','gamepad_analysis','shadow_bench_plot');if ~isfolder(w),mkdir(w);end
unzip(fullfile(p,'bench_capture.zip'),w);T=readtable(fullfile(w,'contact_shadow.csv'));t=(T.imu_us-T.imu_us(1))*1e-6;
set(groot,'defaultAxesFontName','Microsoft YaHei');f=figure('Visible','off','Color','w','Position',[80 80 1400 850]);tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;plot(t,sqrt(T.acc_x_g.^2+T.acc_y_g.^2+T.acc_z_g.^2));hold on;plot(t,T.gravity_norm_g,'--');grid on;xlabel('时间 / s');ylabel('加速度幅值 / g');title('加速度量程 ±8 g：静止噪声检查');legend({'采样幅值','初始化重力幅值'},'Location','best');
nexttile;plot(t,[T.gyro_x_rad_s T.gyro_y_rad_s T.gyro_z_rad_s]*180/pi);grid on;xlabel('时间 / s');ylabel('角速度 / °·s^{-1}');title('陀螺量程 ±1000°/s：零偏校准后');legend({'X','Y','Z'},'Location','best');
nexttile;plot(t,[T.residual_left T.residual_right]);grid on;xlabel('时间 / s');ylabel('轮身加速度差 / m·s^{-2}');title('台架静止：候选状态保持未知，不宣称接地');legend({'左轮','右轮'},'Location','best');
nexttile;plot(t,T.calculate_us);grid on;xlabel('时间 / s');ylabel('影子计算耗时 / μs');title('影子观测在 Core 0 运行，未接入控制决策');
sgtitle({'接触/滑移影子观测与 IMU 新量程台架验证','0.9 m/s、120°/s 映射已编译；本轮平衡未开启，未做地面或滞空试验'});
exportgraphics(f,fullfile(p,'bench_review.png'),'Resolution',140);savefig(f,fullfile(p,'bench_review.fig'));close(f);disp('SHADOW_BENCH_PLOTS_OK');