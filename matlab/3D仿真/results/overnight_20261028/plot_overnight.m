p=fileparts(mfilename('fullpath'));a=readtable(fullfile(p,'trend.csv'));b=readtable(fullfile(p,'shadow_trend.csv'));
f=figure('Visible','off','Position',[60 60 1500 1050]);tiledlayout(4,2,'TileSpacing','compact');
nexttile;plot(a.hour,a.yaw_change_deg,a.hour,a.encoder_yaw_change_deg);ylabel('相对航向（度）');legend('姿态估计','编码器推算','Location','northwest');title('航向：估计角稳定，编码器仍显示少量慢转');grid on;
nexttile;plot(a.hour,a.yaw_difference_deg);ylabel('估计减编码器（度）');title('相对航向误差；编码器不是独立真值');grid on;
nexttile;plot(a.hour,a.pitch_deg,a.hour,a.roll_deg);ylabel('角度（度）');legend('俯仰','原遥测横向倾角');title('姿态长期变化；横向常值可能含安装偏置');grid on;
nexttile;plot(a.hour,(a.distance_cm-a.distance_cm(1))*10);ylabel('相对位置（毫米）');title('控制器里程计：不是外部位置测量');grid on;
nexttile;plot(a.hour,a.command_a_A,a.hour,a.command_b_A);ylabel('目标电流（安）');legend('左轮','右轮');title('平衡目标电流，限值仍为每轮4安');grid on;
nexttile;plot(a.hour,a.bus_V);ylabel('母线电压（伏）');title('母线长期维持约8伏；瞬时点不等同电池电量');grid on;
nexttile;plot(a.hour,a.dt_ms);ylabel('控制周期（毫秒）');title('图中为降采样趋势；最大值以全速统计为准');grid on;
nexttile;plot(a.hour,a.yaw_integral);ylabel('转向积分（弧度·秒）');ylim([-.55 .55]);title('转向积分长时间位于上限');grid on;
for k=1:8;ax=nexttile(k);xlabel(ax,'记录相对时间（小时）');end
sgtitle('过夜平衡记录：约6.5小时，未观察到故障或退出平衡');exportgraphics(f,fullfile(p,'overnight_overview.png'),'Resolution',140);savefig(f,fullfile(p,'overnight_overview.fig'));close(f);
f=figure('Visible','off','Position',[100 100 1250 750]);tiledlayout(2,2,'TileSpacing','compact');
nexttile;plot(b.hour,b.gravity_difference_deg);ylabel('方向夹角（度）');title('估计重力与加速度方向：一致性而非绝对精度');grid on;
nexttile;plot(b.hour,b.acc_norm_g);ylabel('加速度模长（重力加速度）');title('静止模长约0.868，绝对尺度需进一步标定');grid on;
nexttile;plot(a.hour,a.calculate_us);ylabel('计算耗时（微秒）');title('上层计算耗时趋势');grid on;
nexttile;plot(b.hour,b.raw_gyro_z);ylabel('原始陀螺仪读数');title('陀螺仪竖直轴原始读数变化');grid on;
for k=1:4;ax=nexttile(k);xlabel(ax,'记录相对时间（小时）');end
sgtitle('过夜传感器及运算指标');exportgraphics(f,fullfile(p,'overnight_sensors.png'),'Resolution',140);savefig(f,fullfile(p,'overnight_sensors.fig'));close(f);
