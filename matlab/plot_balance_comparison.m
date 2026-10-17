function plot_balance_comparison
% 从真实3D工况数据生成LQR/LQI对照，不重新仿真。
here=fileparts(mfilename('fullpath'));out=fullfile(here,'results','suite');
f=figure('Visible','off','Color','w','Position',[70 70 1100 820]);tiledlayout(2,2);
for k=1:2
 kinds={'lqr','lqi'};kind=kinds{k};
 bias=readtable(fullfile(out,['bias_' kind],'trajectory.csv'));
 terrain=readtable(fullfile(out,['terrain_' kind],'body_pose.csv'));
 nexttile(1);hold on;plot(bias.time_s,1000*(bias.distance_m-bias.distance_ref_m),'LineWidth',1.4);
 nexttile(2);hold on;plot(bias.time_s,rad2deg(bias.yaw_rad-bias.yaw_ref_rad),'LineWidth',1.4);
 nexttile(3);hold on;plot(terrain.x_m,1000*terrain.z_m,'LineWidth',1.4);
 nexttile(4);hold on;plot(terrain.x_m,terrain.roll_deg,'LineWidth',1.4);
end
nexttile(1);grid on;xlabel('时间 (s)');ylabel('路程误差 (mm)');title('持续扰动：0.08N水平力');legend('LQR','LQI');
nexttile(2);grid on;xlabel('时间 (s)');ylabel('偏航误差 (°)');title('持续扰动：0.0004N·m偏航转矩');legend('LQR','LQI');
nexttile(3);grid on;xlabel('世界X (m)');ylabel('轮轴离地高度 (mm)');title('真实接触：3mm坡台与单侧0.8mm凸起');legend('LQR','LQI');
nexttile(4);grid on;xlabel('世界X (m)');ylabel('横滚 (°)');title('通过单侧凸起后的侧倾与恢复');legend('LQR','LQI');
sgtitle('同一个原始3D多体被控对象：LQR / LQI对照');
exportgraphics(f,fullfile(out,'comparison.png'),'Resolution',150);
savefig(f,fullfile(out,'comparison.fig'));close(f);
d=design_balance_suite;save(fullfile(out,'controller_design.mat'),'d');
fid=fopen(fullfile(out,'controller_design.json'),'w');fprintf(fid,'%s',jsonencode(d,PrettyPrint=true));fclose(fid);
end
