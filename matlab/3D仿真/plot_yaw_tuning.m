function plot_yaw_tuning
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));folder=fullfile(root,'esp32/lqi_firmware/results/identification');out=fullfile(here,'results/yaw_speed');if ~isfolder(out),mkdir(out);end
colors=[0 .447 .741;.85 .325 .098];
names={'yaw_speed_base_30_1s','yaw_speed_C_30_1s_repeat_validate'};data=cell(1,2);
for n=1:2
 p=fullfile(folder,names{n});r=jsondecode(fileread(fullfile(p,'report.json')));s=readtable(fullfile(p,'samples.csv'));y=readtable(fullfile(p,'yaw.csv'));y=y(y.phase<4 & y.time_us>=r.start_us,:);t0=y.time_us(1);data{n}=struct('s',s,'y',y,'ts',(s.sample_us-t0)*1e-6,'ty',(y.time_us-t0)*1e-6);
end
f=figure('Visible','off','Color','w','Position',[80 80 1200 950]);tiledlayout(4,1);
nexttile;plot(data{1}.ty,data{1}.y.target*180/pi,'k--');hold on;
for n=1:2,plot(data{n}.ts,data{n}.s.yaw*180/pi,'Color',colors(n,:));end
xlim([0 12]);grid on;ylabel('航向 (度)');legend('目标','原PID 3.1/0.8/0.7','新PID 9/0.6/2');
nexttile;plot(data{1}.ty,data{1}.y.target_rate*180/pi,'k--');hold on;
for n=1:2,plot(data{n}.ts,data{n}.s.yaw_rate*180/pi,'Color',colors(n,:));end
xlim([0 12]);grid on;ylabel('角速度 (度/s)');
nexttile;hold on;for n=1:2,plot(data{n}.ts,data{n}.s.command_b-data{n}.s.command_a,'Color',colors(n,:));end
xlim([0 12]);grid on;ylabel('左右指令电流差 (A)');
nexttile;hold on;for n=1:2,plot(data{n}.ts,data{n}.s.distance*100,'Color',colors(n,:));end
xlim([0 12]);grid on;ylabel('累计前后位移 (cm)');xlabel('测试时间 (s)');
sgtitle('实机30°、1秒转向：原PID与优选参数的独立重复对比；前后PID未改动');
exportgraphics(f,fullfile(out,'pid_comparison.png'),'Resolution',140);savefig(f,fullfile(out,'pid_comparison.fig'));close(f);
end
