function summary=summarize_measured_car
% Read saved simulations only; never connects to hardware.
here=fileparts(mfilename('fullpath'));out=fullfile(here,'car_validation');if ~isfolder(out),mkdir(out);end
names={'recover_tilt3_sensor1_macro250us','drive_tilt3_sensor1_macro500us','turn_tilt3_sensor1_macro500us'};
summary=struct;summary.cases=cell(1,3);
for k=1:3
 folder=fullfile(here,'results',names{k});summary.cases{k}=jsondecode(fileread(fullfile(folder,'validation.json')));
 copyfile(fullfile(folder,'truth_vs_sensed.png'),fullfile(out,[names{k} '.png']));
end
a=load(fullfile(here,'results','recover_tilt3_sensor1_macro500us','simulation_measured.mat'),'o');
b=load(fullfile(here,'results','recover_tilt3_sensor1_macro250us','simulation_measured.mat'),'o');
[ta,va]=values(a.o.raw3d);[tb,vb]=values(b.o.raw3d);stop=min([2 ta(end) tb(end)]);t=(0:.005:stop)';
aa=balance_control_coordinates(interp1(ta,va,t));bb=balance_control_coordinates(interp1(tb,vb,t));
pa=pitch(aa);pb=pitch(bb);errorPitch=rad2deg(pa-pb);distanceError=aa(:,2)-bb(:,2);
c=struct('comparison_duration_s',stop,'coarse_macro_step_s',.0005,'fine_macro_step_s',.00025, ...
 'pitch_difference_rms_deg',sqrt(mean(errorPitch.^2)),'pitch_difference_max_deg',max(abs(errorPitch)), ...
 'forward_position_difference_rms_m',sqrt(mean(distanceError.^2)),'forward_position_difference_max_m',max(abs(distanceError)));
c.target_pitch_difference_max_deg=.15;c.target_position_difference_max_m=.002;
c.pass=c.pitch_difference_max_deg<c.target_pitch_difference_max_deg&&c.forward_position_difference_max_m<c.target_position_difference_max_m;
summary.coarse_500us_comparison=c;
fine=load(fullfile(here,'results','recover_tilt3_sensor1_macro100us','simulation_measured.mat'),'o');
[tf,vf]=values(fine.o.raw3d);stop=min([2 tb(end) tf(end)]);t2=(0:.005:stop)';
ba=balance_control_coordinates(interp1(tb,vb,t2));fa=balance_control_coordinates(interp1(tf,vf,t2));ep=rad2deg(pitch(ba)-pitch(fa));ed=ba(:,2)-fa(:,2);
d=struct('comparison_duration_s',stop,'coarse_macro_step_s',.00025,'fine_macro_step_s',.0001, ...
 'pitch_difference_rms_deg',sqrt(mean(ep.^2)),'pitch_difference_max_deg',max(abs(ep)), ...
 'forward_position_difference_rms_m',sqrt(mean(ed.^2)),'forward_position_difference_max_m',max(abs(ed)), ...
 'target_pitch_difference_max_deg',.15,'target_position_difference_max_m',.002);
d.pass=d.pitch_difference_max_deg<.15&&d.forward_position_difference_max_m<.002;
summary.default_250us_comparison=d;
summary.pass=all(cellfun(@(r)r.pass,summary.cases))&&d.pass;
summary.hardware_ground_vehicle_validated=false;summary.notes='Default250us recovery; fast500us drive/turn; current-limited averaged actuator; conditional torque constant; hanging-fixture observation noise; 3deg initial recovery envelope.';
f=figure('Visible','off','Color','w');tiledlayout(2,1);nexttile;plot(t,rad2deg(pa),t,rad2deg(pb),'--',t2,rad2deg(pitch(fa)),':');grid on;ylabel('俯仰 / deg');legend('500us宏步长','250us宏步长','100us宏步长');
nexttile;plot(t,distanceError*1000,t2,ed*1000);grid on;ylabel('前向位置差 / mm');legend('500us−250us','250us−100us');xlabel('时间 / s');exportgraphics(f,fullfile(out,'macro_step_convergence.png'),'Resolution',140);savefig(f,fullfile(out,'macro_step_convergence.fig'));close(f);
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);disp(summary);disp(c);disp(d);
fid=fopen(fullfile(out,'README.md'),'w');cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# 实测FOC与观测扰动整车验证\n\n实测base电流PI固定，单侧指令限幅0.7A；以下都是仿真结果。恢复采用默认250us，直行/转弯保留500us快速工况。\n\n');
fprintf(fid,'| 工况 | 时长s | 宏步长us | 真实最大倾斜deg | 末段路程误差mm | 末段偏航误差deg | 通过 |\n|---|---:|---:|---:|---:|---:|---|\n');
for k=1:3
 rr=summary.cases{k};fprintf(fid,'| %s | %g | %g | %.3f | %.3f | %.3f | %d |\n',rr.scenario,rr.duration_s,rr.mechanical_macro_step_s*1e6,rr.max_true_tilt_error_deg,rr.true_final_distance_error_m*1000,rr.true_final_yaw_error_deg,rr.pass);
end
fprintf(fid,'\n前2秒宏步长对比：500→250us最大姿态差%.4fdeg、位置差%.3fmm，未达到0.15deg目标；250→100us为%.4fdeg、%.3fmm，达到目标。未通过的粗步长记录保留在JSON中。\n',c.pitch_difference_max_deg,1000*c.forward_position_difference_max_m,d.pitch_difference_max_deg,1000*d.forward_position_difference_max_m);
fprintf(fid,'\n完整数据在相邻results目录。条件转矩常数、未辨识电感、架空振动向落地环境迁移的限制见上级README；硬件尚未完成落地平衡验证。\n');
end
function p=pitch(a)
q=a(:,8:11);r11=1-2*(q(:,3).^2+q(:,4).^2);r21=2*(q(:,2).*q(:,3)+q(:,1).*q(:,4));r31=2*(q(:,2).*q(:,4)-q(:,1).*q(:,3));p=atan2(-r31,hypot(r11,r21));
end
function [t,v]=values(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end
[t,i]=unique(t,'stable');v=v(i,:);
end
