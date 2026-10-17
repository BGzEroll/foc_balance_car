function report=report_balance_3d(simout,outputDir)
% REPORT_BALANCE_3D 验证空间多体求解结果；不使用平面模型轨迹。
if nargin<2,outputDir=fullfile(fileparts(mfilename('fullpath')),'results','3d');end
if ~isfolder(outputDir),mkdir(outputDir);end
v=simout.states3d; t=v.Time; s=squeeze(v.Data);
if size(s,1)~=numel(t),s=s';end
q=s(:,4:7); w=q(:,1);x=q(:,2);y=q(:,3);z=q(:,4);
pitch=atan2(2*(w.*y+x.*z),1-2*(x.^2+y.^2));
roll=atan2(2*(w.*x+y.*z),1-2*(x.^2+y.^2));
yaw=atan2(2*(w.*z+x.*y),1-2*(y.^2+z.^2));
report=struct('engine','Simscape Multibody R2025b','duration_s',t(end), ...
 'initial_pitch_deg',rad2deg(pitch(1)),'final_pitch_deg',rad2deg(pitch(end)), ...
 'max_abs_pitch_deg',max(abs(rad2deg(pitch))),'max_abs_roll_deg',max(abs(rad2deg(roll))), ...
 'max_abs_yaw_deg',max(abs(rad2deg(yaw))),'final_x_m',s(end,1), ...
 'max_abs_x_m',max(abs(s(:,1))),'min_axle_height_m',min(s(:,3)), ...
 'final_axle_height_m',s(end,3),'max_quaternion_norm_error',max(abs(sum(q.^2,2)-1)), ...
 'max_each_torque_Nm',max(abs(simout.torque3d.Data(:))), ...
 'final_left_normal_N',simout.left_contact_N.Data(end), ...
 'final_right_normal_N',simout.right_contact_N.Data(end));
disp(report);
assert(isempty(simout.ErrorMessage),'Simulation reported an error');
assert(all(isfinite(s),'all'),'Nonfinite state');
assert(report.max_quaternion_norm_error<1e-5,'Quaternion normalization drift');
assert(report.max_abs_pitch_deg<20 && report.max_abs_roll_deg<5,'3D model fell');
assert(abs(report.final_pitch_deg)<.5 && abs(report.final_x_m)<.02,'3D recovery failed');
assert(report.final_left_normal_N>0 && report.final_right_normal_N>0,'Wheel contact lost');
assert(abs(report.final_left_normal_N+report.final_right_normal_N-.4025*9.80665)<.15,'Weight support mismatch');
tbl=table(t,s(:,1),s(:,2),s(:,3),rad2deg(pitch),rad2deg(roll),rad2deg(yaw),s(:,8), ...
 'VariableNames',{'time_s','x_m','vx_mps','axle_z_m','pitch_deg','roll_deg','yaw_deg','wy_radps'});
writetable(tbl,fullfile(outputDir,'body_states.csv'));
fid=fopen(fullfile(outputDir,'validation.json'),'w'); assert(fid>=0);
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
f=figure('Visible','off','Color','w','Position',[80 80 1100 800]);
tiledlayout(3,2);
nexttile;plot(t,rad2deg(pitch));grid on;ylabel('俯仰 (°)');
nexttile;plot(t,s(:,1));grid on;ylabel('前进位置 (m)');
nexttile;plot(t,[rad2deg(roll) rad2deg(yaw)]);grid on;legend('侧倾','偏航');ylabel('角度 (°)');
nexttile;plot(t,s(:,3));grid on;ylabel('轮轴离地高度 (m)');
nexttile;plot(simout.torque3d.Time,squeeze(simout.torque3d.Data));grid on;ylabel('单轮转矩 (N·m)');
nexttile;plot(simout.left_contact_N.Time,squeeze(simout.left_contact_N.Data));hold on;
plot(simout.right_contact_N.Time,squeeze(simout.right_contact_N.Data));grid on;legend('左轮','右轮');ylabel('地面支持力 (N)');
sgtitle('Simscape Multibody：原有3D模型 + 自由基座 + 轮地接触 + 采样LQR');
exportgraphics(f,fullfile(outputDir,'dynamics_3d.png'),'Resolution',140);close(f);
save(fullfile(outputDir,'simulation_3d.mat'),'simout','report','-v7.3');
end
