function summarize_online_lqi(folder)
% Summarize recorded hardware windows; does not open or command a serial port.
T=readtable(fullfile(folder,'samples.csv'));j=jsondecode(fileread(fullfile(folder,'report.json')));
labels=string({j.history.label})';pitch=[j.history.pitch_rms_deg]';rate=[j.history.pitch_rate_rms]';
displayLabels=replace(labels,["A0_baseline","B1","A1_return","reference_validation"],...
 ["温和参考A0","候选B","温和参考A1","60秒复核"]);
speed=[j.history.speed_rms]';current=[j.history.current_rms_A]';yaw=[j.history.yaw_rms_deg]';
metrics=table(labels,pitch,rate,speed,yaw,current,'VariableNames',...
 {'阶段','俯仰RMS_deg','俯仰角速度RMS_rad_s','速度RMS_m_s','偏航RMS_deg','目标电流RMS_A'});
writetable(metrics,fullfile(folder,'comparison.csv'));
fig=figure('Visible','off','Position',[100 100 1400 950]);tiledlayout(3,2);
nexttile;bar(categorical(displayLabels,displayLabels),pitch);ylabel('俯仰 RMS (°)');grid on;
nexttile;bar(categorical(displayLabels,displayLabels),rate);ylabel('俯仰角速度 RMS (rad/s)');grid on;
nexttile;bar(categorical(displayLabels,displayLabels),speed);ylabel('速度 RMS (m/s)');grid on;
nexttile;bar(categorical(displayLabels,displayLabels),current);ylabel('每轮目标电流 RMS (A)');grid on;
valid=T.mode==2&T.fault==0;t=(double(T.sample_us)-double(T.sample_us(1)))*1e-6;
nexttile;plot(t(valid),(T.pitch(valid)-T.target_pitch(valid))*180/pi);ylabel('俯仰偏差 (°)');xlabel('记录时间 (s)');grid on;
nexttile;plot(t(valid),double(T.revision(valid))-double(T.revision(1)));ylabel('在线参数版本增量');xlabel('记录时间 (s)');grid on;
sgtitle('实机 LQI：温和参考 A、增大角速度权重候选 B 与复核');
exportgraphics(fig,fullfile(folder,'comparison.png'),'Resolution',140);close(fig);
fid=fopen(fullfile(folder,'matlab_summary.json'),'w');
fprintf(fid,'%s',jsonencode(struct('windows',j.history,'selection',j.extra,...
 'scope','Measured natural-balance A/B windows; no standardized physical push tests'),PrettyPrint=true));fclose(fid);
fprintf('HARDWARE_COMPARISON_READY windows=%d\n',height(metrics));
end
