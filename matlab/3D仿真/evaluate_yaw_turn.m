function report=evaluate_yaw_turn
folder=fullfile(fileparts(mfilename('fullpath')),'results/yaw_observer/turn_shadow_45');
t=readtable(fullfile(folder,'yaw.csv'));time=(t.time_us-t.time_us(1))*1e-6;
gyro=cumtrapz(time,t.gyro_rate-t.bias(1));encoder=t.encoder_yaw-t.encoder_yaw(1);
X=[ones(size(time)),encoder,time];b=X\gyro;
for pass=1:3
 residual=gyro-X*b;w=min(1,(.5*pi/180)./max(abs(residual),1e-8));
 b=(X.*sqrt(w))\(gyro.*sqrt(w));
end
residual=gyro-X*b;
report=struct('scale_gyro_over_cad_encoder',b(2),'effective_track_m',.1325364/b(2),...
 'residual_bias_deg_s',b(3)*180/pi,'residual_rms_deg',rms(residual)*180/pi,...
 'residual_peak_deg',max(abs(residual))*180/pi,...
 'scope','Short-turn gyro integration versus wheel odometry, with initial static bias removed. No external heading reference; does not prove absence of slip.');
fid=fopen(fullfile(folder,'geometry_fit.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
fig=figure('Visible','off','Position',[80 80 1050 600]);tiledlayout(2,1);
nexttile;plot(time,180/pi*[gyro,encoder,X*b]);grid on;ylabel('转角 (°)');legend('陀螺积分','CAD轮距编码器','拟合编码器');
nexttile;plot(time,residual*180/pi);grid on;ylabel('拟合残差 (°)');xlabel('时间 (s)');
sgtitle('实机 ±45° 转向：编码器比例与陀螺短时积分核对');exportgraphics(fig,fullfile(folder,'geometry_fit.png'),'Resolution',140);close(fig);
disp(report);
end
