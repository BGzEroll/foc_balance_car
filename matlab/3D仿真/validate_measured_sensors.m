function result=validate_measured_sensors
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));folder=fullfile(root,'esp32/lqi_firmware/results/identification/linear_08_raw_validate_b_20261026');
s=readtable(fullfile(folder,'samples.csv'));i=readtable(fullfile(folder,'imu.csv'));[~,idx]=unique(s.sample_us);s=s(idx,:);
t=(i.time_us-i.time_us(1))*1e-6;pitch=interp1(s.imu_us,s.pitch,i.time_us,'linear');
n=sqrt(i.acc_x_raw.^2+i.acc_y_raw.^2+i.acc_z_raw.^2);gravity_pitch=-asin(i.acc_y_raw./n);
rest=t>t(end)-1;bias=median(gravity_pitch(rest)-pitch(rest));error=(gravity_pitch-pitch-bias)*180/pi;
result=struct('peak_apparent_tilt_error_deg',max(abs(error)),'rms_apparent_tilt_error_deg',rms(error),'samples',height(i),'duration_s',t(end),'scope','Acceleration-only apparent tilt minus ESKF pitch after removing final-rest offset; not independent attitude truth.');
out=fullfile(here,'results/measured_identification');fid=fopen(fullfile(out,'sensor_validation.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
f=figure('Visible','off','Color','w','Position',[80 80 1100 700]);tiledlayout(3,1);
nexttile;plot(t,[pitch,gravity_pitch-bias]*180/pi);ylabel('前后倾角 (度)');legend('四元数ESKF','仅由加速度求倾角');grid on;
nexttile;plot(t,error);ylabel('二者差值 (度)');grid on;
nexttile;plot(t,n/16384);ylabel('加速度模长 (g)');xlabel('8 cm独立验证时间 (s)');grid on;
sgtitle('实机运动加速度对重力倾角的影响；两种结果均不是外部姿态真值');
exportgraphics(f,fullfile(out,'sensor_validation.png'),'Resolution',140);savefig(f,fullfile(out,'sensor_validation.fig'));close(f);disp(result);
end
