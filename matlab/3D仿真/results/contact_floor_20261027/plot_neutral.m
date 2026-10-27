p=fileparts(mfilename('fullpath'));b=readtable(fullfile(p,'neutral_control.csv'));s=readtable(fullfile(p,'neutral_imu.csv'));
f=figure('Visible','off','Position',[100 100 1100 850]);tiledlayout(4,1);t=b.sample_us/1e6-319.365;ts=s.imu_us/1e6-319.365;
nexttile;plot(t,[b.speed b.target_speed]);ylabel('Speed (m/s)');legend('Measured','Controller target');grid on;title('Both sticks neutral from t=0 to 9.375 s');xlim([-1 9.5]);
nexttile;plot(t,(b.distance-b.distance(find(t>=0,1)))*100);ylabel('Odometry (cm)');grid on;xlim([-1 9.5]);
nexttile;plot(t,b.pitch*180/pi,ts,s.acc_pitch_rad*180/pi);ylabel('Pitch (deg)');legend('EKF','Accelerometer apparent tilt');grid on;xlim([-1 9.5]);
nexttile;plot(t,b.velocity_integral);yline(.5,'r--');ylabel('Velocity integral');xlabel('Time after stick release (s)');grid on;xlim([-1 9.5]);
exportgraphics(f,fullfile(p,'neutral_review.png'),'Resolution',140);savefig(f,fullfile(p,'neutral_review.fig'));close(f);

