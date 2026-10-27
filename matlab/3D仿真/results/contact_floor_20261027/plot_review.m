p=fileparts(mfilename('fullpath')); b=readtable(fullfile(p,'control_plot.csv')); s=readtable(fullfile(p,'imu_plot.csv')); tf=411.750016; t0=257.499624;
f=figure('Visible','off','Position',[100 100 1250 850]); tiledlayout(3,2);
nexttile;plot((b.sent_us/1e6-t0),b.speed);ylabel('Wheel-derived speed (m/s)');grid on;title('Floor run: full active interval');
nexttile;plot(s.imu_us/1e6-t0,s.force_ratio);ylabel('Specific force / rest');grid on;
nexttile;plot(b.sent_us/1e6-t0,[b.command_a b.command_b]);ylabel('Current request (A)');grid on;
nexttile;plot(b.sent_us/1e6-t0,[b.iq_a b.iq_b]);ylabel('Measured Iq (A)');grid on;
nexttile;plot(b.sent_us/1e6-tf,b.fault);xlim([-.3 1]);ylabel('Fault code');xlabel('Time relative to fault (s)');grid on;title('Fault 7 precedes fall');
nexttile;plot(s.imu_us/1e6-tf,s.gyro_x_rad_s*180/pi);xlim([-.3 1]);xline(0,'r--');ylabel('Raw gyro X (deg/s)');xlabel('Time relative to fault (s)');grid on;
exportgraphics(f,fullfile(p,'floor_review.png'),'Resolution',140);savefig(f,fullfile(p,'floor_review.fig'));close(f);
