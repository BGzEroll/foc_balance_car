function result=replay_measured_plant
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));out=fullfile(here,'results/measured_identification');
m=jsondecode(fileread(fullfile(out,'identified_plant.json')));old=m;old.torque_scale=1;old.friction_N=.015;old.viscous_N_s_m=.02;old.pitch_damping_N_m_s=.0001;old.constant_force_N=0;old.pitch_sensor_offset_rad=0;
files=dir(fullfile(root,'esp32/lqi_firmware/results/identification/*validate*/report.json'));horizons=[.025 .05 .1];E=cell(2,3);truth=[];pred=[];
for f=1:numel(files)
 r=jsondecode(fileread(fullfile(files(f).folder,files(f).name)));if ~strcmp(r.kind,'linear'),continue;end
 s=readtable(fullfile(files(f).folder,'samples.csv'));s=s(s.sample_us>=r.start_us & s.sample_us<=r.start_us+16e6,:);
 for a=1:20:height(s)-21
  if max(diff(s.sample_us(a:a+20)))>8000 || mean(abs(s.speed(a:a+20)))<.008,continue;end
  for v=1:2
   if v==1,p=old;else,p=m;end
   x=[s.distance(a);s.speed(a);s.pitch(a)-p.pitch_sensor_offset_rad;s.pitch_rate(a);s.angle_a(a);s.angle_b(a)];hi=1;
   for k=a:a+19
    dt=(s.sample_us(k+1)-s.sample_us(k))*1e-6;u=[s.iq_a(k);s.iq_b(k)];
    for sub=1:5
     h=dt/5;k1=measured_plant_rhs(x,u,p);k2=measured_plant_rhs(x+h*k1/2,u,p);k3=measured_plant_rhs(x+h*k2/2,u,p);k4=measured_plant_rhs(x+h*k3,u,p);x=x+h*(k1+2*k2+2*k3+k4)/6;
    end
    elapsed=(s.sample_us(k+1)-s.sample_us(a))*1e-6;
    if hi<=3 && elapsed>=horizons(hi)-.001
     e=[x(2)-s.speed(k+1);x(4)-s.pitch_rate(k+1)];E{v,hi}=[E{v,hi},e]; %#ok<AGROW>
     if hi==3 && v==2,truth=[truth,[s.speed(k+1);s.pitch_rate(k+1)]];pred=[pred,x([2,4])];end %#ok<AGROW>
     hi=hi+1;
    end
   end
  end
 end
end
result=struct('horizon_s',horizons,'nominal_speed_rms_m_s',cellfun(@(x)rms(x(1,:)),E(1,:)),'identified_speed_rms_m_s',cellfun(@(x)rms(x(1,:)),E(2,:)),'nominal_pitch_rate_rms_rad_s',cellfun(@(x)rms(x(2,:)),E(1,:)),'identified_pitch_rate_rms_rad_s',cellfun(@(x)rms(x(2,:)),E(2,:)),'scope','RK4 open-loop short rollouts on held-out whole runs, initialized from measured states. Actual measured iq input; cogging map included. Not a closed-loop or absolute ground-position validation.');
fid=fopen(fullfile(out,'rollout_validation.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
f=figure('Visible','off','Color','w','Position',[80 80 1100 700]);tiledlayout(2,1);
nexttile;plot([truth(1,:);pred(1,:)]');legend('实测观测值','100 ms模型向前积分');ylabel('速度 (m/s)');grid on;
nexttile;plot([truth(2,:);pred(2,:)]');legend('实测观测值','100 ms模型向前积分');ylabel('俯仰角速度 (rad/s)');xlabel('独立验证窗口');grid on;
sgtitle('实测电流驱动非线性模型：独立验证段短时开环预测，非闭环全程复现');
exportgraphics(f,fullfile(out,'rollout_validation.png'),'Resolution',140);savefig(f,fullfile(out,'rollout_validation.fig'));close(f);disp(result);
end
