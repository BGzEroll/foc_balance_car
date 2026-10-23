% Physical-parameter closed-loop check, distinct from controller replay.
addpath('D:/code/Embedded-related/balance_car/matlab/LQR参数计算');
addpath('D:/code/Embedded-related/balance_car/matlab/3D仿真');
d=calculate_lqr; motor=balance_current_parameters; Ts=.005;
profiles=[.5 .2 30.048168 .375602 3.756021 1.3; ...
          .5 .2 30.048168 .375602 1.878011 1.3; ...
          .3 .1 30.048168 .375602 2.4 1.3; ...
          1 .03 30.048168 .375602 .04/motor.Kt_nominal 1.3];
names={'wired_B','D_half','reduced_velocity','simulation_normal'};
report=struct;
for k=1:height(profiles)
 radius=[];
 for delay=0:2
  F=closed_matrix(d.Ad(1:4,1:4),d.Bd(1:4,1)*motor.Kt_nominal,profiles(k,:),Ts,delay);
  radius(end+1)=max(abs(eig(F))); %#ok<SAGROW>
 end
 report.(names{k})=struct('gains',profiles(k,:),'spectral_radius_delay_0_1_2',radius);
 fprintf('%s: spectral radius [delay=0,5,10ms] = %.9f %.9f %.9f\n',names{k},radius);
end
report.note='Exact linearized mechanical plant, ideal current tracking, discrete PID integrators; yaw, nonlinear contact and sensor filters excluded. Stable poles are necessary screening only.';
fid=fopen('D:/code/Embedded-related/balance_car/esp32/tuning_preparation/results/physical_pid_check.json','w');fprintf(fid,'%s',jsonencode(report));fclose(fid);

function F=closed_matrix(Ad,Bd,g,Ts,delay)
 n=6+delay;F=zeros(n);
 for i=1:n
  z=zeros(n,1);z(i)=1;
  eV=-g(6)*z(1)-z(2);
  vi=z(5)+Ts*eV;
  eP=z(3)-g(1)*eV-g(2)*vi;
  pi=z(6)+Ts*eP;
  u=g(3)*eP+g(4)*pi+g(5)*z(4);
  if delay==0,applied=u;else,applied=z(6+delay);end
  next=[Ad*z(1:4)+Bd*applied;vi;pi];
  if delay>0,next=[next;u;z(7:6+delay-1)];end %#ok<AGROW>
  F(:,i)=next;
 end
end
