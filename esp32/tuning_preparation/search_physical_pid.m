% Screen gains on the existing mechanical model before further powered trials.
addpath('D:/code/Embedded-related/balance_car/matlab/LQR参数计算');
addpath('D:/code/Embedded-related/balance_car/matlab/3D仿真');
d=calculate_lqr; motor=balance_current_parameters; Ad=d.Ad(1:4,1:4);
Bd=d.Bd(1:4,1)*motor.Kt_nominal; results=[]; profiles={};
for kp=[12 16 20 24]
 for kd=[.8 1 1.2 1.5 1.878]
  for vp=[.3 .5 .8 1]
   for vi=[.01 .03 .05 .1]
    g=[vp vi kp .375602 kd .6];worst=0;ok=true;
    for kt=[.7 1 1.3]
     for delay=1:2
      [F,C]=matrix(Ad,Bd*kt,g,.005,delay);
      if max(abs(eig(F)))>=1,ok=false;break;end
      Q=diag([10 1 10 .005 zeros(1,2+delay)])+.005*(C'*C);
      P=dlyap(F',Q);
      cost=P(1,1)*.01^2+P(2,2)*.05^2+P(3,3)*deg2rad(5)^2;
      worst=max(worst,cost);
     end
     if ~ok,break;end
    end
    if ok,results(end+1,:)=[g worst];profiles{end+1}=g;end %#ok<SAGROW>
   end
  end
 end
end
assert(~isempty(results),'No robust linear candidates');
[~,order]=sort(results(:,7));results=results(order,:);
disp(array2table(results(1:min(8,height(results)),:),'VariableNames',{'velocity_P','velocity_I','pitch_P','pitch_I','pitch_D','position_P','cost'}));
g=results(1,1:6);checks=[];
for kt=[.7 1 1.3]
 for delay=1:2
  [F,C]=matrix(Ad,Bd*kt,g,.005,delay);z=zeros(6+delay,1);z(3)=deg2rad(8);
  history=[];
  for k=1:4000
   u=C*z;actual=max(-4,min(4,u));next=F*z;
   next(7)=actual;z=next;
   history(end+1,:)=[z(3),z(2),u/2]; %#ok<SAGROW>
  end
  checks(end+1,:)=[kt delay max(abs(history(:,1))) max(abs(history(:,2))) max(abs(history(:,3))) sqrt(mean(history(end-399:end,1).^2))]; %#ok<SAGROW>
 end
end
assert(all(checks(:,3)<deg2rad(15)) && all(checks(:,4)<.35),'Candidate failed bounded linear recovery screen');
base=jsondecode(fileread('D:/code/Embedded-related/balance_car/esp32/tuning_preparation/profiles/wifi_D_half_2A.json'));
base.gains.velocity_kp=g(1);base.gains.velocity_ki=g(2);base.gains.pitch_kp=g(3);
base.gains.pitch_ki=g(4);base.gains.pitch_kd=g(5);base.gains.position_kp=g(6);
base.origin='Candidate screened on existing mechanical model, Kt scale .7..1.3 and 5/10ms command delay; linear planar model only. Requires physical validation.';
base.linear_recovery_checks=checks;
fid=fopen('D:/code/Embedded-related/balance_car/esp32/tuning_preparation/profiles/physical_model_candidate_2A.json','w');fprintf(fid,'%s',jsonencode(base));fclose(fid);
writematrix(results,'D:/code/Embedded-related/balance_car/esp32/tuning_preparation/results/physical_pid_candidates.csv');
disp(checks);

function [F,C]=matrix(Ad,Bd,g,Ts,delay)
 n=6+delay;F=zeros(n);C=zeros(1,n);
 for i=1:n
  z=zeros(n,1);z(i)=1;eV=-g(6)*z(1)-z(2);vi=z(5)+Ts*eV;
  eP=z(3)-g(1)*eV-g(2)*vi;pi=z(6)+Ts*eP;
  u=g(3)*eP+g(4)*pi+g(5)*z(4);C(i)=u;
  F(:,i)=[Ad*z(1:4)+Bd*z(6+delay);vi;pi;u;z(7:6+delay-1)];
 end
end
