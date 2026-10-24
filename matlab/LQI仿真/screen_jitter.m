function screen_jitter
% Single-column LQI feedback experiment; preserves integrators and other gains.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'esp32','lqi_firmware','results','jitter');
selected=jsondecode(fileread(fullfile(root,'esp32','lqi_firmware','selected_lqi.json')));
d=design_lqi;d.current_limit_each=6;d.torque_limit_each=6*d.Kt_nominal;
base=selected.scale(:)';reports=struct([]);
for factor=[1 .9 .82 .75]
 scale=base;scale(4)=scale(4)*factor;K=d.Kaug.*scale;
 radii=[];
 for kt=[.7 1 1.3]
  for delay=0:2
   for gyroLag=[0 .0048]
    n=16+2*delay;F=zeros(n);
    for j=1:n,e=zeros(n,1);e(j)=1;F(:,j)=step(e,d,K,kt,delay,gyroLag);end
    radii(end+1)=max(abs(eig(F))); %#ok<AGROW>
   end
  end
 end
 trial=d;trial.Kaug=K;trial.K=K(:,1:6);trial.Ki=K(:,7:8);cases={};
 for scenario={'recover','push','bias'}
  result=simulate_lqi(trial,scenario{1},struct('duration',12,'current_lag_s',.003,...
   'delay_cycles',2,'Kt_scale',.7));cases{end+1}=result.report; %#ok<AGROW>
 end
 item=struct('rate_factor',factor,'scale',scale,'Kaug',K,...
  'nominal_augmented_radius',max(abs(eig(d.Aa-d.Ba*K))),...
  'sensor_actuator_uncertainty_radii',radii,'max_radius',max(radii),...
  'nonlinear_pass',all(cellfun(@(z)z.pass,cases)),'nonlinear_cases',{cases});
 if isempty(reports),reports=item;else,reports(end+1)=item;end %#ok<AGROW>
 fprintf('RATE_FACTOR %.2f nominal_radius %.6f delayed_radius %.6f nonlinear_pass %d\n',...
  factor,item.nominal_augmented_radius,item.max_radius,item.nonlinear_pass);
end
fid=fopen(fullfile(out,'screen.json'),'w');fprintf(fid,'%s',jsonencode(struct('cases',reports,...
 'scope','Single rate-feedback-column experiment; not a fresh Riccati optimum. Sensor delay model is a hypothesis, not identification.'),PrettyPrint=true));fclose(fid);
assert(all([reports.nonlinear_pass]),'Do not apply candidates failing nonlinear screen');
end
function next=step(z,d,K,kt,delay,gyroLag)
% x6, xi2, measured6, current-common/differential2, command delay states.
Ts=d.Ts;x=z(1:6);xi=z(7:8);m=z(9:14);iq=z(15:16);
tau=[0 .003 gyroLag gyroLag gyroLag gyroLag]';
alpha=zeros(6,1);mask=tau>0;alpha(mask)=exp(-Ts./tau(mask));m=alpha.*m+(1-alpha).*x;
u=-K*[m;xi];xi=xi+Ts*d.C*m;
if delay>0
 q=reshape(z(17:end),2,delay);target=q(:,1);q=[q(:,2:end),u];
else,target=u;q=[];end
iq=target+(iq-target)*exp(-Ts/.003);x=d.Ad*x+d.Bd*(kt*iq);
next=[x;xi;m;iq;q(:)];
end
