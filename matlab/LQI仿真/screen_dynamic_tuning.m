function screen_dynamic_tuning
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'esp32','lqi_firmware','results','dynamic');
selected=jsondecode(fileread(fullfile(root,'esp32','lqi_firmware','selected_lqi.json')));
d=hardware_lqi_design;d.current_limit_each=6;base=selected.scale(:)';reports=struct([]);
factors=[1 1 1 1 1;.95 1.05 1.03 1.05 1;1 1 1 .85 1];
for k=1:size(factors,1)
 scale=base;scale([1 2 3 4 7])=base([1 2 3 4 7]).*factors(k,:);K=d.Kaug.*scale;radii=[];
 for kt=[.7 1 1.3]
  for delay=0:2
   for lag=[0 .0048]
    n=16+2*delay;F=zeros(n);
    for j=1:n,e=zeros(n,1);e(j)=1;F(:,j)=step(e,d,K,kt,delay,lag);end
    radii(end+1)=max(abs(eig(F)));
   end
  end
 end
 trial=d;trial.Kaug=K;trial.K=K(:,1:6);trial.Ki=K(:,7:8);cases={};
 for scenario={'bounded_drive'}
  z=simulate_lqi(trial,scenario{1},struct('duration',60,'current_lag_s',.003,'delay_cycles',2,'Kt_scale',.7,'noise_pitch',.001,'noise_rate',.02));
  quiet=abs(z.state(:,3))<deg2rad(.5)&abs(z.state(:,2))<.01;
  last=find(~quiet,1,'last');if isempty(last),settle=0;else,settle=z.t(last);end
  c=z.report;c.position_tracking_rms_m=sqrt(mean((z.state(:,1)-z.reference(:,1)).^2));c.position_peak_m=max(abs(z.state(:,1)));c.speed_tracking_rms=sqrt(mean((z.state(:,2)-z.reference(:,2)).^2));c.settle_to_half_deg_and_1cms_s=settle;cases{end+1}=c;
 end
 item=struct('scale',scale,'Kaug',K,'max_radius',max(radii),'nonlinear_pass',all(cellfun(@(z)z.pass,cases)),'nonlinear_cases',{cases});
 if isempty(reports),reports=item;else,reports(end+1)=item;end
 fprintf('DYNAMIC_CANDIDATE %d radius %.7f nonlinear %d\n',k,item.max_radius,item.nonlinear_pass);
end
assert(all([reports.max_radius]<1)&&all([reports.nonlinear_pass]));assert(all(vertcat(reports.scale)>=.5 & vertcat(reports.scale)<=1.5,'all'));
fid=fopen(fullfile(out,'tuning_screen.json'),'w');fprintf(fid,'%s',jsonencode(struct('base',base,'cases',reports,'scope','Hypothesis screen, not identified physical damping or newly solved optimal K.'),PrettyPrint=true));fclose(fid);
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
