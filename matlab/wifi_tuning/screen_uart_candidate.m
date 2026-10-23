function report=screen_uart_candidate(folder)
% Cross-check a measured-data candidate against the existing physical model.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'LQR参数计算'));addpath(fullfile(root,'3D仿真'));
d=calculate_lqr;motor=balance_current_parameters;
data=load(fullfile(folder,'identified_plant.mat'),'model');base=data.model.baseline;
assert(data.model.valid_for_search,'Measured model rejected');
proposal=jsondecode(fileread(fullfile(folder,'candidate_review_only.json')));
profiles={base,base,proposal.gains};profiles{2}.pitch_kd=1.8;
labels={'baseline','D_only_1p8','measured_model_proposal'};rows=[];
for j=1:numel(profiles)
 g=profiles{j};v=[g.velocity_kp g.velocity_ki g.pitch_kp g.pitch_ki g.pitch_kd g.position_kp];
 for kt=[.7 1 1.3]
  for delay=1:2
   [F,C]=closed_matrix(d.Ad(1:4,1:4),d.Bd(1:4,1)*motor.Kt_nominal*kt,v,.005,delay);
   rho=max(abs(eig(F)));Q=diag([10 1 10 .005 zeros(1,2+delay)])+.005*(C'*C);
   cost=Inf;if rho<1,P=dlyap(F',Q);cost=P(1,1)*.01^2+P(2,2)*.05^2+P(3,3)*deg2rad(5)^2;end
   rows(end+1,:)=[j kt delay rho cost]; %#ok<AGROW>
  end
 end
end
report=struct('labels',{labels},'columns',{{'profile','Kt_scale','delay_cycles','spectral_radius','cost'}},...
 'rows',rows,'scope','Linear planar physical-model screen; not a guarantee of real-world stability.');
fid=fopen(fullfile(folder,'physical_candidate_screen.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
disp(array2table(rows,'VariableNames',report.columns));
end

function [F,C]=closed_matrix(Ad,Bd,g,Ts,delay)
n=6+delay;F=zeros(n);C=zeros(1,n);
for i=1:n
 z=zeros(n,1);z(i)=1;eV=-g(6)*z(1)-z(2);vi=z(5)+Ts*eV;
 eP=z(3)-g(1)*eV-g(2)*vi;pi=z(6)+Ts*eP;
 u=g(3)*eP+g(4)*pi+g(5)*z(4);C(i)=u;
 F(:,i)=[Ad*z(1:4)+Bd*z(6+delay);vi;pi;u;z(7:6+delay-1)];
end
end
