function f=calibrate_current_covariance
% Correlated per-motor effective AB noise: train on 0.10/0.50A, hold out 0.30A/mid/q.
p=measured_foc_parameters;r=fileparts(mfilename('fullpath'));file=fullfile(r,'identified_parameters.json');f=jsondecode(fileread(file));
if ~isfield(f,'raw_differenced_sigma_AB_A'),f.raw_differenced_sigma_AB_A=f.noise_sigma_AB_A;end
names={'base_d0p1','base_d0p5'};C=zeros(2,2,2);target=zeros(2,2);
for name=names
 T=readtable(fullfile(p.data,[name{1} '.csv']));J=jsondecode(fileread(fullfile(p.data,[name{1} '.json'])));tt=(T.us-J.meta.profile_start)*1e-6;
 for m=1:2
  I=T.motor==m&tt>.14&tt<.19;angle=mean(T.theta(I));H=[cos(angle)+sin(angle)/sqrt(3) 2*sin(angle)/sqrt(3);-sin(angle)+cos(angle)/sqrt(3) 2*cos(angle)/sqrt(3)];
  Cdq=cov([T.id(I) T.iq(I)]);C(:,:,m)=C(:,:,m)+(H\Cdq)/H'/2;target(:,m)=target(:,m)+diag(Cdq)/2;
 end
end
pred=zeros(2,2);
for iteration=1:2
 pred(:)=0;
 for name=names
  T=readtable(fullfile(p.data,[name{1} '.csv']));J=jsondecode(fileread(fullfile(p.data,[name{1} '.json'])));tt=(T.us-J.meta.profile_start)*1e-6;
  cfg=p.cfg;cfg(10)=.015;cfg(11)=.12;
  for m=1:2,L=chol(C(:,:,m)+eye(2)*1e-14,'lower');cfg(21+4*(m-1):24+4*(m-1))=L(:)';end
  q=zeros(1,2);for m=1:2,I=find(T.motor==m,1);q(m)=J.setup.direction(m)*T.angle(I)-J.setup.zero_offset(m)/7;end
  x=measured_foc_initial(q);ts=(0:p.Ts:.35)';ylog=zeros(numel(ts),4);amp=max(T.id_ref);
  for k=1:numel(ts)
   t=ts(k);d=0;if t>=.05&&t<.1,d=amp/2;elseif t>=.1&&t<.2,d=amp;elseif t>=.2&&t<.3,d=amp/2;end
   [x,y]=measured_foc_step(x,[d d;0 0],q,[0 0],k-1,cfg);ylog(k,:)=[y(4:5,1)' y(4:5,2)'];
  end
  for m=1:2,I=T.motor==m&tt>.14&tt<.19;sim=interp1(ts,ylog(:,(m-1)*2+(1:2)),tt(I));pred(:,m)=pred(:,m)+var(sim)'/2;end
 end
 for m=1:2,C(:,:,m)=C(:,:,m)*sum(target(:,m))/sum(pred(:,m));end
end
f.effective_cov_AB_A2=C;f.noise_sigma_AB_A=sqrt(mean([C(1,1,1) C(2,2,1) C(1,1,2) C(2,2,2)]));
f.noise_calibration=struct('training_cases',{names},'observed_dq_variance_A2',target,'model_dq_variance_before_final_adjustment_A2',pred,'method','correlated per-motor AB disturbance; includes ripple and feedback response, not isolated ADC noise');
fid=fopen(file,'w');fprintf(fid,'%s',jsonencode(f,PrettyPrint=true));fclose(fid);fprintf('CURRENT_NOISE_CALIBRATED covariance_AB=%s\n',mat2str(reshape(C,2,4),4));
end
