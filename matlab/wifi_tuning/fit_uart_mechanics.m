function fit=fit_uart_mechanics(folder)
% Grey-box correction of existing CAD mechanics. No mass/inertia fitting.
% Integrate 0.1s blocks to reduce sensitivity to 40Hz current aliasing.
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'LQR参数计算'));
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'3D仿真'));
sync_uart_session(folder);
load(fullfile(folder,'measured_observations.mat'),'observed');T=observed;
T=T(~T.transition_window & T.imu_age_us<=20000 & T.encoder_age_us<=1500 & ...
 T.current_age_a_us<1000 & T.current_age_b_us<1000,:);
d=calculate_lqr;p=d.parameters;motor=balance_current_parameters;
M=p.body_mass;r=p.wheel_radius;l=hypot(p.com_height,p.com_forward);
D=[M+2*p.wheel_mass+2*p.wheel_inertia/r^2 M*l;M*l p.pitch_inertia+M*l^2];
B=D\[1/r;-1];E=D\[0;1];G=d.A([2 4],3);
t=T.t_us/1e6;edges=[1;find(diff(t)>.04 | diff(t)<=0)+1;height(T)+1];
R=[];Y=[];
for j=1:numel(edges)-1
 index=edges(j):edges(j+1)-1;if numel(index)<11,continue;end
 a=T(index,:);tt=t(index);
 v=movmean(a.speed,3);q=movmean(a.pitch_rate,3);
 theta=movmean(a.pitch-(-.00156420808869),3);
 u=movmean(-a.iq_a+a.iq_b,3);
 omega=movmean((-a.raw_a+a.raw_b)/2,3);
 for k=3:5:height(a)-7
  z=k:k+4;dt=tt(z(end))-tt(z(1));
  if dt<.095 || dt>.105,continue;end
  y=[v(z(end))-v(z(1));q(z(end))-q(z(1))]-G*trapz(tt(z),theta(z));
  H=[B*motor.Kt_nominal*trapz(tt(z),u(z)),...
    -B*trapz(tt(z),omega(z)),-B*trapz(tt(z),tanh(omega(z)/.3)),E*dt];
  R(:,:,end+1)=H;Y(:,end+1)=y; %#ok<AGROW>
 end
end
assert(size(Y,2)>=100,'Not enough usable integral blocks');
n=size(Y,2);split=floor(.7*n);scale=max(std(Y(:,1:split),0,2),[.0005;.005]);
parameter_scale=[1 .001 .005 .005];
X=[];b=[];
for k=1:split,X=[X;(R(:,:,k)./scale).*parameter_scale];b=[b;Y(:,k)./scale];end %#ok<AGROW>
condition=cond(X./max(vecnorm(X),1e-12));
% Small bounded LS problem: enumerate each coefficient free/lower/upper.
% beta=[Kt scale, viscous Nm/(rad/s), Coulomb Nm, body-bias Nm].
lb=[.5 0 0 -4];ub=[1.5 10 4 4];
X=[X;diag([.1 .01 .01 .01])];b=[b;[.1;0;0;0]];
best=Inf;zbest=[];
for state=0:80
 code=dec2base(state,3,4)-'0';fixed=code~=0;z=zeros(4,1);
 z(code==1)=lb(code==1);z(code==2)=ub(code==2);
 free=~fixed;if any(free),z(free)=X(:,free)\(b-X(:,fixed)*z(fixed));end
 if any(z<lb'-1e-9 | z>ub'+1e-9),continue;end
 cost=norm(X*z-b)^2;if cost<best,best=cost;zbest=z;end
end
assert(~isempty(zbest),'Bounded fit failed');beta=zbest.*parameter_scale';
baseline=zeros(2,n-split);prediction=baseline;
for k=split+1:n
 baseline(:,k-split)=R(:,:,k)*[1;0;0;0];prediction(:,k-split)=R(:,:,k)*beta;
end
truth=Y(:,split+1:end);normer=max(std(truth,0,2),[.0005;.005]);
before=sqrt(mean((baseline-truth).^2,2))./normer;
after=sqrt(mean((prediction-truth).^2,2))./normer;
accepted=condition<1e4 && all(after<.8) && all(after<=before+.02) && mean(after)<.85*mean(before);
fit=struct('parameters',beta','parameter_names',{{'Kt_scale','viscous_Nm_per_rad_s','Coulomb_Nm','body_bias_Nm'}},...
 'blocks',n,'condition',condition,'holdout_baseline_nrmse',before','holdout_fitted_nrmse',after',...
 'accepted_for_simulation',accepted,'mechanics',p,'Kt_nominal',motor.Kt_nominal,...
 'limitations','Closed-loop data; 40Hz retained current is not an interval average. Fit rejected unless both held-out dynamics pass. Millisecond delays cannot be identified here.');
save(fullfile(folder,'mechanical_correction.mat'),'fit','truth','baseline','prediction');
fid=fopen(fullfile(folder,'mechanical_correction.json'),'w');fprintf(fid,'%s',jsonencode(fit));fclose(fid);disp(fit);
end
