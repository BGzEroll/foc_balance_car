function report=fit_uart200_physics(folder,train_window,test_window)
% CAD-constrained 200Hz identification. Validation never changes source mechanics.
assert(train_window(2)<test_window(1)||test_window(2)<train_window(1),'Training and validation must not overlap');
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'LQR参数计算'));addpath(fullfile(root,'3D仿真'));
[T,~]=analyze_session(folder);d=calculate_lqr;p=d.parameters;motor=balance_current_parameters;
assert(numel(unique(T.revision))==1,'Split fixed-gain segments first');
M=p.body_mass;r=p.wheel_radius;l=hypot(p.com_height,p.com_forward);
D=[M+2*p.wheel_mass+2*p.wheel_inertia/r^2 M*l;M*l p.pitch_inertia+M*l^2];
B=D\[1/r;-1];E=D\[0;1];G=d.A([2 4],3);
% Integrate ten 5ms intervals. No interpolating missing cycles or stale sensors.
R=[];Y=[];times=[];
for k=1:10:height(T)-10
    z=k:k+10;a=T(z,:);tt=a.sample_us/1e6;
    if any(diff(a.cycle)~=1)||any(a.mode~=2)||any(a.fault~=0)||...
       any(bitand(uint32(a.flags),uint32(7))~=7)||any(a.imu_age_us>20000)||...
       any(a.encoder_age_us>1500)||any(a.current_age_a_us>=1000)||any(a.current_age_b_us>=1000),continue;end
    theta=a.pitch-d.pitch_trim;v=a.speed;q=a.pitch_rate;
    u=-a.iq_a+a.iq_b;omega=(-a.raw_a+a.raw_b)/2;
    y=[v(end)-v(1);q(end)-q(1)]-G*trapz(tt,theta);
    R(:,:,end+1)=[B*motor.Kt_nominal*trapz(tt,u),...
        -B*trapz(tt,omega),-B*trapz(tt,tanh(omega/.3)),E*(tt(end)-tt(1))]; %#ok<AGROW>
    Y(:,end+1)=y;times(end+1)=mean(a.time_s); %#ok<AGROW>
end
train=find(times>=train_window(1)&times<=train_window(2));
test=find(times>=test_window(1)&times<=test_window(2));
assert(numel(train)>=100&&numel(test)>=100,'Insufficient independent integral blocks');
scale=max(std(Y(:,train),0,2),[.0005;.005]);ps=[1 .001 .005 .005];X=[];b=[];
for k=train,X=[X;(R(:,:,k)./scale).*ps];b=[b;Y(:,k)./scale];end %#ok<AGROW>
condition=cond(X./max(vecnorm(X),1e-12));
% Enumerate bounded least squares, avoiding an Optimization Toolbox dependency.
lb=[.5 0 0 -4];ub=[1.5 10 4 4];best=Inf;bestz=[];
for state=0:80
    code=dec2base(state,3,4)-'0';fixed=code~=0;z=zeros(4,1);
    z(code==1)=lb(code==1);z(code==2)=ub(code==2);free=~fixed;
    if any(free),z(free)=X(:,free)\(b-X(:,fixed)*z(fixed));end
    if any(z<lb'-1e-9|z>ub'+1e-9),continue;end
    cost=norm(X*z-b)^2;if cost<best,best=cost;bestz=z;end
end
assert(~isempty(bestz),'Fit failed');beta=bestz.*ps';
before=zeros(2,numel(test));after=before;truth=Y(:,test);
for j=1:numel(test),before(:,j)=R(:,:,test(j))*[1;0;0;0];after(:,j)=R(:,:,test(j))*beta;end
den=max(std(truth,0,2),[.0005;.005]);
e0=sqrt(mean((before-truth).^2,2))./den;e1=sqrt(mean((after-truth).^2,2))./den;
accepted=condition<1e4&&all(e1<.8)&&all(e1<=e0)&&mean(e1)<.85*mean(e0);
report=struct('parameters',beta','parameter_names',{{'Kt_scale','viscous_Nm_per_rad_s','Coulomb_Nm','body_bias_Nm'}},...
    'train_window_s',train_window,'test_window_s',test_window,'train_blocks',numel(train),'test_blocks',numel(test),...
    'condition',condition,'baseline_nrmse',e0','corrected_nrmse',e1',...
    'accepted_for_simulation',accepted,'CAD_parameters',p,'Kt_nominal',motor.Kt_nominal,...
    'scope','50ms integrated measured-current plant replay; closed-loop bias and unmeasured external forces remain. Acceptance is local; full closed-loop disturbance replay remains required.');
save(fullfile(folder,'physics_correction_200hz.mat'),'report','truth','before','after');
fid=fopen(fullfile(folder,'physics_correction_200hz.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);disp(report);
end
