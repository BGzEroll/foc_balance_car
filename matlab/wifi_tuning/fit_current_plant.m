function model=fit_current_plant(folder,window_s)
% Local empirical plant, input = measured forward sum Iq, NOT target current.
% A time window must be selected after inspection: ground, hands-free, no cable pull.
assert(nargin==2&&numel(window_s)==2,'Select an inspected hands-free time window explicitly');
[T,~]=analyze_session(folder);
valid=T.time_s>=window_s(1)&T.time_s<=window_s(2)&T.mode==2&...
 bitand(uint32(T.flags),uint32(7))==7&T.fault==0&...
 T.imu_age_us<=20000&T.encoder_age_us<=1500&T.current_age_a_us<1000&T.current_age_b_us<1000;
T=T(valid,:);assert(height(T)>=600,'Need at least 600 valid active cycles; idle data cannot identify the plant');
assert(numel(unique(T.revision))==1,'Fit a fixed-parameter segment; split revisions');
assert(max(diff(T.sample_us))<15000,'Large data gap: select a contiguous segment');
t=(T.sample_us-T.sample_us(1))/1e6;
X=[T.pitch-T.pitch_trim_rad,T.pitch_rate,T.speed];u=-T.iq_a+T.iq_b;
assert(std(u)>.03&&range(X(:,1))>.01,'Insufficient excitation; no identifiable model');
Xs=movmean(X,5);us=movmean(u,5);
Y=[gradient(Xs(:,2),t),gradient(Xs(:,3),t)];R=[Xs,us,ones(size(us))];
R=R(4:end-3,:);Y=Y(4:end-3,:);split=floor(.7*size(R,1));
scale=max(std(R(1:split,1:4)),1e-6);Rn=[R(:,1:4)./scale,R(:,5)];
condition=cond(Rn(1:split,:));assert(rank(Rn(1:split,:))==5&&condition<1e5,'Poorly identifiable closed-loop data');
beta=Rn(1:split,:)\Y(1:split,:);BETA=beta;BETA(1:4,:)=beta(1:4,:)./scale';
pred=R(split+1:end,:)*BETA;truth=Y(split+1:end,:);
nrmse=sqrt(mean((pred-truth).^2))./max(std(truth),1e-6);
model=struct('A',[0 1 0;BETA(1:3,1)';BETA(1:3,2)'],...
 'B',[0;BETA(4,:)'],'bias',[0;BETA(5,:)'],'holdout_nrmse',nrmse,...
 'condition',condition,'window_s',window_s,'samples',height(T),...
 'gains',jsondecode(fileread(fullfile(folder,'metadata.json'))),...
 'valid_for_search',all(nrmse<.6),'source','actual measured-current local linear fit');
names={'velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd','position_kp','pitch_trim_rad'};
for k=1:numel(names),model.baseline.(names{k})=T.(names{k})(1);end
assert(all(T.current_limit_A>0)&&range(T.current_limit_A)<1e-6,'Select a fixed current-limit segment');
model.current_limit_A=T.current_limit_A(1);
save(fullfile(folder,'identified_plant.mat'),'model');
fid=fopen(fullfile(folder,'identified_plant.json'),'w');fprintf(fid,'%s',jsonencode(model));fclose(fid);
assert(model.valid_for_search,'Holdout error too large; do not explore gains on this fit');
end
