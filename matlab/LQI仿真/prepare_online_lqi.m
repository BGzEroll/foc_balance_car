function prepare_online_lqi(csvpath,makePlot)
% Measured baseline -> actuator diagnostics -> LQI Q/R candidates -> simulation screen.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
out=fullfile(root,'esp32','lqi_firmware','results','online_lqi');
if nargin<1,csvpath=fullfile(out,'baseline.csv');end
if nargin<2,makePlot=false;end
T=readtable(csvpath);T=T(T.mode==2&T.fault==0,:);
T=T(T.sample_us>=T.sample_us(end)-120e6,:);assert(height(T)>4000);
d=design_lqi;d.current_limit_each=6;d.torque_limit_each=6*d.Kt_nominal;
t=double(T.sample_us)*1e-6;t=t-t(1);Ts=d.Ts;
pitch=T.pitch-T.target_pitch;rate=T.pitch_rate;speed=T.speed;
Ia=-T.iq_a;Ib=T.iq_b;command=[T.command_a T.command_b];
metrics=struct('duration_s',t(end),'samples',height(T),'pitch_rms_deg',rms(pitch)*180/pi,...
 'pitch_ac_rms_deg',std(pitch)*180/pi,'pitch_rate_rms',rms(rate),'speed_rms',rms(speed),...
 'yaw_ac_rms_deg',std(unwrap(T.yaw))*180/pi,'current_rms_A',rms(command(:)),...
 'saturation_fraction',mean(T.saturated),'bus_mean_V',mean(T.bus));
x=pitch-mean(pitch);n=numel(x);w=.5-.5*cos(2*pi*(0:n-1)'/(n-1));
f=(0:floor(n/2))'/(n*Ts);P=abs(fft(x.*w)).^2;P=P(1:numel(f));
band=f>=.3&f<=8;fb=f(band);pb=P(band);[~,ix]=max(pb);
metrics.dominant_pitch_Hz=fb(ix);
tracking=[];
for lag=0:4
 e=[Ia(1+lag:end) Ib(1+lag:end)]-command(1:end-lag,:);
 tracking=[tracking;sqrt(mean(e.^2,1))]; %#ok<AGROW>
end
[~,best]=min(sum(tracking.^2,2));delay=(best-1)*Ts;
metrics.current_tracking_rms_by_delay_A=tracking;
metrics.best_observed_command_delay_s=delay;
% Closed-loop regression is diagnostic only: correlated feedback/no excitation
% cannot uniquely identify the mechanical plant.
h=4;theta=movmean(pitch,5);q=movmean(rate,5);u=movmean(Ia+Ib,5);
Z=[theta(1:end-h),q(1:end-h),u(1:end-h),ones(n-h,1)];
y=(q(1+h:end)-q(1:end-h))/(h*Ts);split=floor(.65*numel(y));
beta=Z(1:split,:)\y(1:split);err=y(split+1:end)-Z(split+1:end,:)*beta;
fit=struct('coefficients',beta,'validation_rmse_rad_s2',rms(err),...
 'validation_explained_fraction',1-var(err)/var(y(split+1:end)),...
 'used_as_design_plant',false,'reason','Closed-loop data, no independent excitation; diagnostic only');
X=[T.distance T.speed pitch T.pitch_rate T.yaw T.yaw_rate T.velocity_integral T.yaw_integral];
baseEff=-X*d.Kaug';baseCmd=[(baseEff(:,1)-baseEff(:,2))/2,(baseEff(:,1)+baseEff(:,2))/2];
pool=struct([]);idx=[1 1 1 1 2 2 1 2];
for r=[1 1.3 1.7]
 for qrate=[1 2 4]
  for qi=[.6 1]
   Q=d.Q;R=d.R;R(1,1)=R(1,1)*r;Q(4,4)=Q(4,4)*qrate;Q(7,7)=Q(7,7)*qi;
   K=dlqr(d.Aa,d.Ba,Q,R);scale=zeros(1,8);
   for col=1:8,scale(col)=K(idx(col),col)/d.Kaug(idx(col),col);end
   if any(scale<.5|scale>1.5),continue;end
   actual=d.Kaug.*scale;
   assert(norm(K-actual,'fro')<1e-7,'Column scales must reproduce MATLAB LQI');
   stable=true;worst=0;
   for kt=[.7 1 1.3]
    for lag=0:2
     [ok,score]=screen(d,actual,kt,lag);stable=stable&&ok;worst=max(worst,score);
    end
   end
   if ~stable,continue;end
   effort=-X*actual';cmd=[(effort(:,1)-effort(:,2))/2,(effort(:,1)+effort(:,2))/2];
   replayRatio=rms(cmd(:))/rms(baseCmd(:));
   entry=struct('scale',scale,'Kaug',actual,'Q',diag(Q)','R',diag(R)',...
    'radius',max(abs(eig(d.Aa-d.Ba*actual))),'screen_score',worst,...
    'replay_current_ratio',replayRatio,'replay_is_counterfactual_closed_loop',false,...
    'rank_score',worst+.1*replayRatio,'nonlinear_pass',false);
   if isempty(pool),pool=entry;else,pool(end+1)=entry;end %#ok<AGROW>
  end
 end
end
assert(~isempty(pool));[~,order]=sort([pool.rank_score]);pool=pool(order);
for k=1:min(5,numel(pool))
 trial=d;trial.Kaug=pool(k).Kaug;trial.K=trial.Kaug(:,1:6);trial.Ki=trial.Kaug(:,7:8);
 reports={};
 for scenario={'recover','push','bias'}
  result=simulate_lqi(trial,scenario{1},struct('duration',12,'current_lag_s',.003,...
    'delay_cycles',min(2,max(1,round(delay/Ts))),'noise_pitch',0,'noise_rate',0));
  reports{end+1}=result.report; %#ok<AGROW>
 end
 pool(k).nonlinear_pass=all(cellfun(@(z)z.pass,reports));
 pool(k).nonlinear_reports=reports;
end
valid=find([pool.nonlinear_pass]);assert(~isempty(valid));selected=pool(valid(1:min(3,numel(valid))));
report=struct('baseline',metrics,'effective_fit_diagnostic',fit,'candidates',selected,...
 'base_Kaug',d.Kaug,'base_revision',1348836412,'current_limit_A',6,'voltage_ratio',.48,...
 'state_order',{{'distance','speed','pitch','pitch_rate','yaw','yaw_rate','position_integral','yaw_integral'}},...
 'limits','Nominal CAD/Kt and lag uncertainty; no validated full inverter/ADC plant. Real A/B decides.');
fid=fopen(fullfile(out,'candidates.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
save(fullfile(out,'matlab_analysis.mat'),'report','pool','T','f','P');
if makePlot
fig=figure('Visible','off');tiledlayout(3,1);nexttile;plot(t,pitch*180/pi);ylabel('pitch (deg)');grid on;
nexttile;plot(f,P);xlim([0 8]);ylabel('pitch spectrum');grid on;
nexttile;plot(t,[command Ia Ib]);ylabel('current (A)');xlabel('time (s)');grid on;
exportgraphics(fig,fullfile(out,'baseline.png'));close(fig);
end
fprintf('BASELINE pitch_rms=%.4fdeg rate_rms=%.4f speed_rms=%.4f dominant=%.3fHz\n',...
 metrics.pitch_rms_deg,metrics.pitch_rate_rms,metrics.speed_rms,metrics.dominant_pitch_Hz);
fprintf('MATLAB_CANDIDATES_READY %d\n',numel(selected));
end
function [ok,cost]=screen(d,K,kt,lag)
% Discrete augmented plant plus per-wheel current lag/delay and integral cap.
x=zeros(8,1);x(3)=deg2rad(5);queue=zeros(2,lag+1);iq=zeros(2,1);cost=0;ok=true;
for k=1:2000
 effort=-K*x;u=[(effort(1)-effort(2))/2;(effort(1)+effort(2))/2];
 u=max(-6,min(6,u));queue=[queue(:,2:end) u];iq=queue(:,1)+(iq-queue(:,1))*exp(-d.Ts/.003);
 effort=[sum(iq);iq(2)-iq(1)];x=d.Aa*x+d.Ba*(kt*effort);
 x(7:8)=max(-d.integral_limit,min(d.integral_limit,x(7:8)));
 cost=cost+d.Ts*((x(3)/.1)^2+.05*(x(4)/1)^2+.05*sum(u.^2)+.2*(x(1)/.08)^2);
 if any(~isfinite(x))||abs(x(3))>pi/4,ok=false;cost=1e9;return;end
end
ok=abs(x(1))<.02&&abs(x(3))<.01;
end
