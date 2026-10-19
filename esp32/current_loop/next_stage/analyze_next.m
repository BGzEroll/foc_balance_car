function S=analyze_next(name)
% Offline only: measured current/encoder data; no serial or motor commands.
root=fileparts(mfilename('fullpath'));T=readtable(fullfile(root,[name '.csv']));
J=jsondecode(fileread(fullfile(root,[name '.json'])));M=J.meta;
t=(T.us-M.profile_start)*1e-6;v=-T.speed;pos=-T.left;
fns=fieldnames(J.lut_mv);la=J.lut_mv.(fns{1});lb=J.lut_mv.(fns{2});
ia=(la(T.raw_a+1)*.001-M.offset_V(1))*M.gain_A_per_V;
ib=(lb(T.raw_b+1)*.001-M.offset_V(2))*M.gain_A_per_V;ic=-ia-ib;
ticks_per_us=M.period_ticks/100;guard=M.guard_us*ticks_per_us;
margin=min([T.a0-T.ca-guard,M.period_ticks-T.ca-T.a1-guard,...
 T.b0-T.cb-guard,M.period_ticks-T.cb-T.b1-guard],[],2)/ticks_per_us;
active=bitand(uint32(T.flags),1)>0;
S=struct('name',name,'mode',M.mode,'reason',M.reason,'rows',height(T),...
 'valid_fraction',M.full_rate_valid/max(1,M.full_rate_active),...
 'pi_updates',M.pi_updates,'max_update_gap_us',M.max_sample_gap_us,...
 'missed_commit',M.missed_commit,'min_window_margin_us',min(margin(active)),...
 'max_abs_phase_current_A',max(abs([ia;ib;ic])),...
 'max_abs_speed_rad_s',max(abs(v)),...
 'iq_error_rmse_A',sqrt(mean((T.iq(active)-T.iq_ref(active)).^2)),...
 'id_error_rmse_A',sqrt(mean((T.id(active)-T.id_ref(active)).^2)),...
 'voltage_magnitude_max_V',max(hypot(T.vd,T.vq)),...
 'feedback_filter_tf_s',M.filter_tf);
ix=T.applied_sample_us>0;
S.sample_to_latch_us=prctile(T.latch_us(ix)-T.applied_sample_us(ix),[0 50 95 100]);
S.calc_age_us=prctile(T.calc_us-T.us,[0 50 95 100]);
v20=nan(size(t));
for k=1:numel(t)
 ii=abs(t-t(k))<=.01;
 if sum(ii)>=5,p=polyfit(t(ii)-t(k),pos(ii),1);v20(k)=p(1);end
end
if M.mode==6
 ii=T.stage==2&t>.65&active;
 if sum(ii)>10
  p=polyfit(t(ii)-mean(t(ii)),pos(ii),1);
  S.hold=struct('start_s',min(t(ii)),'end_s',max(t(ii)),...
   'target_rad_s',J.setup.speed_target,'angle_fit_rad_s',p(1),...
   'speed_mean_rad_s',mean(v(ii)),'speed_std_rad_s',std(v(ii)),...
   'speed20_mean_rad_s',mean(v20(ii),'omitnan'),'speed20_std_rad_s',std(v20(ii),'omitnan'),...
   'speed20_min_rad_s',min(v20(ii)),'speed20_max_rad_s',max(v20(ii)),...
   'id_mean_A',mean(T.id(ii)),'id_std_A',std(T.id(ii)),...
   'iq_mean_A',mean(T.iq(ii)),'iq_std_A',std(T.iq(ii)),...
   'iq_error_rmse_A',sqrt(mean((T.iq(ii)-T.iq_ref(ii)).^2)),...
   'current_limit_fraction',mean(abs(T.iq_ref(ii))>.99*J.setup.current_limit));
 end
else
 edges=[1;find(diff(T.iq_ref)~=0)+1;height(T)+1];S.pulses=struct([]);
 for k=1:numel(edges)-1
  ii=edges(k):edges(k+1)-1;if abs(T.iq_ref(ii(end)))<.01,continue;end
  ii=ii(t(ii)>t(ii(end))-.008);
  S.pulses=[S.pulses;struct('iq_ref_A',mean(T.iq_ref(ii)),...
   'iq_mean_A',mean(T.iq(ii)),'iq_std_A',std(T.iq(ii)),...
   'id_mean_A',mean(T.id(ii)),'start_s',t(ii(1)),'end_s',t(ii(end)))]; %#ok<AGROW>
 end
end
if M.mode==3
 ii=t>.14&t<.19;
 S.d_hold=struct('mean_A',mean(T.id(ii)),'std_A',std(T.id(ii)));
 ii=t>=.095&t<=.13;tt=t(ii)-.1;yy=T.id(ii);
 cost=@(p)mean((yy-(p(1)+p(2)*(1-exp(-max(tt-p(3),0)/exp(p(4)))))).^2);
 p=fminsearch(cost,[.15 .15 .0002 log(.001)],optimset('Display','off'));
 S.fit=struct('tau_s',exp(p(4)),'delay_s',p(3),'equivalent_fc_Hz',1/(2*pi*exp(p(4))),'rmse_A',sqrt(cost(p)));
end
if isfield(J,'health')&&~isempty(J.health)
 S.health_first=J.health(1);S.health_last=J.health(end);
end
f=figure('Visible','off','Color','w','Position',[50 50 1500 1050]);
set(groot,'defaultAxesFontName','Microsoft YaHei');tl=tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
nexttile;plot(t,v,'Color',[.7 .8 1]);hold on;plot(t,v20,'b',t,T.speed_ref,'k--');grid on;
ylabel('机械速度 / rad/s');legend('编码器原有3ms低通','离线20ms角度拟合','速度目标');
nexttile;plot(t,T.id,'Color',[.5 .7 1]);hold on;plot(t,T.id_ref,'b--',t,T.iq,'Color',[1 .6 .4]);
plot(t,T.iq_ref,'r--');grid on;ylabel('d/q电流 / A');legend('Id','Id目标','Iq','Iq目标');
nexttile;plot(t,[ia ib ic]);grid on;ylabel('相电流 / A');legend('A采样','B采样','C重构');
nexttile;plot(t,[T.vd T.vq hypot(T.vd,T.vq)]);hold on;yline(M.voltage_limit,'k--');
grid on;ylabel('PI电压命令 / V');legend('Vd','Vq','幅值','限制');
nexttile;plot(t,margin);hold on;yline(0,'r--');grid on;ylabel('扣除保护量后的窗口余量 / us');
nexttile;plot(t,T.calc_us-T.us,'.',t(ix),T.latch_us(ix)-T.applied_sample_us(ix),'.');grid on;
ylabel('时间 / us');legend('采样至计算结束','所用采样至PWM生效');
for ax=findall(f,'Type','axes')',xlabel(ax,'运行时间 / s');end
title(tl,sprintf('%s | 电流反馈无滤波 | 有效 %.4f%% | reason=%d',...
 strrep(name,'_','\_'),S.valid_fraction*100,M.reason));
exportgraphics(f,fullfile(root,[name '.png']),'Resolution',150);
savefig(f,fullfile(root,[name '.fig']));save(fullfile(root,[name '.mat']),'T','J','S','v20');close(f);
fid=fopen(fullfile(root,[name '_analysis.json']),'w');fwrite(fid,jsonencode(S,PrettyPrint=true));fclose(fid);
disp(jsonencode(S,PrettyPrint=true));
end
