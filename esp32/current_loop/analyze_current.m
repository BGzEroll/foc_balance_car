function S=analyze_current(name)
% 实物采样数据分析；不将理想PWM推算电压当作示波器测量。
root=fileparts(mfilename('fullpath'));
T=readtable(fullfile(root,[name '.csv']));
J=jsondecode(fileread(fullfile(root,[name '.json']))); M=J.meta;
t=(T.us-M.profile_start)*1e-6; dt=median(diff(t));
S=struct('name',name,'reason',M.reason,'valid_fraction',M.full_rate_valid/max(1,M.full_rate_active),...
 'rows',height(T),'logged_dt_s',dt,'kp',M.kp,'ki',M.ki,'filter_tf_s',M.filter_tf,...
 'nominal_phase_R_ohm',.09,'nominal_phase_L_H',24e-6);
S.calc_age_us=prctile(T.calc_us-T.us,[0 50 95 100]);
ix=T.applied_sample_us>0;
S.sample_to_latch_us=prctile(T.latch_us(ix)-T.applied_sample_us(ix),[0 50 95 100]);
lutfields=fieldnames(J.lut_mv);
lutA=J.lut_mv.(lutfields{1});lutB=J.lut_mv.(lutfields{2});
ia=(lutA(T.raw_a+1)*.001-M.offset_V(1))*M.gain_A_per_V;
ib=(lutB(T.raw_b+1)*.001-M.offset_V(2))*M.gain_A_per_V;
beta=(ia+2*ib)/sqrt(3);
S.park_recompute_max_error_A=max(abs([ia.*cos(T.theta)+beta.*sin(T.theta)-T.id;...
 beta.*cos(T.theta)-ia.*sin(T.theta)-T.iq]));
f=figure('Visible','off','Position',[80 80 1440 850],'Color','w');
set(groot,'defaultAxesFontName','Microsoft YaHei');
tl=tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
if M.mode<=2
 nexttile;plot(t,[ia ib]);grid on;ylabel('相电流 / A');legend('A','B');
 nexttile;plot(t,T.left);grid on;ylabel('编码器机械角 / rad');
 nexttile;plot(t,[T.ca T.cb T.cc]);grid on;ylabel('实际生效比较值 / tick');legend('A','B','C');
else
 % 仅为阅读清晰加约1ms移动平均线；闭环是否滤波由filter_tf决定。
 w=max(1,round(.001/dt));
 nexttile;plot(t,T.id,'Color',[.7 .8 .95]);hold on;
 plot(t,movmean(T.id,w),'b',t,T.id_ref,'k--','LineWidth',1.2);grid on;
 ylabel('I_d / A');legend('原始反馈','约1ms显示均值','指令');
 nexttile;plot(t,T.iq,'Color',[1 .8 .7]);hold on;
 plot(t,movmean(T.iq,w),'r',t,T.iq_ref,'k--','LineWidth',1.2);grid on;
 ylabel('I_q / A');legend('原始反馈','约1ms显示均值','指令');
 nexttile;plot(t,[T.vd T.vq]);grid on;ylabel('PI 电压命令 / V');legend('V_d','V_q');
 S.steady=[];
 edges=[1;find(diff(T.stage)~=0)+1;height(T)+1];
 for k=1:numel(edges)-1
  ii=edges(k):edges(k+1)-1; ii=ii(t(ii)>t(ii(end))-.01);
  if isempty(ii),continue;end
  z=struct('stage',T.stage(ii(1)),'start_s',t(edges(k)),...
   'id_ref',mean(T.id_ref(ii)),'id_mean',mean(T.id(ii)),'id_std',std(T.id(ii)),...
   'iq_ref',mean(T.iq_ref(ii)),'iq_mean',mean(T.iq(ii)),'iq_std',std(T.iq(ii)),...
   'speed_mean',mean(T.speed(ii)));
  S.steady=[S.steady;z]; %#ok<AGROW>
 end
 if M.mode==3
  % 用0.1s上升阶跃的实测均值拟合闭环一阶近似，非电机R/L辨识。
  ii=t>=.095&t<=.13; tt=t(ii)-.1; yy=T.id(ii); ref=T.id_ref(ii);
  cost=@(p)mean((yy-(p(1)+p(2)*(1-exp(-max(tt-p(3),0)/exp(p(4)))))).^2);
  p=fminsearch(cost,[M.current_amp/2 M.current_amp/2 .0002 log(.002)],optimset('Display','off'));
  S.closed_loop_fit=struct('offset_A',p(1),'step_A',p(2),'delay_s',p(3),...
   'tau_s',exp(p(4)),'equivalent_bandwidth_Hz',1/(2*pi*exp(p(4))),...
   'rmse_A',sqrt(cost(p)),'meaning','empirical closed-loop approximation; not R/L identification');
 end
end
nexttile;plot(t,T.calc_us-T.us,'.','MarkerSize',3);hold on;
plot(t(ix),T.latch_us(ix)-T.applied_sample_us(ix),'.','MarkerSize',3);
grid on;ylabel('时间 / us');legend('采样时间戳至计算结束','所用采样至PWM生效');
for ax=findall(f,'Type','axes')',xlabel(ax,'运行时间 / s');end
title(tl,sprintf('%s | 10kHz PWM | PI %.5g / %.5g | TF %.2g s | valid %.3f%%',...
 strrep(name,'_','\_'),M.kp,M.ki,M.filter_tf,S.valid_fraction*100));
exportgraphics(f,fullfile(root,[name '.png']),'Resolution',160);
savefig(f,fullfile(root,[name '.fig']));save(fullfile(root,[name '.mat']),'T','J','S');close(f);
fid=fopen(fullfile(root,[name '_matlab.json']),'w');fwrite(fid,jsonencode(S,PrettyPrint=true));fclose(fid);
disp(jsonencode(S,PrettyPrint=true));
end
