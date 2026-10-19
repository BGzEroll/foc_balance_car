function compare_openloop_voltage(labels)
% All runs have a 20rad/s target; compare Uq while preserving the measured data.
root=fileparts(mfilename('fullpath'));
if nargin==0
 labels={'ma_pwm10k_012v_20rad_full','ma_pwm10k_020v_20rad_full'};
end
n=numel(labels);D=cell(1,n);S=cell(1,n);
for k=1:n
 analyze_openloop(labels{k});D{k}=load(fullfile(root,[labels{k} '_analysis.mat']));
 d=D{k};T=d.T;t=(T.us-T.us(1))*1e-6;left=unwrap(T.left);theta=unwrap(T.theta);
 A=[ones(height(T),1),t];slip=theta+7*left;p=A\slip;
 S{k}=d.result;S{k}.uq_V=d.J.meta.uq;
 S{k}.target_follow_ratio=-S{k}.left_fit_speed_rad_s/d.J.meta.omega;
 S{k}.electrical_slip_slope_rad_s=p(2);S{k}.electrical_slip_range_rad=range(slip);
 S{k}.buffer_active_samples=nnz(bitand(uint32(T.flags),1));
 S{k}.buffer_active_last_time_ms=NaN;S{k}.last_active_ia_A=NaN;
 S{k}.last_active_ib_A=NaN;S{k}.last_active_window_valid=false;
 on=bitand(uint32(T.flags),1)>0;
 if any(on)
  ix=find(on);S{k}.buffer_active_last_time_ms=t(ix(end))*1000;
  S{k}.last_active_ia_A=d.ia(ix(end));S{k}.last_active_ib_A=d.ib(ix(end));
  S{k}.last_active_window_valid=d.valid(ix(end));
 end
 if nnz(on)<10 || d.J.meta.reason~=1
  % A protection stop is not evidence of a stable 20rad/s operating point.
  S{k}.target_follow_ratio=NaN;
  S{k}.electrical_slip_slope_rad_s=NaN;
 end
end
f=figure('Visible','off','Color','w','Position',[30 30 max(1400,560*n) 1000]);
tl=tiledlayout(3,n,'TileSpacing','compact','Padding','compact');
for k=1:n
 d=D{k};T=d.T;t=(T.us-T.us(1))*1e-6;left=unwrap(T.left);theta=unwrap(T.theta);
 nexttile(k);plot(t,d.ia,t,d.ib);grid on;xlabel('相对时间 (s)');ylabel('标称电流 (A)');
 title(sprintf('v%d Uq=%.2fV：原始电流，停止原因%d',d.J.meta.version,S{k}.uq_V,S{k}.reason));legend('A','B');
 if S{k}.reason~=1,xline(S{k}.buffer_active_last_time_ms/1000,'r--','保护停止');end
 nexttile(k+n);plot(t,left-left(1));hold on;plot(t,-20*t,'k--');grid on;
 xlabel('相对时间 (s)');ylabel('机械角位移 (rad)');legend('编码器','目标20rad/s参考斜率');
 if S{k}.reason==1
  title(sprintf('实际平均 %.3f rad/s；有效运行采样%u组',S{k}.left_fit_speed_rad_s,S{k}.full_rate_active));
 else
  title(sprintf('含保护停机区间；实际运行采样%u组',S{k}.full_rate_active));
  xline(S{k}.buffer_active_last_time_ms/1000,'r--','保护停止');
 end
 nexttile(k+2*n);slip=theta+7*left;yyaxis left;plot(t,slip-slip(1));ylabel('相对电角差变化 (rad)');
 yyaxis right;plot(t,min(d.margin_a,d.margin_b));hold on;yline(2,'k--');ylabel('最小窗口间距 (us)');
 grid on;xlabel('相对时间 (s)');
 if S{k}.reason==1
  title(sprintf('全程判窗%.3f%%；电角差斜率%.3f rad/s',S{k}.full_rate_valid_percent,S{k}.electrical_slip_slope_rad_s));
 else
  title(sprintf('运行段判窗%.3f%%；保护后不判断同步',S{k}.full_rate_valid_percent));
  xline(S{k}.buffer_active_last_time_ms/1000,'r--','保护停止');
 end
end
title(tl,'实物开环电压对照：目标20rad/s，PWM/ADC均10kHz，限时2秒');
exportgraphics(f,fullfile(root,'voltage_at20rad.png'),'Resolution',120);
savefig(f,fullfile(root,'voltage_at20rad.fig'));close(f);
results=[S{:}];
fid=fopen(fullfile(root,'voltage_at20rad.json'),'w');fwrite(fid,jsonencode(results,PrettyPrint=true),'char');fclose(fid);
save(fullfile(root,'voltage_at20rad.mat'),'results');disp(results);
end
