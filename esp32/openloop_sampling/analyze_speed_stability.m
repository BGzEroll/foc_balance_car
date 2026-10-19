function R = analyze_speed_stability(label)
% Encoder speed from an approximately 20ms centered angle-difference window.
% This is an observation at that averaging time, not instantaneous shaft speed.
root=fileparts(mfilename('fullpath'));
if nargin==0,label='ma_sync_012v_5rad_stability';end
analyze_openloop(label);D=load(fullfile(root,[label '_analysis.mat']));T=D.T;M=D.J.meta;
t=(T.us-T.us(1))*1e-6;a=unwrap(T.left);theta=unwrap(T.theta);
h=max(1,round(.010/median(diff(t))));ii=(1+h):(height(T)-h);
w=-(a(ii+h)-a(ii-h))./(t(ii+h)-t(ii-h));ts=t(ii);
A=[ones(height(T),1),t];pa=A\a;slip=theta+7*a;ps=A\slip;
R=D.result;R.window_span_ms=median(t(ii+h)-t(ii-h))*1000;
R.window_speed_mean_rad_s=mean(w);R.window_speed_std_rad_s=std(w);
R.window_speed_min_rad_s=min(w);R.window_speed_max_rad_s=max(w);
ws=sort(w);qp=linspace(0,1,numel(ws));
R.window_speed_p05_rad_s=interp1(qp,ws,.05);R.window_speed_p95_rad_s=interp1(qp,ws,.95);
R.window_speed_cv_percent=100*std(w)/abs(mean(w));
R.mean_speed_error_percent=100*(abs(pa(2))-M.omega)/M.omega;
R.angle_fit_r_squared=1-sum((a-A*pa).^2)/sum((a-mean(a)).^2);
R.slip_slope_rad_s=ps(2);R.slip_range_rad=range(slip);
R.measured_duration_s=t(end)-t(1);R.fault_seen=0;
if isfield(M,'fault_seen'),R.fault_seen=M.fault_seen;end
f=figure('Visible','off','Color','w','Position',[70 70 1400 950]);
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;plot(t,a-a(1));hold on;plot(t,-M.omega*t,'k--');grid on;
xlabel('缓存相对时间 (s)');ylabel('机械角位移 (rad)');legend('编码器','目标参考斜率');
title(sprintf('整体拟合 %.4f rad/s；平均误差 %.2f%%',pa(2),R.mean_speed_error_percent));
nexttile;plot(ts,w);hold on;yline(M.omega,'k--');grid on;
xlabel('缓存相对时间 (s)');ylabel('速度幅值 (rad/s)');
title(sprintf('约%.1fms窗口速度：均值%.3f，标准差%.3f',R.window_span_ms,mean(w),std(w)));
nexttile;plot(t,slip-slip(1));grid on;xlabel('缓存相对时间 (s)');ylabel('相对电角差变化 (rad)');
title(sprintf('变化范围%.3frad；趋势斜率%.3frad/s',range(slip),ps(2)));
nexttile;plot(t,D.ia,t,D.ib);grid on;xlabel('缓存相对时间 (s)');ylabel('标称电流 (A)');legend('A','B');
title(sprintf('全速率判窗 %.4f%%，停止原因%d',R.full_rate_valid_percent,M.reason));
title(tl,sprintf('5rad/s稳定性核查：v%d，Uq=%.2fV，PWM/ADC均10kHz',M.version,M.uq));
exportgraphics(f,fullfile(root,[label '_stability.png']),'Resolution',125);
savefig(f,fullfile(root,[label '_stability.fig']));close(f);
save(fullfile(root,[label '_stability.mat']),'R','ts','w','slip','T');
fid=fopen(fullfile(root,[label '_stability.json']),'w');fwrite(fid,jsonencode(R,PrettyPrint=true),'char');fclose(fid);
disp(R);
end
