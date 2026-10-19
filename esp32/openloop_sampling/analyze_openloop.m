function result = analyze_openloop(label)
% 实测串口数据核查。换算电流采用板上 ADC 校准表；4 A/V 为标称值。
% 软件时戳包住整个转换，不等同于示波器测得的采样保持孔径/栅极波形。
root = fileparts(mfilename('fullpath'));
if nargin==0, label='baseline'; end
T = readtable(fullfile(root,[label '.csv']));
J = jsondecode(fileread(fullfile(root,[label '.json']))); M=J.meta;
periodTicks=500;
if isfield(M,'period_ticks'), periodTicks=M.period_ticks; end
% JSON numeric object keys become x0/x1/x2 in MATLAB.
L = fieldnames(J.lut_mv); lut=zeros(4096,3);
for k=1:3, lut(:,k)=J.lut_mv.(L{k}); end
ia=(lut(T.raw_a+1,1)*1e-3-M.offset_V(1))*M.gain_A_per_V;
ib=(lut(T.raw_b+1,2)*1e-3-M.offset_V(2))*M.gain_A_per_V;
bus=lut(T.raw_bus+1,3)*.003;
t=(T.us-T.us(1))*1e-6;
on=bitand(uint32(T.flags),1)>0;
adcok=bitand(uint32(T.flags),6)==6 & T.raw_a>0 & T.raw_a<4095 & T.raw_b>0 & T.raw_b<4095;
cycleok=bitand(uint32(T.flags),32)>0;
margin_a=min(T.a0-T.ca,periodTicks-T.ca-T.a1)*.1;
margin_b=min(T.b0-T.cb,periodTicks-T.cb-T.b1)*.1;
wa=margin_a>=M.guard_us; wb=margin_b>=M.guard_us;
valid=adcok & cycleok & (~on | (wa & wb));
flagwa=bitand(uint32(T.flags),8)>0; flagwb=bitand(uint32(T.flags),16)>0;
assert(all(flagwa==(wa|~on)) && all(flagwb==(wb|~on)),'Host/firmware window check mismatch');
left=unwrap(T.left); right=unwrap(T.right); theta=unwrap(T.theta);
sel=on & valid;
if nnz(sel)<10, sel=valid; end
fitA=[ones(nnz(sel),1),sin(theta(sel)),cos(theta(sel))];
if range(theta(sel))<pi
 coefA=[mean(ia(sel));0;0];coefB=[mean(ib(sel));0;0];
else
 coefA=fitA\ia(sel);coefB=fitA\ib(sel);
end
resA=ia(sel)-fitA*coefA;resB=ib(sel)-fitA*coefB;
speedfit=[ones(height(T),1),t]\left;
result=struct('label',label,'samples',height(T),'active_samples',nnz(on), ...
 'valid_percent',100*mean(valid),'active_valid_percent',100*nnz(valid&on)/max(nnz(on),1), ...
 'window_a_percent',100*nnz(wa&on)/max(nnz(on),1),'window_b_percent',100*nnz(wb&on)/max(nnz(on),1), ...
 'margin_a_min_us',min(margin_a(on)),'margin_b_min_us',min(margin_b(on)), ...
 'adc_a_median_us',median((T.a1-T.a0)*.1),'adc_b_median_us',median((T.b1-T.b0)*.1), ...
 'pair_median_us',median((T.b1-T.a0)*.1),'pair_max_us',max((T.b1-T.a0)*.1), ...
 'interval_median_us',median(diff(T.us)),'interval_max_us',max(diff(T.us)), ...
 'bus_mean_V',mean(bus),'ia_mean_A',mean(ia(sel)),'ib_mean_A',mean(ib(sel)), ...
 'ia_std_A',std(ia(sel)),'ib_std_A',std(ib(sel)), ...
 'ia_abs_max_A',max(abs(ia)),'ib_abs_max_A',max(abs(ib)), ...
 'ia_fundamental_A',hypot(coefA(2),coefA(3)),'ib_fundamental_A',hypot(coefB(2),coefB(3)), ...
 'ia_fit_residual_rms_A',sqrt(mean(resA.^2)),'ib_fit_residual_rms_A',sqrt(mean(resB.^2)), ...
 'left_delta_rad',left(end)-left(1),'right_delta_rad',right(end)-right(1), ...
 'left_fit_speed_rad_s',speedfit(2),'command_speed_rad_s',M.omega, ...
 'electrical_cycles',(theta(end)-theta(1))/(2*pi),'reason',M.reason, ...
 'cmp_errors',M.cmp_errors,'late_commit',M.late_commit);
if isfield(M,'full_rate_active')
 result.full_rate_active=M.full_rate_active;
 result.full_rate_valid=M.full_rate_valid;
 result.full_rate_valid_percent=100*M.full_rate_valid/max(M.full_rate_active,1);
 result.full_rate_cross=M.full_rate_cross;
end
f=figure('Visible','off','Color','w','Position',[80 80 1440 1000]);
tl=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
nexttile;plot(t,ia,'Color',[.65 .75 1]);hold on;plot(t,ib,'Color',[1 .75 .65]);
plot(t(sel),ia(sel),'.','MarkerSize',3);plot(t(sel),ib(sel),'.','MarkerSize',3);
grid on;xlabel('时间 (s)');ylabel('标称电流 (A)');title('MA 实测 A/B 相电流');legend('A 全部','B 全部','A 窗口有效','B 窗口有效');
nexttile;plot(t,left-left(1),t,right-right(1));grid on;xlabel('时间 (s)');ylabel('机械角位移 (rad)');title(sprintf('编码器观测：MA 拟合速度 %.3f rad/s',speedfit(2)));legend('left 字段','right 字段');
nexttile;plot(t,T.ca*.1,t,T.cb*.1);hold on;plot(t,T.a0*.1,'.',t,T.a1*.1,'.',t,T.b0*.1,'.',t,T.b1*.1,'.');
grid on;xlabel('时间 (s)');ylabel('TEZ 后相位 (us)');title('生效比较值及转换起止');legend('A 低侧起点','B 低侧起点','A 开始','A 结束','B 开始','B 结束');
nexttile;plot(t,margin_a,t,margin_b);hold on;yline(M.guard_us,'k--');grid on;xlabel('时间 (s)');ylabel('至相邻开关沿的最小间距 (us)');title(sprintf('按整段转换判窗，有效 %.2f%%',result.active_valid_percent));legend('A','B','实验保护间隔');
nexttile;histogram(resA,60);hold on;histogram(resB,60);grid on;xlabel('单基波拟合残差 (A)');ylabel('样本数');title('残差包含谐波、噪声和转动不匀');legend('A','B');
nexttile;plot(t,bus);grid on;xlabel('时间 (s)');ylabel('母线 (V)');title(sprintf('Uq=%.3f V，目标 %.2f rad/s，停止原因 %d',M.uq,M.omega,M.reason));
title(tl,[label '：实机开环采样核查'],'Interpreter','none');
exportgraphics(f,fullfile(root,[label '_analysis.png']),'Resolution',120);
savefig(f,fullfile(root,[label '_analysis.fig']));close(f);
save(fullfile(root,[label '_analysis.mat']),'T','J','ia','ib','bus','valid','margin_a','margin_b','result');
fid=fopen(fullfile(root,[label '_summary.json']),'w');fwrite(fid,jsonencode(result,PrettyPrint=true),'char');fclose(fid);
disp(result);
end
