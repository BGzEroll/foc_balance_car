function compare_openloop_speed()
% Same measured 10kHz firmware/Uq; compare commanded mechanical speed 5 vs 20.
root=fileparts(mfilename('fullpath'));
labels={'ma_pwm10k_012v_5rad_full','ma_pwm10k_012v_20rad_full'};
names={'目标 5 rad/s','目标 20 rad/s'};D=cell(1,2);S=cell(1,2);
for k=1:2
 if k==2 || ~isfile(fullfile(root,[labels{k} '_analysis.mat']))
  analyze_openloop(labels{k});
 end
 D{k}=load(fullfile(root,[labels{k} '_analysis.mat']));
 d=D{k};T=d.T;t=(T.us-T.us(1))*1e-6;left=unwrap(T.left);theta=unwrap(T.theta);
 A=[ones(height(T),1),t];q=A\left;commandFit=A\theta;
 slip=theta+7*left;slipFit=A\slip;
 S{k}=d.result;
 S{k}.measured_to_command_speed_ratio=-q(2)/d.J.meta.omega;
 S{k}.speed_fit_r_squared=1-sum((left-A*q).^2)/max(sum((left-mean(left)).^2),eps);
 S{k}.command_electrical_speed_rad_s=commandFit(2);
 S{k}.electrical_slip_slope_rad_s=slipFit(2);
 S{k}.electrical_slip_range_rad=range(slip);
end
f=figure('Visible','off','Color','w','Position',[60 60 1400 1000]);
tl=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for k=1:2
 d=D{k};T=d.T;t=(T.us-T.us(1))*1e-6;left=unwrap(T.left);theta=unwrap(T.theta);
 nexttile(k);plot(t,d.ia,t,d.ib);grid on;xlabel('相对时间 (s)');ylabel('标称电流 (A)');
 title([names{k} '：全部原始电流']);legend('A','B');
 nexttile(k+2);plot(t,left-left(1));hold on;plot(t,-d.J.meta.omega*t,'k--');grid on;
 xlabel('相对时间 (s)');ylabel('机械角位移 (rad)');legend('编码器','跟上目标时的参考斜率');
 title(sprintf('实测平均 %.3f rad/s，目标跟随率 %.1f%%',S{k}.left_fit_speed_rad_s,100*S{k}.measured_to_command_speed_ratio));
 nexttile(k+4);slip=theta+7*left;yyaxis left;plot(t,slip-slip(1));ylabel('相对电角差变化 (rad)');
 yyaxis right;plot(t,min(d.margin_a,d.margin_b));hold on;yline(2,'k--');ylabel('两相最小窗口间距 (us)');
 grid on;xlabel('相对时间 (s)');title(sprintf('判窗有效 %.4f%%；相对电角差斜率 %.3f rad/s',S{k}.full_rate_valid_percent,S{k}.electrical_slip_slope_rad_s));
end
title(tl,'实物开环速度对比：PWM/ADC 均 10kHz，Uq=0.12V，运行 2秒');
exportgraphics(f,fullfile(root,'speed5_vs_20.png'),'Resolution',130);
savefig(f,fullfile(root,'speed5_vs_20.fig'));close(f);
results=struct('speed5',S{1},'speed20',S{2});
fid=fopen(fullfile(root,'speed5_vs_20.json'),'w');fwrite(fid,jsonencode(results,PrettyPrint=true),'char');fclose(fid);
save(fullfile(root,'speed5_vs_20.mat'),'results');disp(results.speed20);
end
