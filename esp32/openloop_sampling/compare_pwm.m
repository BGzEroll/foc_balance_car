function compare_pwm()
% Compare measured runs; both use MA open-loop Uq=.12V, 5rad/s and ADC 10kHz.
root=fileparts(mfilename('fullpath'));
labels={'ma_final_012v_5rad_full','ma_pwm10k_012v_5rad_full'};
names={'PWM 20 kHz','PWM 10 kHz'};D=cell(1,2);
for k=1:2
 analyze_openloop(labels{k});
 D{k}=load(fullfile(root,[labels{k} '_analysis.mat']));
end
f=figure('Visible','off','Color','w','Position',[60 60 1400 1000]);
tl=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for k=1:2
 d=D{k};T=d.T;t=(T.us-T.us(1))*1e-6;
 nexttile(k);plot(t,d.ia,'Color',[.3 .5 .9]);hold on;plot(t,d.ib,'Color',[.95 .5 .2]);grid on;
 xlabel('相对时间 (s)');ylabel('标称电流 (A)');ylim([-1.2 1.2]);title([names{k} '：全部原始电流']);legend('A','B');
 nexttile(k+2);plot(t,d.margin_a,t,d.margin_b);hold on;yline(2,'k--');grid on;
 xlabel('相对时间 (s)');ylabel('最小开关沿间距 (us)');ylim([-15 30]);
 title(sprintf('%s：全程有效 %.4f%%',names{k},d.result.full_rate_valid_percent));legend('A','B','2us 实验间隔');
end
nexttile(5);values=zeros(2,2);
for k=1:2,values(k,:)=[D{k}.result.ia_fit_residual_rms_A,D{k}.result.ib_fit_residual_rms_A]*1000;end
bar(categorical(names,names,'Ordinal',true),values);grid on;ylabel('单基波拟合残差 RMS (mA)');legend('A','B');title('仅使用窗口有效组；残差包含畸变');
nexttile(6);
for k=1:2
 d=D{k};T=d.T;t=(T.us-T.us(1))*1e-6;
 plot(t,unwrap(T.left)-T.left(1));hold on;
end
grid on;xlabel('相对时间 (s)');ylabel('机械角位移 (rad)');legend(names);title('稳定段编码器实测');
title(tl,'实物对照：相同 0.12V / 5rad/s / ADC 10kHz，PWM 20kHz → 10kHz');
exportgraphics(f,fullfile(root,'pwm20k_vs_10k.png'),'Resolution',130);
savefig(f,fullfile(root,'pwm20k_vs_10k.fig'));close(f);
results=struct('pwm20k',D{1}.result,'pwm10k',D{2}.result);
fid=fopen(fullfile(root,'pwm20k_vs_10k.json'),'w');fwrite(fid,jsonencode(results,PrettyPrint=true),'char');fclose(fid);
disp(results.pwm20k);disp(results.pwm10k);
end
