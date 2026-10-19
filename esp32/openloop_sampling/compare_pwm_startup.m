function compare_pwm_startup()
root=fileparts(mfilename('fullpath'));
labels={'ma_pwm10k_020v_20rad_start','ma_sync_zero'};
names={'v4 逐路立即解除强制','v5 同一TEZ解除；三相等占空比检查'};
D=cell(1,2);f=figure('Visible','off','Color','w','Position',[50 50 1400 850]);
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
for k=1:2
 analyze_openloop(labels{k});d=load(fullfile(root,[labels{k} '_analysis.mat']));D{k}=d;
 T=d.T;on=bitand(uint32(T.flags),1)>0;i=find(on,1);
 if isempty(i),i=1;end
 t=(T.us-T.us(i))*1e-3;
 nexttile(k);plot(t,d.ia,t,d.ib);hold on;xline(0,'k--');grid on;xlim([-.2 2]);ylim([-5.5 3]);
 xlabel('相对首个运行样本 (ms)');ylabel('标称电流 (A)');title(names{k});legend('A','B','首个运行样本');
 nexttile(k+2);plot(t,T.ca,t,T.cb,t,T.cc);grid on;xlim([-.2 2]);ylim([245 255]);
 xlabel('相对首个运行样本 (ms)');ylabel('当时生效比较值');title('相同50%比较值不代表原启动路径无瞬态');legend('A','B','C');
end
title(tl,'启动故障排查：首个运行ADC读数与硬件同步启动零线电压检查');
exportgraphics(f,fullfile(root,'pwm_startup_check.png'),'Resolution',120);
savefig(f,fullfile(root,'pwm_startup_check.fig'));close(f);
end
