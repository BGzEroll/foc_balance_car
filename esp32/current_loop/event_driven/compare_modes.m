function compare_modes
% Offline saved measurements; no serial communication or motor commands.
r=fileparts(mfilename('fullpath'));modes={'poll','event','merged'};
names={'轮询合并','事件拆分','事件合并'};L=nan(3,2);I=L;G=L;C=nan(3,1);
for k=1:3
 p=fullfile(r,[modes{k} '_summary.json']);if ~isfile(p),continue;end
 J=jsondecode(fileread(p));ix=find([J.target_rad_s]==27.5,1);if isempty(ix),continue;end;J=J(ix);
 for m=1:2
  L(k,m)=J.latency(m).first_activation_mean_us;
  I(k,m)=J.metric(m).iq_error_rms_A*1000;
  G(k,m)=J.meta.max_sample_gap_us(m);
 end
 C(k)=J.meta.missed_commit/40000*100;
end
set(groot,'defaultAxesFontName','Microsoft YaHei');
f=figure('Visible','off','Color','w','Position',[40 40 1350 900]);t=tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile;bar(L);ylabel('平均首次生效延迟 / μs');legend('MA','MB','Location','northwest');grid on;
nexttile;bar(I);ylabel('平台Iq误差RMS / mA');legend('MA','MB','Location','northwest');grid on;
nexttile;bar(G);ylabel('最大PI更新间隔 / μs');legend('MA','MB','Location','northwest');grid on;
nexttile;bar(C);ylabel('错过提交时隙 / %');grid on;
for ax=findall(f,'Type','axes')',xticks(ax,1:3);xticklabels(ax,names);end
title(t,'双电机27.5 rad/s：相同采样、PI和限幅；每方案一次4秒，不代表统计最优');
exportgraphics(f,fullfile(r,'scheduling_comparison.png'),'Resolution',140);
savefig(f,fullfile(r,'scheduling_comparison.fig'));save(fullfile(r,'scheduling_comparison.mat'),'L','I','G','C','modes','names');close(f);
end
