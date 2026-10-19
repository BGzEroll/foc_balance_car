function C=compare_current
root=fileparts(mfilename('fullpath'));
names={'d_100hz_raw','d_100hz_hires','d_200hz_hires'};
labels={'10MHz计数 / 100Hz PI','80MHz计数 / 100Hz PI','80MHz计数 / 200Hz PI'};
f=figure('Visible','off','Color','w','Position',[80 80 1400 850]);
set(groot,'defaultAxesFontName','Microsoft YaHei');
tl=tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
colors=lines(3);C=struct([]);
for k=1:3
 T=readtable(fullfile(root,[names{k} '.csv']));J=jsondecode(fileread(fullfile(root,[names{k} '.json'])));
 m=J.meta;t=(T.us-m.profile_start)*1e-6;ix=t>.14&t<.19;
 z=struct('name',names{k},'id_mean_A',mean(T.id(ix)),'id_std_A',std(T.id(ix)),...
  'iq_mean_A',mean(T.iq(ix)),'iq_std_A',std(T.iq(ix)),...
  'kp',m.kp,'ki',m.ki,'filter_tf_s',m.filter_tf,...
  'full_rate_valid_fraction',m.full_rate_valid/m.full_rate_active,...
  'pi_updates',m.pi_updates,'active_samples',m.full_rate_active,'missed_commit',m.missed_commit);
 C=[C;z]; %#ok<AGROW>
 nexttile(1);hold on;ii=t>.095&t<.12;
 plot((t(ii)-.1)*1e3,T.id(ii),'Color',colors(k,:));
 nexttile(2);hold on;plot(t(ix),T.id(ix)-.3,'Color',colors(k,:));
 nexttile(3);hold on;plot(t(ix),T.iq(ix),'Color',colors(k,:));
end
nexttile(1);grid on;yline(.3,'k--');xlabel('相对0.15→0.30 A阶跃 / ms');ylabel('I_d 原始反馈 / A');legend(labels,'Location','southeast');
nexttile(2);grid on;xlabel('运行时间 / s');ylabel('I_d - 0.30 A / A');
nexttile(3);grid on;xlabel('运行时间 / s');ylabel('I_q 原始反馈 / A');
nexttile(4);bar([[C.id_std_A]' [C.iq_std_A]']*1000);grid on;
xticklabels(labels);xtickangle(15);ylabel('平台标准差 / mA');legend('I_d','I_q');
title(tl,'MA实物电流闭环对比：PWM均为10kHz，三组均未加反馈滤波；统计窗口0.14–0.19s');
exportgraphics(f,fullfile(root,'current_loop_comparison.png'),'Resolution',160);
savefig(f,fullfile(root,'current_loop_comparison.fig'));save(fullfile(root,'current_loop_comparison.mat'),'C');close(f);
fid=fopen(fullfile(root,'current_loop_comparison.json'),'w');fwrite(fid,jsonencode(C,PrettyPrint=true));fclose(fid);
disp(jsonencode(C,PrettyPrint=true));
end
