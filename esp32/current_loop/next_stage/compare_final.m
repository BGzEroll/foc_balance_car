function summary=compare_final
root=fileparts(mfilename('fullpath'));names={'speed5_final4s','speed_minus3_final4s'};
f=figure('Visible','off','Color','w','Position',[50 50 1500 850]);
set(groot,'defaultAxesFontName','Microsoft YaHei');tl=tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
summary=struct([]);tables=cell(1,2);
for k=1:2
 d=load(fullfile(root,[names{k} '.mat']));T=d.T;J=d.J;S=d.S;t=(T.us-J.meta.profile_start)*1e-6;
 nexttile(k);plot(t,-T.speed,'Color',[.7 .8 1]);hold on;plot(t,d.v20,'b',t,T.speed_ref,'k--');grid on;
 xlabel('时间 / s');ylabel('机械速度 / rad/s');legend('编码器速度（原有3ms低通）','离线20ms角度拟合','目标');
 title(sprintf('目标 %.0f，平台整体角度拟合 %.3f rad/s',J.setup.speed_target,S.hold.angle_fit_rad_s));
 nexttile(k+2);plot(t,T.iq,'Color',[1 .6 .4]);hold on;plot(t,T.iq_ref,'k--',t,T.id,'Color',[.5 .7 1]);
 grid on;xlabel('时间 / s');ylabel('电流 / A');legend('Iq反馈','Iq指令','Id反馈');
 title(sprintf('全速有效 %d/%d；保存样本窗口余量最小 %.2f us',J.meta.full_rate_valid,J.meta.full_rate_active,S.min_window_margin_us));
 z=struct('name',names{k},'target_rad_s',J.setup.speed_target,'angle_fit_rad_s',S.hold.angle_fit_rad_s,...
  'speed20_std_rad_s',S.hold.speed20_std_rad_s,'iq_error_rmse_A',S.hold.iq_error_rmse_A,...
  'valid',J.meta.full_rate_valid,'active',J.meta.full_rate_active,'max_update_gap_us',J.meta.max_sample_gap_us);
 summary=[summary;z];tables{k}=T; %#ok<AGROW>
end
title(tl,'MA裸转子双向低速实测 | 10kHz PWM | 电流反馈无滤波 | Id=0 | Iq指令限制±0.7A');
exportgraphics(f,fullfile(root,'bidirectional_final.png'),'Resolution',160);
savefig(f,fullfile(root,'bidirectional_final.fig'));save(fullfile(root,'bidirectional_final.mat'),'summary','tables');close(f);
fid=fopen(fullfile(root,'bidirectional_final.json'),'w');fwrite(fid,jsonencode(summary,PrettyPrint=true));fclose(fid);
disp(jsonencode(summary,PrettyPrint=true));
end
