function plot_settling(folder)
% Hardware A/B evidence, no serial commands.
T=readtable(fullfile(folder,'samples.csv'));r=jsondecode(fileread(fullfile(folder,'report.json')));
h=r.history;
if numel(h)==1 && isfield(r.extra,'reference')
 a=h;a.label='A0_reference_mean';a.scale=r.base;names=fieldnames(r.extra.reference);
 for j=1:numel(names),a.(names{j})=r.extra.reference.(names{j});end
 h=[a;h];
end
labels=string({h.label})';display=labels;
for i=1:numel(labels)
 if labels(i)=="A0_reference_mean",display(i)="原参数前后均值";
 elseif startsWith(labels(i),"A0"),display(i)="原参考A0";
 elseif labels(i)=="B1"
  if h(i).scale(2)>r.base(2)+.001,display(i)="速度阻尼+12%";else,display(i)="位置-20% 积分-25%";end
 elseif labels(i)=="B2",display(i)="阻尼增强及回拉减弱";
 elseif contains(labels(i),"_return"),display(i)="返回原参数"+i;
 elseif startsWith(labels(i),"A_return"),display(i)="恢复参考A1";
 elseif labels(i)=="B_notch",display(i)="12.5Hz陷波开启";
 elseif labels(i)=="B_validation",display(i)="候选120秒复核";
 else,display(i)=replace(labels(i),"B_rate_","角速度项×");end
end
rate=[h.band_rate_rms]';burst=[h.burst_fraction]'*100;
pitch=[h.pitch_rms_deg]';current=[h.current_rms]';speed=[h.speed_rms]';
metrics=table(labels,rate,burst,pitch,current,speed);writetable(metrics,fullfile(folder,'comparison.csv'));
fig=figure('Visible','off','Position',[80 80 1450 1050]);tiledlayout(3,2);
nexttile;bar(categorical(display,display),rate);ylabel('0.8Hz附近角速度 RMS (rad/s)');grid on;
nexttile;bar(categorical(display,display),speed*1000);ylabel('速度 RMS (mm/s)');grid on;
nexttile;bar(categorical(display,display),current);ylabel('每轮目标电流 RMS (A)');grid on;
nexttile;bar(categorical(display,display),pitch);ylabel('俯仰 RMS (°)');grid on;
t=(T.sample_us-T.sample_us(1))*1e-6;
nexttile;plot(t,T.band_rate);ylabel('低频角速度 (rad/s)');xlabel('记录时间 (s)');grid on;
nexttile;plot(t,(T.pitch-T.target_pitch)*180/pi);ylabel('俯仰偏差 (°)');xlabel('记录时间 (s)');grid on;
if any(labels=="B_notch"),sgtitle('12.5Hz公共电流陷波：连续平衡实机对比');
else,sgtitle('低频往复摆动：速度阻尼与位置积分在线对比');end
exportgraphics(fig,fullfile(folder,'comparison.png'),'Resolution',140);close(fig);
fprintf('JITTER_FIGURE_READY %d measured windows\n',numel(h));
end
