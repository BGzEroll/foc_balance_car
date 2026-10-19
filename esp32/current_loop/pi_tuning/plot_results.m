function plot_results
% Offline only: reads saved data, never connects to hardware.
r=fileparts(mfilename('fullpath'));
set(groot,'defaultAxesFontName','Microsoft YaHei');
names={'base','soft','mid','lowki'};labels={'MA 无轮','MB 带轮'};
f=figure('Visible','off','Color','w','Position',[40 40 1400 850]);tl=tiledlayout(2,2,'Padding','compact');
for motor=1:2
 for row=1:2
  nexttile((row-1)*2+motor);hold on;
  for k=1:numel(names)
   files=dir(fullfile(r,[names{k} '_d030*_analysis.json']));n='';
   for q=1:numel(files),A=jsondecode(fileread(fullfile(r,files(q).name)));if A.progression_pass,n=erase(files(q).name,'_analysis.json');break;end,end
   if isempty(n),continue;end
   T=readtable(fullfile(r,[n '.csv']));J=jsondecode(fileread(fullfile(r,[n '.json'])));I=T.motor==motor;
   plot((T.us(I)-J.meta.profile_start)*.001,T.id(I),'DisplayName',names{k});
  end
  yline(.3,'k--','HandleVisibility','off');grid on;legend('Location','best');xlabel('时间 / ms');ylabel('Id / A');title(labels{motor});
  if row==1,xlim([95 112]);else,xlim([140 190]);end
 end
end
title(tl,'同一 v18 合并轮询：0.15→0.30 A 阶跃和稳态纹波，保存间隔约0.4 ms');
exportgraphics(f,fullfile(r,'pi_step_comparison.png'),'Resolution',150);savefig(f,fullfile(r,'pi_step_comparison.fig'));close(f);
files=dir(fullfile(r,'*_analysis.json'));S={};
for k=1:numel(files),j=jsondecode(fileread(fullfile(r,files(k).name)));if j.meta.mode==6,S{end+1}=j;end,end
if isempty(S),return;end
f=figure('Visible','off','Color','w','Position',[40 40 1400 550]);tl=tiledlayout(1,2,'Padding','compact');
for motor=1:2
 nexttile;hold on;
 for k=1:numel(names)
  xy=[];
  for q=1:numel(S),j=S{q};if startsWith(j.name,[names{k} '_']),xy(end+1,:)=[j.motors(motor).speed_fit_rad_s,1000*j.metric(motor).iq_error_rms_A];end,end
  if ~isempty(xy),xy=sortrows(xy);plot(xy(:,1),xy(:,2),'o-','DisplayName',names{k});end
 end
 grid on;legend('Location','best');title(labels{motor});xlabel('实测平台速度 / rad/s');ylabel('全速累计 Iq 跟踪误差 RMS / mA');
end
title(tl,'速度外环参数保持相同的电流 PI 对照');exportgraphics(f,fullfile(r,'pi_speed_comparison.png'),'Resolution',150);savefig(f,fullfile(r,'pi_speed_comparison.fig'));close(f);
cases={'base_d0p1','id','id_ref','0.10 A d轴阶跃';'base_d0p5','id','id_ref','0.50 A d轴阶跃';'base_q0p2','iq','iq_ref','正负0.20 A q轴脉冲'};
if ~all(cellfun(@(n)isfile(fullfile(r,[n '.csv'])),cases(:,1))),return;end
f=figure('Visible','off','Color','w','Position',[40 40 1400 1050]);tl=tiledlayout(3,2,'Padding','compact');
for q=1:3
 T=readtable(fullfile(r,[cases{q,1} '.csv']));J=jsondecode(fileread(fullfile(r,[cases{q,1} '.json'])));
 for motor=1:2
  nexttile((q-1)*2+motor);I=T.motor==motor;t=(T.us(I)-J.meta.profile_start)*.001;
  plot(t,T.(cases{q,2})(I));hold on;plot(t,T.(cases{q,3})(I),'k--');grid on;legend('反馈','目标','Location','best');xlabel('时间 / ms');ylabel('电流 / A');title([labels{motor} '：' cases{q,4}]);
 end
end
title(tl,'推荐原参数：Kp=0.060318579，Ki=226.194672，无额外电流滤波');exportgraphics(f,fullfile(r,'current_amplitudes.png'),'Resolution',150);savefig(f,fullfile(r,'current_amplitudes.fig'));close(f);
end
