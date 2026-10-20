function fig=measured_pid_panel
% Single active GUI: PID -> measured FOC -> Multibody; simulation-only limits.
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));balance_setup;addpath(here,'-begin');
tag='measured_pid_foc_panel';existing=findall(groot,'Type','figure','Tag',tag);if ~isempty(existing),fig=existing(1);figure(fig);return;end
fig=figure('Name','平衡车：PID + FOC 综合性能仿真','NumberTitle','off','Tag',tag,'MenuBar','none','ToolBar','none','Color',[.96 .97 .98],'Position',[120 120 940 680]);
text([.04 .87 .92 .1],{'位置P → 速度PI → 俯仰PID / 偏航PID → 目标Iq → base电流环 → 3D小车', ...
 '上层PID默认200Hz，编码器与IMU均200Hz。本面板仅控制仿真，高电流档不是实物额定值。'},13);
text([.04 .77 .18 .05],'仿真场景',12);scenarios={'mixed','recover','drive','turn','route','push','limit','limit'};
pick=uicontrol(fig,'Style','popupmenu','String',{'综合：快慢速 / 坡中停 / 交错凸起 / 倒车转向 / 推扰','3°倾斜恢复','前进与后退','原地转向','边走边转','外力与偏航推扰','高速压力：2.5m/s（原200Hz曾通过）','高速压力：4m/s（原200Hz姿态超限）'},'Value',1,'Units','normalized','Position',[.23 .77 .72 .05],'FontSize',12);
text([.04 .68 .18 .05],'上层PID',12);tunings={'selected','baseline'};
tune=uicontrol(fig,'Style','popupmenu','String',{'候选参数（已跑综合场景）','原参数（对照）'},'Value',1,'Units','normalized','Position',[.23 .68 .31 .05],'FontSize',12);
text([.57 .68 .13 .05],'运行档位',12);envelopes={'explore6','explore3','explore1p5','explore10','bench'};
env=uicontrol(fig,'Style','popupmenu','String',{'6A 性能探索','3A 性能探索','1.5A 性能探索','10A 外推压力测试','0.7A 原台架范围'},'Value',1,'Units','normalized','Position',[.70 .68 .25 .05],'FontSize',12);
noise=uicontrol(fig,'Style','checkbox','String','实测IMU/编码器扰动','Value',1,'Units','normalized','Position',[.04 .59 .34 .05],'FontSize',12,'BackgroundColor',fig.Color);
text([.39 .59 .20 .05],'机械步长',12);step=uicontrol(fig,'Style','popupmenu','String',{'500us 快速探索','250us 精度复核'},'Value',1,'Units','normalized','Position',[.60 .59 .35 .05],'FontSize',12);
text([.04 .535 .18 .04],'上层PID周期',12);rate=uicontrol(fig,'Style','popupmenu','String',{'5ms / 200Hz（默认，编码器与IMU均200Hz）','1ms / 1kHz（周期对照，IMU仍200Hz）'},'Value',1,'Units','normalized','Position',[.23 .535 .72 .04],'FontSize',12);
text([.04 .43 .91 .08],{'综合场景约39秒：1.48m/s峰值指令 → 0.094m/s慢行 → 12°坡中停4秒', ...
 '→ 平台/下坡/3mm左凸起/2mm右凸起/1mm减速带 → 倒车/左右转向/推扰。'},12);
button=uicontrol(fig,'Style','pushbutton','String','运行所选场景，打开3D结果','Units','normalized','Position',[.04 .35 .61 .075],'FontSize',14,'Callback',@runCase);
curve=uicontrol(fig,'Style','pushbutton','String','打开最近结果曲线','Units','normalized','Position',[.68 .35 .27 .075],'FontSize',12,'Callback',@showCurves);
replay=uicontrol(fig,'Style','pushbutton','String','直接回放已保存综合场景','Units','normalized','Position',[.04 .25 .43 .065],'FontSize',12,'Callback',@(~,~)replay_measured_course);
compare=uicontrol(fig,'Style','pushbutton','String','打开PID验证对比','Units','normalized','Position',[.51 .25 .44 .065],'FontSize',12,'Callback',@showComparison);
status=text([.04 .04 .91 .18],{'已选综合性能场景。可直接回放已保存的机械真值，无需重新计算。', ...
 '500us用于快速探索；250us用于精度复核，具体资格见验证报告。', ...
 '高电流与高速结果属于模型外推；改变档位会重新计算，不控制实物。'},12);
latest='';controls=[pick tune env noise step rate button curve replay compare];
 function h=text(pos,content,sz)
  h=uicontrol(fig,'Style','text','String',content,'Units','normalized','Position',pos,'HorizontalAlignment','left','FontSize',sz,'BackgroundColor',fig.Color);
 end
 function runCase(~,~)
  name=scenarios{pick.Value};tuning=tunings{tune.Value};envelope=envelopes{env.Value};macro=[.0005 .00025];set(controls,'Enable','off');
  status.String={['正在计算：' name ' / ' tuning ' / ' envelope], '完整3D接触与FOC计算需要时间；命令窗口每20秒显示进度。'};drawnow;
  try
   periods=[.005 .001];durations=[39 6 11 9 10 9 7 7];speeds=[0 0 0 0 0 0 2.5 4];[o,r]=run_measured_car(name,true,3,logical(noise.Value),macro(step.Value),durations(pick.Value),tuning,envelope,speeds(pick.Value),periods(rate.Value));
   assignin('base','simout_pid',o);assignin('base','report_pid',r);
   latest=r.result_folder;
   status.String={sprintf('已完成：通过=%d，真实最大倾斜=%.2f°，电流指令上限=%.2fA。',r.pass,r.max_true_tilt_error_deg,r.current_limit_A), ...
    '切到 balance_car_pid_foc 的Multibody Explorer，底部播放键重播本次结果。'};
  catch err
   status.String={['运行失败：' err.message],'完整错误在MATLAB命令窗口。'};fprintf(2,'%s\n',getReport(err,'extended'));
  end
  if isgraphics(fig),set(controls,'Enable','on');end
 end
 function showCurves(~,~)
  if isempty(latest),status.String='请先运行一个工况；历史结果也保存在实测FOC/results。';return;end
  file=fullfile(latest,'mixed_course.png');if ~isfile(file),file=fullfile(latest,'truth_vs_sensed.png');end
  f=figure('Name','PID + FOC 最近结果','NumberTitle','off','Color','w');ax=axes(f);image(ax,imread(file));axis(ax,'image');axis(ax,'off');
 end
 function showComparison(~,~)
  file=fullfile(here,'pid_rate_validation','rate_comparison.png');if ~isfile(file),file=fullfile(here,'pid_validation','pid_comparison.png');end
  f=figure('Name','PID验证对比','NumberTitle','off','Color','w');ax=axes(f);image(ax,imread(file));axis(ax,'image');axis(ax,'off');
 end
end
