function fig=balance_demo_panel
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
% BALANCE_DEMO_PANEL 中文工况选择入口；直接调用原有3D多体仿真API。
tag='balance_car_demo_panel';existing=findall(groot,'Type','figure','Tag',tag);
if ~isempty(existing)
 if isequal(existing(1).UserData,'lqr-pid-current-v5'),fig=existing(1);figure(fig);return;end
 delete(existing);
end
here=fileparts(mfilename('fullpath'));addpath(here);configure_balance_cache;
names={'recover','drive','turn','route','push','terrain','bias','slope','slope_hold'};
labels={'倾斜恢复：前倾8°后站稳','前进与后退：先前进30厘米，再后退20厘米', ...
 '原地转向：转到60°，再回到20°','边走边转：前进60厘米，途中转向60°', ...
 '外力推扰：水平推一下，再施加偏航转矩','起伏地形：上坡、平台、下坡、单侧凸起', ...
 '持续扰动：持续施加小推力和偏航转矩', ...
 '明显坡道：上坡、平台、下坡、3mm凸起','坡中停车：斜面中段停12秒，再继续行驶'};
descriptions={ ...
 '6秒。车身起初前倾，轮子向前接住重心，然后恢复到原位。', ...
 '11秒。1秒后开始前进，6秒后后退；最后停在起点前方约10厘米。', ...
 '9秒。1～3秒原地转向至60°，5～7秒再转回20°。观察车头和轮轴的方向变化。', ...
 '10秒。小车沿弯曲路径行驶，而不是原地转。观察行驶方向逐渐改变。', ...
 '9秒。2秒时受水平推力，4秒时受偏航转矩；外力消失后尝试回到原位和原航向。', ...
 '13秒。向前经过3毫米坡台和左轮0.8毫米凸起，观察升降与短暂侧倾。', ...
 '18秒。2秒后持续施加扰动，查看LQR在恒定外力下的稳态误差。', ...
 '26秒。默认12°坡、高约12.8厘米，完整通过上坡、平台、下坡与凸起。', ...
 '34秒。先到上坡中段，7～19秒停车，随后继续通过坡顶和下坡。检查是否溜车。'};
fig=figure('Name','平衡车3D仿真：选择工况','NumberTitle','off','Tag',tag, ...
 'MenuBar','none','ToolBar','none','Color',[.96 .97 .98],'Position',[90 80 880 650],'UserData','lqr-pid-current-v5');
textBox([.04 .87 .92 .10],{'每次选择一个工况 → 点击运行 → 在 Multibody Explorer 点击播放', ...
 '切换下拉框不会自动改变3D动画；运行后，同一3D标签会更新为新工况。'},14);
textBox([.04 .79 .16 .045],'仿真工况',12);
pick=uicontrol(fig,'Style','popupmenu','String',labels,'Value',3,'Tag','case_selector','Units','normalized', ...
 'Position',[.20 .79 .53 .05],'FontSize',12,'Callback',@selectScenario);
profileNames={'normal','agile','slope'};
profilePick=uicontrol(fig,'Style','popupmenu','String',{'常规','快速（大倾角）','坡道整定'},'Value',1, ...
 'Tag','motion_profile_selector','Units','normalized','Position',[.76 .79 .19 .05],'FontSize',12,'Callback',@describe);
textBox([.04 .71 .16 .045],'控制器',12);
controllerNames={'lqr','pid','pid_current'};
controllerPick=uicontrol(fig,'Style','popupmenu','String',{'LQR：六状态反馈','PID：转矩输出（原版）','PID：目标电流输出'}, ...
 'Value',1,'Tag','controller_selector','Units','normalized','Position',[.20 .71 .38 .05],'FontSize',12,'Callback',@selectController);
limitLabel=textBox([.60 .71 .20 .045],'单轮转矩上限',12);
design=design_balance_suite;
limit=uicontrol(fig,'Style','edit','String',num2str(design.torque_limit_each),'Tag','torque_limit','Units','normalized', ...
 'Position',[.80 .71 .15 .05],'FontSize',12,'BackgroundColor','w');
textBox([.04 .65 .28 .04],'实际/指令转矩增益：左 / 右',11);
gainLeft=uicontrol(fig,'Style','edit','String','1','Tag','gain_left','Units','normalized', ...
 'Position',[.34 .65 .14 .045],'FontSize',12,'BackgroundColor','w');
gainRight=uicontrol(fig,'Style','edit','String','1','Tag','gain_right','Units','normalized', ...
 'Position',[.51 .65 .14 .045],'FontSize',12,'BackgroundColor','w');
textBox([.69 .65 .26 .04],'1=标称；0.6=弱40%',11);
detail=textBox([.04 .56 .91 .08],descriptions{3},12);
unitHint=textBox([.04 .51 .91 .04],'上限单位N·m（指令侧）；Inf不限幅。增益模拟Kt误差；传感器仍理想。',10);
runButton=uicontrol(fig,'Style','pushbutton','String','运行所选工况，更新3D窗口','Tag','run_selected', ...
 'Units','normalized','Position',[.04 .43 .58 .075],'FontSize',14,'Callback',@runCase);
curveButton=uicontrol(fig,'Style','pushbutton','String','查看所选工况的曲线','Tag','view_curves', ...
 'Units','normalized','Position',[.65 .43 .30 .075],'FontSize',12,'Callback',@showCurves);
status=textBox([.04 .27 .91 .14],{'当前尚未从本面板运行。建议先选“原地转向”，点击上面的运行按钮。', ...
 '计算可能需要几十秒。计算结束后，切到Multibody Explorer，用播放按钮反复观看。'},12);
status.Tag='status_text';
textBox([.04 .08 .91 .16],{'三个窗口各管一件事：', ...
 '① 本面板：选择并运行工况。 ② Simulink：查看控制器和机械连接。', ...
 '③ Multibody Explorer：播放当前这一次仿真的3D结果。', ...
 '它的播放按钮只重播当前结果；要换动作，请回到本面板选择并重新运行。'},11);
    function h=textBox(pos,txt,size)
        h=uicontrol(fig,'Style','text','String',txt,'Units','normalized','Position',pos, ...
          'HorizontalAlignment','left','FontSize',size,'BackgroundColor',[.96 .97 .98]);
    end
    function describe(~,~)
        detail.String=descriptions{pick.Value};
        if profilePick.Value==2
            fastDescriptions={'6秒倾斜恢复；快速档与常规档使用相同初始条件。', ...
             '7秒。1～2.2秒前进30厘米，3.8～5秒后退20厘米；允许更大目标倾角。', ...
             '7秒。1～2.2秒转至60°，4～5.2秒返回20°。', ...
             '8秒。1～4秒行驶60厘米，1.5～3秒转向60°。', ...
             descriptions{5},'地形保持原速度；快速档目前只加快平地行驶和转向。',descriptions{7},descriptions{8},descriptions{9}};
            detail.String=fastDescriptions{pick.Value};
        end
        status.String={'已选择新工况，3D窗口仍保留上一次结果。','请点击“运行所选工况，更新3D窗口”。'};
    end
    function selectScenario(~,~)
        if pick.Value>=8,profilePick.Value=3;end
        describe([],[]);
    end
    function selectController(~,~)
        if controllerPick.Value==3
            motor=balance_current_parameters;limit.String=num2str(motor.current_limit_each);
            limitLabel.String='单轮iq上限 / A';
            unitHint.String=sprintf('电流接口：Kt暂估%.5g Nm/A，理想电流跟踪；左右增益表示实际Kt/标称Kt。',motor.Kt_nominal);
        else
            limit.String=num2str(design.torque_limit_each);limitLabel.String='单轮转矩上限';
            unitHint.String='上限单位N·m（指令侧）；Inf不限幅。增益模拟Kt误差；传感器仍理想。';
        end
        describe([],[]);
    end
    function runCase(~,~)
        cap=str2double(limit.String);
        if ~isscalar(cap)||isnan(cap)||cap<=0
            status.String='输出上限请输入正数：转矩模式单位Nm，电流模式单位A；或输入Inf。';return;
        end
        gains=[str2double(gainLeft.String) str2double(gainRight.String)];
        if any(~isfinite(gains))||any(gains<=0),status.String='左右执行增益请输入正数，例如1、0.6或1.4。';return;end
        controller=controllerNames{controllerPick.Value};
        profile=profileNames{profilePick.Value};
        name=names{pick.Value};
        controls=[pick profilePick controllerPick limit gainLeft gainRight runButton curveButton];set(controls,'Enable','off');
        status.String={['正在计算：' labels{pick.Value} ' / ' upper(controller)], ...
            '稍后会更新Multibody Explorer。请等待计算完成。'};drawnow;
        try
            [simout,report]=run_balance_case(name,true,cap,controller,gains,profile);
            assignin('base','simout_selected',simout);assignin('base','report_selected',report);
            if isgraphics(fig)
                status.String={sprintf('已完成：%s / %s，%.0f秒，检查通过=%d。',name,upper(controller),report.duration_s,report.pass), ...
                 '切到 Multibody Explorer 的 balance_car_3d_suite 标签，点击 ▶ 播放。', ...
                 '可拖动时间轴查看某一瞬间；同一标签现在已替换为这次工况。'};
            end
        catch err
            if isgraphics(fig),status.String={['运行失败：' err.message],'完整错误已显示在MATLAB命令窗口。'};end
            fprintf(2,'%s\n',getReport(err));
        end
        if isgraphics(fig),set(controls,'Enable','on');end
    end
    function showCurves(~,~)
        controller=controllerNames{controllerPick.Value};gains=[str2double(gainLeft.String) str2double(gainRight.String)];
        if any(~isfinite(gains))||any(gains<=0),status.String='请先填写有效的左右执行增益。';return;end
        file=fullfile(balance_result_folder(paths.results,controller,gains,profileNames{profilePick.Value}),[names{pick.Value} '_' controller],'results.png');
        if ~isfile(file),status.String='这个组合还没有保存的曲线，请先运行。';return;end
        delete(findall(groot,'Type','figure','Tag','balance_curves'));
        f=figure('Name',['仿真曲线：' labels{pick.Value} ' / ' upper(controller)],'NumberTitle','off','Color','w','Tag','balance_curves');
        ax=axes(f);image(ax,imread(file));axis(ax,'image');axis(ax,'off');
    end
end

