function plot_balance_axes(out)
% 使用实际仿真资产与视觉变换绘制俯视坐标图。
paths=balance_setup;f=figure('Visible','off','Color','w','Position',[50 50 950 850]);ax=axes(f);hold(ax,'on');
folder=fullfile(paths.root,'模型','仿真模型','通用URDF','meshes');
names={'body.stl','right_wheel.stl','left_wheel.stl'};
offsets=[0 0 0;-.0662682 0 0;.0662682 0 0];
R=[0 1 0;-1 0 0;0 0 1];
for k=1:3
 tri=stlread(fullfile(folder,names{k}));v=tri.Points*R'+offsets(k,:);
 color=[.12 .4 .65];if k>1,color=[.25 .25 .25];end
 patch(ax,'Faces',tri.ConnectivityList,'Vertices',v,'FaceColor',color,'EdgeColor','none');
end
quiver3(ax,0,0,.12,.055,0,0,0,'r','LineWidth',2);text(ax,.06,.005,.12,'+X 右');
quiver3(ax,0,0,.12,0,.055,0,0,'g','LineWidth',2);text(ax,-.025,.065,.12,'+Y 前 / USB');
text(ax,-.09,-.045,.12,'左 MA / -X');text(ax,.048,-.045,.12,'右 MB / +X');
view(ax,0,90);axis(ax,'equal');xlim(ax,[-.11 .11]);ylim(ax,[-.07 .09]);grid(ax,'on');
xlabel(ax,'X 向右 / m');ylabel(ax,'Y 向前 / m');title(ax,'俯视：X右、Y前、Z上（朝观察者）');
exportgraphics(f,fullfile(out,'axes.png'),'Resolution',140);close(f);
end
