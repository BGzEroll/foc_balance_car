function fig=replay_measured_course(resultFolder)
% Replay saved native Multibody truth; this function performs no new physics.
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));paths=balance_setup;
if nargin<1,resultFolder=fullfile(here,'results','mixed_selected_explore6_tilt0_sensor1_macro500us');end
s=load(fullfile(resultFolder,'simulation_measured.mat'),'o');r=jsondecode(fileread(fullfile(resultFolder,'validation.json')));
[t,raw]=data(s.o.raw3d);[tl,ql]=data(s.o.left_wheel_q);[tr,qr]=data(s.o.right_wheel_q);[tc,c]=data(s.o.control3d);
tf=(0:1/25:t(end))';state=interp1(t,raw,tf);wq=[interp1(tl,ql,tf) interp1(tr,qr,tf)];
a=balance_control_coordinates(state);q=a(:,8:11);yaw=atan2(2*(q(:,1).*q(:,4)+q(:,2).*q(:,3)),1-2*(q(:,3).^2+q(:,4).^2));
pitch=atan2(-2*(q(:,2).*q(:,4)-q(:,1).*q(:,3)),hypot(1-2*(q(:,3).^2+q(:,4).^2),2*(q(:,2).*q(:,3)+q(:,1).*q(:,4))));
speed=cos(yaw).*a(:,3)+sin(yaw).*a(:,5);
fig=figure('Name','PID + FOC：综合场景机械真值回放','NumberTitle','off','Color','w','Position',[40 70 1440 850],'CloseRequestFcn',@closeReplay);
ax=axes(fig,'Position',[.04 .17 .59 .76]);hold(ax,'on');grid(ax,'on');axis(ax,'equal');view(ax,[-40 22]);xlabel(ax,'X / 右 (m)');ylabel(ax,'Y / 前 (m)');zlabel(ax,'Z / 上 (m)');
patch(ax,'Vertices',[-1 -1 0;1 -1 0;1 9 0;-1 9 0],'Faces',[1 2 3 4],'FaceColor',[.86 .89 .91],'EdgeColor','none');
course=measured_course_definition;ang=deg2rad(course.slope_deg);th=.02;track=.1325364;
top=[0 course.ramp_start+course.ramp_run/2 course.height/2;0 course.ramp_start+course.ramp_run+course.platform_length/2 course.height;0 course.ramp_start+1.5*course.ramp_run+course.platform_length course.height/2; ...
 -track/2 course.bump_start+course.bump_length/2 course.bump_height;track/2 course.right_bump_start+.0175 course.right_bump_height;0 course.full_bump_start+.015 course.full_bump_height];
angles=[ang 0 -ang 0 0 0];dims=[.45 course.ramp_run/cos(ang) th;.45 course.platform_length th;.45 course.ramp_run/cos(ang) th;.06 course.bump_length th;.06 .035 th;.45 .03 th];
for j=1:6
 v=([-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1].*dims(j,:))/2;
 v=(rx(angles(j))*v')'+top(j,:)-[0 -th/2*sin(angles(j)) th/2*cos(angles(j))];
 patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],'FaceColor',[.70 .46 .20],'EdgeColor',[.45 .30 .12]);
end
files={'body.stl','right_wheel.stl','left_wheel.stl'};colors={[.15 .43 .68],[.16 .17 .18],[.16 .17 .18]};mesh=cell(1,3);handles=gobjects(1,3);Rvisual=[0 1 0;-1 0 0;0 0 1];
for j=1:3
 tri=stlread(fullfile(paths.root,'模型','仿真模型','通用URDF','meshes',files{j}));[faces,vertices]=reducepatch(tri.ConnectivityList,tri.Points,.15);mesh{j}=(Rvisual*vertices')';
 handles(j)=patch(ax,'Faces',faces,'Vertices',mesh{j},'FaceColor',colors{j},'EdgeColor','none','FaceLighting','gouraud');
end
camlight(ax,'headlight');material(ax,'dull');
map=axes(fig,'Position',[.70 .58 .27 .31]);plot(map,state(:,4),state(:,2),'Color',[.7 .7 .7]);hold(map,'on');point=plot(map,state(1,4),state(1,2),'o','MarkerFaceColor',[.1 .45 .8]);grid(map,'on');xlabel(map,'前向世界 Y / m');ylabel(map,'横向世界 X / m');title(map,'整段真实轨迹');
vax=axes(fig,'Position',[.70 .24 .27 .25]);plot(vax,tf,speed,tc,c(:,10),'--');hold(vax,'on');cursor=xline(vax,0,'r');grid(vax,'on');xlabel(vax,'时间 / s');ylabel(vax,'速度 / m/s');legend(vax,'机械真值','指令');
label=uicontrol(fig,'Style','text','Units','normalized','Position',[.04 .935 .92 .04],'BackgroundColor','w','HorizontalAlignment','left','FontSize',13);
uicontrol(fig,'Style','text','Units','normalized','Position',[.04 .01 .92 .035],'BackgroundColor','w','String',sprintf('已保存的完整Multibody结果回放；PID / %s / 通过=%d。仅仿真，未提高实物限流。',r.operating_envelope,r.pass),'FontSize',12);
play=uicontrol(fig,'Style','togglebutton','String','播放','Value',0,'Units','normalized','Position',[.04 .075 .09 .055],'FontSize',12,'Callback',@toggle);
slider=uicontrol(fig,'Style','slider','Min',0,'Max',tf(end),'Value',0,'Units','normalized','Position',[.16 .085 .80 .035],'Callback',@seek);
baseTime=0;started=tic;update(1);
 function update(k)
  qq=state(k,8:11);qq=qq/norm(qq);ww=qq(1);x=qq(2);y=qq(3);z=qq(4);
  R=[1-2*(y*y+z*z) 2*(x*y-ww*z) 2*(x*z+ww*y);2*(x*y+ww*z) 1-2*(x*x+z*z) 2*(y*z-ww*x);2*(x*z-ww*y) 2*(y*z+ww*x) 1-2*(x*x+y*y)];pos=state(k,[2 4 6]);
  handles(1).Vertices=(R*mesh{1}')'+pos;
  for m=1:2,offset=[(2*m-3)*track/2 0 0];handles(m+1).Vertices=(R*((rx(-wq(k,m))*mesh{m+1}')+offset'))'+pos;end
  xlim(ax,pos(1)+[-.27 .27]);ylim(ax,pos(2)+[-.35 .42]);zlim(ax,[max(0,pos(3)-.08) max(.30,pos(3)+.22)]);
  point.XData=pos(2);point.YData=pos(1);cursor.Value=tf(k);slider.Value=tf(k);
  label.String=sprintf('t = %.2f / %.1f s     真实速度 = %.2f m/s     真实俯仰 = %.1f°     坡度12° / 交错凸起 / 倒车转向 / 推扰',tf(k),tf(end),speed(k),rad2deg(pitch(k)));drawnow limitrate;
 end
 function toggle(~,~)
  if ~play.Value,play.String='播放';return;end
  baseTime=slider.Value;started=tic;play.String='暂停';
  while isgraphics(fig)&&play.Value
   now=mod(baseTime+toc(started),tf(end));[~,k]=min(abs(tf-now));update(k);pause(.04);
  end
 end
 function seek(~,~),baseTime=slider.Value;started=tic;[~,k]=min(abs(tf-baseTime));update(k);end
 function closeReplay(~,~),delete(fig);end
end
function R=rx(a),R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function [t,v]=data(ts),t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end;[t,i]=unique(t,'stable');v=v(i,:);end
