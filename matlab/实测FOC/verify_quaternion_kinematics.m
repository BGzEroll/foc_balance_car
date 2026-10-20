function result=verify_quaternion_kinematics(out)
% Prescribed 3-axis motion isolates kinematics from wheel/controller dynamics.
dt=.005;t=(0:dt:20)';n=numel(t);R=zeros(3,3,n);truth=zeros(n,4);
for k=1:n
 roll=.35*sin(.8*t(k));pitch=.55*sin(.6*t(k));yaw=.7*t(k);
 cr=cos(roll/2);sr=sin(roll/2);cp=cos(pitch/2);sp=sin(pitch/2);cy=cos(yaw/2);sy=sin(yaw/2);
 q=[cr*cp*cy;sr*cp*cy;cr*sp*cy;cr*cp*sy]+[sr*sp*sy;-cr*sp*sy;sr*cp*sy;-sr*sp*cy];
 truth(k,:)=q';R(:,:,k)=rot(q);
end
q=truth(1,:)';angles=zeros(3,1);qe=zeros(n,1);ee=qe;normError=qe;
for k=2:n
 % Body angular velocity from the exact quaternion increment.
 qa=truth(k-1,:)';qb=truth(k,:)';dq=[dot(qa,qb);qa(1)*qb(2:4)-qb(1)*qa(2:4)-cross(qa(2:4),qb(2:4))];
 if dq(1)<0,dq=-dq;end
 v=norm(dq(2:4));gyro=zeros(3,1);if v>1e-12,gyro=dq(2:4)*(2*atan2(v,dq(1))/(dt*v));end
 acc=R(:,:,k-1)'*[0;0;1];q=measured_quaternion_step(q,gyro,acc,dt,4,1);
 ap=atan2(acc(2),acc(3));ar=atan2(-acc(1),hypot(acc(2),acc(3)));
 angles(1)=.98*(angles(1)+gyro(1)*dt)+.02*ap;angles(2)=.98*(angles(2)+gyro(2)*dt)+.02*ar;angles(3)=angles(3)+gyro(3)*dt;
 a=angles/2;cx=cos(a(1));sx=sin(a(1));cy=cos(a(2));sy=sin(a(2));cz=cos(a(3));sz=sin(a(3));
 old=[cx*cy*cz+sx*sy*sz;sx*cy*cz-cx*sy*sz;cx*sy*cz+sx*cy*sz;cx*cy*sz-sx*sy*cz];
 qe(k)=2*acos(min(1,abs(dot(q,qb))));ee(k)=2*acos(min(1,abs(dot(old,qb))));normError(k)=abs(norm(q)-1);
end
result=struct('quaternion_max_error_deg',rad2deg(max(qe)),'legacy_max_error_deg',rad2deg(max(ee)), ...
 'max_unit_norm_error',max(normError),'note','Prescribed ideal three-axis IMU motion; not closed-loop car performance');
assert(result.quaternion_max_error_deg<.05&&result.max_unit_norm_error<1e-12,'Quaternion kinematics failed');
f=figure('Visible','off','Color','w');plot(t,rad2deg(ee),t,rad2deg(qe),'LineWidth',1.2);grid on;
xlabel('时间 / s');ylabel('姿态整体旋转误差 / °');legend('当前欧拉角互补','四元数互补');title('理想多轴运动：解算运动学对照');
exportgraphics(f,fullfile(out,'kinematics.png'),'Resolution',150);savefig(f,fullfile(out,'kinematics.fig'));close(f);
end
function R=rot(q)
w=q(1);x=q(2);y=q(3);z=q(4);R=[1-2*(y*y+z*z) 2*(x*y-w*z) 2*(x*z+w*y);2*(x*y+w*z) 1-2*(x*x+z*z) 2*(y*z-w*x);2*(x*z-w*y) 2*(y*z+w*x) 1-2*(x*x+y*y)];
end
