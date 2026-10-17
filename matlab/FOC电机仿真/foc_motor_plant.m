function next=foc_motor_plant(x,u,ref,cfg)
%#codegen
% x=[id iq omega_mechanical theta_mechanical Ein Ecu Eload]。
% 一个控制周期内保持逆变器alpha/beta平均电压，RK4积分电磁与机械方程。
next=x;h=cfg(9)/cfg(14);
for k=1:round(cfg(14))
 a=derivative(next,u,ref,cfg);
 b=derivative(next+h*a/2,u,ref,cfg);
 c=derivative(next+h*b/2,u,ref,cfg);
 d=derivative(next+h*c,u,ref,cfg);
 next=next+h*(a+2*b+2*c+d)/6;
end
end
function dx=derivative(x,u,ref,cfg)
p=cfg(1);R=cfg(2);Ld=cfg(3);Lq=cfg(4);flux=cfg(5);
id=x(1);iq=x(2);w=x(3);theta=p*x(4);we=p*w;
vd=cos(theta)*u(1)+sin(theta)*u(2);
vq=-sin(theta)*u(1)+cos(theta)*u(2);
torque=1.5*p*(flux*iq+(Ld-Lq)*id*iq);
dx=zeros(7,1);
dx(1)=(vd-R*id+we*Lq*iq)/Ld;
dx(2)=(vq-R*iq-we*(Ld*id+flux))/Lq;
if ref(4)<.5
 dx(3)=(torque-ref(3)-cfg(7)*w)/cfg(6);dx(4)=w;
end
dx(5)=1.5*(vd*id+vq*iq);
dx(6)=1.5*R*(id*id+iq*iq);
dx(7)=(ref(3)+cfg(7)*w)*w;
end
