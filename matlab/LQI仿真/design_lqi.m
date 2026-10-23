function d=design_lqi(integralScale)
% 状态 [路程 速度 俯仰偏差 俯仰角速度 偏航 偏航角速度]；输入[公共电流 差分电流] A。
if nargin<1,integralScale=1;end
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
j=jsondecode(fileread(fullfile(root,'模型','参数与图片','机械参数.json')));b=j.balance_parameters;
p=struct('body_mass',j.mass_kg.body,'wheel_mass',j.mass_kg.left_drive,'wheel_radius',b.wheel_radius_m,...
 'track',b.contact_track_m,'com_height',b.body_com_height_above_axle_m,'com_forward',b.body_com_forward_offset_m,...
 'pitch_inertia',b.body_Ixx_at_com_kg_m2,'wheel_inertia',b.left_drive_J_axis_kg_m2,...
 'yaw_inertia',b.effective_inplace_yaw_kg_m2,'gravity',9.80665);
M=p.body_mass;r=p.wheel_radius;l=hypot(p.com_height,p.com_forward);
D=[M+2*p.wheel_mass+2*p.wheel_inertia/r^2 M*l;M*l p.pitch_inertia+M*l^2];
g=D\[0;M*p.gravity*l];v=D\[1/r;-1];Kt=1.5*60/(2*pi*sqrt(3)*700);
A=zeros(6);A(1,2)=1;A(2,3)=g(1);A(3,4)=1;A(4,3)=g(2);A(5,6)=1;
B=zeros(6,2);B([2 4],1)=v*Kt;B(6,2)=p.track/(2*r*p.yaw_inertia)*Kt;
Ts=.005;E=expm([A B;zeros(2,8)]*Ts);Ad=E(1:6,1:6);Bd=E(1:6,7:8);
C=zeros(2,6);C(1,1)=1;C(2,5)=1;
Aa=[Ad zeros(6,2);Ts*C eye(2)];Ba=[Bd;zeros(2,2)];
Q=diag([156.25 16 44.4444 .25 4 .111111 100*integralScale 4*integralScale]);R=diag([.4 .6]);
assert(rank(ctrb(Aa,Ba))==8,'LQI augmented system is not controllable');
[K,~,pole]=dlqr(Aa,Ba,Q,R);assert(max(abs(pole))<1,'LQI design unstable');
d=struct('Ts',Ts,'A',A,'B',B,'Ad',Ad,'Bd',Bd,'Aa',Aa,'Ba',Ba,'C',C,'Kaug',K,...
 'K',K(:,1:6),'Ki',K(:,7:8),'Q',Q,'R',R,'poles',pole,'integral_scale',integralScale,...
 'parameters',p,'pitch_trim',atan2(-p.com_forward,p.com_height),'current_limit_each',2,...
 'torque_limit_each',2*Kt,'Kt_nominal',Kt,'integral_limit',[.3;1],...
 'current_parameters',struct('Kt_nominal',Kt,'current_limit_each',2,'ideal_current_tracking',true),...
 'scope','CAD mechanics + nominal 700KV Kt. Current gain/lag/noise uncertainties are screened; not a validated hardware plant.');
end
