function d=design_balance_suite
% 六状态：[纵向路程 纵向速度 俯仰偏差 俯仰角速度 偏航 偏航角速度]。
here=fileparts(mfilename('fullpath'));
src=jsondecode(fileread(fullfile(here,'..','模型','参数与图片','机械参数.json')));
p=src.balance_parameters; M=src.mass_kg.body;m=src.mass_kg.left_drive;
r=p.wheel_radius_m;l=p.body_com_distance_to_axle_m;I=p.body_Ixx_at_com_kg_m2;J=p.left_drive_J_axis_kg_m2;
D=[M+2*m+2*J/r^2 M*l;M*l I+M*l^2];g=D\[0;M*9.80665*l];b=D\[1/r;-1];
A=zeros(6); A(1,2)=1;A(2,3)=g(1);A(3,4)=1;A(4,3)=g(2);A(5,6)=1;
B=zeros(6,2);B([2 4],1)=b;B(6,2)=p.contact_track_m/(2*r*p.effective_inplace_yaw_kg_m2);
Ts=.005;E=expm([A B;zeros(2,8)]*Ts);Ad=E(1:6,1:6);Bd=E(1:6,7:8);
Q=diag([100 4 1/deg2rad(8)^2 .25 1/deg2rad(20)^2 .2]);R=diag([625 625]);
assert(rank(ctrb(Ad,Bd))==6,'Six-state model not controllable');
K=dlqr(Ad,Bd,Q,R);
C=zeros(2,6);C(1,1)=1;C(2,5)=1;
Ai=[Ad zeros(6,2);-Ts*C eye(2)];Bi=[Bd;zeros(2)];
Ki=dlqr(Ai,Bi,blkdiag(Q,diag([300 15])),R);
assert(max(abs(eig(Ai-Bi*Ki)))<1,'LQI not stabilizable');
d=struct('Ts',Ts,'K',K,'Ki',Ki,'A',A,'B',B,'Ad',Ad,'Bd',Bd, ...
 'pitch_trim',atan2(-p.body_com_forward_offset_m,p.body_com_height_above_axle_m), ...
 'torque_limit_each',.10,'source',src.mass_kg,'track',p.contact_track_m,'radius',r);
% cfg=[mode scenario limit trim K(:)' Ki(:)']; mode=0 LQR, mode=1 LQI.
d.config=[0 1 d.torque_limit_each d.pitch_trim K(:)' Ki(:)'];
end
