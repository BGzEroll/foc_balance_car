function d=solve_balance_lqr(p,Q,R,Ts,torqueLimit)
% 平衡点线性化，零阶保持精确离散化，再使用dlqr计算K。
positive=[p.body_mass p.wheel_mass p.wheel_radius p.track p.com_height ...
 p.pitch_inertia p.wheel_inertia p.yaw_inertia p.gravity Ts];
assert(all(isfinite(positive))&&all(positive>0),'质量、惯量、尺寸和周期必须为正有限数');
assert(isscalar(torqueLimit)&&torqueLimit>0,'单轮限幅必须为正数或Inf');
assert(isequal(size(Q),[6 6])&&all(isfinite(Q),'all')&&norm(Q-Q','fro')<1e-10&&min(eig(Q))>=0,'Q须为6x6对称半正定矩阵');
assert(isequal(size(R),[2 2])&&all(isfinite(R),'all')&&norm(R-R','fro')<1e-10&&min(eig(R))>0,'R须为2x2对称正定矩阵');
assert(isscalar(p.com_forward)&&isfinite(p.com_forward),'质心前向偏移必须为有限数');
M=p.body_mass;m=p.wheel_mass;r=p.wheel_radius;
l=hypot(p.com_forward,p.com_height);I=p.pitch_inertia;J=p.wheel_inertia;
D=[M+2*m+2*J/r^2 M*l;M*l I+M*l^2];g=D\[0;M*p.gravity*l];b=D\[1/r;-1];
A=zeros(6);A(1,2)=1;A(2,3)=g(1);A(3,4)=1;A(4,3)=g(2);A(5,6)=1;
B=zeros(6,2);B([2 4],1)=b;B(6,2)=p.track/(2*r*p.yaw_inertia);
E=expm([A B;zeros(2,8)]*Ts);Ad=E(1:6,1:6);Bd=E(1:6,7:8);
assert(rank(ctrb(Ad,Bd))==6,'Six-state model not controllable');
K=dlqr(Ad,Bd,Q,R);
assert(max(abs(eig(Ad-Bd*K)))<1,'Discrete LQR not stable');
d=struct('Ts',Ts,'K',K,'A',A,'B',B,'Ad',Ad,'Bd',Bd,'Q',Q,'R',R,'parameters',p, ...
 'pitch_trim',atan2(-p.com_forward,p.com_height),'torque_limit_each',torqueLimit, ...
 'source',struct('body',M,'left_drive',m,'right_drive',m,'total',M+2*m),'track',p.track,'radius',r);
d.config=[1 torqueLimit d.pitch_trim K(:)'];
end
