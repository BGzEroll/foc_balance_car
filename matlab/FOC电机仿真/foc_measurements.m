function measured=foc_measurements(x,cfg)
%#codegen
% 理想相电流与转子角度测量。幅值不变逆Park及逆Clarke。
theta=cfg(1)*x(4);c=cos(theta);s=sin(theta);
alpha=c*x(1)-s*x(2);beta=s*x(1)+c*x(2);
measured=[alpha;-.5*alpha+sqrt(3)/2*beta;-.5*alpha-sqrt(3)/2*beta;theta;x(3)];
end
