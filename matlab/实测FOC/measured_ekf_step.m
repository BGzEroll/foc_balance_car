function [q,bias,P]=measured_ekf_step(q,bias,P,gyro,acc,dt,accScale)
%#codegen
% Multiplicative error-state EKF: nominal body->world quaternion + gyro bias.
% Local/right orientation error: q_true=q_nominal (*) exp(delta_theta/2).
% Covariance state [delta_theta(3); delta_bias(3)], no heading observation.
p=measured_ekf_parameters;q=q/max(norm(q),1e-12);omega=gyro-bias;
q=multiply(q,increment(omega*dt));q=q/norm(q);
F=[-skew(omega) -eye(3);zeros(3) zeros(3)];
Phi=eye(6)+F*dt+.5*(F*F)*dt^2;
Q=diag([ones(1,3)*p.gyro_std_rad_s^2*dt^2 ones(1,3)*p.bias_walk_rad_s_sqrt_s^2*dt]);
P=Phi*P*Phi'+Q;
mag=norm(acc);mismatch=abs(mag/max(accScale,1e-9)-1);
if mag>1e-9 && mismatch<p.acc_norm_gate
 w=q(1);x=q(2);y=q(3);z=q(4);
 gravity=[2*(x*z-w*y);2*(y*z+w*x);1-2*(x*x+y*y)];
 innovation=acc/mag-gravity;
 % Approximate normalized-direction noise with an isotropic conservative R.
 inflation=1+(norm(innovation)/p.innovation_scale)^2+(mismatch/.05)^2;
 R=eye(3)*p.acc_direction_std^2*inflation;
 H=[skew(gravity) zeros(3)];S=H*P*H'+R;K=(P*H')/S;
 dx=K*innovation;
 q=multiply(q,increment(dx(1:3)));q=q/norm(q);bias=bias+dx(4:6);
 A=eye(6)-K*H;P=A*P*A'+K*R*K'; % Joseph covariance update.
 reset=eye(6);reset(1:3,1:3)=eye(3)-.5*skew(dx(1:3));
 P=reset*P*reset'; % Reset covariance into the corrected local tangent frame.
end
P=(P+P')/2+eye(6)*1e-15;
end
function s=skew(v)
s=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end
function dq=increment(v)
a=norm(v);if a>1e-12,dq=[cos(a/2);v*(sin(a/2)/a)];else,dq=[1;v/2];end
end
function q=multiply(a,b)
q=[a(1)*b(1)-dot(a(2:4),b(2:4));a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end
