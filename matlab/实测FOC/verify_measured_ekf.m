function r=verify_measured_ekf
% Independent observation-Jacobian, convergence, covariance and yaw checks.
dt=.005;p=measured_ekf_parameters;P0=diag([ones(1,3)*p.initial_attitude_std_rad^2 ones(1,3)*p.initial_bias_std_rad_s^2]);
q=[cos(.3);sin(.3)*[1;2;3]/sqrt(14)];g=gravity(q);H=[0 -g(3) g(2);g(3) 0 -g(1);-g(2) g(1) 0];J=zeros(3);
for j=1:3
 v=zeros(3,1);v(j)=1e-6;dq=[cos(norm(v)/2);v*sin(norm(v)/2)/norm(v)];
 qq=[q(1)*dq(1)-dot(q(2:4),dq(2:4));q(1)*dq(2:4)+dq(1)*q(2:4)+cross(q(2:4),dq(2:4))];
 J(:,j)=(gravity(qq)-g)/1e-6;
end
r.jacobian_max_error=max(abs(J-H),[],'all');assert(r.jacobian_max_error<1e-6);
q=[cos(4*pi/180);sin(4*pi/180);0;0];b=zeros(3,1);P=P0;truthBias=[.003;-.002;.001];minEigen=inf;normError=0;
for k=1:6000
 [q,b,P]=measured_ekf_step(q,b,P,truthBias,[0;0;1],dt,1);
 minEigen=min(minEigen,min(eig(P)));normError=max(normError,abs(norm(q)-1));
end
r.static_final_tilt_deg=acosd(min(1,gravity(q)'*[0;0;1]));r.bias_xy_error_rad_s=norm(b(1:2)-truthBias(1:2));
r.min_covariance_eigenvalue=minEigen;r.max_norm_error=normError;
assert(r.static_final_tilt_deg<.3&&r.bias_xy_error_rad_s<.001&&minEigen>0&&normError<1e-12,'EKF static convergence failed');
q=[1;0;0;0];b=zeros(3,1);P=P0;
for k=1:1000,[q,b,P]=measured_ekf_step(q,b,P,[0;0;.01],[0;0;1],dt,1);end
yaw=atan2(2*q(1)*q(4),1-2*q(4)^2);r.unobservable_yaw_drift_deg=rad2deg(yaw);r.yaw_bias_estimate_rad_s=b(3);
assert(abs(yaw-.05)<1e-5&&abs(b(3))<1e-8,'EKF must not invent a gravity yaw observation');
[a,~,~]=measured_ekf_step([1;0;0;0],zeros(3,1),P0,[.1;.2;.3],[0;0;2],dt,1);
[bq,~,~]=measured_ekf_step(-[1;0;0;0],zeros(3,1),P0,[.1;.2;.3],[0;0;2],dt,1);
r.quaternion_sign_error=norm(a+bq);assert(r.quaternion_sign_error<1e-12);
r.pass=true;disp(r);
end
function g=gravity(q)
w=q(1);x=q(2);y=q(3);z=q(4);g=[2*(x*z-w*y);2*(y*z+w*x);1-2*(x*x+y*y)];
end
