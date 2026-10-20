function q=measured_quaternion_step(q,gyro,acc,dt,kp,accScale)
%#codegen
% Scalar-first body-to-world quaternion, world X right / Y forward / Z up.
% Mahony-style proportional gravity correction; no magnetic heading reference.
q=q/max(norm(q),1e-12);
w=q(1);x=q(2);y=q(3);z=q(4);
gravity=[2*(x*z-w*y);2*(y*z+w*x);1-2*(x*x+y*y)];
mag=norm(acc);omega=gyro;
if mag>1e-9
 % Reject large non-gravitational specific force; preserve calibrated scale.
 mismatch=abs(mag/max(accScale,1e-6)-1);
 weight=max(0,min(1,1-mismatch/.2));
 omega=gyro+kp*weight*cross(acc/mag,gravity);
end
a=norm(omega)*dt;
if a>1e-12,dq=[cos(a/2);omega*(sin(a/2)/norm(omega))];
else,dq=[1;omega*dt/2];end
q=[q(1)*dq(1)-dot(q(2:4),dq(2:4)); ...
 q(1)*dq(2:4)+dq(1)*q(2:4)+cross(q(2:4),dq(2:4))];
q=q/max(norm(q),1e-12);
end
