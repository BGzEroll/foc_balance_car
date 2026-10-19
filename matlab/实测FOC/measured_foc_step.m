function [next,y]=measured_foc_step(x,reference,q,w,tick,cfg)
%#codegen
% cfg: R L ke Kp Ki dt controlTicks delayTicks bus ratio cap Imax noise
%      counts encoderDelayTicks polePairs B_L B_R Tc_L Tc_R
% Per motor y=[tau_em id iq id_meas iq_meas vd vq vd_cmd vq_cmd sat limit iq_ref encoder_w encoder_q].
next=x;y=zeros(14,2);h=cfg(6);period=round(cfg(7));delay=max(1,min(8,round(cfg(8))));
edelay=max(1,min(8,round(cfg(15))));vmax=min(cfg(11),cfg(10)*cfg(9));p=cfg(16);
for m=1:2
 ab=x(5:6,m);
 for k=1:delay-1,next(3+2*k:4+2*k,m)=x(5+2*k:6+2*k,m);end
 next(23,m)=q(m);next(24:30,m)=x(23:29,m);
 offset=(m-1)*floor(period/2);
 if mod(tick-offset,period)==0
  seed=mod(1664525*x(33,m)+1013904223,4294967296);u=(seed+1)/4294967297;
  seed=mod(1664525*seed+1013904223,4294967296);vnoise=(seed+1)/4294967297;
  z=sqrt(-2*log(u));zAB=[z*cos(2*pi*vnoise);z*sin(2*pi*vnoise)];
  seed=mod(1664525*seed+1013904223,4294967296);u2=(seed+1)/4294967297;
  seed=mod(1664525*seed+1013904223,4294967296);v2=(seed+1)/4294967297;next(33,m)=seed;
  encnoise=cfg(28+m)*sqrt(-2*log(u2))*cos(2*pi*v2);
  qt=round(x(22+edelay,m)*cfg(14)/(2*pi)+encnoise)*2*pi/cfg(14);
  Tc=h*period;v=(qt-x(31,m))/Tc;
  vm=(.003*x(32,m)+(qt-x(31,m)))/(.003+Tc);next(31,m)=qt;next(32,m)=vm;
  estimated=p*(qt+vm*edelay*h);trueTheta=p*q(m);
  alpha=cos(trueTheta)*x(1,m)-sin(trueTheta)*x(2,m);
  beta=sin(trueTheta)*x(1,m)+cos(trueTheta)*x(2,m);
  ia=alpha;ib=-.5*alpha+sqrt(3)/2*beta;
  Lnoise=reshape(cfg(21+4*(m-1):24+4*(m-1)),2,2);noise=Lnoise*zAB;ia=ia+noise(1);ib=ib+noise(2);
  alpha=ia;beta=(ia+2*ib)/sqrt(3);
  measured=[cos(estimated)*alpha+sin(estimated)*beta;-sin(estimated)*alpha+cos(estimated)*beta];
  ref=reference(:,m);ref(2)=max(-cfg(12),min(cfg(12),ref(2)));
  error=ref-measured;candidate=x(3:4,m)+cfg(5)*Tc*error;voltage=cfg(4)*error+candidate;
  mag=norm(voltage);sat=mag>vmax;
  if sat,voltage=voltage*vmax/mag;next(3:4,m)=voltage-cfg(4)*error;else,next(3:4,m)=candidate;end
  vab=[cos(estimated)*voltage(1)-sin(estimated)*voltage(2);sin(estimated)*voltage(1)+cos(estimated)*voltage(2)];
  phase=[vab(1);-.5*vab(1)+sqrt(3)/2*vab(2);-.5*vab(1)-sqrt(3)/2*vab(2)];
  common=-(max(phase)+min(phase))/2;cmp=round(4000*(.5+(phase+common)/cfg(9)));cmp=max(320,min(3680,cmp));
  phase=cfg(9)*(cmp-mean(cmp))/4000;next(21:22,m)=[phase(1);(phase(2)-phase(3))/sqrt(3)];
  next(34:35,m)=measured;next(36,m)=double(sat);next(37,m)=ref(2);
 end
 next(3+2*delay:4+2*delay,m)=next(21:22,m);
 current=x(1:2,m);
 % Small RK4 electrical substeps; shaft q/w are imposed by the mechanics.
 for k=1:2
  sub=h/2;theta=p*(q(m)+w(m)*(k-1)*sub);
  a=rhs(current,ab,theta,w(m),cfg);
  b=rhs(current+sub*a/2,ab,theta+p*w(m)*sub/2,w(m),cfg);
  c=rhs(current+sub*b/2,ab,theta+p*w(m)*sub/2,w(m),cfg);
  d=rhs(current+sub*c,ab,theta+p*w(m)*sub,w(m),cfg);
  current=current+sub*(a+2*b+2*c+d)/6;
 end
 next(1:2,m)=current;theta=p*q(m);vdq=[cos(theta)*ab(1)+sin(theta)*ab(2);-sin(theta)*ab(1)+cos(theta)*ab(2)];
 cmd=next(21:22,m);cmd_dq=[cos(theta)*cmd(1)+sin(theta)*cmd(2);-sin(theta)*cmd(1)+cos(theta)*cmd(2)];
 y(:,m)=[1.5*cfg(3)*current(2);current;next(34:35,m);vdq;cmd_dq;next(36,m);vmax;next(37,m);next(32,m);next(31,m)];
end
end
function d=rhs(i,ab,theta,w,cfg)
vd=cos(theta)*ab(1)+sin(theta)*ab(2);vq=-sin(theta)*ab(1)+cos(theta)*ab(2);we=cfg(16)*w;
d=[(vd-cfg(1)*i(1)+we*cfg(2)*i(2))/cfg(2);(vq-cfg(1)*i(2)-we*cfg(2)*i(1)-cfg(3)*w)/cfg(2)];
end
