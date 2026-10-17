function u=foc_current_controller(measured,ref,cfg)
%#codegen
% Clarke/Park -> dq PI+解耦 -> 圆形电压限幅 -> SVPWM平均占空比。
persistent integral
if isempty(integral),integral=zeros(2,1);end
alpha=(2*measured(1)-measured(2)-measured(3))/3;
beta=(measured(2)-measured(3))/sqrt(3);
c=cos(measured(4));s=sin(measured(4));
id=c*alpha+s*beta;iq=-s*alpha+c*beta;
iqref=min(max(ref(2),-cfg(13)),cfg(13));error=[ref(1)-id;iqref-iq];
we=cfg(1)*measured(5);
ff=[-we*cfg(4)*iq;we*(cfg(3)*id+cfg(5))];
unsat=[cfg(10)*error(1);cfg(11)*error(2)]+integral+ff;
vmax=cfg(8)/sqrt(3);scale=min(1,vmax/max(norm(unsat),1e-12));vdq=unsat*scale;
% 反算抗积分饱和：回算时间常数采用Ld/Rs。
kaw=cfg(12)/cfg(10);
integral=integral+cfg(9)*(cfg(12)*error+kaw*(vdq-unsat));
va=c*vdq(1)-s*vdq(2);vb=s*vdq(1)+c*vdq(2);
phase=[va;-.5*va+sqrt(3)/2*vb;-.5*va-sqrt(3)/2*vb];
common=-(max(phase)+min(phase))/2;
duty=.5+(phase+common)/cfg(8);duty=min(max(duty,0),1);
% 两电平逆变器的平均相电压，去除浮动中性点共模。
applied=cfg(8)*(duty-sum(duty)/3);
ab=[(2*applied(1)-applied(2)-applied(3))/3;(applied(2)-applied(3))/sqrt(3)];
% u=[v_alpha v_beta vd vq dutyA dutyB dutyC saturated iq_limited]。
u=[ab;vdq;duty;double(scale<1-1e-9);iqref];
end
