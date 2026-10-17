function ref=foc_reference(t,cfg)
%#codegen
% ref=[id指令 iq原始指令 外负载转矩 锁轴标志]。
iq=0;load=0;locked=0;scenario=round(cfg(15));
if scenario==1
 locked=1;
 if t>=.02&&t<.10,iq=2;
 elseif t>=.10&&t<.18,iq=4;
 elseif t>=.18&&t<.26,iq=-3;
 elseif t>=.26&&t<.34,iq=8;end
elseif scenario==2
 if t>=.02&&t<.20,iq=3;
 elseif t>=.20&&t<.60,iq=-3;end
elseif scenario==3
 if t>=.02,iq=2;end
 if t>=.15&&t<.35,load=.020;end
else
 if t>=.02,iq=6;end
end
ref=[0;iq;load;locked];
end
