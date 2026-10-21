function [ref,F,T]=measured_car_reference(t,scenario,moves)
%#codegen
ref=zeros(6,1);F=zeros(3,1);T=zeros(3,1);
if scenario==10
 for k=1:size(moves,1)
  [s,v]=smooth(t,moves(k,1),moves(k,2),moves(k,3));
  [a,w]=smooth(t,moves(k,1),moves(k,2),moves(k,4));
  ref=ref+[s;v;0;0;a;w];
 end
 if t>=36.2&&t<36.35,F(2)=.4;end
 if t>=36.7&&t<36.85,T(3)=.002;end
elseif scenario==11
 [ref(1),ref(2)]=smooth(t,moves(1,1),moves(1,2),moves(1,3));
elseif scenario==12 % Straight, left turn, hold, right return; flat ground.
 [ref(1),ref(2)]=smooth(t,1,11,.6);
 [a,b]=smooth(t,4,2,pi/6);[c,d]=smooth(t,9,2,-pi/6);ref(5:6)=[a+c;b+d];
elseif scenario==2
 [a,b]=smooth(t,1,3,.3);[c,d]=smooth(t,6,2,-.2);ref(1:2)=[a+c;b+d];
elseif scenario==3
 [a,b]=smooth(t,1,2,pi/3);[c,d]=smooth(t,5,2,-2*pi/9);ref(5:6)=[a+c;b+d];
elseif scenario==4
 [ref(1),ref(2)]=smooth(t,1,6,.6);[ref(5),ref(6)]=smooth(t,2,3,pi/3);
elseif scenario==5
 if t>=2&&t<2.15,F(2)=.4;end
 if t>=4&&t<4.15,T(3)=.002;end
elseif scenario==6
 [ref(1),ref(2)]=smooth(t,1,8,.75);
elseif scenario==7
 if t>=2,F(2)=.08;T(3)=.0004;end
elseif scenario==8
 [ref(1),ref(2)]=smooth(t,1,20,1.95);
elseif scenario==9
 [a,b]=smooth(t,1,6,.55);[c,d]=smooth(t,19,11,1.4);ref(1:2)=[a+c;b+d];
end
end
function [s,v]=smooth(t,start,duration,amplitude)
u=max(0,min(1,(t-start)/duration));s=amplitude*(10*u^3-15*u^4+6*u^5);
v=0;if t>start&&t<start+duration,v=amplitude*(30*u^2-60*u^3+30*u^4)/duration;end
end
