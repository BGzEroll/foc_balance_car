function x=measured_foc_initial(q)
% State has electrical, PI, PWM-delay, encoder-history states only; no inertia.
x=zeros(37,2);
for m=1:2,x(23:30,m)=q(m);x(31,m)=q(m);x(33,m)=4199+1009*m;end
end
