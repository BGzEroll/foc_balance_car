function out=measured_vehicle_actuator(mechanics,iq,t,cfg)
%#codegen
% Mechanics are supplied by Multibody. Do not integrate wheel inertia here.
persistent x
if isempty(x),x=measured_foc_initial(mechanics(1:2));end
q=mechanics(1:2);w=mechanics(3:4);refs=[0 0;iq(1) iq(2)];tick=round(t/cfg(6));torque=zeros(2,1);steps=round(cfg(31));y=zeros(14,2);
for k=0:steps-1
 [x,y]=measured_foc_step(x,refs,q+w*(k*cfg(6)),w,tick+k,cfg);torque=torque+y(1,:)';
end
friction=[cfg(17);cfg(18)].*w+[cfg(19);cfg(20)].*tanh(w/.5);
out=[torque/steps-friction;y(:)];
end
