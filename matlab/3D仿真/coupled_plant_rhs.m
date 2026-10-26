function dx=coupled_plant_rhs(x,net_iq,m)
dx=net_plant_rhs(x,net_iq,m);p=m.parameters;v=x(2);theta=x(3);q=x(4);relative=v-p.r*q;
torque=-p.r*(m.motor_friction_N*tanh(relative/.008)+m.motor_viscous_N_s_m*relative);
D=[p.M+2*p.mw+2*p.Jw/p.r^2,p.M*p.l*cos(theta);p.M*p.l*cos(theta),p.I+p.M*p.l^2];a=D\[torque/p.r;-torque];dx(2)=dx(2)+a(1);dx(4)=dx(4)+a(2);
end
