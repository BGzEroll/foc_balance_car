function dx=measured_plant_rhs(x,iq,m)
% Longitudinal model: x=[travel speed pitch pitch_rate encoder_a encoder_b].
% iq is measured motor-frame [iq_a iq_b], including compensation current.
% Torque from cogging is approximated by the negative deployed feedforward map.
p=m.parameters;v=x(2);theta=x(3);q=x(4);
point=mod(floor(mod(x(5:6),2*pi)*4096/(2*pi)+.5),4096)+1;
net=[-(iq(1)-p.cogging_table_A(point(1),1));iq(2)-p.cogging_table_A(point(2),2)];
tau=p.Kt*m.torque_scale*sum(net);
D=[p.M+2*p.mw+2*p.Jw/p.r^2,p.M*p.l*cos(theta);p.M*p.l*cos(theta),p.I+p.M*p.l^2];
f=[tau/p.r+p.M*p.l*sin(theta)*q*q-m.friction_N*tanh(v/.008)-m.viscous_N_s_m*v+m.constant_force_N;p.M*p.g*p.l*sin(theta)-tau-m.pitch_damping_N_m_s*q];
a=D\f;dx=[v;a(1);q;a(2);-(v/p.r-q);v/p.r-q];
end
