function dx=net_plant_rhs(x,net_iq,m)
% Motor-frame net current after subtracting deployed cogging compensation.
p=m.parameters;point=mod(floor(mod(x(5:6),2*pi)*4096/(2*pi)+.5),4096)+1;
iq=net_iq+[p.cogging_table_A(point(1),1);p.cogging_table_A(point(2),2)];
dx=measured_plant_rhs(x,iq,m);
end
