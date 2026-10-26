function dx=commanded_plant_rhs(x,command,m)
% Effective command-input model: cogging map is cancelled by ideal feedforward.
p=m.parameters;point=mod(floor(mod(x(5:6),2*pi)*4096/(2*pi)+.5),4096)+1;
iq=[-command(1)+p.cogging_table_A(point(1),1);command(2)+p.cogging_table_A(point(2),2)];
dx=measured_plant_rhs(x,iq,m);
end
