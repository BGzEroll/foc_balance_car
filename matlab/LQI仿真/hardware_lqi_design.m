function d=hardware_lqi_design
% Mechanical plant plus the exact exported MCU matrix, not a fresh Riccati design.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
fw=fullfile(root,'esp32','lqi_firmware');
j=jsondecode(fileread(fullfile(fw,'lqi_design.json')));
s=jsondecode(fileread(fullfile(fw,'selected_lqi.json')));
assert(strcmp(j.design_sha256,s.base_design_sha256),'Hardware design identity mismatch');
d=design_lqi;assert(abs(d.Ts-j.Ts)<1e-12);
d.Kaug=j.Kaug;d.K=d.Kaug(:,1:6);d.Ki=d.Kaug(:,7:8);
d.current_limit_each=s.current_limit_each_A;d.integral_limit=j.integral_limit(:);
d.current_parameters.current_limit_each=d.current_limit_each;
d.torque_limit_each=d.current_limit_each*d.Kt_nominal;
d.Q=diag(s.Q);d.R=diag(s.R);d.poles=eig(d.Aa-d.Ba*d.Kaug);
assert(max(abs(d.Kaug.*s.scale(:)'-s.Kaug),[],'all')<1e-6,'Saved and reconstructed hardware matrices differ');
d.design_sha256=j.design_sha256;
d.scope='CAD mechanics with exact exported hardware K; measured column scales remain a separate step.';
end
