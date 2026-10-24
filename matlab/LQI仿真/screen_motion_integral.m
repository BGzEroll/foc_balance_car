function screen_motion_integral
% Reproduce the motion-integral hypothesis screen; this is not plant identification.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'esp32','lqi_firmware','results','dynamic');
selected=jsondecode(fileread(fullfile(root,'esp32','lqi_firmware','selected_lqi.json')));
d=hardware_lqi_design;d.current_limit_each=6;d.Kaug=d.Kaug.*selected.scale(:)';
d.K=d.Kaug(:,1:6);d.Ki=d.Kaug(:,7:8);reports=struct([]);
for limit=[inf .04 .08]
 for friction=[.1 .4]
  options=struct('duration',65,'current_lag_s',.003,'delay_cycles',2,...
   'stiction',true,'rolling_coulomb_N',friction,'motion_integral_delta_limit',limit,...
   'motion_integral_return_tau',inf,'motion_reversal_reset',true);
  z=simulate_lqi(d,'bounded_drive',options);c=z.report;
  c.tracking_rms_m=sqrt(mean((z.state(:,1)-z.reference(:,1)).^2));
  if isempty(reports),reports=c;else,reports(end+1)=c;end
  fprintf('INTEGRAL limit=%g friction=%g rms=%.5f pitch=%.3f tail=%.5f fell=%d\n',...
   limit,friction,c.tracking_rms_m,c.peak_pitch_deg,c.tail_position_error_m,c.fell);
 end
end
assert(~any([reports.fell]));
fid=fopen(fullfile(out,'integral_reversal_screen.json'),'w');
fprintf(fid,'%s',jsonencode(reports,PrettyPrint=true));fclose(fid);
end
