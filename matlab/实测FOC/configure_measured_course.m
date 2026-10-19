function terrain=configure_measured_course(model,on)
% Real geometry/contact. Extra contacts are constructed by the builder.
d=measured_course_definition;terrain=balance_terrain_parameters('mild');
terrain.angle_deg=d.slope_deg;terrain.ramp_start=d.ramp_start;terrain.ramp_run=d.ramp_run;terrain.height=d.height;
terrain.platform_length=d.platform_length;terrain.ramp_end=d.ramp_start+d.ramp_run;terrain.platform_end=terrain.ramp_end+d.platform_length;
terrain.downhill_end=terrain.platform_end+d.ramp_run;terrain.bump_start=d.bump_start;terrain.bump_length=d.bump_length;terrain.bump_height=d.bump_height;
terrain.hold_y=d.hold_y;terrain.finish_distance=d.finish_distance;terrain.hold_reference_start=10.5;terrain.hold_reference_end=14.5;terrain.hold_check_start=12;terrain.hold_check_end=14.3;
a=deg2rad(d.slope_deg);thick=.02;track=.1325364;
top=[0 d.ramp_start+d.ramp_run/2 d.height/2;0 terrain.ramp_end+d.platform_length/2 d.height;0 terrain.platform_end+d.ramp_run/2 d.height/2; ...
 -track/2 d.bump_start+d.bump_length/2 d.bump_height;track/2 d.right_bump_start+.0175 d.right_bump_height;0 d.full_bump_start+.015 d.full_bump_height];
angle=[a;0;-a;0;0;0];dims=[.45 d.ramp_run/cos(a) thick;.45 d.platform_length thick;.45 d.ramp_run/cos(a) thick;.06 d.bump_length thick;.06 .035 thick;.45 .03 thick];
for j=1:6
 name=['terrain_' num2str(j)];position=top(j,:)-[0 -thick/2*sin(angle(j)) thick/2*cos(angle(j))];if ~on,position(3)=position(3)-1;end
 set_param([model '/' name '_pose'],'TranslationCartesianOffset',mat2str(position,16),'RotationStandardAxis','+X','RotationAngle',num2str(rad2deg(angle(j)),16));
 set_param([model '/' name],'BrickDimensions',mat2str(dims(j,:),16));
end
set_param([model '/Ground'],'GraphicPlaneWidth','2.5','GraphicPlaneHeight','14');
end
