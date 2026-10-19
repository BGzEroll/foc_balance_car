function d=measured_course_definition
% One continuous route; all rows are [start duration delta_distance delta_yaw].
d.name='综合性能场景';d.duration=39;
d.moves=[1 3.8 3 0;5 2 .1 0;7.5 3 .45 0;14.5 6 1.6 0; ...
 21.5 2 -.05 0;24 2 0 pi/4;27 3 .4 -pi/3;31 2 0 pi/12;34 2 .1 0];
d.labels={'快速起步/制动','低速爬行','缓入坡道','坡中停车','坡顶/下坡/交错凸起','倒车','原地转向','弧线运动','回正','慢速停车','推扰恢复'};
d.phases=[1 4.8;5 7;7.5 10.5;10.5 14.5;14.5 20.5;21.5 23.5;24 26;27 30;31 33;34 36;36.2 39];
d.fast_reference_peak_mps=1.875*3/3.8;d.slow_reference_peak_mps=1.875*.1/2;
d.slope_deg=12;d.ramp_start=3.3;d.ramp_run=.5;d.height=.5*tand(12);d.platform_length=.25;
d.bump_start=4.75;d.bump_length=.035;d.bump_height=.003;
d.right_bump_start=4.90;d.right_bump_height=.002;
d.full_bump_start=5.01;d.full_bump_height=.001;
d.hold_y=3.55;d.finish_distance=5.60;d.hold_check=[12 14.3];
d.notes='Simulation performance exploration. Fixed-voltage battery, rigid tires, empirical hanging noise; no thermal or hardware rating claim.';
end
