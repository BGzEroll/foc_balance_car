% CAD几何＋实测质量初步估算，SI单位；非辨识结果。
M_body_kg = 0.2771;
m_drive_each_kg = 0.0627;
M_total_kg = 0.4025;
r_m = 0.0325;
track_m = 0.1325364;
h_m = 0.03652922742030107;
com_forward_offset_m = 5.7139359606250176e-05;
l_m = 0.03652927210923442;
I_body_com_x_kgm2 = 0.00031184766836104503;
I_body_axle_x_kgm2 = 0.0006816065058031743;
J_drive_axis_kgm2 = 2.996788998409844e-05;
g_mps2 = 9.80665;
rim_mass_kg = 0.0203;
tyre_mass_kg = 0.0203;
rotor_mass_kg = 0.0212;
stator_mass_kg = 0.0342;
tyre_wall_equivalent_m = 0.0082;

% YAW_PARAMETERS
% 保留CAD轴：X沿轮轴，车前为-Y，Z向上；前向偏移量已按USB侧为前换算；O为轮轴中点。直立、无滑动的原地差速转向。
% effective_inplace_yaw 已含轮组自转折算项；显式建模两轮自转时不要重复加入。
whole_Izz_at_com_kg_m2 = 0.0009545634717723205;
whole_Izz_at_axle_midpoint_kg_m2 = 0.0009545644863429122;
wheel_spin_yaw_addition_kg_m2 = 0.00024918952828703925;
effective_inplace_yaw_kg_m2 = 0.0012037540146299514;
