"""Read current CAD/model inputs, export bench load and speed references."""
from pathlib import Path
import json,math
r=Path(__file__).resolve().parent;repo=r.parents[2]
j=json.loads((repo/'模型/参数与图片/机械参数.json').read_text(encoding='utf-8'))
p=j['balance_parameters'];radius=p['wheel_radius_m'];kt=1.5*60/(2*math.pi*math.sqrt(3)*700)
out={'source':'模型/参数与图片/机械参数.json','radius_m':radius,'vehicle_mass_kg':j['mass_kg']['total'],'loaded_MB_rotating_mass_kg':j['mass_kg']['right_drive'],'loaded_MB_axial_inertia_kg_m2':p['right_drive_J_axis_kg_m2'],'bare_MA_note':'Wheel and tyre absent; do not apply the complete wheel-group inertia to MA.','velocity_model_source':'matlab/3D仿真/pid_balance_parameters.m','normal_reference_m_s':.35,'agile_reference_m_s':.9,'normal_omega_rad_s':.35/radius,'agile_omega_rad_s':.9/radius,'candidate_steps':[{'omega_rad_s':w,'equivalent_ground_m_s':radius*w,'rpm':w*60/(2*math.pi)} for w in [5,10,15,20,27.5]],'nominal_Kt_Nm_A':kt,'acceleration_estimate_0_to_27p5_in_0p65s':{'alpha_rad_s2':27.5/.65,'inertia_torque_Nm':p['right_drive_J_axis_kg_m2']*27.5/.65,'inertia_current_A_nominal':p['right_drive_J_axis_kg_m2']*27.5/.65/kt},'limits':'CAD inertia and nominal KV are estimates, not identified inertia/Kt or certified vehicle speed. Suspended wheel tests exclude rolling load, chassis translation and balance.'}
(r/'model_reference.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(out,ensure_ascii=False))
