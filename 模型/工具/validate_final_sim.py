import sys,pathlib,json,math,xml.etree.ElementTree as E
import numpy as np
root=pathlib.Path(__file__).resolve().parents[1];sim=root/'仿真模型'
sys.path.insert(0,str(sim/'MuJoCo'))
from load_model import load_model
import mujoco
out={}
for f in [sim/'通用URDF'/'urdf'/'balance_car.urdf',sim/'ROS2'/'balance_car_description'/'urdf'/'balance_car.urdf']:
 xml=E.parse(f).getroot();links=xml.findall('link');joints=xml.findall('joint')
 assert len(links)==3 and len(joints)==2
 assert all(j.attrib['type']=='continuous' for j in joints)
 mass=sum(float(x.find('inertial/mass').attrib['value']) for x in links);assert abs(mass-.4025)<1e-12
 for x in links:
  mesh=x.find('visual/geometry/mesh').attrib['filename']
  file=(sim/'ROS2'/'balance_car_description'/mesh.split('balance_car_description/')[1]) if mesh.startswith('package:') else (f.parent/mesh)
  assert file.is_file(),str(file)
 out[f.parent.parent.name]={'links':len(links),'continuous_joints':len(joints),'mass_kg':mass,'mesh_paths_passed':True}
E.parse(sim/'Gazebo_SDF'/'model.sdf')
m=load_model();d=mujoco.MjData(m);mujoco.mj_forward(m,d)
assert m.nq==9 and m.nv==8 and m.nu==2
assert abs(sum(m.body_mass)-.4025)<1e-12
qmat=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,d,qmat)
p=json.loads((root/'参数与图片'/'机械参数.json').read_text(encoding='utf-8'))['balance_parameters']
v=np.zeros(m.nv);v[5]=1;v[6]=-p['contact_track_m']/(2*p['wheel_radius_m']);v[7]=-v[6]
effective=float(v@qmat@v)
relative_error=abs(effective-p['effective_inplace_yaw_kg_m2'])/p['effective_inplace_yaw_kg_m2']
assert relative_error<1e-6,(effective,p['effective_inplace_yaw_kg_m2'])
for i in range(1000):mujoco.mj_step(m,d)
assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
assert not np.any([x.number for x in d.warning]),[(x.number,x.lastinfo) for x in d.warning]
out['mujoco']={'version':mujoco.__version__,'nq':m.nq,'nv':m.nv,'actuators':m.nu,'mass_kg':float(sum(m.body_mass)),'yaw_equivalent_from_kinetic_energy_kgm2':effective,'yaw_relative_error':relative_error,'yaw_relative_tolerance':1e-6,'time_s':d.time,'steps':1000,'finite_state':True,'warnings':0,'note':'零转矩步进仅验证数值可运行，不是平衡闭环验证。'}
(root/'参数与图片'/'仿真文件验证.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False,indent=2))
