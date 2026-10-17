import pathlib,json,math,shutil,xml.etree.ElementTree as E
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[1]
SIM=ROOT/'仿真模型';DATA=ROOT/'参数与图片'
r=json.loads((DATA/'机械参数.json').read_text(encoding='utf-8'));p=r['balance_parameters']
rot=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
def txt(v):return ' '.join('%.12g'%float(x) for x in v)
def save(root,path):
 path.parent.mkdir(parents=True,exist_ok=True);E.indent(root);E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
names=['body','left_wheel','right_wheel'];origins=[np.array([0,0,.0325]),np.array([-p['contact_track_m']/2,0,.0325]),np.array([p['contact_track_m']/2,0,.0325])]
comkeys=['body_com_m','left_drive_com_m','right_drive_com_m'];tkeys=['body_at_com','left_drive_at_com','right_drive_at_com'];mkeys=['body','left_drive','right_drive']
links=[]
for name,origin,ck,tk,mk in zip(names,origins,comkeys,tkeys,mkeys):
 links.append({'name':name,'mass':r['mass_kg'][mk],'com':rot@(np.array(r['coordinates'][ck])-origin),'inertia':rot@np.array(r['inertia_tensors_kg_m2'][tk])@rot.T})
assert abs(sum(x['mass'] for x in links)-.4025)<1e-12
for a in links:assert np.linalg.eigvalsh(a['inertia']).min()>0
b=p['contact_track_m'];radius=p['wheel_radius_m']
def inertial(parent,a,kind='urdf'):
 j=a['inertia'];attrs=dict(ixx=str(j[0,0]),iyy=str(j[1,1]),izz=str(j[2,2]),ixy=str(j[0,1]),ixz=str(j[0,2]),iyz=str(j[1,2]))
 node=E.SubElement(parent,'inertial')
 if kind=='urdf':E.SubElement(node,'origin',xyz=txt(a['com']),rpy='0 0 0');E.SubElement(node,'mass',value=str(a['mass']));E.SubElement(node,'inertia',**attrs)
 else:
  E.SubElement(node,'pose').text=txt(a['com'])+' 0 0 0';E.SubElement(node,'mass').text=str(a['mass']);ij=E.SubElement(node,'inertia')
  for k,v in attrs.items():E.SubElement(ij,k).text=v
 return node
def bodycollisions():
 return [('box',[0,0,.005],[.056,.096,.038]),('box',[0,0,.0713],[.038,.093,.020])]
def makeurdf(path,meshprefix):
 robot=E.Element('robot',name='balance_car_zd2808')
 for i,a in enumerate(links):
  link=E.SubElement(robot,'link',name='base_link' if i==0 else a['name']);inertial(link,a)
  vis=E.SubElement(link,'visual');E.SubElement(vis,'geometry');E.SubElement(vis.find('geometry'),'mesh',filename=meshprefix+a['name']+'.stl',scale='1 1 1')
  mat=E.SubElement(vis,'material',name='body_blue' if i==0 else 'wheel_dark');E.SubElement(mat,'color',rgba='.12 .4 .65 1' if i==0 else '.16 .18 .16 1')
  if i==0:
   for typ,pos,size in bodycollisions():
    co=E.SubElement(link,'collision');E.SubElement(co,'origin',xyz=txt(pos),rpy='0 0 0');E.SubElement(E.SubElement(co,'geometry'),'box',size=txt(size))
  else:
   co=E.SubElement(link,'collision');E.SubElement(co,'origin',xyz='0 0 0',rpy=f'{math.pi/2} 0 0');E.SubElement(E.SubElement(co,'geometry'),'cylinder',radius=str(radius),length='.028')
 for name,sign in [('left_wheel',1),('right_wheel',-1)]:
  joint=E.SubElement(robot,'joint',name=name+'_joint',type='continuous');E.SubElement(joint,'parent',link='base_link');E.SubElement(joint,'child',link=name);E.SubElement(joint,'origin',xyz=txt([0,sign*b/2,0]),rpy='0 0 0');E.SubElement(joint,'axis',xyz='0 1 0')
 save(robot,path)
common=SIM/'通用URDF';makeurdf(common/'urdf'/'balance_car.urdf','../meshes/')
pkg=SIM/'ROS2'/'balance_car_description';(pkg/'meshes').mkdir(parents=True,exist_ok=True)
# Mesh copying is performed after CAD export finishes; all paths are relative/package based.
makeurdf(pkg/'urdf'/'balance_car.urdf','package://balance_car_description/meshes/')
(pkg/'launch').mkdir(parents=True,exist_ok=True)
(pkg/'launch'/'display.launch.py').write_text('''from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    share=Path(get_package_share_directory('balance_car_description'))
    return LaunchDescription([
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(share/'urdf'/'balance_car.urdf').read_text()}]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz'/'balance_car.rviz')])])
''',encoding='utf-8')
(pkg/'rviz').mkdir(exist_ok=True)
(pkg/'rviz'/'balance_car.rviz').write_text('''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: base_link
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Durability Policy: Transient Local
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.05
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 0.4
      Focal Point: {X: 0, Y: 0, Z: 0.03}
''',encoding='utf-8')
(pkg/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.8)
project(balance_car_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
''',encoding='utf-8')
(pkg/'package.xml').write_text('''<?xml version="1.0"?>
<package format="3"><name>balance_car_description</name><version>1.0.0</version><description>Measured-mass ZD2808 two-wheel balance-car description.</description><maintainer email="maintainer@example.invalid">Local model maintainer</maintainer><license>LicenseRef-Source-Models-Not-Specified</license><buildtool_depend>ament_cmake</buildtool_depend><exec_depend>robot_state_publisher</exec_depend><exec_depend>joint_state_publisher_gui</exec_depend><exec_depend>rviz2</exec_depend><exec_depend>ament_index_python</exec_depend><export><build_type>ament_cmake</build_type></export></package>
''',encoding='utf-8')
# Standalone SDF model: no version-specific drive/controller plugin is silently assumed.
sdf=E.Element('sdf',version='1.9');model=E.SubElement(sdf,'model',name='balance_car');E.SubElement(model,'pose').text='0 0 0.0335 0 0 0'
for i,a in enumerate(links):
 li=E.SubElement(model,'link',name='base_link' if i==0 else a['name']);pos=[0,0,0] if i==0 else [0,(1 if i==1 else -1)*b/2,0];E.SubElement(li,'pose').text=txt(pos)+' 0 0 0';inertial(li,a,'sdf')
 vis=E.SubElement(li,'visual',name=a['name']+'_visual');mesh=E.SubElement(E.SubElement(vis,'geometry'),'mesh');E.SubElement(mesh,'uri').text='../通用URDF/meshes/'+a['name']+'.stl'
 if i==0:
  for k,(_,pos,size) in enumerate(bodycollisions()):
   co=E.SubElement(li,'collision',name='body_collision_'+str(k));E.SubElement(co,'pose').text=txt(pos)+' 0 0 0';E.SubElement(E.SubElement(E.SubElement(co,'geometry'),'box'),'size').text=txt(size)
 else:
  co=E.SubElement(li,'collision',name='wheel_collision');E.SubElement(co,'pose').text='0 0 0 1.570796326795 0 0';cy=E.SubElement(E.SubElement(co,'geometry'),'cylinder');E.SubElement(cy,'radius').text=str(radius);E.SubElement(cy,'length').text='.028'
for name,sign in [('left_wheel',1),('right_wheel',-1)]:
 j=E.SubElement(model,'joint',name=name+'_joint',type='revolute');E.SubElement(j,'parent').text='base_link';E.SubElement(j,'child').text=name;ax=E.SubElement(j,'axis');E.SubElement(ax,'xyz').text='0 1 0';lim=E.SubElement(ax,'limit');E.SubElement(lim,'lower').text='-1e16';E.SubElement(lim,'upper').text='1e16'
save(sdf,SIM/'Gazebo_SDF'/'model.sdf')
# MuJoCo physics starter: floating base, two hinge joints, torque actuators, simple contact geometry.
mj=E.Element('mujoco',model='balance_car');E.SubElement(mj,'compiler',angle='radian',meshdir='../通用URDF/meshes',inertiafromgeom='false');E.SubElement(mj,'option',timestep='.001',gravity='0 0 -9.80665',integrator='implicitfast')
default=E.SubElement(mj,'default');E.SubElement(default,'geom',friction='1 .005 .0001',condim='3');E.SubElement(default,'joint',damping='0')
asset=E.SubElement(mj,'asset')
for n in names:E.SubElement(asset,'mesh',name=n,file=n+'.stl')
world=E.SubElement(mj,'worldbody');E.SubElement(world,'light',pos='0 -1 2',dir='0 0 -1');E.SubElement(world,'geom',name='ground',type='plane',size='2 2 .05',rgba='.8 .82 .85 1')
base=E.SubElement(world,'body',name='base_link',pos='0 0 .0335');E.SubElement(base,'freejoint',name='floating_base')
for i,a in enumerate(links):
 bo=base if i==0 else E.SubElement(base,'body',name=a['name'],pos=txt([0,(1 if i==1 else -1)*b/2,0]));j=a['inertia']
 E.SubElement(bo,'inertial',pos=txt(a['com']),mass=str(a['mass']),fullinertia=txt([j[0,0],j[1,1],j[2,2],j[0,1],j[0,2],j[1,2]]))
 E.SubElement(bo,'geom',name=a['name']+'_visual',type='mesh',mesh=a['name'],contype='0',conaffinity='0',group='2',rgba='.15 .45 .65 1' if i==0 else '.15 .18 .15 1')
 if i==0:
  E.SubElement(bo,'site',name='imu',pos='0 0 .03',size='.002')
  for k,(_,pos,size) in enumerate(bodycollisions()):E.SubElement(bo,'geom',name='body_contact_'+str(k),type='box',pos=txt(pos),size=txt(np.array(size)/2),group='3',rgba='.3 .5 .8 .15')
 else:
  E.SubElement(bo,'joint',name=a['name']+'_joint',type='hinge',axis='0 1 0')
  E.SubElement(bo,'geom',name=a['name']+'_contact',type='cylinder',size=f'{radius} .014',quat='.707106781187 .707106781187 0 0',group='3',rgba='.2 .2 .2 .15')
act=E.SubElement(mj,'actuator')
for n in ['left_wheel','right_wheel']:E.SubElement(act,'motor',name=n+'_torque',joint=n+'_joint',gear='1')
sensor=E.SubElement(mj,'sensor');E.SubElement(sensor,'gyro',name='body_gyro',site='imu');E.SubElement(sensor,'accelerometer',name='body_accel',site='imu')
save(mj,SIM/'MuJoCo'/'balance_car.xml')
(SIM/'MuJoCo'/'run_demo.py').write_text('''"""Direct joint torque input in N*m; no balance controller is included."""
from pathlib import Path
import time
import mujoco
import mujoco.viewer
model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('balance_car.xml')))
data=mujoco.MjData(model)
with mujoco.viewer.launch_passive(model,data) as viewer:
    while viewer.is_running():
        start=time.perf_counter()
        data.ctrl[:]=0.0  # Replace with controller/policy outputs: left and right torque.
        mujoco.mj_step(model,data)
        viewer.sync()
        time.sleep(max(0,model.opt.timestep-(time.perf_counter()-start)))
''',encoding='utf-8')
(SIM/'MATLAB').mkdir(exist_ok=True)
(SIM/'MATLAB'/'打开模型.m').write_text('''% Three rigid bodies + two wheel joints. Geometry is in metres, inertia in kg*m^2.
here = fileparts(mfilename('fullpath'));
urdfFile = fullfile(here,'..','通用URDF','urdf','balance_car.urdf');
meshDir = fullfile(here,'..','通用URDF','meshes');
robot = importrobot(urdfFile,'MeshPath',meshDir,'DataFormat','column');
robot.Gravity = [0 0 -9.80665];
figure('Name','Balance car final'); show(robot,'Visuals','on','Collisions','off'); axis equal; view(135,20);
% Robot is fixed at the root in rigidBodyTree; this preview is not a floating-base contact simulation.
% To create a Simscape Multibody model (requires its toolbox), run:
% addpath(meshDir); smimport(urdfFile,'ModelName','balance_car_imported');
% Then configure a floating base, gravity, wheel-ground contacts and the controller.
''',encoding='utf-8')
for ext in ['py','m']:shutil.copy2(DATA/('机械参数.'+ext),SIM/'MATLAB'/('lqr_mechanical_params.'+ext))
payload={'axes':'X forward, Y left, Z up; origin at axle midpoint','cad_to_sim_rotation':rot.tolist(),'wheel_joint_origins_m':[[0,b/2,0],[0,-b/2,0]],'links':[{**a,'com':a['com'].tolist(),'inertia':a['inertia'].tolist()} for a in links]}
(DATA/'仿真刚体参数_SI.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
print('URDF, ROS2 package, SDF, MJCF, MATLAB starter and rotated inertia tensors generated.')
