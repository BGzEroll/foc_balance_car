"""只读核对USB朝前后的URDF、SDF、MuJoCo和SI参数。无需第三方库。"""
from pathlib import Path
import json, math
import xml.etree.ElementTree as E

def verify():
    folder=Path(__file__).resolve().parent
    parameters=json.loads((folder.parent/'参数与图片'/'仿真刚体参数_SI.json').read_text(encoding='utf-8'))
    assert parameters['front_side']=='USB'
    assert parameters['cad_to_sim_rotation']==[[-1,0,0],[0,-1,0],[0,0,1]]
    assert parameters['axis_convention']=='x_right_y_forward_z_up'
    urdf=E.parse(folder/'通用URDF'/'urdf'/'balance_car.urdf').getroot()
    refs={link.get('name'):link for link in urdf.findall('link')}
    filenames={'base_link':'body.stl','left_wheel':'right_wheel.stl','right_wheel':'left_wheel.stl'}
    for robot in [urdf,E.parse(folder/'ROS2'/'balance_car_description'/'urdf'/'balance_car.urdf').getroot()]:
        for link in robot.findall('link'):
            name=link.get('name');visual=link.find('visual')
            assert math.isclose(float(visual.find('origin').get('rpy').split()[2]),-math.pi/2)
            assert visual.find('geometry/mesh').get('filename').endswith(filenames[name])
        for side,sign in [('left',-1),('right',1)]:
            joint=robot.find(f"joint[@name='{side}_wheel_joint']")
            assert float(joint.find('origin').get('xyz').split()[0])*sign>0
            assert joint.find('axis').get('xyz')=='-1 0 0'
    for key,item in zip(('base_link','left_wheel','right_wheel'),parameters['links']):
        xml_com=list(map(float,refs[key].find('inertial/origin').get('xyz').split()))
        assert all(math.isclose(a,b,abs_tol=1e-13) for a,b in zip(xml_com,item['com']))
    for link in E.parse(folder/'Gazebo_SDF'/'model.sdf').findall('model/link'):
        assert math.isclose(float(link.find('visual/pose').text.split()[5]),-math.pi/2)
        assert link.find('visual/geometry/mesh/uri').text.endswith(filenames[link.get('name')])
        assert link.find('inertial/pose').text.split()[:3]==refs[link.get('name')].find('inertial/origin').get('xyz').split()
    for body in E.parse(folder/'MuJoCo'/'balance_car.xml').findall('.//body'):
        visual=next(g for g in body.findall('geom') if g.get('type')=='mesh')
        quat=list(map(float,visual.get('quat').split()))
        assert math.isclose(quat[0],math.sqrt(.5)) and math.isclose(quat[3],-math.sqrt(.5))
        expected={'base_link':'body','left_wheel':'right_wheel','right_wheel':'left_wheel'}[body.get('name')]
        assert visual.get('mesh')==expected
        assert body.find('inertial').get('pos')==refs[body.get('name')].find('inertial/origin').get('xyz')
    print('PASS: X right, Y forward/USB, Z up; MA -X, MB +X; URDF/ROS/SDF/MuJoCo consistent')

if __name__=='__main__':
    verify()
