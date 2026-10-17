import FreeCAD as A,Mesh,json,pathlib,shutil
root=pathlib.Path(__file__).resolve().parents[1];folder=root/'仿真模型'/'通用URDF'/'meshes';report=[]
for name,target in [('body',90000),('left_wheel',40000),('right_wheel',40000)]:
 f=folder/(name+'.stl');m=Mesh.Mesh(str(f));before=m.CountFacets
 if before>target:m.decimate(target)
 m.write(str(f));assert m.CountFacets<200000
 for dest in [root/'仿真模型'/'ROS2'/'balance_car_description'/'meshes',root/'仿真模型'/'MuJoCo'/'meshes']:shutil.copy2(f,dest/f.name)
 report.append({'name':name,'original_faces':before,'simulation_visual_faces':m.CountFacets,'note':'仅简化仿真显示网格；显式质量惯量与简化碰撞体不变。CAD导出保留完整网格。'})
(root/'参数与图片'/'仿真网格简化.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=False))
