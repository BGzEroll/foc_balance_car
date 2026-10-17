import FreeCAD as A,Mesh,Part,json,pathlib,math
root=pathlib.Path(__file__).resolve().parents[1]
d=A.openDocument(str(root/'freeCAD工程'/'平衡车_最终.FCStd'));src=A.openDocument(str(root/'源数据与依据'/'整理前_R6.FCStd'))
def oldleaves(o):
 if o.TypeId in ['Part::Feature','Mesh::Feature']:return [o]
 return [x for c in getattr(o,'Group',[]) for x in oldleaves(c)]
def oldbbox(o):
 if o.TypeId=='Mesh::Feature':m=Mesh.Mesh(o.Mesh);m.Placement=o.getGlobalPlacement();return m.BoundBox
 s=o.Shape.copy();s.Placement=o.getGlobalPlacement();return s.BoundBox
def bbvalues(bb):return [bb.XMin,bb.YMin,bb.ZMin,bb.XMax,bb.YMax,bb.ZMax]
mapping=json.loads((root/'参数与图片'/'工程树映射.json').read_text(encoding='utf-8'))
for row in mapping:
 o=d.getObject(row['name']);before=A.BoundBox()
 for name in row['source_objects']:before.add(oldbbox(src.getObject(name)))
 after=(o.Mesh if o.TypeId=='Mesh::Feature' else o.Shape).BoundBox
 assert max(abs(a-b) for a,b in zip(bbvalues(before),bbvalues(after)))<1e-4,(o.Name,before,after)
assert abs(d.Assembly.GroupMass_g-402.5)<1e-9
assert abs(d.Motion.Track.Value-132.5364)<1e-8
assert len(d.Assembly.Group)==3
for angle in [10,-10,0]:
 d.Motion.Pitch=angle;d.recompute();v=d.BodyMotion.Placement.multVec(A.Vector(0,0,32.5));assert (v-A.Vector(0,0,32.5)).Length<1e-7
d.Motion.Pitch=0;d.Motion.Travel=32.5*math.pi*2;d.recompute();assert abs(d.Motion.LeftWheelAngle.Value+360)<1e-7
d.Motion.Travel=0;d.Motion.Heading=90;d.recompute();assert abs(d.Motion.LeftWheelAngle.Value+d.Motion.RightWheelAngle.Value)<1e-7
d.Motion.Heading=0;d.recompute()
m=Mesh.Mesh(str(root/'导出的模型'/'STL_mm'/'balance_car_assembly.stl'));bb=m.BoundBox
assert abs(bb.XLength-169.7364)<.15 and abs(bb.YLength-65)<.15 and abs(bb.ZLength-112.8000254)<.15
out={'component_world_bounds_match':True,'merged_parts_checked':len(mapping),'top_level_movement_groups':3,'mass_g':402.5,'wheel_track_mm':132.5364,'pitch_axle_invariant':True,'travel_wheel_rotation_passed':True,'yaw_counter_rotation_passed':True,'STL_bbox_mm':[bb.XLength,bb.YLength,bb.ZLength],'STL_bounds_tolerance_mm':.15,'note':'网格近似造成外包尺寸微小误差；原生实体尺寸与原装配一致。'}
(root/'参数与图片'/'FreeCAD整理验证.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False))
