import FreeCAD as A, FreeCADGui as G, Part, Mesh, MeshPart
import pathlib, json, math, traceback, shutil, time
from PySide6 import QtCore
FINAL=pathlib.Path(__file__).resolve().parents[1]
SOURCE=FINAL/'源数据与依据'/'整理前_R6.FCStd'
DATA=FINAL/'参数与图片'; DATA.mkdir(parents=True,exist_ok=True)
LOG=DATA/'整理进度.txt'
def log(s):
 with LOG.open('a',encoding='utf-8') as f:f.write(s+'\n')
 print(s,flush=True)
def run():
 src=A.openDocument(str(SOURCE)); src.Motion.Travel=0;src.Motion.Heading=0;src.Motion.Pitch=0;src.recompute()
 r=json.loads((DATA/'机械参数.json').read_text(encoding='utf-8'));p=r['balance_parameters']
 d=A.newDocument('BalanceCarFinal');d.Label='平衡车_最终_402.5g_ZD2808'
 root=d.addObject('App::Part','Assembly');root.Label='01_平衡车整机'
 def group(name,label,parent,part=False):
  o=d.addObject('App::Part' if part else 'App::DocumentObjectGroup',name);o.Label=label
  parent.addObject(o);return o
 body=group('BodyMotion','01_车身固定组_277.1g',root,True)
 left=group('LeftDrive','02_左轮旋转组_62.7g',root,True)
 right=group('RightDrive','03_右轮旋转组_62.7g',root,True)
 framegroup=group('FrameGroup','01_车架与安装件',body)
 pcbgroup=group('Electronics','02_电路板与编码器',body)
 batterygroup=group('Power','03_电池与电池座',body)
 statorgroup=group('Stators','04_左右电机定子',body)
 meta=d.addObject('App::DocumentObjectGroup','Documentation');meta.Label='02_参数与使用说明'
 allfeatures=[];mapping=[]
 def leaves(o):
  if o.TypeId in ['Part::Feature','Mesh::Feature']:return [o]
  return [f for child in getattr(o,'Group',[]) for f in leaves(child)]
 def add(name,label,sources,parent,mass=None,note=''):
  fs=[x for n in sources for x in leaves(src.getObject(n))];fs=list({o.Name:o for o in fs}.values())
  meshes=[o for o in fs if o.TypeId=='Mesh::Feature'];shapes=[o for o in fs if o.TypeId=='Part::Feature' and not o.Shape.isNull()]
  assert not(meshes and shapes),(name,'mixed geometry')
  if meshes:
   o=d.addObject('Mesh::Feature',name);combined=Mesh.Mesh()
   for old in meshes:
    m=Mesh.Mesh(old.Mesh);m.Placement=old.getGlobalPlacement()
    vertices,faces=m.Topology
    # addMesh ignores the input mesh Placement: explicitly bake world vertices first.
    combined.addMesh(Mesh.Mesh([(vertices[a],vertices[b],vertices[c]) for a,b,c in faces]))
   o.Mesh=combined
   o.ViewObject.ShapeColor=meshes[0].ViewObject.ShapeColor
  else:
   outshapes=[];colors=[]
   for old in shapes:
    shape=old.Shape.copy();shape.Placement=old.getGlobalPlacement();outshapes.append(shape)
    cs=list(old.ViewObject.DiffuseColor);base=old.ViewObject.ShapeColor
    if len(cs)==len(shape.Faces):colors.extend(cs)
    else:colors.extend([base]*len(shape.Faces))
   o=d.addObject('Part::Feature',name);o.Shape=Part.makeCompound(outshapes)
   if colors:o.ViewObject.DiffuseColor=colors
  o.Label=label;parent.addObject(o)
  if o.TypeId=='Part::Feature':o.ViewObject.DisplayMode='Flat Lines'
  # Shaded faces retain original PCB and motor material colors without thousands of tree nodes.
  if o.TypeId=='Part::Feature':o.ViewObject.DisplayMode='Flat Lines'
  o.addProperty('App::PropertyString','Contents','部件说明');o.Contents=note or '合并展示对象；原始细分模型保存在源数据与依据。'
  o.addProperty('App::PropertyStringList','SourceObjects','溯源');o.SourceObjects=[x.Name for x in fs]
  if mass is not None:o.addProperty('App::PropertyFloat','AccountingMass_g','质量口径');o.AccountingMass_g=mass
  allfeatures.append(o);mapping.append({'name':name,'label':label,'source_objects':[x.Name for x in fs],'geometry_type':o.TypeId,'mass_g':mass,'note':o.Contents})
  return o
 add('FrameGeometry','车架_含连接螺钉计重45.4g',['Frame'],framegroup,45.4,'原车架STL；45.4g含4颗短铜柱连接螺钉。')
 add('PlasticPosts','塑料长柱_4根_合计3.0g',['TopSpacer'+str(i) for i in range(4)],framegroup,3)
 add('CopperPosts','短铜柱_4个_合计3.7g',['LowSpacer'+str(i) for i in range(4)],framegroup,3.7)
 add('EncoderBolts','编码器安装螺钉_外观示意',['EncoderBolt'+side+str(i) for side in ['Left','Right'] for i in range(2)],framegroup,None,'示意几何；未额外增加质量，沿用编码器原合称口径。')
 add('Guards','电路板四角护件_作者外形',['Protectors'],framegroup,None,'保留原外观；无单独称重，机械参数不另加质量。')
 add('MainBoard','主控电路板_23.7g',['MainPCB'],pcbgroup,23.7,'整板与器件合成一个可选部件；保留各面颜色。')
 for side,cn in [('Left','左'),('Right','右')]:add('Encoder'+side,cn+'AS5600编码器_1.0g',['Encoder'+side],pcbgroup,1,'PCB贴车架内侧槽，沿用已校正位置。')
 add('BatteryHolder','电池座_PCB及4颗螺钉_36.9g',['BatteryPCB']+['TopBolt'+str(i) for i in range(4)],batterygroup,36.9)
 for i in range(2):add('Battery'+str(i+1),'18650电芯_'+str(i+1)+'_44.6g',['Cell'+str(i),'Terminal'+str(i)+'m1','Terminal'+str(i)+'1'],batterygroup,44.6,'标准18×65mm示意；电芯端子包含在44.6g内。')
 for side,cn in [('Left','左'),('Right','右')]:add('Stator'+side,cn+'电机定子_ZD2808_34.2g',['Motor'+side],statorgroup,34.2,'后座、铁芯、铜线及后轴承合成；单台拆机实测应用到两侧。')
 for side,cn,parent in [('Left','左',left),('Right','右',right)]:
  add('Rim'+side,cn+'轮毂_20.3g',['Rim'+side],parent,20.3)
  add('Tyre'+side,cn+'轮胎_20.3g_外径65',['Tyre'+side],parent,20.3,'显示保留已配合外形；惯量采用外径65、壁厚8.2mm的等效环。')
  add('Rotor'+side,cn+'电机转子_21.2g',['RotorBell'+side,'BearingFront'+side,'RearAdaptShaft'+side],parent,21.2,'外壳、前轴承及改轴合并；转子实测包含内壁永磁体。')
  add('EncoderMagnet'+side,cn+'编码器磁铁_0.6g',['Magnet'+side],parent,.6)
  add('HubBolt'+side,cn+'轮毂中轴螺钉_0.3g',['WheelBolt'+side],parent,.3)
 for o,m in [(body,277.1),(left,62.7),(right,62.7),(root,402.5)]:o.addProperty('App::PropertyFloat','GroupMass_g','质量口径');o.GroupMass_g=m
 ctl=d.addObject('App::FeaturePython','Motion');ctl.Label='01_运动控制_选中后修改距离及角度';meta.addObject(ctl)
 for typ,n,val in [('App::PropertyDistance','Travel',0),('App::PropertyAngle','Heading',0),('App::PropertyAngle','Pitch',0),('App::PropertyLength','WheelRadius',32.5),('App::PropertyLength','Track',p['contact_track_m']*1000),('App::PropertyAngle','LeftWheelAngle',0),('App::PropertyAngle','RightWheelAngle',0)]:ctl.addProperty(typ,n,'运动控制');setattr(ctl,n,val)
 ctl.setExpression('LeftWheelAngle','-(Travel - Heading/(180 deg)*pi*Track/2)/WheelRadius*(180 deg)/pi')
 ctl.setExpression('RightWheelAngle','-(Travel + Heading/(180 deg)*pi*Track/2)/WheelRadius*(180 deg)/pi')
 for o,angle in [(body,'Pitch'),(left,'LeftWheelAngle'),(right,'RightWheelAngle')]:
  o.Placement.Rotation=A.Rotation(A.Vector(1,0,0),0);o.setExpression('Placement.Rotation.Angle','Motion.'+angle)
  o.setExpression('Placement.Base.y','32.5 mm * sin(Motion.'+angle+')');o.setExpression('Placement.Base.z','32.5 mm * (1 - cos(Motion.'+angle+'))')
 root.setExpression('Placement.Base.y','Motion.Travel');root.Placement.Rotation=A.Rotation(A.Vector(0,0,1),0);root.setExpression('Placement.Rotation.Angle','Motion.Heading')
 info=d.addObject('App::FeaturePython','ModelInfo');info.Label='02_模型说明_坐标与质量口径';meta.addObject(info)
 for n,v in [('Coordinates','CAD：X沿左右轮轴（右为正），Y向前，Z向上；地面Z=0。仿真：X向前，Y向左，Z向上；原点为轮轴中点。'),('MassAccounting','整车402.5g＝车身277.1g＋左右旋转组各62.7g。电机安装螺钉8颗合计4.8g计入车身参数，外观未单独添加。护件无单独称重；不额外加质量。'),('Geometry','PCB及电机细节合并为可识别部件，保留颜色与安装位置。合并为复合形状，未强行布尔融合；可在原始工程继续细分修改。'),('Usage','运动控制仅为运动学；仿真文件包含物理惯性与轮轴关节，摩擦及控制器需按实车进一步标定。')]:info.addProperty('App::PropertyString',n,'使用说明');setattr(info,n,v)
 est=d.addObject('App::FeaturePython','MechanicalEstimates');est.Label='03_机械参数_俯仰X_自转X_偏航Z';meta.addObject(est)
 for n,val in p.items():
  if isinstance(val,(float,int)):est.addProperty('App::PropertyFloat',n,'SI估算');setattr(est,n,val)
 est.addProperty('App::PropertyStringList','Definitions','含义');est.Definitions=[x['description']+'：'+x['value']+'；'+x['definition'] for x in r['parameter_list']]
 mass=d.addObject('App::FeaturePython','MeasuredInputs');mass.Label='04_实测质量输入_仅记录不自动辨识';meta.addObject(mass)
 vals={'RimEach_g':20.3,'TyreEach_g':20.3,'RotorEach_g':21.2,'StatorEach_g':34.2,'MotorBoltEach_g':.6,'HubBoltEach_g':.3,'MagnetEach_g':.6,'FrameWithBolts_g':45.4,'MainBoard_g':23.7,'BatteryHolderWithBolts_g':36.9,'PlasticPosts4_g':3.,'CopperPosts4_g':3.7,'EncoderEach_g':1.,'CellEach_g':44.6}
 for n,v in vals.items():mass.addProperty('App::PropertyFloat',n,'称重记录_g');setattr(mass,n,v)
 dimroot=group('Dimensions','05_尺寸标注_默认隐藏_展开按空格显示',meta)
 for oldgroup in src.Dimensions.Group:
  dg=group('Final'+oldgroup.Name,oldgroup.Label,dimroot)
  for old in oldgroup.Group:
   c=d.copyObject(old,False);dg.addObject(c);c.Visibility=False
 d.recompute();A.setActiveDocument(d.Name);G.activateWorkbench('PartWorkbench')
 G.activeDocument().activeView().viewAxonometric();G.activeDocument().activeView().fitAll();G.updateGui()
 native=FINAL/'freeCAD工程'/'平衡车_最终.FCStd';native.parent.mkdir(exist_ok=True);d.saveAs(str(native))
 (DATA/'工程树映射.json').write_text(json.dumps(mapping,ensure_ascii=False,indent=2),encoding='utf-8')
 log('Native document saved: '+str(len(src.Objects))+' source objects -> '+str(len(d.Objects))+' document objects / '+str(len(allfeatures))+' visible merged parts')
 # Native view images are suitable for the workbook and remain separate usable illustrations.
 view=G.activeDocument().activeView()
 for name,fn in [('整车轴测图',view.viewAxonometric),('正视图',view.viewRear),('侧视图',view.viewRight),('俯视图',view.viewTop)]:
  fn();view.fitAll();G.updateGui();view.saveImage(str(DATA/(name+'.png')),1400,1000,'White')
 view.viewAxonometric();view.fitAll();G.updateGui()
 log('Four native images saved')
 # Export CAD geometry in millimetres. STEP uses BRep conversion only for originally meshed parts.
 exports=FINAL/'导出的模型';stepdir=exports/'STEP_mm';stldir=exports/'STL_mm';objdir=exports/'OBJ_mm'
 for q in [stepdir,stldir,objdir]:q.mkdir(parents=True,exist_ok=True)
 exportdoc=A.newDocument('ExportSolids');stepobjects=[];meshgroups={}
 def mesh_of(o):
  if o.TypeId=='Mesh::Feature':m=Mesh.Mesh(o.Mesh)
  else:m=MeshPart.meshFromShape(Shape=o.Shape,LinearDeflection=.15,AngularDeflection=.35,Relative=False)
  return m
 for key,parent in [('body',body),('left_wheel',left),('right_wheel',right)]:
  fs=[o for o in allfeatures if o in parent.OutListRecursive];meshgroups[key]=[(o,mesh_of(o)) for o in fs]
  combined=Mesh.Mesh()
  for o,m in meshgroups[key]:combined.addMesh(m)
  combined.write(str(stldir/(key+'.stl')))
  for o in fs:
   e=exportdoc.addObject('Part::Feature',o.Name);e.Label=o.Label
   if o.TypeId=='Mesh::Feature':
    shape=Part.Shape();shape.makeShapeFromMesh(o.Mesh.Topology,.01)
    if o.Mesh.isSolid():
     try:shape=Part.makeSolid(shape)
     except Exception:pass
    e.Shape=shape
   else:e.Shape=o.Shape.copy()
   e.ViewObject.ShapeColor=o.ViewObject.ShapeColor
   if o.TypeId=='Part::Feature':e.ViewObject.DiffuseColor=o.ViewObject.DiffuseColor
   stepobjects.append(e)
  log('Prepared '+key+' mesh/solid export')
 combined=Mesh.Mesh()
 for pairs in meshgroups.values():
  for o,m in pairs:combined.addMesh(m)
 combined.write(str(stldir/'balance_car_assembly.stl'))
 exportdoc.recompute()
 import Import
 log('STEP export started (STL-derived surfaces retain triangular facets)')
 Import.export(stepobjects,str(stepdir/'balance_car_assembly.step'))
 log('STEP export complete')
 # Simple OBJ + MTL with per-part colors; CAD version in mm, simulation visual files in metres.
 def objwrite(path,pairs,origin=(0,0,0),ros=False):
  materials=[];offset=1
  with path.open('w',encoding='utf-8') as f:
   f.write('mtllib '+path.with_suffix('.mtl').name+'\n')
   for idx,(o,m) in enumerate(pairs):
    mat='mat_'+str(idx);color=tuple(o.ViewObject.ShapeColor)[:3];materials.append((mat,color));f.write('o '+o.Name+'\nusemtl '+mat+'\n')
    vertices,faces=m.Topology
    for v in vertices:
     x,y,z=v.x-origin[0],v.y-origin[1],v.z-origin[2]
     if ros:x,y,z=y/1000,-x/1000,z/1000
     f.write('v %.9g %.9g %.9g\n'%(x,y,z))
    for a,b,c in faces:f.write('f %d %d %d\n'%(a+offset,b+offset,c+offset))
    offset+=len(vertices)
  with path.with_suffix('.mtl').open('w',encoding='utf-8') as f:
   for mat,col in materials:f.write('newmtl '+mat+'\nKd %.5f %.5f %.5f\nKa 0.12 0.12 0.12\n'%col)
 objwrite(objdir/'balance_car_assembly.obj',[pair for pairs in meshgroups.values() for pair in pairs])
 simmesh=FINAL/'仿真模型'/'通用URDF'/'meshes';simmesh.mkdir(parents=True,exist_ok=True)
 for key,pairs in meshgroups.items():
  origin=(0,0,32.5) if key=='body' else ((-1 if key=='left_wheel' else 1)*p['contact_track_m']*500,0,32.5)
  objwrite(simmesh/(key+'.obj'),pairs,origin,True)
  outmesh=Mesh.Mesh()
  for o,m in pairs:
   verts,faces=m.Topology;vs=[A.Vector((v.y-origin[1])/1000,-(v.x-origin[0])/1000,(v.z-origin[2])/1000) for v in verts]
   outmesh.addMesh(Mesh.Mesh([(vs[a],vs[b],vs[c]) for a,b,c in faces]))
  outmesh.write(str(simmesh/(key+'.stl')))
 log('STL/OBJ CAD and SI simulation meshes saved')
 A.closeDocument(exportdoc.Name);A.setActiveDocument(d.Name);G.activeDocument().activeView().viewAxonometric();G.activeDocument().activeView().fitAll()
 # Open only the three meaningful movement branches. Keep detail and documentation compact.
 G.activeDocument().activeView().redraw();G.activeDocument().activeView().fitAll()
 for o in [root,body,left,right]:G.activeDocument().getObject(o.Name).show()
 G.activeDocument().activeView().viewAxonometric();G.updateGui();d.save()
 sourcebbox=combined.BoundBox
 dimensions=[]
 for dg in src.Dimensions.Group:
  for dim in dg.Group:
   dimensions.append({'name':dim.Label,'length_mm':(dim.End-dim.Start).Length})
 report={'source_objects':len(src.Objects),'final_objects':len(d.Objects),'merged_geometry_objects':len(allfeatures),'total_mass_kg':r['mass_kg']['total'],'cad_bbox_mm':{'x':sourcebbox.XLength,'y':sourcebbox.YLength,'z':sourcebbox.ZLength},'dimensions':dimensions,'native_file':str(native),'status':'CAD_EXPORT_COMPLETE'}
 (DATA/'CAD整理结果.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 log('CAD_EXPORT_COMPLETE')
try:run()
except Exception:
 (DATA/'CAD整理错误.txt').write_text(traceback.format_exc(),encoding='utf-8');log(traceback.format_exc())

