import FreeCAD as App, FreeCADGui as Gui, math,os,json,traceback
from PySide6 import QtCore,QtWidgets
OUT=os.path.join(os.path.dirname(os.path.dirname(App.ActiveDocument.FileName)),'参数与图片','动作演示');os.makedirs(OUT,exist_ok=True)
class CarMotionPanel(QtWidgets.QDockWidget):
 def __init__(self,doc):
  super().__init__('平衡车动作控制',Gui.getMainWindow());self.doc=doc;self.ctl=doc.Motion;self.setObjectName('BalanceCarMotionPanel')
  w=QtWidgets.QWidget();lay=QtWidgets.QVBoxLayout(w);self.setWidget(w)
  note=QtWidgets.QLabel('轮子、转子和磁铁随驱动转动；\n车架、定子和编码器随车身俯仰。\n这是运动学演示。');lay.addWidget(note)
  self.inputs={}
  for prop,label,lo,hi,unit in [('Travel','前进距离',-100,100,' mm'),('Heading','原地转向',-180,180,'°'),('Pitch','车身俯仰',-12,12,'°')]:
   row=QtWidgets.QHBoxLayout();row.addWidget(QtWidgets.QLabel(label));spin=QtWidgets.QDoubleSpinBox();spin.setRange(lo,hi);spin.setDecimals(1);spin.setSuffix(unit);row.addWidget(spin);lay.addLayout(row)
   spin.valueChanged.connect(lambda val,p=prop:self.set_value(p,val));self.inputs[prop]=spin
  self.status=QtWidgets.QLabel('就绪');lay.addWidget(self.status)
  for title,fn in [('播放 / 暂停',self.toggle),('复位到直立',self.reset),('录制动作 GIF',self.record)]:
   b=QtWidgets.QPushButton(title);b.clicked.connect(fn);lay.addWidget(b)
  lay.addStretch();self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.setInterval(40)
  self.clock=QtCore.QElapsedTimer();self.elapsed=0.;self.recording=False;self.frames=[];self.record_index=0
 def set_value(self,prop,val):
  setattr(self.ctl,prop,val);self.doc.recompute();Gui.updateGui()
 def pose(self,travel,heading,pitch):
  for p,val in [('Travel',travel),('Heading',heading),('Pitch',pitch)]:
   self.inputs[p].blockSignals(True);self.inputs[p].setValue(val);self.inputs[p].blockSignals(False);setattr(self.ctl,p,val)
  self.doc.recompute();Gui.updateGui()
 def motion(self,u):
  # Three closed phases avoid accumulated yaw/translation drift.
  if u<4:return 75*math.sin(2*math.pi*u/4),0,0
  if u<8:return 0,0,10*math.sin(2*math.pi*(u-4)/4)
  return 0,75*math.sin(2*math.pi*(u-8)/4),0
 def toggle(self):
  if self.timer.isActive():self.elapsed+=self.clock.elapsed()/1000;self.timer.stop();self.status.setText('已暂停');return
  self.clock.start();self.timer.start();self.status.setText('前后行驶 → 俯仰 → 原地转向')
 def reset(self):
  self.timer.stop();self.elapsed=0.;self.recording=False;self.pose(0,0,0);self.status.setText('已复位')
 def tick(self):
  try:
   u=(self.elapsed+self.clock.elapsed()/1000)%12
   self.pose(*self.motion(u))
   if self.elapsed+self.clock.elapsed()/1000>=24:
    self.reset();self.status.setText('演示完成；可继续播放或手动调节')
  except Exception:
   self.timer.stop();self.status.setText('动作停止，文档可能已关闭')
 def record(self):
  self.reset();self.recording=True;self.record_index=0;self.frames=[]
  folder=os.path.join(OUT,'动作帧');os.makedirs(folder,exist_ok=True)
  # One fixed orthographic camera for every frame.
  view=Gui.activeDocument().activeView();view.setCameraType('Orthographic');view.viewAxonometric();view.fitAll()
  # Increase view height to include the +/-75 mm travel envelope.
  # Camera node height is a scene property, independent of viewport dimensions.
  node=view.getCameraNode();node.height.setValue(float(node.height.getValue())*1.75)
  self.status.setText('录制中：每帧直接从FreeCAD导出')
  self.rec_timer=QtCore.QTimer(self);self.rec_timer.timeout.connect(self.record_frame);self.rec_timer.start(70)
 def record_frame(self):
  try:
   if self.record_index>=72:
    self.rec_timer.stop();self.pose(0,0,0)
    from PIL import Image,ImageDraw,ImageFont
    images=[]
    for i,p in enumerate(self.frames):
     im=Image.open(p).convert('RGB');draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',18)
     label=['前后行驶：车轮按半径联动','车身俯仰：绕轮轴摆动','原地转向：左右轮反向'][min(i//24,2)]
     draw.rectangle((0,0,im.width,38),fill='white');draw.text((14,7),label,fill=(25,45,65),font=font);images.append(im)
    images[0].save(os.path.join(OUT,'平衡车_动作演示.gif'),save_all=True,append_images=images[1:],duration=167,loop=0,optimize=True)
    self.recording=False;self.status.setText('GIF已保存；播放键可继续演示');self.doc.recompute();self.doc.save()
    open(os.path.join(OUT,'ANIMATION_READY.txt'),'w').write('72 frames / 12 seconds')
    # Start a visible smooth demo after the movie is recorded.
    QtCore.QTimer.singleShot(1300,self.toggle)
    return
   u=self.record_index/6.;self.pose(*self.motion(u));Gui.activeDocument().activeView().redraw()
   path=os.path.join(OUT,'动作帧','frame_%03d.png'%self.record_index)
   Gui.activeDocument().activeView().saveImage(path,900,700,'White');self.frames.append(path);self.record_index+=1
  except Exception:
   self.rec_timer.stop();open(os.path.join(OUT,'ANIMATION_ERROR.txt'),'w',encoding='utf-8').write(traceback.format_exc());self.status.setText('录制失败，请查看错误文件')
panel=CarMotionPanel(App.ActiveDocument)
Gui.getMainWindow().addDockWidget(QtCore.Qt.RightDockWidgetArea,panel)
panel.show()
App._balance_motion_panel=panel
if globals().get('AUTO_RECORD_R3',False):QtCore.QTimer.singleShot(1600,panel.record)
