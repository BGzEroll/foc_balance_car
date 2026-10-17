import FreeCAD as App,FreeCADGui as Gui,json,pathlib
from PySide6 import QtWidgets,QtCore
d=App.ActiveDocument
root=pathlib.Path(d.FileName).resolve().parents[1]
r=json.loads((root/'参数与图片'/'机械参数.json').read_text(encoding='utf-8'))
panel=QtWidgets.QDockWidget('机械参数 · 直立姿态估算',Gui.getMainWindow());panel.setObjectName('FinalMechanicsPanel')
w=QtWidgets.QWidget();lay=QtWidgets.QVBoxLayout(w);panel.setWidget(w)
note=QtWidgets.QLabel('车身不含两侧旋转组；整车包含全部已计重部件。\nCAD：X沿轮轴，Y向前，Z向上。');note.setWordWrap(True);lay.addWidget(note)
table=QtWidgets.QTableWidget(len(r['parameter_list']),2);table.setHorizontalHeaderLabels(['部件 / 运动 / 轴线','估算值'])
for i,row in enumerate(r['parameter_list']):
 for j,k in enumerate(['description','value']):
  item=QtWidgets.QTableWidgetItem(row[k]);item.setToolTip(row['definition']);table.setItem(i,j,item)
table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers);table.horizontalHeader().setSectionResizeMode(0,QtWidgets.QHeaderView.Stretch);table.setColumnWidth(1,165);table.verticalHeader().hide();table.resizeRowsToContents();lay.addWidget(table)
foot=QtWidgets.QLabel('等效偏航惯量已含纯滚动轮组自转附加项。参数为快照；改动质量或几何后需重新计算。');foot.setWordWrap(True);lay.addWidget(foot)
panel.setMinimumWidth(650);Gui.getMainWindow().addDockWidget(QtCore.Qt.RightDockWidgetArea,panel);panel.show();App._final_mechanics_panel=panel
