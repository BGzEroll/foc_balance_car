"""Generate MA v12 from the saved, measured v11; no hardware operations."""
from pathlib import Path
r=Path(__file__).resolve().parent
s=(r/'v11_before.cpp').read_text(encoding='utf-8')
def sub(a,b):
 global s
 assert a in s,a
 s=s.replace(a,b)
sub('voltage_limit=.12f;', 'voltage_limit=.12f,bus_ratio=.015f;')
sub('calc_us,stage,pi_seq,speed_ref;', 'calc_us,stage,pi_seq,speed_ref,vlimit;')
sub('sizeof(Row)==120','sizeof(Row)==124')
sub('fabsf(p.left_encoder.speed_rad_s)>15','fabsf(p.left_encoder.speed_rad_s)>40')
sub('float dt=dtus*1e-6f;', '''float dt=dtus*1e-6f;''')
sub('float id=ia*cs+beta*sn,iq=beta*cs-ia*sn,rd=0,rq=0,vd=0,vq=0,speed_ref=0;', '''float bus=voltage_mv[4][s.bus]*.003f;
  if(!isfinite(bus)||bus<5||bus>13||now-__atomic_load_n(&bus_stamp,__ATOMIC_ACQUIRE)>3000){stop_output(3);continue;}
  // The 0.04*Vbus ceiling also reserves the shared low-side sampling window.
  // The absolute ceiling is a separate, explicitly staged bench restriction.
  float limit=fminf(voltage_limit,bus_ratio*bus);
  float id=ia*cs+beta*sn,iq=beta*cs-ia*sn,rd=0,rq=0,vd=0,vq=0,speed_ref=0;''')
sub('if(mag>voltage_limit){float a=voltage_limit/mag;', 'if(mag>limit){float a=limit/mag;')
sub('r.speed_ref=bits(speed_ref);', 'r.vlimit=bits(limit);r.speed_ref=bits(speed_ref);')
sub('"row_bytes\\\":120', '"row_bytes\\\":124')
sub(r'\"version\":11',r'\"version\":12')
sub('MA_CURRENT_V11','MA_CURRENT_V12_BUS')
sub('speed_ref");','speed_ref,vlimit");')
sub('%.7f\\n",s.us','%.7f,%.7f\\n",s.us')
sub('unbits(r.speed_ref));','unbits(r.speed_ref),unbits(r.vlimit));')
sub('float a=0,b=0,c=0,d=0;', '''float a=0,b=0,c=0,d=0;
 if(sscanf(line,"LIMIT %f %f",&a,&b)==2){
  if(!isfinite(a)||!isfinite(b)||a<.005f||a>.04f||b<=0||b>.4f){puts("ERROR bus limit");return;}
  bus_ratio=a;voltage_limit=b;puts("LIMIT_OK");return;
 }''')
sub('d>.2f','d>.4f')
sub('fabsf(a)>5','fabsf(a)>28')
sub('health();printf("SETUP ', 'health();printf("BUS_LIMIT {\\"ratio\\":%.8f,\\"absolute_V\\":%.6f}\\n",bus_ratio,voltage_limit);printf("SETUP ')
(r/'foc_current.cpp').write_text(s,encoding='utf-8',newline='\n')
host=(r.parent/'next_stage/run.py').read_text(encoding='utf-8')
host=host.replace("p.add_argument('--velocity',nargs=3,type=float)","p.add_argument('--velocity',nargs=3,type=float)\np.add_argument('--buslimit',nargs=2,type=float)")
host=host.replace('if a.velocity:setting(',"if a.buslimit:setting('LIMIT '+' '.join(map(str,a.buslimit)),'LIMIT_OK')\n if a.velocity:setting(")
host=host.replace("meta=None;rows=[];luts={};header=None;health=[];setup={}","meta=None;rows=[];luts={};header=None;health=[];setup={};buslimit={}")
host=host.replace("if line.startswith('META '):", "if line.startswith('BUS_LIMIT '):buslimit=json.loads(line[10:])\n elif line.startswith('META '):")
host=host.replace("'meta':meta,'setup':setup", "'meta':meta,'bus_limit':buslimit,'setup':setup")
(r/'run.py').write_text(host,encoding='utf-8',newline='\n')
