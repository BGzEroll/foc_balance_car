"""Online PID comparison through the existing serial logger; no port reset."""
import csv,json,os,pathlib,re,sys,time
h=pathlib.Path(__file__).resolve().parent
candidate=json.loads(pathlib.Path(sys.argv[1]).read_text()) if len(sys.argv)>1 else None
amp=float(sys.argv[2]) if len(sys.argv)>2 else .1
source=max((h/'results').glob('live_*'),key=lambda p:p.stat().st_mtime)
f=(source/'samples.csv').open();fields=next(csv.reader([f.readline()]));f.seek(0,2);buf='';last=None
log=(source/'commands.log').open();log.seek(0,2);text='';began=None;rows=[]
folder=h/'results'/('pid_trial_'+time.strftime('%Y%m%d_%H%M%S'));folder.mkdir();out=(folder/'samples.csv').open('w',newline='');w=csv.writer(out);w.writerow(fields)
fd=os.open('/dev/ttyACM0',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
def send(s):os.write(fd,(s+'\n').encode())
def pump(seconds):
 global buf,last,text
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  buf+=f.read();text+=log.read()
  while '\n' in buf:
   line,buf=buf.split('\n',1);v=next(csv.reader([line]))
   if len(v)==len(fields):
    last={k:float(x) for k,x in zip(fields,v)}
    if began is not None:rows.append(last.copy());w.writerow(v)
  out.flush();time.sleep(.01)
names=['velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd','position_kp','pitch_trim_rad']
try:
 pump(.4);assert last and last['mode']==2 and last['fault']==0,last
 original=[last[k] for k in names];g=candidate['gains'] if candidate else original
 for n in range(1,11):
  v=[a+(b-a)*n/10 for a,b in zip(original,g)]
  send('PIDSET '+' '.join(format(x,'.9g') for x in v));pump(.15)
 assert max(abs(last[k]-v) for k,v in zip(names,g))<3e-6
 pump(4);text='';began=time.monotonic();send('DYNTEST '+str(amp));pump(.3);send('DYNSTAT');pump(.3)
 assert 'DYNTEST accepted=1' in text,text
 match=next(x for x in re.findall(r'DYNSTAT [^\r\n]+',text) if 'armed=1 ' in x)
 origin=float(re.search(r'origin=([^ ]+)',match).group(1));start=float(re.search(r'began_us=([^ ]+)',match).group(1))
 print('START '+str(folder)+' gains='+str(g),flush=True);due=2;ended=False
 while time.monotonic()-began<32:
  pump(.08);elapsed=time.monotonic()-began
  if last['mode']!=2 or last['fault']!=0:break
  if elapsed>due:
   send('DYNSTAT');print('t=%.1f x_cm=%.2f v=%.3f'%(elapsed,100*(last['distance']-origin),last['speed']),flush=True);due+=2
  if elapsed>=17 and not ended:send('DYNEND');ended=True
 send('STATUS');pump(.2)
 late=[r for r in rows if (r['sample_us']-start)*1e-6>=22]
 def target(t):
  if t>=16:return 0
  phase=max(0,int(t/4));u=max(0,min(1,(t-phase*4)/2));return [0,amp,0,-amp][phase]+([amp,0,-amp,0][phase]-[0,amp,0,-amp][phase])*u**3*(10-15*u+6*u*u)
 track=[r['distance']-origin-target((r['sample_us']-start)*1e-6) for r in rows if 0<=(r['sample_us']-start)*1e-6<16]
 report=dict(gains=g,amplitude_m=amp,origin_m=origin,forward_peak_cm=100*max(r['distance']-origin for r in rows),backward_peak_cm=-100*min(r['distance']-origin for r in rows),tracking_rms_cm=100*(sum(e*e for e in track)/len(track))**.5,late_span_cm=100*(max(r['distance'] for r in late)-min(r['distance'] for r in late)) if late else None,late_speed_rms_m_s=(sum(r['speed']**2 for r in late)/len(late))**.5 if late else None,peak_current_A=max(abs(r[k]) for r in rows for k in ['command_a','command_b']),saturated_samples=sum(r['saturated'] for r in rows),final_mode=last['mode'],final_fault=last['fault'],dyn_status=re.findall(r'DYNSTAT [^\r\n]+',text))
 (folder/'report.json').write_text(json.dumps(report,indent=2));print('REPORT '+json.dumps({k:v for k,v in report.items() if k!='dyn_status'}),flush=True)
finally:
 os.close(fd);f.close();log.close();out.close()
