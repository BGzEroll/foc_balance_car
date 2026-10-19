"""Upgrade PWM counter resolution, preserving the measured v6 baseline."""
from pathlib import Path
r=Path(__file__).resolve().parent
s=(r/'v6_baseline/foc_current.cpp').read_text(encoding='utf-8')
def sub(a,b):
 global s
 assert a in s,a
 s=s.replace(a,b)
sub('constexpr unsigned ZERO=250U|(250U<<10)|(250U<<20);','constexpr unsigned ZERO=2000;')
sub('epoch=0,queued=ZERO,applied=ZERO;','epoch=0;\nunsigned queued[3]={ZERO,ZERO,ZERO},applied[3]={ZERO,ZERO,ZERO};')
sub('struct Command {unsigned tag,seq,sample_us,pack;}','struct Command {unsigned tag,seq,sample_us,ca,cb,cc;}')
sub('(s.timer_direction?1000-s.timer_value:s.timer_value)%1000','(s.timer_direction?8000-s.timer_value:s.timer_value)%8000')
sub('++epoch;applied=queued;','++epoch;for(unsigned i=0;i<3;++i)applied[i]=queued[i];')
sub('p>=500&&p<800','p>=4000&&p<6400')
sub('c.pack=ZERO;c.seq=0;','c.ca=ZERO;c.cb=ZERO;c.cc=ZERO;c.seq=0;')
sub('if(p<800||p>930)','if(p<6400||p>7440)')
sub('if(c.pack!=queued){for(unsigned i=0;i<3;++i)cmp_errors+=mcpwm_comparator_set_compare_value(cmp[i],(c.pack>>(10*i))&1023)!=ESP_OK;}',
    'unsigned levels[3]={c.ca,c.cb,c.cc};for(unsigned i=0;i<3;++i)if(levels[i]!=queued[i])cmp_errors+=mcpwm_comparator_set_compare_value(cmp[i],levels[i])!=ESP_OK;')
sub('end>980','end>7840')
sub('queued=c.pack;queued_seq','for(unsigned i=0;i<3;++i){queued[i]=levels[i];}queued_seq')
sub('s.ca=applied&1023;s.cb=(applied>>10)&1023;s.cc=(applied>>20)&1023;',
    's.ca=applied[0];s.cb=applied[1];s.cc=applied[2];')
sub('(esp_cpu_get_cycle_count()-t0+15)/16','(esp_cpu_get_cycle_count()-t0+1)/2')
sub('s.ca+20&&s.a1+20<=1000-s.ca','s.ca+160&&s.a1+160<=8000-s.ca')
sub('s.cb+20&&s.b1+20<=1000-s.cb','s.cb+160&&s.b1+160<=8000-s.cb')
sub('s.b1<1000','s.b1<8000');sub('s.b1>=1000','s.b1>=8000')
sub('unsigned modulation(float vd,float vq,float theta,float bus)',
    'Command modulation(float vd,float vq,float theta,float bus)')
sub('unsigned pack=0;','Command result{};unsigned *out=&result.ca;')
sub('unsigned c=unsigned((.5f+(v[i]+z)/bus)*500+.5f);c=c<40?40:(c>460?460:c);pack|=c<<(10*i);}return pack;',
    'unsigned c=unsigned((.5f+(v[i]+z)/bus)*4000+.5f);c=c<320?320:(c>3680?3680:c);out[i]=c;}return result;')
sub('else publish_command(Command{tag,pi_seq,s.us,modulation(vd,vq,theta,bus)});',
    'else {Command c=modulation(vd,vq,theta,bus);c.tag=tag;c.seq=pi_seq;c.sample_us=s.us;publish_command(c);}')
sub(r'\"version\":6',r'\"version\":7')
sub(r'\"period_ticks\":1000',r'\"period_ticks\":8000')
sub('MA_CURRENT_V6','MA_CURRENT_V7')
sub('Command c{};bool coherent=false;',
    'const bool active=running||__atomic_load_n(&requested,__ATOMIC_ACQUIRE);\n Command c{};bool coherent=false;')
sub('++late_commit;return false;','if(active)++late_commit;return false;')
sub('kp=.015079645f,ki=56.548668f','kp=.030159289f,ki=113.097336f')
sub('records=0;reason=0;all_on=0;','records=0;reason=0;cmp_errors=0;late_commit=0;all_on=0;')
sub('cfg.resolution_hz=10000000;cfg.period_ticks=1000;',
    'cfg.resolution_hz=80000000;cfg.period_ticks=8000;')
sub('mcpwm_comparator_set_compare_value(cmp[i],250)','mcpwm_comparator_set_compare_value(cmp[i],2000)')
sub('p.count_value=200','p.count_value=1600')
# Avoid pointer arithmetic across struct members in the PWM conversion.
sub('Command result{};unsigned *out=&result.ca;','Command result{};unsigned out[3]{};')
sub('out[i]=c;}return result;','out[i]=c;}result.ca=out[0];result.cb=out[1];result.cc=out[2];return result;')
sub(r'\"version\":7',r'\"version\":8')
sub('MA_CURRENT_V7','MA_CURRENT_V8')
(r/'foc_current.cpp').write_text(s,encoding='utf-8',newline='\n')
