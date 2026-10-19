"""Keep the measured 20kHz sources intact; generate a separate 10kHz variant."""
from pathlib import Path
r=Path(__file__).resolve().parent
out=r/'pwm10k';out.mkdir(exist_ok=True)
s=(r/'foc_current.cpp').read_text(encoding='utf-8')
def change(old,new):
 global s
 assert old in s,old
 s=s.replace(old,new)
change('125U|(125U<<8)|(125U<<16)','250U|(250U<<10)|(250U<<20)')
change('commit_before_min=500','commit_before_min=1000')
change('commit_after_min=500','commit_after_min=1000')
change('(s.timer_direction?500-s.timer_value:s.timer_value)%500','(s.timer_direction?1000-s.timer_value:s.timer_value)%1000')
change('commit_hist[before/50]','commit_hist[before/100]')
change('else if(before<250)','else if(before<500)')
change('(next>>(8*i))&255','(next>>(10*i))&1023')
change('if(after<350&&after>=before)','if(after<700&&after>=before)')
change('if(++n>=20&&ma_parity<2&&(epoch&1U)!=ma_parity)','if(++n>=10)')
change('// Learn MA parity from actual ISR observations; startup ordering can shift it.\n // The other cycle has no MA pair, so use it for the 1kHz bus conversion.',
       '// 100us PWM leaves time for the 1kHz bus conversion near TEZ,\n // well before the MA trigger at TEZ+35us. MA pair stays at 10kHz.')
change('(applied>>(8*i))&255','(applied>>(10*i))&1023')
change('p<500-mx-20','p<1000-mx-20')
change(' if(current&1U)return false; // MA pair 10 kHz; MB remains input pulldown.',' // MA pair each PWM cycle: 10kHz; MB remains input pulldown.')
change('active&255','active&1023')
change('(active>>8)&255','(active>>10)&1023')
change('(active>>16)&255','(active>>20)&1023')
change('<=500-row.ca','<=1000-row.ca')
change('<=500-row.cb','<=1000-row.cb')
change('row.b1<500','row.b1<1000')
change('row.b1>=500','row.b1>=1000')
change('(current/2)%decimate','current%decimate')
change('(.5f+(v[i]+z)/bus)*250+.5f','(.5f+(v[i]+z)/bus)*500+.5f')
change('c=c<20?20:(c>230?230:c);pack|=c<<(8*i);','c=c<40?40:(c>460?460:c);pack|=c<<(10*i);')
change('\\"version\\":3','\\"version\\":4,\\"pwm_hz\\":10000,\\"period_ticks\\":1000,\\"sample_hz\\":10000')
change('\\"trigger_us\\":11','\\"trigger_us\\":35')
change('READY MA_OPENLOOP v3','READY MA_OPENLOOP v4 PWM10kHz')
change('cfg.period_ticks=500','cfg.period_ticks=1000')
change('set_compare_value(cmp[i],125)','set_compare_value(cmp[i],250)')
change('p.count_value=140','p.count_value=150')
assert 'row.b1<500' not in s and '>>(8*i)' not in s
(out/'foc_current.cpp').write_text(s,encoding='utf-8',newline='\n')
for name in ['foc_current.h','test.cpp']:
 (out/name).write_text((r/name).read_text(encoding='utf-8'),encoding='utf-8',newline='\n')
