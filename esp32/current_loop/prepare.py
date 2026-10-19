from pathlib import Path
r=Path(__file__).resolve().parent
base=(r.parent/'openloop_sampling/pwm10k_sync_start/foc_current.cpp').read_text(encoding='utf-8')
read=base[base.index('        bool IRAM_ATTR read_adc'):base.index('inline unsigned IRAM_ATTR now32')]
init=base[base.index('bool init(){'):]
init=init.replace('p.count_value=150','p.count_value=200')
init=init.replace('sb.on_full=on_sample;','sb.on_full=on_sample;sb.on_empty=on_commit;')
init=init.replace('"sensor_openloop",6144','"sensor_current",8192').replace('"openloop_console",6144','"current_console",8192')
text=(r/'experiment.inc').read_text(encoding='utf-8').replace('/* ADC */',read).replace('/* INIT */',init)
text=text.replace(r'gain_A_per_V\":4',r'gain_A_per_V\":-4')
(r/'foc_current.cpp').write_text(text,encoding='utf-8',newline='\n')
for name in ['foc_current.h','test.cpp']:
 (r/name).write_text((r.parent/'openloop_sampling/pwm10k_sync_start'/name).read_text(encoding='utf-8'),encoding='utf-8',newline='\n')
