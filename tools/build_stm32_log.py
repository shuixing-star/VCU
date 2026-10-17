"""Offline ARMCC5 build of the dedicated VCU logging target; never flashes."""
import argparse,subprocess,xml.etree.ElementTree as ET
from pathlib import Path

def build(toolchain):
    base=Path(__file__).resolve().parents[1]/'STM32F407/MDK-ARM'
    target=ET.parse(base/'STM32F407_FlightLog.uvprojx').find('./Targets/Target')
    option=target.find('TargetOption/TargetArmAds/Cads/VariousControls')
    includes=option.findtext('IncludePath').split(';')
    defines=option.findtext('Define').split(',')
    out=base/'FlightLog';out.mkdir(exist_ok=True)
    scatter=out/'VCU_STM32_Log_1_0_0.sct'
    scatter.write_text('''LR_IROM1 0x08000000 0x00080000 {
 ER_IROM1 0x08000000 0x00080000 {
  *.o (RESET, +First)
  *(InRoot$$Sections)
  .ANY (+RO)
  .ANY (+XO)
 }
 RW_IRAM1 0x20000000 0x0001C000 { .ANY (+RW +ZI) }
 RW_IRAM2 0x2001C000 0x00004000 { .ANY (+RW +ZI) }
}
''')
    objects=[]
    for group in target.findall('Groups/Group'):
        if group.findtext('GroupOption/CommonProperty/IncludeInBuild')=='0':continue
        for file in group.findall('Files/File'):
            if file.findtext('FileOption/CommonProperty/IncludeInBuild')=='0':continue
            source=Path(file.findtext('FilePath').replace('\\','/'))
            if source.suffix.lower() not in ('.c','.s'):continue
            obj=out/(source.stem+'.o');objects.append(obj)
            if source.suffix.lower()=='.s':
                args=[str(toolchain/'armasm.exe'),'--cpu=Cortex-M4.fp.sp','--apcs=interwork','-g']
            else:
                args=[str(toolchain/'armcc.exe'),'--cpu=Cortex-M4.fp.sp','--apcs=interwork','--c99','-O3','--split_sections','-g','-c']
                for path in includes:args+=['-I',path]
                for define in defines:args+=['-D',define]
            args += ['-o',str(obj),str(source)]
            print('Compiling',source,flush=True)
            subprocess.run(args,cwd=base,check=True)
    axf=out/'VCU_STM32_Log_1_0_0.axf'
    subprocess.run([str(toolchain/'armlink.exe'),'--cpu=Cortex-M4.fp.sp','--strict','--scatter',str(scatter),
        '--summary_stderr','--info','summarysizes','--map','--xref','--callgraph','--symbols','--info','sizes,totals,unused,veneers',
        '--list',str(out/'VCU_STM32_Log_1_0_0.map'),'-o',str(axf)]+list(map(str,objects)),cwd=base,check=True)
    subprocess.run([str(toolchain/'fromelf.exe'),'--i32combined','--output',str(out/'VCU_STM32_Log_1_0_0.hex'),str(axf)],cwd=base,check=True)
    subprocess.run([str(toolchain/'fromelf.exe'),'--bin','--output',str(out/'VCU_STM32_Log_1_0_0.bin'),str(axf)],cwd=base,check=True)
    print('Offline STM32 build complete:',axf)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--toolchain',type=Path,default=Path('D:/APP_DATA/APP/Keil_v5/ARM/ARMCC/bin'))
    build(p.parse_args().toolchain)
