"""Offline VCU logging tests. Every generated capture is simulated."""
import argparse,csv,importlib.util,subprocess
from pathlib import Path
def run(cc,out):
    vcu=Path(__file__).resolve().parents[1]
    out=out.resolve();out.mkdir(parents=True,exist_ok=True)
    common=[cc,'-std=c11','-Wall','-Wextra','-Werror','-D__USE_MINGW_ANSI_STDIO=1']
    for inc in ['common','tests','STM32F407/App/Inc','STM32F407/Protocol/Inc','STM32F407/Bsp/CAN']:
        common+=['-I',str(vcu/inc)]
    suites={
      'test_log_core':['tests/test_log_core.c','common/vcu_log_wire.c','common/vcu_log_store.c'],
      'test_log_stm32':['tests/test_log_stm32.c','STM32F407/App/Src/vcu_flight_log.c','common/vcu_log_wire.c'],
      'test_log_export':['tests/test_log_export.c','common/vcu_log_wire.c','common/vcu_log_store.c','common/vcu_log_motor_decode.c']}
    for name,sources in suites.items():
        exe=out/(name+'.exe')
        subprocess.run(common+[str(vcu/s) for s in sources]+['-o',str(exe)],check=True)
        subprocess.run([str(exe)]+([out.as_posix()+'/'] if name=='test_log_export' else []),check=True)
    folder=sorted((out/'mx_log').glob('flight_*'))[-1]
    for path in folder.glob('*.csv'):
        with path.open(newline='') as f:data=list(csv.reader(f))
        assert all(len(r)==len(data[0]) for r in data),path
    with (folder/'snapshots.csv').open(newline='') as f:data=list(csv.DictReader(f))
    assert data[0]['throttle_raw']=='' and data[0]['left_voltage_dV']==''
    assert data[1]['left_voltage_dV']=='720' and data[1]['left_bus_current_dA']=='100'
    assert data[2]['left_fault_code']=='5' and data[2]['txok_mask']=='1'
    assert data[2]['left_txok_target_rpm']=='350' and data[2]['right_txok_target_rpm']==''
    assert data[3]['left_fault_code']=='0'
    spec=importlib.util.spec_from_file_location('inspect_log',vcu/'tools/inspect_vcu_log.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assert len(module.inspect(folder))==9
    assert 'SIMULATED_HOST_TEST_NOT_FIELD_DATA' in (folder/'metadata.csv').read_text()
    print('PASS CSV columns/units/missing values and timeline; all data SIMULATED')
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cc',default='gcc');p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args();run(a.cc,a.output_dir)
