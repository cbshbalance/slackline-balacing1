"""Host adapter for the exact C++ header included by the OpenCR sketch."""
import ctypes as ct
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / '.tools'))
KEYS = ['p2r','wphi','wphid','wbetad','offset','gain','trigger','relax','return_speed','dead',
        'limit','step_cap','vmax','amax','tolerance','rest','tau']
OUT = ['alpha','beta','dphi','dbeta','Ahat','hold','ref','vref','elapsed','phase','event','folds','saturations']
DEFAULT = dict(zip(KEYS, [.412217905378982,.607864411665610,.102707995607934,.168864153239234,
                         0,9.5,.6,.3,3,1,55,0,420,8000,.5,.060,.030]))

def build(single=False):
    folder = HERE / 'build'; folder.mkdir(exist_ok=True)
    lib = folder / (('core_float' if single else 'core') + ('.dll' if os.name=='nt' else '.so'))
    sources = [HERE/'native.cpp', HERE/'firmware/v23_controller/control_core.h']
    if lib.exists() and lib.stat().st_mtime > max(p.stat().st_mtime for p in sources):
        return lib
    zig = shutil.which('zig')
    if not zig:
        import ziglang
        zig = str(Path(ziglang.__file__).parent / ('zig.exe' if os.name=='nt' else 'zig'))
    cmd = [zig,'c++','-shared','-O2','-std=c++17',str(sources[0]),'-o',str(lib)]
    if os.name != 'nt': cmd += ['-fPIC']
    if single: cmd += ['-DV23_FLOAT']
    env = dict(os.environ, ZIG_GLOBAL_CACHE_DIR=str(folder/'zig-cache'), ZIG_LOCAL_CACHE_DIR=str(folder/'zig-local'))
    subprocess.run(cmd, check=True, env=env)
    return lib

class Core:
    def __init__(self, config=None, single=False):
        self.lib = ct.CDLL(str(build(single)))
        self.lib.core_new.restype = ct.c_void_p
        self.lib.core_delete.argtypes = [ct.c_void_p]
        self.lib.core_reset.argtypes = [ct.c_void_p,ct.c_double]
        self.lib.core_config.argtypes = [ct.c_void_p,ct.POINTER(ct.c_double)]
        self.lib.core_tick.argtypes = [ct.c_void_p,ct.POINTER(ct.c_double),ct.c_int,ct.POINTER(ct.c_double)]
        self.ptr = self.lib.core_new()
        self.config = dict(DEFAULT, **(config or {}))
        self.lib.core_config(self.ptr, (ct.c_double*len(KEYS))(*(self.config[k] for k in KEYS)))
    def reset(self, delta=0): self.lib.core_reset(self.ptr,delta)
    def tick(self, phi, ank, delta, local_delta=None, local_velocity=0, enabled=True):
        a=(ct.c_double*5)(phi,ank,delta,delta if local_delta is None else local_delta,local_velocity)
        out=(ct.c_double*len(OUT))()
        self.lib.core_tick(self.ptr,a,int(enabled),out)
        return dict(zip(OUT,map(float,out)))
    def close(self):
        if self.ptr: self.lib.core_delete(self.ptr); self.ptr=None
    def __del__(self):
        if getattr(self,'ptr',None): self.close()
