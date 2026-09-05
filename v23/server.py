"""Local-only validation, raw recording, synchronized twin and sample replay."""
import argparse
import asyncio
import csv
import io
import json
import math
from pathlib import Path
import sys
import time
import uuid

from aiohttp import web
from core import Core, DEFAULT, HERE
from validation import simulate, save, source_hashes, measured_plane

ROOT=HERE.parent
sys.path.append(str(ROOT/'v22_logger'))
import serial_bridge as sb
from dataset_v22 import Dataset
import analysis_v22

LOGS=HERE/'logs';LOGS.mkdir(exist_ok=True)
STATIC=HERE/'static'

def clean(x):
    if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [clean(v) for v in x]
    if hasattr(x,'tolist'):return clean(x.tolist())
    if isinstance(x,float) and not math.isfinite(x):return None
    return x

class Recording:
    def __init__(self,name='USB'):
        self.data=dict(schema=1,name=name,kind='recording',config=dict(DEFAULT),rows=[],events=[],notes=[],
                       hashes=source_hashes(),geom=dict(R=.433,L1=.259,L2=.375),plane=dict(measured_plane()),summary={})
        self.header=[];self.core=Core();self.last_seq=None;self.complete=False;self.last_delta=0;self.has_config=False;self.last_drop=0
    def line(self,line):
        if line.startswith('# CFG '):
            c=json.loads(line[6:]);changed=any(self.data['config'].get(k)!=v for k,v in c.items())
            self.data['config'].update(c)
            self.data['events'].append(dict(seq=len(self.data['rows']),event='CONFIG',config=c))
            if changed or not self.has_config:
                self.core.close();self.core=Core({k:v for k,v in c.items() if k in DEFAULT});self.core.reset(self.last_delta)
                self.complete=False
            self.has_config=True;return
        if line.startswith('# D,'):self.header=line[4:].split(',');return
        if line.startswith('E,'):
            fields=line.split(',');name=fields[2]
            self.data['events'].append(dict(seq=len(self.data['rows']),t_ms=float(fields[1]),event=name))
            if name in ('PREPARE','STOP','TORQUE_OFF','ZERO_UPRIGHT','ZERO_HANG','CONFIG','ESTIMATOR_RESET'):
                delta=float(fields[4]) if len(fields)>4 else self.last_delta
                self.core.reset(0 if name.startswith('ZERO') else delta);self.complete=self.has_config
            if name in ('DEADLINE','SENSOR_OR_ANGLE','FOLD_TIMEOUT','WRITE_ERROR','TORQUE_ERROR'):
                self.complete=False
            return
        if not line.startswith('D,'):return
        if not self.header:raise ValueError('D header missing; use hdr')
        vals=line[2:].split(',')
        if len(vals)!=len(self.header):raise ValueError('D column count mismatch')
        row={k:float(v) for k,v in zip(self.header,vals)}
        if not all(math.isfinite(v) for v in row.values()):raise ValueError('Non-finite sample')
        for k in ('phi','ank','del_now','t_ms'):
            if k not in row:raise ValueError('Missing '+k)
        if self.last_seq is not None and row.get('seq',self.last_seq+1)!=self.last_seq+1:
            self.complete=False;self.data['notes'].append('Sample gap/reset at row '+str(len(self.data['rows'])))
        self.last_seq=row.get('seq',len(self.data['rows']));self.last_delta=row['del_now']
        if row.get('tx_drop',0)!=self.last_drop:self.complete=False
        self.last_drop=row.get('tx_drop',0)
        out=self.core.tick(row['phi'],row['ank'],row['del_now'],enabled=row.get('start',0)==4)
        row['replay_Ahat']=out['Ahat'];row['replay_hold']=out['hold']
        row['error_Ahat']=abs(out['Ahat']-row.get('Ahat',out['Ahat'])) if self.complete else None
        row['error_hold']=abs(out['hold']-row.get('hold',out['hold'])) if self.complete else None
        row['mismatch']=int(self.complete and (row['error_Ahat']>1e-3 or row['error_hold']>1e-3))
        row['replay_valid']=int(self.complete)
        row['alpha']=row['ank']-row['phi'];row['beta']=row['alpha']+self.data['config']['p2r']*row['del_now']
        c=self.data['config']
        if all(abs(c[k]-DEFAULT[k])<1e-9 for k in ('wphi','wphid','wbetad','p2r')):
            self.data['plane']=dict(measured_plane())
        else:
            # No full physical model accompanies edited weights. Preserve A, but
            # explicitly distinguish this projection from the v21 modal predictor.
            self.data['plane']=dict(P=[[c['wphid']/c['wphi'],0],[0,c['wbetad']]],
                                    prediction_kind='coefficient_projection')
        P=self.data['plane']['P']
        row['beta_pred']=row['beta']+P[1][0]*out['dphi']+P[1][1]*out['dbeta']
        row['phi_pred']=row['phi']+P[0][0]*out['dphi']+P[0][1]*out['dbeta']
        self.data['rows'].append(row)
    def close(self):self.core.close()

class Hub:
    def __init__(self):
        self.session=None;self.source=None;self.recording=None;self.raw=None;self.stem=None
        self.clients=set();self.sent=0;self.last_receive=0.;self.busy=False;self.message=''
    async def publish(self,m):
        for ws in list(self.clients):
            if not ws.closed:await ws.send_json(clean(m))
    async def replace(self,data):
        self.session=data;self.sent=len(data['rows'])
        await self.publish(dict(type='session',data=data))
    def disconnect(self):
        if self.source:self.source.close();self.source=None
        if self.raw:self.raw.close();self.raw=None
        if self.recording:
            data=self.recording.data
            if data['rows']:save(data,self.stem)
            self.recording.close();self.recording=None
    async def loop(self):
        while True:
            if self.source:
                lines=self.source.drain()
                for _,line in lines:
                    self.raw.write(line+'\n')
                    try:self.recording.line(line)
                    except (ValueError,KeyError,json.JSONDecodeError) as ex:
                        self.message=str(ex);self.recording.complete=False
                if lines:self.raw.flush();self.last_receive=time.monotonic()
                if self.source.error:self.message=self.source.error
                self.session=self.recording.data
                if len(self.session['rows'])>self.sent:
                    await self.publish(dict(type='append',rows=self.session['rows'][self.sent:],
                        events=self.session['events'],config=self.session['config'],plane=self.session['plane'],notes=self.session['notes']))
                    self.sent=len(self.session['rows'])
            await self.publish(dict(type='status',connected=self.source is not None,
               stale=self.source is not None and time.monotonic()-self.last_receive>.5,busy=self.busy,message=self.message))
            await asyncio.sleep(.1)

HUB=Hub()

async def ws_handler(request):
    ws=web.WebSocketResponse(heartbeat=20);await ws.prepare(request);HUB.clients.add(ws)
    if HUB.session:await ws.send_json(clean(dict(type='session',data=HUB.session)))
    try:
        async for _ in ws:pass
    finally:HUB.clients.discard(ws)
    return ws

async def api(request):
    action=request.match_info['action']
    try:
        if action not in ('ports','files') and request.method!='POST':
            raise ValueError('POST required')
        if request.method=='POST' and request.content_type!='application/json':
            raise ValueError('JSON required')
        p=await request.json() if request.method=='POST' else {}
        if action=='ports':return web.json_response(dict(ports=sb.list_ports()[0]))
        if action=='files':
            files=[x.name for x in sorted(LOGS.glob('*.json'))]+['baseline.json']
            return web.json_response(dict(files=files))
        if action=='disconnect':HUB.disconnect();return web.json_response(dict(ok=True))
        if action=='connect':
            if HUB.busy:raise ValueError('Simulation is running')
            HUB.disconnect();HUB.recording=Recording(p['port']);HUB.session=HUB.recording.data;HUB.sent=0
            HUB.stem=LOGS/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
            HUB.raw=HUB.stem.with_suffix('.raw.txt').open('w',encoding='utf-8')
            try:
                HUB.source=sb.SerialSource(p['port'],1000000);HUB.source.start()
            except Exception:
                HUB.disconnect();raise
            HUB.message='';HUB.last_receive=time.monotonic()
            await HUB.replace(HUB.recording.data)
            return web.json_response(dict(ok=True))
        if action=='command':
            if not HUB.source:raise ValueError('USB is not connected')
            cmd=str(p['command']).strip()
            if '\n' in cmd or '\r' in cmd or len(cmd)>90:raise ValueError('One command only')
            HUB.source.write(cmd);HUB.raw.write('# CMD '+cmd+'\n');HUB.raw.flush()
            return web.json_response(dict(ok=True))
        if action=='simulate':
            if HUB.source:raise ValueError('Disconnect USB before simulation')
            if HUB.busy:raise ValueError('Simulation is running')
            allowed={'seconds':(1,60),'beta':(-5,5),'phi':(-8,8),'gain':(.5,30),'noise':(0,.3),
                     'release_ms':(-300,500),'arm_ms':(-300,500)}
            args={}
            for key,(lo,hi) in allowed.items():
                if key in p:
                    v=float(p[key])
                    if not math.isfinite(v) or not lo<=v<=hi:raise ValueError(key+' out of range')
                    args[key]=v
            HUB.busy=True
            try:
                result=await asyncio.to_thread(simulate,**args)
                name=time.strftime('sim_%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
                result['name']=name;save(result,LOGS/name);await HUB.replace(result)
            finally:HUB.busy=False
            return web.json_response(clean(result['summary']))
        if action=='load':
            if HUB.source or HUB.busy:raise ValueError('Disconnect USB / wait for simulation')
            name=Path(p['name']).name
            path=HERE/'reports/baseline.json' if name=='baseline.json' else LOGS/name
            await HUB.replace(json.loads(path.read_text(encoding='utf-8')))
            return web.json_response(dict(ok=True))
        if action=='upload':
            if HUB.source or HUB.busy:raise ValueError('Disconnect USB / wait for simulation')
            text=p['text'];name=Path(p.get('name','recording')).name
            if name.endswith('.json'):
                data=json.loads(text)
                if data.get('schema')!=1 or not isinstance(data.get('rows'),list):raise ValueError('Unsupported session')
            else:
                rec=Recording(name)
                if '# D,' in text or text.startswith('D,'):
                    for line in text.splitlines():rec.line(line.strip())
                else:
                    reader=csv.DictReader(io.StringIO(text));rows=list(reader)
                    if not reader.fieldnames:raise ValueError('No CSV header')
                    # v22 recordings: normalize known raw channel aliases only.
                    aliases={'phi_deg':'phi','ank_deg':'ank','del_now_deg':'del_now'}
                    rec.header=[aliases.get(k,k) for k in reader.fieldnames]
                    for row in rows:rec.line('D,'+','.join(row[k] for k in reader.fieldnames))
                    rec.data['notes'].append('CSV has no complete config/events; replay is diagnostic only.')
                    for row in rec.data['rows']:row['replay_valid']=0;row['mismatch']=0
                data=rec.data;rec.close()
            if not data['rows']:raise ValueError('No samples')
            await HUB.replace(data);return web.json_response(dict(ok=True))
        if action=='analyze':
            if not HUB.session:raise ValueError('No session')
            data=HUB.session;c=data['config']
            ds=Dataset(pipe=dict(p2r=c['p2r'],r=-1/c['wphi'],wf=c['wphid'],wb=c['wbetad'],
                                vg=1,c0=-c['offset']/c['wphi'],tau_ms=c['tau']*1000,alpha_mode='ank-phi'))
            header=['t_ms','phi','ank','del_now','hold'];ds.set_header(header)
            for row in data['rows']:
                ds.add_data_row(','.join(str(row.get(k,0)) for k in header))
            result=await asyncio.to_thread(analysis_v22.run,ds,p.get('tool','stats'),p.get('args',{}))
            return web.json_response(clean(result))
        raise ValueError('Unknown operation')
    except Exception as ex:
        return web.json_response(dict(error=f'{type(ex).__name__}: {ex}'),status=400)

@web.middleware
async def local_requests(request,handler):
    if request.host.split(':')[0] not in ('127.0.0.1','localhost'):
        raise web.HTTPForbidden(text='Local host required')
    origin=request.headers.get('Origin')
    if origin and origin!='http://'+request.host:
        raise web.HTTPForbidden(text='Same origin required')
    return await handler(request)

def app():
    a=web.Application(client_max_size=128*1024*1024,middlewares=[local_requests])
    a.router.add_get('/health',lambda r:web.json_response(dict(app='eoreumi-v23',workspace=str(HERE))))
    a.router.add_get('/',lambda r:web.FileResponse(STATIC/'index.html'))
    a.router.add_get('/ws23',ws_handler)
    a.router.add_route('*','/api/{action}',api)
    a.router.add_static('/static/',STATIC)
    a.router.add_get('/three.min.js',lambda r:web.FileResponse(ROOT/'v22_logger/static/three.min.js'))
    a.router.add_get('/twin.js',lambda r:web.FileResponse(ROOT/'v22_logger/static/lg_3d.js'))
    async def lifecycle(a):
        task=asyncio.create_task(HUB.loop())
        yield
        task.cancel();HUB.disconnect()
        try:await task
        except asyncio.CancelledError:pass
    a.cleanup_ctx.append(lifecycle)
    return a

def run(port=8230,sock=None):
    path=HERE/'reports/baseline.json'
    if path.exists():HUB.session=json.loads(path.read_text(encoding='utf-8'))
    if sock is None:web.run_app(app(),host='127.0.0.1',port=port)
    else:web.run_app(app(),sock=sock)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--port',type=int,default=8230);args=ap.parse_args()
    run(port=args.port)
